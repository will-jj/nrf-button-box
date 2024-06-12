/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <soc.h>
#include <assert.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/util.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/led_strip.h>

#include <zephyr/settings/settings.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/usb/class/hid.h>

#include <zephyr/bluetooth/services/bas.h>
#include <bluetooth/services/hids.h>
#include <zephyr/bluetooth/services/dis.h>
#include <dk_buttons_and_leds.h>

#include "app_nfc.h"
#include "battery.h"

#define WHEEL_START_BIT CONFIG_NUM_ON_BOARD_BUTTONS
#define WHEEL_BT_0 BIT(WHEEL_START_BIT)

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define BASE_USB_HID_SPEC_VERSION 0x0101

#define OUTPUT_REPORT_MAX_LEN 1
#define OUTPUT_REPORT_BIT_MASK_CAPS_LOCK 0x02
#define INPUT_REP_KEYS_REF_ID 0
#define OUTPUT_REP_KEYS_REF_ID 0
#define MODIFIER_KEY_POS 0
#define SHIFT_KEY_CODE 0x02
#define SCAN_CODE_POS 2
#define KEYS_MAX_LEN (INPUT_REPORT_KEYS_MAX_LEN - \
					  SCAN_CODE_POS)

#define ADV_LED_BLINK_INTERVAL 500

#define ADV_STATUS_LED DK_LED1
#define CON_STATUS_LED DK_LED2
#define LED_CAPS_LOCK DK_LED3
#define NFC_LED DK_LED4
#define KEY_TEXT_MASK DK_BTN1_MSK
#define KEY_SHIFT_MASK DK_BTN2_MSK
#define KEY_ADV_MASK DK_BTN4_MSK

/* Key used to accept or reject passkey value */
#define KEY_PAIRING_ACCEPT DK_BTN1_MSK
#define KEY_PAIRING_REJECT DK_BTN2_MSK

/* HIDs queue elements. */
#define HIDS_QUEUE_SIZE 10

/* ********************* */
/* Buttons configuration */

/* Note: The configuration below is the same as BOOT mode configuration
 * This simplifies the code as the BOOT mode is the same as REPORT mode.
 * Changing this configuration would require separate implementation of
 * BOOT mode report generation.
 */
#define KEY_CTRL_CODE_MIN 224 /* Control key codes - required 8 of them */
#define KEY_CTRL_CODE_MAX 231 /* Control key codes - required 8 of them */
#define KEY_CODE_MIN 0		  /* Normal key codes */
#define KEY_CODE_MAX 101	  /* Normal key codes */
#define KEY_PRESS_MAX 6		  /* Maximum number of non-control keys \
							   * pressed simultaneously             \
							   */

/* Number of bytes in key report
 *
 * 1B - control keys
 * 1B - reserved
 * rest - non-control keys
 */
#define INPUT_REPORT_KEYS_MAX_LEN (1 + 1 + KEY_PRESS_MAX)

uint8_t button_values = 0;
int8_t axis_values = -127;

/* Current report map construction requires exactly 8 buttons */
BUILD_ASSERT((KEY_CTRL_CODE_MAX - KEY_CTRL_CODE_MIN) + 1 == 8);

/* OUT report internal indexes.
 *
 * This is a position in internal report table and is not related to
 * report ID.
 */
enum
{
	OUTPUT_REP_KEYS_IDX = 0
};

/* INPUT report internal indexes.
 *
 * This is a position in internal report table and is not related to
 * report ID.
 */
enum
{
	INPUT_REP_KEYS_IDX = 0
};

/* HIDS instance. */
BT_HIDS_DEF(hids_obj,
			OUTPUT_REPORT_MAX_LEN,
			INPUT_REPORT_KEYS_MAX_LEN);

static volatile bool is_adv;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE,
				  (CONFIG_BT_DEVICE_APPEARANCE >> 0) & 0xff,
				  (CONFIG_BT_DEVICE_APPEARANCE >> 8) & 0xff),
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_HIDS_VAL),
				  BT_UUID_16_ENCODE(BT_UUID_BAS_VAL)),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static struct conn_mode
{
	struct bt_conn *conn;
	bool in_boot_mode;
} conn_mode[CONFIG_BT_HIDS_MAX_CLIENT_COUNT];

struct gamepad_report_t
{
	//		uint8_t report_id;
	uint16_t buttons;
	int8_t x_axis;
	int8_t y_axis;
	int8_t z_axis;
	int8_t rz_axis;
};
struct gamepad_report_t gamepad;

#define STRIP_NODE DT_ALIAS(led_strip)
#define STRIP_NUM_PIXELS DT_PROP(DT_ALIAS(led_strip), chain_length)
#define THREAD_REDLINE_PRIORITY 5
#define REDLINE_STACK_SIZE 500

K_THREAD_STACK_DEFINE(redline_stack_area, REDLINE_STACK_SIZE);
struct k_thread redline_thread_data;
bool red_lining = false;
#define REDLINE_DELAY_TIME K_MSEC(50)
struct led_rgb pixels[STRIP_NUM_PIXELS];
static const struct device *const strip = DEVICE_DT_GET(STRIP_NODE);

#define RGB(_r, _g, _b)                 \
	{                                   \
		.r = (_r), .g = (_g), .b = (_b) \
	}

static const struct led_rgb pix_red[] = {RGB(0xFF, 0x00, 0x00)};
static const struct led_rgb pix_off[] = {RGB(0x00, 0x00, 0x00)};

/** A discharge curve specific to the power source. */
static const struct battery_level_point levels[] = {
	/* "Curve" here eyeballed from captured data for the [Adafruit
	 * 3.7v 2000 mAh](https://www.adafruit.com/product/2011) LIPO
	 * under full load that started with a charge of 3.96 V and
	 * dropped about linearly to 3.58 V over 15 hours.  It then
	 * dropped rapidly to 3.10 V over one hour, at which point it
	 * stopped transmitting.
	 *
	 * Based on eyeball comparisons we'll say that 15/16 of life
	 * goes between 4.2 and 3.55 V, and 1/16 goes between 3.55 V
	 * and 3.1 V.
	 */

	{10000, 4200},
	{625, 3550},
	{0, 3100},
};

#if CONFIG_NFC_OOB_PAIRING
static struct k_work adv_work;
#endif

static struct k_work pairing_work;
struct pairing_data_mitm
{
	struct bt_conn *conn;
	unsigned int passkey;
};

K_MSGQ_DEFINE(mitm_queue,
			  sizeof(struct pairing_data_mitm),
			  CONFIG_BT_HIDS_MAX_CLIENT_COUNT,
			  4);

static void advertising_start(void)
{
	int err;
	struct bt_le_adv_param *adv_param = BT_LE_ADV_PARAM(
		BT_LE_ADV_OPT_CONNECTABLE |
			BT_LE_ADV_OPT_ONE_TIME,
		BT_GAP_ADV_FAST_INT_MIN_2,
		BT_GAP_ADV_FAST_INT_MAX_2,
		NULL);

	err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd,
						  ARRAY_SIZE(sd));
	if (err)
	{
		if (err == -EALREADY)
		{
			printk("Advertising continued\n");
		}
		else
		{
			printk("Advertising failed to start (err %d)\n", err);
		}

		return;
	}

	is_adv = true;
	printk("Advertising successfully started\n");
}

#if CONFIG_NFC_OOB_PAIRING
static void delayed_advertising_start(struct k_work *work)
{
	advertising_start();
}

void nfc_field_detected(void)
{
	dk_set_led_on(NFC_LED);

	for (int i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++)
	{
		if (!conn_mode[i].conn)
		{
			k_work_submit(&adv_work);
			break;
		}
	}
}

void nfc_field_lost(void)
{
	dk_set_led_off(NFC_LED);
}
#endif

static void pairing_process(struct k_work *work)
{
	int err;
	struct pairing_data_mitm pairing_data;

	char addr[BT_ADDR_LE_STR_LEN];

	err = k_msgq_peek(&mitm_queue, &pairing_data);
	if (err)
	{
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(pairing_data.conn),
					  addr, sizeof(addr));

	printk("Passkey for %s: %06u\n", addr, pairing_data.passkey);
	printk("Press Button 1 to confirm, Button 2 to reject.\n");
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err)
	{
		printk("Failed to connect to %s (%u)\n", addr, err);
		return;
	}

	printk("Connected %s\n", addr);
	dk_set_led_on(CON_STATUS_LED);

	err = bt_hids_connected(&hids_obj, conn);

	if (err)
	{
		printk("Failed to notify HID service about connection\n");
		return;
	}

	for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++)
	{
		if (!conn_mode[i].conn)
		{
			conn_mode[i].conn = conn;
			conn_mode[i].in_boot_mode = false;
			break;
		}
	}

#if CONFIG_NFC_OOB_PAIRING == 0
	for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++)
	{
		if (!conn_mode[i].conn)
		{
			advertising_start();
			return;
		}
	}
#endif
	is_adv = false;
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	int err;
	bool is_any_dev_connected = false;
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Disconnected from %s (reason %u)\n", addr, reason);

	err = bt_hids_disconnected(&hids_obj, conn);

	if (err)
	{
		printk("Failed to notify HID service about disconnection\n");
	}

	for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++)
	{
		if (conn_mode[i].conn == conn)
		{
			conn_mode[i].conn = NULL;
		}
		else
		{
			if (conn_mode[i].conn)
			{
				is_any_dev_connected = true;
			}
		}
	}

	if (!is_any_dev_connected)
	{
		dk_set_led_off(CON_STATUS_LED);
	}

#if CONFIG_NFC_OOB_PAIRING
	if (is_adv)
	{
		printk("Advertising stopped after disconnect\n");
		bt_le_adv_stop();
		is_adv = false;
	}
#else
	advertising_start();
#endif
}

static void security_changed(struct bt_conn *conn, bt_security_t level,
							 enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err)
	{
		printk("Security changed: %s level %u\n", addr, level);
	}
	else
	{
		printk("Security failed: %s level %u err %d\n", addr, level,
			   err);
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.security_changed = security_changed,
};

static void caps_lock_handler(const struct bt_hids_rep *rep)
{
	uint8_t report_val = ((*rep->data) & OUTPUT_REPORT_BIT_MASK_CAPS_LOCK) ? 1 : 0;
	if (report_val)
	{
		red_lining = true;
	}
	else
	{
		red_lining = false;
	}
	// dk_set_led(LED_CAPS_LOCK, report_val);
}

static void hids_outp_rep_handler(struct bt_hids_rep *rep,
								  struct bt_conn *conn,
								  bool write)
{
	printk("Output report read\n");
	char addr[BT_ADDR_LE_STR_LEN];

	if (!write)
	{
		printk("Output report read\n");
		return;
	};

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("Output report has been received %s\n", addr);
	caps_lock_handler(rep);
}

void thread_red_line(void)
{
	int rc;

	while (1)
	{
		if (red_lining)
		{
			memset(&pixels, 0x00, sizeof(pixels));
			memcpy(&pixels[0], &pix_red[0], sizeof(struct led_rgb));
			rc = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
			k_sleep(REDLINE_DELAY_TIME);

			memset(&pixels, 0x00, sizeof(pixels));
			memcpy(&pixels[0], &pix_off[0], sizeof(struct led_rgb));
			rc = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
			k_sleep(REDLINE_DELAY_TIME);
		}
		else
		{
			memset(&pixels, 0x00, sizeof(pixels));
			memcpy(&pixels[0], &pix_off[0], sizeof(struct led_rgb));
			rc = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
			k_sleep(REDLINE_DELAY_TIME);
		}
	}
}

static void hids_pm_evt_handler(enum bt_hids_pm_evt evt,
								struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];
	size_t i;

	for (i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++)
	{
		if (conn_mode[i].conn == conn)
		{
			break;
		}
	}

	if (i >= CONFIG_BT_HIDS_MAX_CLIENT_COUNT)
	{
		printk("Cannot find connection handle when processing PM");
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	switch (evt)
	{
	case BT_HIDS_PM_EVT_BOOT_MODE_ENTERED:
		printk("Boot mode entered %s\n", addr);
		conn_mode[i].in_boot_mode = true;
		break;

	case BT_HIDS_PM_EVT_REPORT_MODE_ENTERED:
		printk("Report mode entered %s\n", addr);
		conn_mode[i].in_boot_mode = false;
		break;

	default:
		break;
	}
}

static void hid_init(void)
{
	int err;
	struct bt_hids_init_param hids_init_obj = {0};
	struct bt_hids_inp_rep *hids_inp_rep;
	struct bt_hids_outp_feat_rep *hids_outp_rep;

	static const uint8_t report_map[] = {

		HID_USAGE_PAGE(HID_USAGE_GEN_DESKTOP),
		HID_USAGE(HID_USAGE_GEN_DESKTOP_GAMEPAD),
		HID_COLLECTION(HID_COLLECTION_APPLICATION),
		HID_USAGE_PAGE(HID_USAGE_GEN_BUTTON),
		HID_USAGE_MIN8(1),
		HID_USAGE_MAX8(16),
		HID_LOGICAL_MIN8(0),
		HID_LOGICAL_MAX8(1),
		HID_REPORT_SIZE(1),
		HID_REPORT_COUNT(16),
		// INPUT (Data,Var,Abs)
		HID_INPUT(0x02),
		HID_USAGE_PAGE(HID_USAGE_GEN_DESKTOP),
		HID_USAGE(HID_USAGE_GEN_DESKTOP_X),
		HID_USAGE(HID_USAGE_GEN_DESKTOP_Y),
		// Z
		HID_USAGE(0x32),
		// Rz
		HID_USAGE(0x35),
		HID_LOGICAL_MIN8(-127),
		HID_LOGICAL_MAX8(127),
		HID_REPORT_SIZE(8),
		HID_REPORT_COUNT(4),
		// INPUT (Data,Var,Abs)
		HID_INPUT(0x02),

		// LED test
		0x95, 0x05, // Report Count (5)
		0x75, 0x01, // Report Size (1)
		0x05, 0x08, // Usage Page (LEDs)
		0x19, 0x01, // Usage Minimum (Num Lock)
		0x29, 0x05, // Usage Maximum (Kana)
		0x91, 0x02, // Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		0x95, 0x01, // Report Count (1)
		0x75, 0x03, // Report Size (3)
		0x91, 0x01, // Output (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)

		HID_END_COLLECTION};

	hids_init_obj.rep_map.data = report_map;
	hids_init_obj.rep_map.size = sizeof(report_map);

	hids_init_obj.info.bcd_hid = BASE_USB_HID_SPEC_VERSION;
	hids_init_obj.info.b_country_code = 0x00;
	hids_init_obj.info.flags = (BT_HIDS_REMOTE_WAKE |
								BT_HIDS_NORMALLY_CONNECTABLE);

	hids_inp_rep =
		&hids_init_obj.inp_rep_group_init.reports[0];
	hids_inp_rep->size = 6; // 1; // 2;
	hids_inp_rep->id = INPUT_REP_KEYS_REF_ID;
	hids_init_obj.inp_rep_group_init.cnt++;

	hids_outp_rep =
		&hids_init_obj.outp_rep_group_init.reports[OUTPUT_REP_KEYS_IDX];
	hids_outp_rep->size = OUTPUT_REPORT_MAX_LEN;
	hids_outp_rep->id = OUTPUT_REP_KEYS_REF_ID;
	hids_outp_rep->handler = hids_outp_rep_handler;
	hids_init_obj.outp_rep_group_init.cnt++;

	hids_init_obj.pm_evt_handler = hids_pm_evt_handler;

	err = bt_hids_init(&hids_obj, &hids_init_obj);
	__ASSERT(err == 0, "HIDS initialization failed\n");
}

static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Passkey for %s: %06u\n", addr, passkey);
}

static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
	int err;

	struct pairing_data_mitm pairing_data;

	pairing_data.conn = bt_conn_ref(conn);
	pairing_data.passkey = passkey;

	err = k_msgq_put(&mitm_queue, &pairing_data, K_NO_WAIT);
	if (err)
	{
		printk("Pairing queue is full. Purge previous data.\n");
	}

	/* In the case of multiple pairing requests, trigger
	 * pairing confirmation which needed user interaction only
	 * once to avoid display information about all devices at
	 * the same time. Passkey confirmation for next devices will
	 * be proccess from queue after handling the earlier ones.
	 */
	if (k_msgq_num_used_get(&mitm_queue) == 1)
	{
		k_work_submit(&pairing_work);
	}
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing cancelled: %s\n", addr);
}

#if CONFIG_NFC_OOB_PAIRING
static void auth_oob_data_request(struct bt_conn *conn,
								  struct bt_conn_oob_info *info)
{
	int err;
	struct bt_le_oob *oob_local = app_nfc_oob_data_get();

	printk("LESC OOB data requested\n");

	if (info->type != BT_CONN_OOB_LE_SC)
	{
		printk("Only LESC pairing supported\n");
		return;
	}

	if (info->lesc.oob_config != BT_CONN_OOB_LOCAL_ONLY)
	{
		printk("LESC OOB config not supported\n");
		return;
	}

	/* Pass only local OOB data. */
	err = bt_le_oob_set_sc_data(conn, &oob_local->le_sc_data, NULL);
	if (err)
	{
		printk("Error while setting OOB data: %d\n", err);
	}
	else
	{
		printk("Successfully provided LESC OOB data\n");
	}
}
#endif

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing completed: %s, bonded: %d\n", addr, bonded);
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];
	struct pairing_data_mitm pairing_data;

	if (k_msgq_peek(&mitm_queue, &pairing_data) != 0)
	{
		return;
	}

	if (pairing_data.conn == conn)
	{
		bt_conn_unref(pairing_data.conn);
		k_msgq_get(&mitm_queue, &pairing_data, K_NO_WAIT);
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing failed conn: %s, reason %d\n", addr, reason);
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = auth_passkey_display,
	.passkey_confirm = auth_passkey_confirm,
	.cancel = auth_cancel,
#if CONFIG_NFC_OOB_PAIRING
	.oob_data_request = auth_oob_data_request,
#endif
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed};

static void button_state_changed(uint32_t button_state)
{
	int err;
	gamepad.buttons = (uint16_t)(button_state >> WHEEL_START_BIT);
	err = bt_hids_inp_rep_send(&hids_obj, conn_mode[0].conn, 0, (uint8_t *)&gamepad, sizeof(gamepad), NULL);
}

static void num_comp_reply(bool accept)
{
	struct pairing_data_mitm pairing_data;
	struct bt_conn *conn;

	if (k_msgq_get(&mitm_queue, &pairing_data, K_NO_WAIT) != 0)
	{
		return;
	}

	conn = pairing_data.conn;

	if (accept)
	{
		bt_conn_auth_passkey_confirm(conn);
		printk("Numeric Match, conn %p\n", conn);
	}
	else
	{
		bt_conn_auth_cancel(conn);
		printk("Numeric Reject, conn %p\n", conn);
	}

	bt_conn_unref(pairing_data.conn);

	if (k_msgq_num_used_get(&mitm_queue))
	{
		k_work_submit(&pairing_work);
	}
}

static void button_changed(uint32_t button_state, uint32_t has_changed)
{
	static bool pairing_button_pressed;

	uint32_t buttons = button_state & has_changed;

	if (k_msgq_num_used_get(&mitm_queue))
	{
		if (buttons & KEY_PAIRING_ACCEPT)
		{
			pairing_button_pressed = true;
			num_comp_reply(true);

			return;
		}

		if (buttons & KEY_PAIRING_REJECT)
		{
			pairing_button_pressed = true;
			num_comp_reply(false);

			return;
		}
	}

	/* Do not take any action if the pairing button is released. */
	if (pairing_button_pressed &&
		(has_changed & (KEY_PAIRING_ACCEPT | KEY_PAIRING_REJECT)))
	{
		pairing_button_pressed = false;

		return;
	}

	// we have done an interrupted you're meant to check this against specific buttons like above :)
	// check if it is in our range pro gamer move
	if (has_changed >= WHEEL_BT_0)
	{
		button_state_changed(button_state);
	}

#if CONFIG_NFC_OOB_PAIRING
	if (has_changed & KEY_ADV_MASK)
	{
		size_t i;

		for (i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++)
		{
			if (!conn_mode[i].conn)
			{
				advertising_start();
				return;
			}
		}

		printk("Cannot start advertising, all connections slots are"
			   " taken\n");
	}
#endif
}

static void configure_gpio(void)
{
	int err;

	err = dk_buttons_init(button_changed);
	if (err)
	{
		printk("Cannot init buttons (err: %d)\n", err);
	}

	err = dk_leds_init();
	if (err)
	{
		printk("Cannot init LEDs (err: %d)\n", err);
	}
}

static void bas_notify(void)
{
	int batt_mV = battery_sample();
	unsigned int batt_pptt = battery_level_pptt(batt_mV, levels) / 100;
	bt_bas_set_battery_level(batt_pptt);
}

int main(void)
{

	int rc = battery_measure_enable(true);

	if (rc != 0)
	{
		printk("Failed to initialize battery measurement: %d\n", rc);
		return 0;
	}

	int err;

	int blink_status = 0;

	printk("Starting Bluetooth Peripheral HIDS keyboard example\n");

	configure_gpio();

	err = bt_conn_auth_cb_register(&conn_auth_callbacks);
	if (err)
	{
		printk("Failed to register authorization callbacks.\n");
		return 0;
	}

	err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
	if (err)
	{
		printk("Failed to register authorization info callbacks.\n");
		return 0;
	}

	hid_init();

	err = bt_enable(NULL);
	if (err)
	{
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	printk("Bluetooth initialized\n");

	if (IS_ENABLED(CONFIG_SETTINGS))
	{
		settings_load();
	}

#if CONFIG_NFC_OOB_PAIRING
	k_work_init(&adv_work, delayed_advertising_start);
	app_nfc_init();
#else
	advertising_start();
#endif

	k_work_init(&pairing_work, pairing_process);

	// if (device_is_ready(strip))
	//{
	k_tid_t my_tid = k_thread_create(&redline_thread_data, redline_stack_area,
									 K_THREAD_STACK_SIZEOF(redline_stack_area),
									 thread_red_line,
									 NULL, NULL, NULL,
									 THREAD_REDLINE_PRIORITY, 0, K_NO_WAIT);
	memset(&pixels, 0x00, sizeof(pixels));
	memcpy(&pixels[0], &pix_red[0], sizeof(struct led_rgb));
	rc = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
	//}

	for (;;)
	{
		if (is_adv)
		{
			dk_set_led(ADV_STATUS_LED, (++blink_status) % 2);
		}
		else
		{
			dk_set_led_off(ADV_STATUS_LED);
		}
		bas_notify();
		k_sleep(K_MSEC(ADV_LED_BLINK_INTERVAL));
	}
}
