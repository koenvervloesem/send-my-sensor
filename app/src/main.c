/*
 * Send My Sensor - Upload sensor data via Apple's Find My network
 *
 * Copyright (c) 2021 Koen Vervloesem
 * Copyright (c) 2021 Positive Security
 *
 * SPDX-License-Identifier: AGPL-3.0
 */

#include <bluetooth/addr.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/sensor.h>
#include <drivers/uart.h>
#include <stddef.h>
#include <stdio.h>
#include <sys/printk.h>
#include <sys/util.h>
#include <usb/usb_device.h>
#include <zephyr/types.h>
#include <zephyr.h>

#include "openhaystack.h"
#include "uECC.h"

#ifdef CONFIG_USB_UART_CONSOLE
#include "debug.h"
#endif

#define CHECK_BIT(var, pos) ((var) & (1 << (7 - pos)))

// Bluetooth identity used to advertise
int bt_id = BT_ID_DEFAULT;

// Set custom modem id before flashing:
static uint32_t modem_id = 0x42424242;

static bt_addr_le_t address = {BT_ADDR_LE_RANDOM,
                               {0xFF, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF}};

/*
 * Get a device structure from a devicetree node with compatible
 * "bosch,bme280". (If there are multiple, just pick one.)
 */
static const struct device *get_bme280_device(void)
{
	const struct device *dev = DEVICE_DT_GET_ANY(bosch_bme280);

	if (dev == NULL) {
		/* No such node, or the node does not have status "okay". */
		printk("\nError: no device found.\n");
		return NULL;
	}

	if (!device_is_ready(dev)) {
		printk("\nError: Device \"%s\" is not ready; "
		       "check the driver initialization logs for errors.\n",
		       dev->name);
		return NULL;
	}

	printk("Found device \"%s\", getting sensor data\n", dev->name);
	return dev;
}

uint32_t swap_uint32(uint32_t val) {
  val = ((val << 8) & 0xFF00FF00) | ((val >> 8) & 0xFF00FF);
  return (val << 16) | (val >> 16);
};

int is_valid_pubkey(uint8_t *pub_key_compressed) {
  uint8_t with_sign_byte[29];
  uint8_t pub_key_uncompressed[128];
  const struct uECC_Curve_t *curve = uECC_secp224r1();
  with_sign_byte[0] = 0x02;
  memcpy(&with_sign_byte[1], pub_key_compressed, 28);
  uECC_decompress(with_sign_byte, pub_key_uncompressed, curve);
  if (!uECC_valid_public_key(pub_key_uncompressed, curve)) {
    printk("Generated public key tested as invalid");
    return 0;
  }
  return 1;
}

void pub_from_priv(uint8_t *pub_compressed, uint8_t *priv) {
  const struct uECC_Curve_t *curve = uECC_secp224r1();
  uint8_t pub_key_tmp[128];
  uECC_compute_public_key(priv, pub_key_tmp, curve);
  uECC_compress(pub_key_tmp, pub_compressed, curve);
}

void copy_4b_big_endian(uint8_t *dst, uint8_t *src) {
  dst[0] = src[3];
  dst[1] = src[2];
  dst[2] = src[1];
  dst[3] = src[0];
}

// index as first part of payload to have an often changing MAC address
// [2b magic] [4byte index] [4byte msg_id] [4byte modem_id] [000.000] [1bit]
// There is a trade-off between sending and receiving throughput (e.g. we could
// also use a 1-byte lookup table)
void set_addr_and_payload_for_bit(uint32_t index, uint32_t msg_id,
                                  uint8_t bit) {
  uint32_t valid_key_counter = 0;
  static uint8_t public_key[28] = {0};
  public_key[0] = 0xBA; // magic value
  public_key[1] = 0xBE;
  copy_4b_big_endian(&public_key[2], &index);
  copy_4b_big_endian(&public_key[6], &msg_id);
  copy_4b_big_endian(&public_key[10], &modem_id);
  public_key[27] = bit;
  do {
    copy_4b_big_endian(&public_key[14], &valid_key_counter);
    // here, you could call `pub_from_priv(public_key, private_key)` to instead
    // treat the payload as private key
    valid_key_counter++; // for next round
  } while (!is_valid_pubkey(public_key));
  printk("  pub key to use (%d. try): %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x ... %02x", valid_key_counter, public_key[0], public_key[1], public_key[2], public_key[3], public_key[4], public_key[5], public_key[6], public_key[7], public_key[8], public_key[9], public_key[10], public_key[11], public_key[12], public_key[13],public_key[14], public_key[15],public_key[16],public_key[17],public_key[19], public_key[19], public_key[20], public_key[21], public_key[22], public_key[23], public_key[24], public_key[25], public_key[26],  public_key[27]);
  of_set_address_from_key(&address, public_key);
  of_set_manufacturer_data_from_key(of_manufacturer_data, public_key);
}

void reset_advertising() {

  int err;

  // Stop advertising
  err = bt_le_adv_stop();
  if (err) {
    printk("Advertising failed to stop (err %d)\n", err);
    return;
  }

  err = bt_id_reset(bt_id, &address, NULL);
  if (err < 0) {
    printk("Can't reset identity %d (err %d)\n", bt_id, err);
    return;
  }

  // Start advertising
  err = bt_le_adv_start(&of_adv_param, of_advertising_data, ARRAY_SIZE(of_advertising_data), NULL, 0);
  if (err) {
    printk("Advertising failed to start (err %d)\n", err);
    return;
  }

}

void send_data_once_blocking(uint8_t *data_to_send, uint32_t len,
                             uint32_t msg_id) {

  int err;
  uint8_t current_bit = 0;

  printk("Data to send (msg_id: %d): %s", msg_id, data_to_send);

  // iterate byte-by-byte
  for (int by_i = 0; by_i < len; by_i++) {
    printk("  Sending byte %d/%d (0x%02x)", by_i, len-1, data_to_send[by_i]);
    // iterate bit-by-bit
    for (int bi_i = 0; bi_i < 8; bi_i++) {
      if (CHECK_BIT(data_to_send[by_i], bi_i)) {
        current_bit = 1;
      } else {
        current_bit = 0;
      }
      printk("  Sending byte %d, bit %d: %d", by_i, bi_i, current_bit);
      set_addr_and_payload_for_bit(by_i * 8 + bi_i, msg_id, current_bit);
      printk("    resetting. Will now use device address: %02x %02x %02x %02x %02x %02x", address.a.val[5], address.a.val[4], address.a.val[3], address.a.val[2], address.a.val[1], address.a.val[0]);
      reset_advertising();
      k_sleep(K_MSEC(100));

    }
  }
  // Stop advertising
  err = bt_le_adv_stop();
  if (err) {
    printk("Advertising failed to stop (err %d)\n", err);
    return;
  }
}

void main(void) {
  int err;

  printk("Starting OpenHaystack firmware...\n");

  const struct device *dev = get_bme280_device();

  if (dev == NULL) {
    return;
  }

  // Create identity
  bt_id = bt_id_create(&address, NULL);
  if (bt_id < 0) {
    printk("Can't create new identity (err %d)\n", bt_id);
    return;
  } else {
    printk("Created new identity %d\n", bt_id);
  }
  address.a.val[0]++;
  // Create a second identity because we need to be able to reset it later.
  // Zephyr doesn't allow resetting the default identity.
  bt_id = bt_id_create(&address, NULL);
  if (bt_id < 0) {
    printk("Can't create new identity (err %d)\n", bt_id);
    return;
  } else {
    printk("Created new identity %d\n", bt_id);
  }

  // Initialize the Bluetooth subsystem
  err = bt_enable(NULL);
  if (err) {
    printk("Bluetooth init failed (err %d)\n", err);
  }

  printk("Bluetooth initialized\n");

  // Send initial test message after boot
  uint32_t current_message_id = 0;
  static uint8_t data_to_send[] = "TEST MESSAGE";
  for(int i=0; i<3; i++) {
    send_data_once_blocking(data_to_send, sizeof(data_to_send),
                            current_message_id);
  }

  while(1) {
    static uint8_t data[6];
    struct sensor_value temp;

    sensor_sample_fetch(dev);
    sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
    printk("temp: %06d.%06d\n", temp.val1, temp.val2);

    snprintf(data, 7, "%06d", temp.val1);
    current_message_id++;
    for(int i=0; i<3; i++) {
      send_data_once_blocking(data, sizeof(data),
                              current_message_id);
    }

  }
}

#ifdef CONFIG_USB_UART_CONSOLE
// Task for starting up the USB console
K_THREAD_DEFINE(console_id, STACKSIZE, console_init, NULL, NULL, NULL, PRIORITY - 2, 0, 0);
#endif
