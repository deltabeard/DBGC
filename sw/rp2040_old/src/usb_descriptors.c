/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <tusb.h>
#include <pico/unique_id.h>

/* A combination of interfaces must have a unique product id, since PC will save device driver after the first plug.
 * Same VID/PID with different interface e.g MSC (first), then CDC (later) will possibly cause system error on PC.
 *
 * Auto ProductID layout's Bitmap:
 *   [MSB]         HID | MSC | CDC          [LSB]
 */
#define _PID_MAP(itf, n)  ( (CFG_TUD_##itf) << (n) )
#define USB_PID           (0x4000 | _PID_MAP(CDC, 0) | _PID_MAP(MSC, 1) | _PID_MAP(HID, 2) | \
                           _PID_MAP(MIDI, 3) | _PID_MAP(VENDOR, 4) )

//--------------------------------------------------------------------+
// Device Descriptors
//--------------------------------------------------------------------+

// Invoked when received GET DEVICE DESCRIPTOR
// Application return pointer to descriptor
uint8_t const *tud_descriptor_device_cb(void)
{
	/* FIXME: This makes unauthorised use of the Microchip Vendor. */
	static const tusb_desc_device_t desc_device = {
		.bLength            = sizeof(tusb_desc_device_t),
		.bDescriptorType    = TUSB_DESC_DEVICE,
		.bcdUSB             = 0x0200,
		.bDeviceClass       = 0x00,
		.bDeviceSubClass    = 0x00,
		.bDeviceProtocol    = 0x00,
		.bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
		.idVendor           = 0x04D8, /* Microchip */
		.idProduct          = 0x003F,
		.bcdDevice          = 0x0100,
		.iManufacturer      = 0x01,
		.iProduct           = 0x02,
		.iSerialNumber      = 0x03,
		.bNumConfigurations = 0x01
	};
	return (const uint8_t *)&desc_device;
}

//--------------------------------------------------------------------+
// HID Report Descriptor
//--------------------------------------------------------------------+

static const uint8_t desc_hid_report[] = {
	TUD_HID_REPORT_DESC_GENERIC_INOUT(CFG_TUD_HID_EP_BUFSIZE)
};

// Invoked when received GET HID REPORT DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
const uint8_t *tud_hid_descriptor_report_cb(uint8_t itf)
{
	(void) itf;
	return desc_hid_report;
}

//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+

enum {
	ITF_NUM_HID,
	ITF_NUM_TOTAL
};

#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_HID_INOUT_DESC_LEN)
#define EPNUM_HID   0x01

// Invoked when received GET CONFIGURATION DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
	static const uint8_t desc_configuration[] = {
		// Config number, interface count, string index, total length,
		// 	attribute, power in mA
		TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN,
			TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

		// Interface number, string index, protocol,
		// 	report descriptor len, EP In & Out address,
		// 	size & polling interval
		TUD_HID_INOUT_DESCRIPTOR(ITF_NUM_HID, 0, HID_ITF_PROTOCOL_NONE,
			sizeof(desc_hid_report), EPNUM_HID, 0x80 | EPNUM_HID,
			CFG_TUD_HID_EP_BUFSIZE, 10)
	};
	(void) index; // for multiple configurations
	return desc_configuration;
}

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+

// Invoked when received GET STRING DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
const uint16_t *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
	static uint16_t desc[32];
	uint8_t chr_count;

	(void) langid;

	switch(index)
	{
	/* 0: supported language: English (0x0409) */
	case 0:
		desc[1] = 0x0409;
		chr_count = 1;
		break;
	/* 1: Manufacturer */
	case 1:
	/* 2: Product */
	case 2:
	{
		/* Strings must fit within _desc_str. */
		char const *string_desc_arr[] = {
			"",
			"Deltabeard",
			"Deltabeard's Game Boy Cart",
		};
		const char *str = string_desc_arr[index];
		chr_count = strlen(str);

		// Convert ASCII string into UTF-16
		for(uint8_t i = 0; i < chr_count; i++)
			desc[1 + i] = str[i];

		break;
	}
	/* 3: Serial. */
	case 3:
	{
#define ID_LEN (2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1)
		char id_buf[ID_LEN];
		/* TODO: optimisation: make output wchar. */
		pico_get_unique_board_id_string(id_buf, ID_LEN);
#undef ID_LEN

		/* TODO: strlen could be optimised away here. */
		chr_count = strlen(id_buf);
		// Convert ASCII string into UTF-16
		for(uint8_t i = 0; i < chr_count; i++)
			desc[1 + i] = id_buf[i];

		break;
	}
	default:
		// Note: the 0xEE index string is a Microsoft OS 1.0 Descriptors.
		// https://docs.microsoft.com/en-us/windows-hardware/drivers/usbcon/microsoft-defined-usb-descriptors
		return NULL;
	}

	// first byte is length (including header), second byte is string type
	desc[0] = (TUSB_DESC_STRING << 8) | (2 * chr_count + 2);

	return desc;
}
