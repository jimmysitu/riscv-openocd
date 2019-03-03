/**************************************************************************
 *   Copyright (C) 2016 by besonzore                                       *
 *   1670739974@qq.com                                                     *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#ifndef W_JTAG_SJ_H_
#define W_JTAG_SJ_H_

#include <stdbool.h>
#include "helper/binarybuffer.h"
#include <libusb.h>

/* cmd */
#define WJTAGSJ_CMD_IDLE                 0x00    //doing nothing
#define WJTAGSJ_CMD_TDI_OUTPUT_ONLY      0x01
#define WJTAGSJ_CMD_TDO_INPUT_ONLY       0x02
#define WJTAGSJ_CMD_TDIO_INOUT           0x03
#define WJTAGSJ_CMD_TMS_OUTPUT_ONLY      0x04
#define WJTAGSJ_CMD_ENABLE_SWDIO         0x05
#define WJTAGSJ_CMD_ADAPTER_CTRL         0x06
#define WJTAGSJ_SUBCMD_SET_SPEED            0x00
#define WJTAGSJ_SUBCMD_RST_CONTROL          0x01
#define WJTAGSJ_SUBCMD_ENABLE_SWD           0x02
#define WJTAGSJ_SUBCMD_ENABLE_SWDO          0x03

/* cmd mask */
#define WJTAGSJ_CMD_MASK            0x07
#define WJTAGSJ_LAST_BIT_MASK       0x08
#define WJTAGSJ_LENGTH_MASK         0xF0
#define WJTAGSJ_PARAM_IN_CMD_MSDK   0xF8

struct wjtagsj_ctx {
	libusb_context *usb_ctx;
	libusb_device_handle *usb_dev;
	unsigned int usb_write_timeout;
	unsigned int usb_read_timeout;
	uint8_t in_ep;
	uint8_t out_ep;
	uint16_t max_packet_size;
	uint16_t index;
	uint8_t interface;
	uint8_t *write_buffer;
	unsigned write_size;
	unsigned write_count;
	uint8_t *read_buffer;
	unsigned read_size;
	unsigned read_count;
	uint8_t *read_chunk;
	unsigned read_chunk_size;
	struct bit_copy_queue read_queue;
	int retval;
};

/* Device handling */
struct wjtagsj_ctx *wjtagsj_open(const uint16_t *vid, const uint16_t *pid, const char *description,
	const char *serial, int channel);
void wjtagsj_close(struct wjtagsj_ctx *ctx);

/* Command queuing. These correspond to the wjtagsj commands with the same names, but no need to care
 * about bit/byte transfer or data length limitation. Read data is guaranteed to be available only
 * after the following wjtagsj_flush(). */
void wjtagsj_clock_data_out(struct wjtagsj_ctx *ctx, const uint8_t *out, unsigned out_offset,
			 unsigned length, bool last_bit);
void wjtagsj_clock_data_in(struct wjtagsj_ctx *ctx, uint8_t *in, unsigned in_offset, unsigned length,
			bool last_bit);
void wjtagsj_clock_data(struct wjtagsj_ctx *ctx, const uint8_t *out, unsigned out_offset, uint8_t *in,
		     unsigned in_offset, unsigned length, bool last_bit);
void wjtagsj_clock_tms_cs_out(struct wjtagsj_ctx *ctx, const uint8_t *out, unsigned out_offset,
			   unsigned length);
void wjtagsj_set_clk_speed(struct wjtagsj_ctx *ctx, uint8_t data);
void wjtagsj_set_rst_ctrl(struct wjtagsj_ctx *ctx, uint8_t data);
void wjtagsj_enable_swd(struct wjtagsj_ctx *ctx, uint8_t data);
void wjtagsj_enable_swdio(struct wjtagsj_ctx *ctx, uint8_t data);
void wjtagsj_enable_swo(struct wjtagsj_ctx *ctx, uint8_t data,unsigned int baudrate);
/* Helper to set frequency in Hertz. Returns actual realizable frequency or negative error.
 * Frequency 0 means RTCK. */
int wjtagsj_set_frequency(struct wjtagsj_ctx *ctx, int frequency);

/* Queue handling */
int wjtagsj_flush(struct wjtagsj_ctx *ctx);
void wjtagsj_purge(struct wjtagsj_ctx *ctx);

#endif /* W_JTAG_SJ_H_ */
