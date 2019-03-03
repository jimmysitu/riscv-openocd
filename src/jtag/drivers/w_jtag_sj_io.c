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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "log.h"
#include "libusb1_common.h"


#include <jtag/interface.h>
#include <jtag/commands.h>

#include "w_jtag_sj_io.h"
#include "helper/log.h"


/* Compatibility define for older libusb-1.0 */
#ifndef LIBUSB_CALL
#define LIBUSB_CALL
#endif

#ifdef _DEBUG_JTAG_IO_
#define DEBUG_IO(expr...) LOG_DEBUG(expr)
#define DEBUG_PRINT_BUF(buf, len) \
	do { \
		char buf_string[32 * 3 + 1]; \
		int buf_string_pos = 0; \
		for (int i = 0; i < len; i++) { \
			buf_string_pos += sprintf(buf_string + buf_string_pos, " %02x", buf[i]); \
			if (i % 32 == 32 - 1) { \
				LOG_DEBUG("%s", buf_string); \
				buf_string_pos = 0; \
			} \
		} \
		if (buf_string_pos > 0) \
			LOG_DEBUG("%s", buf_string);\
	} while (0)
#else
#define DEBUG_IO(expr...) do {} while (0)
#define DEBUG_PRINT_BUF(buf, len) do {} while (0)
#endif

#define W_JTAG_SJ_DEVICE_OUT_REQTYPE (LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE)
#define W_JTAG_SJ_RESET_BUF           0x55



static unsigned char clk_speed =6;
static float delay_coeff[9] = {8.6 , 1 , 1 , 1 , 1 , 1.4 , 2, 3.8 , 8.6 };
/* Returns true if the string descriptor indexed by str_index in device matches string */
static bool string_descriptor_equal(libusb_device_handle *device, uint8_t str_index,
                                    const char *string)
{
	int retval;
	char desc_string[256]; /* Max size of string descriptor */
	retval = libusb_get_string_descriptor_ascii(device, str_index, (unsigned char *)desc_string,
	                sizeof(desc_string));
	if (retval < 0) {
		LOG_ERROR("libusb_get_string_descriptor_ascii() failed with %s", libusb_error_name(retval));
		return false;
	}
	return strncmp(string, desc_string, sizeof(desc_string)) == 0;
}

/* Helper to open a libusb device that matches vid, pid, product string and/or serial string.
 * Set any field to 0 as a wildcard. If the device is found true is returned, with ctx containing
 * the already opened handle. ctx->interface must be set to the desired interface (channel) number
 * prior to calling this function. */
static bool open_matching_device(struct wjtagsj_ctx *ctx, const uint16_t *vid, const uint16_t *pid,
                                 const char *product, const char *serial)
{
	libusb_device **list;
	struct libusb_device_descriptor desc;
	struct libusb_config_descriptor *config0;
	int err;
	bool found = false;
	ssize_t cnt = libusb_get_device_list(ctx->usb_ctx, &list);
	if (cnt < 0)
		LOG_ERROR("libusb_get_device_list() failed with %s", libusb_error_name(cnt));

	for (ssize_t i = 0; i < cnt; i++) {
		libusb_device *device = list[i];

		err = libusb_get_device_descriptor(device, &desc);
		if (err != LIBUSB_SUCCESS) {
			LOG_ERROR("libusb_get_device_descriptor() failed with %s", libusb_error_name(err));
			continue;
		}

		if (vid && *vid != desc.idVendor)
			continue;
		if (pid && *pid != desc.idProduct)
			continue;

		err = libusb_open(device, &ctx->usb_dev);
		if (err != LIBUSB_SUCCESS) {
			LOG_ERROR("libusb_open() failed with %s",
			          libusb_error_name(err));
			continue;
		}

		if (product && !string_descriptor_equal(ctx->usb_dev, desc.iProduct, product)) {
			libusb_close(ctx->usb_dev);
			continue;
		}

		if (serial && !string_descriptor_equal(ctx->usb_dev, desc.iSerialNumber, serial)) {
			libusb_close(ctx->usb_dev);
			continue;
		}

		found = true;
		break;
	}

	libusb_free_device_list(list, 1);

	if (!found) {
		LOG_ERROR("no device found");
		return false;
	}

	err = libusb_get_config_descriptor(libusb_get_device(ctx->usb_dev), 0, &config0);
	if (err != LIBUSB_SUCCESS) {
		LOG_ERROR("libusb_get_config_descriptor() failed with %s", libusb_error_name(err));
		libusb_close(ctx->usb_dev);
		return false;
	}

	/* Make sure the first configuration is selected */
	int cfg;
	err = libusb_get_configuration(ctx->usb_dev, &cfg);
	if (err != LIBUSB_SUCCESS) {
		LOG_ERROR("libusb_get_configuration() failed with %s", libusb_error_name(err));
		goto error;
	}

	if (desc.bNumConfigurations > 0 && cfg != config0->bConfigurationValue) {
		err = libusb_set_configuration(ctx->usb_dev, config0->bConfigurationValue);
		if (err != LIBUSB_SUCCESS) {
			LOG_ERROR("libusb_set_configuration() failed with %s", libusb_error_name(err));
			goto error;
		}
	}

	err = libusb_claim_interface(ctx->usb_dev, ctx->interface);
	if (err != LIBUSB_SUCCESS) {
		LOG_ERROR("libusb_claim_interface() failed with %s", libusb_error_name(err));
		goto error;
	}

	/* Reset buf*/
	err = libusb_control_transfer(ctx->usb_dev, W_JTAG_SJ_DEVICE_OUT_REQTYPE,
	                              W_JTAG_SJ_RESET_BUF, 0,
	                              ctx->index, NULL, 0, ctx->usb_write_timeout);
	if (err < 0) {
		LOG_ERROR("failed to reset interface: %s", libusb_error_name(err));
		goto error;
	}

	/* Determine maximum packet size and endpoint addresses */
	if (!(desc.bNumConfigurations > 0 && ctx->interface < config0->bNumInterfaces
	      && config0->interface[ctx->interface].num_altsetting > 0))
		goto desc_error;

	const struct libusb_interface_descriptor *descriptor;
	descriptor = &config0->interface[ctx->interface].altsetting[0];
	if (descriptor->bNumEndpoints != 2)
		goto desc_error;

	ctx->in_ep = 0;
	ctx->out_ep = 0;
	for (int i = 0; i < descriptor->bNumEndpoints; i++) {
		if (descriptor->endpoint[i].bEndpointAddress & 0x80) {
			ctx->in_ep = descriptor->endpoint[i].bEndpointAddress;
			ctx->max_packet_size =
			        descriptor->endpoint[i].wMaxPacketSize;
		} else {
			ctx->out_ep = descriptor->endpoint[i].bEndpointAddress;
		}
	}

	if (ctx->in_ep == 0 || ctx->out_ep == 0)
		goto desc_error;

	libusb_free_config_descriptor(config0);
	return true;

desc_error:
	LOG_ERROR("unrecognized USB device descriptor");
error:
	libusb_free_config_descriptor(config0);
	libusb_close(ctx->usb_dev);
	return false;
}

struct wjtagsj_ctx *wjtagsj_open(const uint16_t *vid, const uint16_t *pid, const char *description,
                                 const char *serial, int channel) {
	struct wjtagsj_ctx *ctx = calloc(1, sizeof(*ctx));
	int err;

	if (!ctx)
		return 0;

	bit_copy_queue_init(&ctx->read_queue);
	ctx->read_chunk_size = 1024*16;
	ctx->read_size = 1024*4;
	ctx->write_size = 1024*8;
	ctx->read_chunk = malloc(ctx->read_chunk_size);
	ctx->read_buffer = malloc(ctx->read_size);
	ctx->write_buffer = malloc(ctx->write_size);
	if (!ctx->read_chunk || !ctx->read_buffer || !ctx->write_buffer)
		goto error;

	ctx->interface = channel;
	ctx->index = channel + 1;
	ctx->usb_read_timeout = 5000;
	ctx->usb_write_timeout = 5000;

	err = libusb_init(&ctx->usb_ctx);
	if (err != LIBUSB_SUCCESS) {
		LOG_ERROR("libusb_init() failed with %s", libusb_error_name(err));
		goto error;
	}

	if (!open_matching_device(ctx, vid, pid, description, serial)) {
		/* Four hex digits plus terminating zero each */
		char vidstr[5];
		char pidstr[5];
		LOG_ERROR("unable to open interface with vid %s, pid %s, description '%s' and "
		          "serial '%s'",
		          vid ? sprintf(vidstr, "%04x", *vid), vidstr : "*",
		          pid ? sprintf(pidstr, "%04x", *pid), pidstr : "*",
		          description ? description : "*",
		          serial ? serial : "*");
		ctx->usb_dev = 0;
		goto error;
	}

	wjtagsj_purge(ctx);

	return ctx;
error:
	wjtagsj_close(ctx);
	return 0;
}

void wjtagsj_close(struct wjtagsj_ctx *ctx)
{
	if (ctx->usb_dev)
		libusb_close(ctx->usb_dev);
	if (ctx->usb_ctx)
		libusb_exit(ctx->usb_ctx);
	bit_copy_discard(&ctx->read_queue);
	if (ctx->write_buffer)
		free(ctx->write_buffer);
	if (ctx->read_buffer)
		free(ctx->read_buffer);
	if (ctx->read_chunk)
		free(ctx->read_chunk);

	free(ctx);
}

void wjtagsj_purge(struct wjtagsj_ctx *ctx)
{
	int err;
	LOG_DEBUG("-");
	ctx->write_count = 0;
	ctx->read_count = 0;
	ctx->retval = ERROR_OK;
	bit_copy_discard(&ctx->read_queue);
	err = libusb_control_transfer(ctx->usb_dev, W_JTAG_SJ_DEVICE_OUT_REQTYPE, W_JTAG_SJ_RESET_BUF,
	                              0, ctx->index, NULL, 0, ctx->usb_write_timeout);
	if (err < 0) {
		LOG_ERROR("unable to purge interface rx and tx buffers: %s", libusb_error_name(err));
		return;
	}
}

static unsigned buffer_write_space(struct wjtagsj_ctx *ctx)
{
	return ctx->write_size - ctx->write_count;
}

static unsigned buffer_read_space(struct wjtagsj_ctx *ctx)
{
	return ctx->read_size - ctx->read_count;
}

static void buffer_write_byte(struct wjtagsj_ctx *ctx, uint8_t data)
{
	DEBUG_IO("%02x", data);
	assert(ctx->write_count < ctx->write_size);
	ctx->write_buffer[ctx->write_count++] = data;
}

static unsigned buffer_write(struct wjtagsj_ctx *ctx, const uint8_t *out, unsigned out_offset,
                             unsigned bit_count)
{
	DEBUG_IO("%d bits", bit_count);
	assert(ctx->write_count + DIV_ROUND_UP(bit_count, 8) <= ctx->write_size);
	bit_copy(ctx->write_buffer + ctx->write_count, 0, out, out_offset, bit_count);
	ctx->write_count += DIV_ROUND_UP(bit_count, 8);
	return bit_count;
}

static unsigned buffer_add_read(struct wjtagsj_ctx *ctx, uint8_t *in, unsigned in_offset,
                                unsigned bit_count, unsigned offset)
{
	DEBUG_IO("%d bits, offset %d", bit_count, offset);
	assert(ctx->read_count + DIV_ROUND_UP(bit_count, 8) <= ctx->read_size);
	bit_copy_queued(&ctx->read_queue, in, in_offset, ctx->read_buffer + ctx->read_count, offset,
	                bit_count);
	ctx->read_count += DIV_ROUND_UP(bit_count, 8);
	return bit_count;
}

void wjtagsj_clock_data_out(struct wjtagsj_ctx *ctx, const uint8_t *out, unsigned out_offset,
                            unsigned length,bool last_bit)
{
	wjtagsj_clock_data(ctx, out, out_offset, 0, 0, length,last_bit);
}

void wjtagsj_clock_data_in(struct wjtagsj_ctx *ctx, uint8_t *in, unsigned in_offset, unsigned length,
                           bool last_bit)
{
	wjtagsj_clock_data(ctx, 0, 0, in, in_offset, length, last_bit);
}

void wjtagsj_clock_data(struct wjtagsj_ctx *ctx, const uint8_t *out, unsigned out_offset, uint8_t *in,
                        unsigned in_offset, unsigned length, bool last_bit)
{
	/* TODO: Fix MSB first modes */
	DEBUG_IO("%s%s %d bits", in ? "in" : "", out ? "out" : "", length);

	if (ctx->retval != ERROR_OK) {
		DEBUG_IO("Ignoring command due to previous error");
		return;
	}
	unsigned int min_bytes;
	unsigned int this_bytes;
	unsigned char cmd_to_send;
	if(out && in) {
		while (length > 0) {
			this_bytes = (((length-1) >>3)+1) ;
			min_bytes = this_bytes <=512 ? this_bytes: 512;

			if(buffer_write_space(ctx) < min_bytes+2 || buffer_read_space(ctx) < min_bytes)
				ctx->retval = wjtagsj_flush(ctx);

			if(min_bytes == this_bytes) {
				if(last_bit)
					cmd_to_send = WJTAGSJ_CMD_TDIO_INOUT | WJTAGSJ_LAST_BIT_MASK;
				else
					cmd_to_send = WJTAGSJ_CMD_TDIO_INOUT ;
				cmd_to_send =  ((((length - 1)>>8) &0x0f)<<4 )| cmd_to_send;

				buffer_write_byte(ctx, cmd_to_send);
				buffer_write_byte(ctx, (length - 1)&0xff);

				out_offset += buffer_write(ctx, out, out_offset, length);
				in_offset += buffer_add_read(ctx, in, in_offset, length, 0);

				length = 0;
				break;

			} else {
				if((512*8 == length) && last_bit)
					cmd_to_send = WJTAGSJ_CMD_TDIO_INOUT | WJTAGSJ_LAST_BIT_MASK;
				else
					cmd_to_send = WJTAGSJ_CMD_TDIO_INOUT ;
				cmd_to_send =  WJTAGSJ_LENGTH_MASK| cmd_to_send;

				buffer_write_byte(ctx, cmd_to_send);
				buffer_write_byte(ctx, (512*8 - 1)&0xff);

				out_offset += buffer_write(ctx, out, out_offset, 512*8);
				in_offset += buffer_add_read(ctx, in, in_offset, 512*8, 0);

				length -= 512*8 ;

				if(0 ==length) break;
			}
		}
	} else if(!out && in ) {
		while(length > 0) {

			this_bytes = (((length-1) >>3)+1) ;
			min_bytes = this_bytes<=512 ? this_bytes: 512;

			if(buffer_write_space(ctx) < 2 || buffer_read_space(ctx) < min_bytes)
				ctx->retval = wjtagsj_flush(ctx);
			if(min_bytes == this_bytes) {
				if(last_bit)
					cmd_to_send = WJTAGSJ_CMD_TDO_INPUT_ONLY | WJTAGSJ_LAST_BIT_MASK;
				else
					cmd_to_send = WJTAGSJ_CMD_TDO_INPUT_ONLY ;
				cmd_to_send =  ((((length - 1)>>8) &0x0f)<<4 )| cmd_to_send;

				buffer_write_byte(ctx, cmd_to_send);
				buffer_write_byte(ctx, (length - 1)&0xff);

				in_offset += buffer_add_read(ctx, in, in_offset, length, 0);

				length = 0;
				break;
			} else {
				if((512*8 == length) && last_bit)
					cmd_to_send = WJTAGSJ_CMD_TDO_INPUT_ONLY | WJTAGSJ_LAST_BIT_MASK;
				else
					cmd_to_send = WJTAGSJ_CMD_TDO_INPUT_ONLY ;
				cmd_to_send =  WJTAGSJ_LENGTH_MASK| cmd_to_send;

				buffer_write_byte(ctx, cmd_to_send);
				buffer_write_byte(ctx, (512*8  - 1)&0xff);

				in_offset += buffer_add_read(ctx, in, in_offset, 512*8 , 0);

				length -= 512*8 ;
				if(0 ==length) break;
			}
		}
	} else {
		while( length > 0 ) {
			this_bytes = (((length-1) >>3)+1) ;

			min_bytes = this_bytes<=512 ? this_bytes : 512;
			if(buffer_write_space(ctx) < min_bytes+2)
				ctx->retval = wjtagsj_flush(ctx);

			if(min_bytes == this_bytes) {
				if(last_bit)
					cmd_to_send = WJTAGSJ_CMD_TDI_OUTPUT_ONLY | WJTAGSJ_LAST_BIT_MASK;
				else
					cmd_to_send = WJTAGSJ_CMD_TDI_OUTPUT_ONLY ;
				cmd_to_send =  ((((length - 1)>>8) &0x0f)<<4 )| cmd_to_send;

				buffer_write_byte(ctx, cmd_to_send);
				buffer_write_byte(ctx, (length - 1)&0xff);

				if(out && !in) {
					out_offset += buffer_write(ctx, out, out_offset, length);
				} else {
					for(unsigned int i =0; i<min_bytes; i++) {
						buffer_write_byte(ctx, 0x00);
					}
				}
				length = 0;
				break;
			} else {
				if((512*8 == length) && last_bit)
					cmd_to_send = WJTAGSJ_CMD_TDI_OUTPUT_ONLY | WJTAGSJ_LAST_BIT_MASK;
				else
					cmd_to_send = WJTAGSJ_CMD_TDI_OUTPUT_ONLY ;
				cmd_to_send =  WJTAGSJ_LENGTH_MASK| cmd_to_send;

				buffer_write_byte(ctx, cmd_to_send);
				buffer_write_byte(ctx, (512*8 - 1)&0xff);

				if(out && !in) {
					out_offset += buffer_write(ctx, out, out_offset, 512*8);
				} else {
					for(int i =0; i<512; i++) {
						buffer_write_byte(ctx, 0x00);
					}
				}
				length -= 512*8 ;
				if(0 ==length) break;
			}
		}
	}
}

void wjtagsj_clock_tms_cs_out(struct wjtagsj_ctx *ctx, const uint8_t *out, unsigned out_offset,
                              unsigned length)
{

	DEBUG_IO("%sout %d bits", out ? "out" : "", length);

	assert(out);

	if (ctx->retval != ERROR_OK) {
		DEBUG_IO("Ignoring command due to previous error");
		return;
	}

	unsigned int min_bytes;
	unsigned int this_bytes;
	unsigned int cmd_to_send;
	while(length > 0) {
		this_bytes = (((length-1) >>3)+1) ;
		min_bytes = this_bytes <=4 ? this_bytes : 4;

		if(buffer_write_space(ctx) < min_bytes+1)
			ctx->retval = wjtagsj_flush(ctx);

		if(min_bytes == this_bytes) {
			cmd_to_send = WJTAGSJ_CMD_TMS_OUTPUT_ONLY | ((length-1)<<3);
			buffer_write_byte(ctx, cmd_to_send);
			out_offset += buffer_write(ctx, out, out_offset, length);

			length = 0 ;
			break;
		} else {
			cmd_to_send = WJTAGSJ_CMD_TMS_OUTPUT_ONLY | (0xf8);
			buffer_write_byte(ctx, cmd_to_send);
			out_offset += buffer_write(ctx, out, out_offset, 32);

			length -= 32 ;
			if(0 == length) break;
		}
	}
}

void wjtagsj_enable_swdio(struct wjtagsj_ctx *ctx, uint8_t data)
{
	DEBUG_IO("-");



	if (ctx->retval != ERROR_OK) {
		DEBUG_IO("Ignoring command due to previous error");
		return;
	}

	if(data)
		data =WJTAGSJ_CMD_ENABLE_SWDIO|0x80 ;
	else
		data =WJTAGSJ_CMD_ENABLE_SWDIO|0x00;

	if (buffer_write_space(ctx) < 1)
		ctx->retval = wjtagsj_flush(ctx);

	buffer_write_byte(ctx, data);
}

void wjtagsj_set_clk_speed(struct wjtagsj_ctx *ctx, uint8_t data)
{
	DEBUG_IO("-");

	if (ctx->retval != ERROR_OK) {
		DEBUG_IO("Ignoring command due to previous error");
		return;
	}

	if(data >8 ) data =8;

	if (buffer_write_space(ctx) < 3)
		ctx->retval = wjtagsj_flush(ctx);

	buffer_write_byte(ctx, WJTAGSJ_CMD_ADAPTER_CTRL);
	buffer_write_byte(ctx, WJTAGSJ_SUBCMD_SET_SPEED);
	buffer_write_byte(ctx, data);

	clk_speed = data;

}

void wjtagsj_set_rst_ctrl(struct wjtagsj_ctx *ctx, uint8_t data)
{
	DEBUG_IO("-");

	if (ctx->retval != ERROR_OK) {
		DEBUG_IO("Ignoring command due to previous error");
		return;
	}

	data = data & 0x03;

	if (buffer_write_space(ctx) < 3)
		ctx->retval = wjtagsj_flush(ctx);

	buffer_write_byte(ctx, WJTAGSJ_CMD_ADAPTER_CTRL);
	buffer_write_byte(ctx, WJTAGSJ_SUBCMD_RST_CONTROL);
	buffer_write_byte(ctx, data);

}

void wjtagsj_enable_swd(struct wjtagsj_ctx *ctx, uint8_t data)
{
	DEBUG_IO("-");



	if (ctx->retval != ERROR_OK) {
		DEBUG_IO("Ignoring command due to previous error");
		return;
	}

	if(data)
		data =1 ;
	else
		data =0;

	if (buffer_write_space(ctx) < 3)
		ctx->retval = wjtagsj_flush(ctx);

	buffer_write_byte(ctx, WJTAGSJ_CMD_ADAPTER_CTRL);
	buffer_write_byte(ctx, WJTAGSJ_SUBCMD_ENABLE_SWD);
	buffer_write_byte(ctx, data);
}

void wjtagsj_enable_swo(struct wjtagsj_ctx *ctx, uint8_t data,unsigned int baudrate)
{
	DEBUG_IO("-");

	if (ctx->retval != ERROR_OK) {
		DEBUG_IO("Ignoring command due to previous error");
		return;
	}

	if (buffer_write_space(ctx) < 7)
		ctx->retval = wjtagsj_flush(ctx);


	buffer_write_byte(ctx, WJTAGSJ_CMD_ADAPTER_CTRL);
	buffer_write_byte(ctx, WJTAGSJ_SUBCMD_ENABLE_SWDO);
	buffer_write_byte(ctx, data);
	buffer_write_byte(ctx, baudrate&0xff);
	buffer_write_byte(ctx, (baudrate>>8)&0xff);
	buffer_write_byte(ctx, (baudrate>>16)&0xff);
	buffer_write_byte(ctx, (baudrate>>24)&0xff);
}



int wjtagsj_set_frequency(struct wjtagsj_ctx *ctx, int frequency)
{
	LOG_DEBUG("target %d Hz", frequency);
	assert(frequency >= 0);

	unsigned char param=0;
	if(0 == frequency) {
		param =0;
		frequency =0;
	} else if(frequency >0 && frequency <375000) {
		param =8;
		frequency =187500;
	} else if(frequency >=375000 && frequency <750000) {
		param =7;
		frequency =375000;
	} else if(frequency >=750000 && frequency <1500000) {
		param =6;
		frequency =750000;
	} else if(frequency >=1500000 && frequency <3000000) {
		param =5;
		frequency =1500000;
	} else if(frequency >=3000000 && frequency <6000000) {
		param =4;
		frequency =3000000;
	} else if(frequency >=6000000 && frequency <12000000) {
		param =3;
		frequency =6000000 ;
	} else if(frequency >=12000000 && frequency <24000000) {
		param =2;
		frequency =12000000;
	} else if(frequency >=24000000) {
		param =1;
		frequency =24000000;
	}

	wjtagsj_set_clk_speed(ctx, param);

	LOG_DEBUG("actually %d Hz", frequency);

	return frequency;
}

/* Context needed by the callbacks */
struct transfer_result {
	struct wjtagsj_ctx *ctx;
	int done;
	unsigned transferred;
};



int wjtagsj_flush(struct wjtagsj_ctx *ctx)
{
	int retval = ctx->retval;

	if (retval != ERROR_OK) {
		DEBUG_IO("Ignoring flush due to previous error");
		assert(ctx->write_count == 0 && ctx->read_count == 0);
		ctx->retval = ERROR_OK;
		return retval;
	}

	DEBUG_IO("write %d%s, read %d", ctx->write_count, ctx->read_count ? "+1" : "",
	         ctx->read_count);
	assert(ctx->write_count > 0 || ctx->read_count == 0); /* No read data without write data */

	if (ctx->write_count == 0)
		return retval;

	int ret_byte_write = jtag_libusb_bulk_write(ctx->usb_dev,ctx->out_ep,(char*)(ctx->write_buffer),ctx->write_count, 900);
	keep_alive();
	if(ret_byte_write == (int)(ctx->write_count)) {
		int read_num =ctx->read_count;
		int ret_bytes =0;
		int loop_time=0;
		if (ctx->read_count) {
			while(1) {
				int delay= (read_num*delay_coeff[clk_speed]/64 + 3) *3;

				if(read_num == (int)ctx->read_count)
					delay = delay + 20;
				if (delay > 320)
					delay =320;
				int ret_byte_read = jtag_libusb_bulk_read(ctx->usb_dev,ctx->in_ep,(char*)(ctx->read_chunk),read_num, delay);
				keep_alive();
				if(ret_byte_read) {

					memcpy(ctx->read_buffer +ret_bytes,
					       ctx->read_chunk,
					       ret_byte_read);
					ret_bytes +=  ret_byte_read;
					read_num = ctx->read_count - ret_bytes;

					if(ret_bytes == (int)(ctx->read_count)) {
						ctx->write_count = 0;
						ctx->read_count = 0;
						bit_copy_execute(&ctx->read_queue);
						retval = ERROR_OK;
						break;
					}
					loop_time = loop_time + delay;
					if(loop_time > 1800) {
						retval = ERROR_FAIL;
						break;
					}
				}
			}
		} else {
			ctx->write_count = 0;
			bit_copy_discard(&ctx->read_queue);
			retval = ERROR_OK;
		}

	} else {
		retval = ERROR_FAIL;
	}

	if (retval != ERROR_OK)
		wjtagsj_purge(ctx);

	return retval;
}
