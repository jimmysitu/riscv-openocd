/**************************************************************************
*   Copyright (C) 2016 by besonzore                              *
*   1670739974@qq.com                                           *
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

/* project specific includes */
#include <jtag/interface.h>
#include <jtag/swd.h>
#include <transport/transport.h>
#include <helper/time_support.h>

#if IS_CYGWIN == 1
#include <windows.h>
#endif

#include <assert.h>

#include <stdint.h>
#include <math.h>

#include "w_jtag_sj_io.h"

#define SWO_MAX_FREQ_FS 2250000
#define SWO_MAX_FREQ_HS 1875000

static char *wjtagsj_device_desc;
static char *wjtagsj_serial;
static uint8_t wjtagsj_channel;

static bool swd_mode;
static bool swo_raw_data =false;

static uint32_t swo_max_freq = 0xFFFFFFFF;

#define MAX_USB_IDS 8
/* vid = pid = 0 marks the end of the list */
static uint16_t wjtagsj_vid[MAX_USB_IDS + 1] = { 0 };
static uint16_t wjtagsj_pid[MAX_USB_IDS + 1] = { 0 };

static struct wjtagsj_ctx *wjtagsj_ctx1;


static struct swd_cmd_queue_entry {
	uint8_t cmd;
	uint32_t *dst;
	uint8_t trn_ack_data_parity_trn[DIV_ROUND_UP(4 + 3 + 32 + 1 + 4, 8)];
} *swd_cmd_queue;
static size_t swd_cmd_queue_length;
static size_t swd_cmd_queue_alloced;
static int queued_retval;
static int freq;

static int wjtagsj_swd_switch_seq(enum swd_special_seq seq);

static const struct command_registration wjtagsj_command_handlers[];

/**
 * Function wjtagsj_move_to_state
 * moves the TAP controller from the current state to a
 * \a goal_state through a path given by tap_get_tms_path().  State transition
 * logging is performed by delegation to clock_tms().
 *
 * @param goal_state is the destination state for the move.
 */
static void wjtagsj_move_to_state(tap_state_t goal_state)
{
	tap_state_t start_state = tap_get_state();

	/*	goal_state is 1/2 of a tuple/pair of states which allow convenient
		lookup of the required TMS pattern to move to this state from the
		start state.
	*/

	/* do the 2 lookups */
	uint8_t tms_bits  = tap_get_tms_path(start_state, goal_state);
	int tms_count = tap_get_tms_path_len(start_state, goal_state);
	assert(tms_count <= 8);

	DEBUG_JTAG_IO("start=%s goal=%s", tap_state_name(start_state), tap_state_name(goal_state));

	/* Track state transitions step by step */
	for (int i = 0; i < tms_count; i++)
		tap_set_state(tap_state_transition(tap_get_state(), (tms_bits >> i) & 1));

	wjtagsj_clock_tms_cs_out(wjtagsj_ctx1,
	                         &tms_bits,
	                         0,
	                         tms_count);
}

static int wjtagsj_speed(int speed)
{
	int retval;
	retval = wjtagsj_set_frequency(wjtagsj_ctx1, speed);

	if (retval < 0) {
		LOG_ERROR("couldn't set TCK speed");
		return retval;
	}

	return ERROR_OK;
}
static int wjtagsj_speed_div(int speed, int *khz)
{
	*khz = speed / 1000;
	return ERROR_OK;

}

static int wjtagsj_khz(int khz, int *jtag_speed)
{
	if (khz >= 24000)
		*jtag_speed = 24000 * 1000;
	else if (khz >= 12000)
		*jtag_speed = 12000 * 1000;
	else if (khz >= 6000)
		*jtag_speed = 6000 * 1000;
	else if (khz >= 3000)
		*jtag_speed = 3000 * 1000;
	else if (khz >= 1500)
		*jtag_speed = 1500 * 1000;
	else if (khz >= 750)
		*jtag_speed = 750 * 1000;
	else if (khz >= 375)
		*jtag_speed = 375 * 1000;
	else if (khz >= 1)
		*jtag_speed = 187 * 1000;
	else
		*jtag_speed = 0;
	return ERROR_OK;
}

static void wjtagsj_end_state(tap_state_t state)
{
	if (tap_is_state_stable(state))
		tap_set_end_state(state);
	else {
		LOG_ERROR("BUG: %s is not a stable end state", tap_state_name(state));
		exit(-1);
	}
}

static void wjtagsj_execute_runtest(struct jtag_command *cmd)
{
	int i;
	uint8_t zero = 0;

	DEBUG_JTAG_IO("runtest %i cycles, end in %s",
	              cmd->cmd.runtest->num_cycles,
	              tap_state_name(cmd->cmd.runtest->end_state));

	if (tap_get_state() != TAP_IDLE)
		wjtagsj_move_to_state(TAP_IDLE);

	/* TODO: Reuse wjtagsj_execute_stableclocks */
	i = cmd->cmd.runtest->num_cycles;
	while (i > 0) {
		/* there are no state transitions in this code, so omit state tracking */
		unsigned this_len = i > 7 ? 7 : i;
		wjtagsj_clock_tms_cs_out(wjtagsj_ctx1, &zero, 0, this_len);
		i -= this_len;
	}

	wjtagsj_end_state(cmd->cmd.runtest->end_state);

	if (tap_get_state() != tap_get_end_state())
		wjtagsj_move_to_state(tap_get_end_state());

	DEBUG_JTAG_IO("runtest: %i, end in %s",
	              cmd->cmd.runtest->num_cycles,
	              tap_state_name(tap_get_end_state()));
}

static void wjtagsj_execute_statemove(struct jtag_command *cmd)
{
	DEBUG_JTAG_IO("statemove end in %s",
	              tap_state_name(cmd->cmd.statemove->end_state));

	wjtagsj_end_state(cmd->cmd.statemove->end_state);

	/* shortest-path move to desired end state */
	if (tap_get_state() != tap_get_end_state() || tap_get_end_state() == TAP_RESET)
		wjtagsj_move_to_state(tap_get_end_state());
}

/**
 * Clock a bunch of TMS (or SWDIO) transitions, to change the JTAG
 * (or SWD) state machine. REVISIT: Not the best method, perhaps.
 */
static void wjtagsj_execute_tms(struct jtag_command *cmd)
{
	DEBUG_JTAG_IO("TMS: %d bits", cmd->cmd.tms->num_bits);

	/* TODO: Missing tap state tracking, also missing from ft2232.c! */
	wjtagsj_clock_tms_cs_out(wjtagsj_ctx1,
	                         cmd->cmd.tms->bits,
	                         0,
	                         cmd->cmd.tms->num_bits);
}

static void wjtagsj_execute_pathmove(struct jtag_command *cmd)
{
	tap_state_t *path = cmd->cmd.pathmove->path;
	int num_states  = cmd->cmd.pathmove->num_states;

	DEBUG_JTAG_IO("pathmove: %i states, current: %s  end: %s", num_states,
	              tap_state_name(tap_get_state()),
	              tap_state_name(path[num_states-1]));

	int state_count = 0;
	unsigned bit_count = 0;
	uint8_t tms_byte = 0;

	DEBUG_JTAG_IO("-");

	/* this loop verifies that the path is legal and logs each state in the path */
	while (num_states--) {

		/* either TMS=0 or TMS=1 must work ... */
		if (tap_state_transition(tap_get_state(), false)
		    == path[state_count])
			buf_set_u32(&tms_byte, bit_count++, 1, 0x0);
		else if (tap_state_transition(tap_get_state(), true)
		         == path[state_count]) {
			buf_set_u32(&tms_byte, bit_count++, 1, 0x1);

			/* ... or else the caller goofed BADLY */
		} else {
			LOG_ERROR("BUG: %s -> %s isn't a valid "
			          "TAP state transition",
			          tap_state_name(tap_get_state()),
			          tap_state_name(path[state_count]));
			exit(-1);
		}

		tap_set_state(path[state_count]);
		state_count++;

		if (bit_count == 7 || num_states == 0) {
			wjtagsj_clock_tms_cs_out(wjtagsj_ctx1,
			                         &tms_byte,
			                         0,
			                         bit_count);
			bit_count = 0;
		}
	}
	tap_set_end_state(tap_get_state());
}

static void wjtagsj_execute_scan(struct jtag_command *cmd)
{
	DEBUG_JTAG_IO("%s type:%d", cmd->cmd.scan->ir_scan ? "IRSCAN" : "DRSCAN",
	              jtag_scan_type(cmd->cmd.scan));

	/* Make sure there are no trailing fields with num_bits == 0, or the logic below will fail. */
	while (cmd->cmd.scan->num_fields > 0
	       && cmd->cmd.scan->fields[cmd->cmd.scan->num_fields - 1].num_bits == 0) {
		cmd->cmd.scan->num_fields--;
		LOG_DEBUG("discarding trailing empty field");
	}

	if (cmd->cmd.scan->num_fields == 0) {
		LOG_DEBUG("empty scan, doing nothing");
		return;
	}

	if (cmd->cmd.scan->ir_scan) {
		if (tap_get_state() != TAP_IRSHIFT)
			wjtagsj_move_to_state(TAP_IRSHIFT);
	} else {
		if (tap_get_state() != TAP_DRSHIFT)
			wjtagsj_move_to_state(TAP_DRSHIFT);
	}

	wjtagsj_end_state(cmd->cmd.scan->end_state);

	struct scan_field *field = cmd->cmd.scan->fields;
	unsigned scan_size = 0;

	for (int i = 0; i < cmd->cmd.scan->num_fields; i++, field++) {
		scan_size += field->num_bits;
		DEBUG_JTAG_IO("%s%s field %d/%d %d bits",
		              field->in_value ? "in" : "",
		              field->out_value ? "out" : "",
		              i,
		              cmd->cmd.scan->num_fields,
		              field->num_bits);

		if (i == cmd->cmd.scan->num_fields - 1 && tap_get_state() != tap_get_end_state()) {
			/* Last field, and we're leaving IRSHIFT/DRSHIFT. Clock last bit during tap
			 * movement. This last field can't have length zero, it was checked above. */
			wjtagsj_clock_data(wjtagsj_ctx1,
			                   field->out_value,
			                   0,
			                   field->in_value,
			                   0,
			                   field->num_bits,
			                   true);
			//to exit1-dr / exit1-ir
			tap_set_state(tap_state_transition(tap_get_state(), 1));
			//to pause-dr / pause-ir
			tap_set_state(tap_state_transition(tap_get_state(), 0));
		} else
			wjtagsj_clock_data(wjtagsj_ctx1,
			                   field->out_value,
			                   0,
			                   field->in_value,
			                   0,
			                   field->num_bits,
			                   false);
	}

	if (tap_get_state() != tap_get_end_state())
		wjtagsj_move_to_state(tap_get_end_state());

	DEBUG_JTAG_IO("%s scan, %i bits, end in %s",
	              (cmd->cmd.scan->ir_scan) ? "IR" : "DR", scan_size,
	              tap_state_name(tap_get_end_state()));
}

static void wjtagsj_execute_reset(struct jtag_command *cmd)
{
	DEBUG_JTAG_IO("reset trst: %i srst %i",
	              cmd->cmd.reset->trst, cmd->cmd.reset->srst);

	if (cmd->cmd.reset->trst == 1
	    || (cmd->cmd.reset->srst
	        && (jtag_get_reset_config() & RESET_SRST_PULLS_TRST)))
		tap_set_state(TAP_RESET);

	unsigned char param =0;
	if (cmd->cmd.reset->trst == 1) {
		param =0x01;
	} else if (jtag_get_reset_config() & RESET_HAS_TRST &&
	           cmd->cmd.reset->trst == 0) {
		param =0x00;
	}

	if (cmd->cmd.reset->srst == 1) {
		param = param | 0x02;
	} else if (jtag_get_reset_config() & RESET_HAS_SRST &&
	           cmd->cmd.reset->srst == 0) {
		param = param | 0x00;
	}

	wjtagsj_set_rst_ctrl(wjtagsj_ctx1, param);

	DEBUG_JTAG_IO("trst: %i, srst: %i",
	              cmd->cmd.reset->trst, cmd->cmd.reset->srst);
}

static void wjtagsj_execute_sleep(struct jtag_command *cmd)
{
	DEBUG_JTAG_IO("sleep %" PRIi32, cmd->cmd.sleep->us);

	wjtagsj_flush(wjtagsj_ctx1);
	jtag_sleep(cmd->cmd.sleep->us);
	DEBUG_JTAG_IO("sleep %" PRIi32 " usec while in %s",
	              cmd->cmd.sleep->us,
	              tap_state_name(tap_get_state()));
}

static void wjtagsj_execute_stableclocks(struct jtag_command *cmd)
{
	/* this is only allowed while in a stable state.  A check for a stable
	 * state was done in jtag_add_clocks()
	 */
	int num_cycles = cmd->cmd.stableclocks->num_cycles;

	/* 7 bits of either ones or zeros. */
	uint8_t tms = tap_get_state() == TAP_RESET ? 0x7f : 0x00;

	// output directly
	wjtagsj_clock_tms_cs_out(wjtagsj_ctx1,&tms, 0,num_cycles);


	DEBUG_JTAG_IO("clocks %i while in %s",
	              cmd->cmd.stableclocks->num_cycles,
	              tap_state_name(tap_get_state()));
}

static void wjtagsj_execute_command(struct jtag_command *cmd)
{
	switch (cmd->type) {
	case JTAG_RESET:
		wjtagsj_execute_reset(cmd);
		break;
	case JTAG_RUNTEST:
		wjtagsj_execute_runtest(cmd);
		break;
	case JTAG_TLR_RESET:
		wjtagsj_execute_statemove(cmd);
		break;
	case JTAG_PATHMOVE:
		wjtagsj_execute_pathmove(cmd);
		break;
	case JTAG_SCAN:
		wjtagsj_execute_scan(cmd);
		break;
	case JTAG_SLEEP:
		wjtagsj_execute_sleep(cmd);
		break;
	case JTAG_STABLECLOCKS:
		wjtagsj_execute_stableclocks(cmd);
		break;
	case JTAG_TMS:
		wjtagsj_execute_tms(cmd);
		break;
	default:
		LOG_ERROR("BUG: unknown JTAG command type encountered: %d", cmd->type);
		break;
	}
}

static int wjtagsj_execute_queue(void)
{
	/* blink, if the current layout has that feature */

	for (struct jtag_command *cmd = jtag_command_queue; cmd; cmd = cmd->next) {
		/* fill the write buffer with the desired command */
		wjtagsj_execute_command(cmd);
	}


	int retval = wjtagsj_flush(wjtagsj_ctx1);
	if (retval != ERROR_OK)
		LOG_ERROR("error while flushing queue: %d", retval);

	return retval;
}

static int wjtagsj_swd_enable_swo(bool en,unsigned int baudrate)
{
	int retval = wjtagsj_flush(wjtagsj_ctx1);
	if (retval != ERROR_OK) {
		LOG_ERROR("error while flushing queue: %d", retval);
		return retval;
	}

	unsigned char en_swo=0;
	if(en) en_swo = 1;
	if(swo_raw_data) en_swo =en_swo | 0x02;
	wjtagsj_enable_swo(wjtagsj_ctx1,en_swo,baudrate);

	if (wjtagsj_ctx1->retval != ERROR_OK) {
		LOG_ERROR("Ignoring command due to previous error");
		return ERROR_FAIL;
	}

	retval = wjtagsj_flush(wjtagsj_ctx1);
	if (retval != ERROR_OK) {
		LOG_ERROR("error while flushing queue: %d", retval);
		return retval;
	}
	return ERROR_OK;
}
static int wjtagsj_poll_trace(uint8_t *buf, size_t *size)
{
	*size = 0;
	return ERROR_OK;
}
static bool wjtagsj_check_trace_freq(uint32_t infreq, uint32_t* ret_freq, uint32_t trace_freq)
{
	/* increase 1/16 each step*/
	int times = infreq *16 /trace_freq;
	double times1 = (times)/16.0;
	double times2 = (times+1)/16.0;

	double min1 = fabs(1.0 -(infreq/times1)/trace_freq);
	double min2 = fabs(1.0 -(infreq/times2)/trace_freq);

	double deviation;

	if(min1 < min2) {
		deviation = min1;
		* ret_freq = (int )(infreq/times1);
	} else {
		deviation = min2;
		* ret_freq = (int )(infreq/times2);
	}


	if (deviation < 0.03) {
		LOG_DEBUG("Found suitable frequency with deviation of "
		          "%.02f %%.", deviation);
		return true;
	}

	LOG_ERROR("Selected trace frequency is not supported by the device. "
	          "Please choose a different trace frequency.");
	LOG_ERROR("Maximum permitted deviation is 3.00 %%, but only %.02f %% "
	          "could be achieved.", deviation * 100);

	* ret_freq =0;

	return false;
}



static int wjtagsj_config_trace(bool enabled, enum tpiu_pin_protocol pin_protocol,
                                uint32_t port_size, unsigned int *trace_freq)
{

	int ret;
	uint32_t infreq;
	uint32_t validfreq;

    if(swo_max_freq == 0xFFFFFFFF)
        {
        swo_max_freq = SWO_MAX_FREQ_FS;
    }

	if(! swd_mode) {
		LOG_INFO("SWO can be used in SWD mode only");
		return ERROR_FAIL;
	}

    if(enabled) {
        if (pin_protocol != TPIU_PIN_PROTOCOL_ASYNC_UART) {
		    LOG_ERROR("Selected pin protocol is not supported.");
		    return ERROR_FAIL;
        }
    	if(*trace_freq > swo_max_freq) {
    		LOG_ERROR("Frequency exceed the limit of max swo baudrate");
    		return ERROR_FAIL;
    	}
    }

	/*disable SWO*/
	if(!enabled) {
		ret = wjtagsj_swd_enable_swo(enabled,*trace_freq);
		if( ERROR_OK!= ret) {
			LOG_ERROR("disable SWO Failed");
			return ERROR_FAIL;
		} else
			return ERROR_OK;
	}

	/*enable SWO*/
	infreq = swo_max_freq;
	validfreq = swo_max_freq;

	if (!*trace_freq)
		*trace_freq = infreq;

	if (!wjtagsj_check_trace_freq(infreq, &validfreq, *trace_freq))
		return ERROR_FAIL;

	*trace_freq = validfreq;
	ret = wjtagsj_swd_enable_swo(enabled,validfreq);

	if( ERROR_OK!= ret) {
		LOG_ERROR("enable SWO Failed");
		return ERROR_FAIL;
	} else
		return ERROR_OK;
}


static int wjtagsj_initialize(void)
{
	if (tap_get_tms_path_len(TAP_IRPAUSE, TAP_IRPAUSE) == 7)
		LOG_DEBUG("interface using 7 step jtag state transitions");
	else
		LOG_DEBUG("interface using shortest path jtag state transitions");

	// Try to open the first one
	for (int i = 0; wjtagsj_vid[i] || wjtagsj_pid[i]; i++) {
		wjtagsj_ctx1 = wjtagsj_open(&wjtagsj_vid[i], &wjtagsj_pid[i], wjtagsj_device_desc,
		                            wjtagsj_serial, wjtagsj_channel);
		if (wjtagsj_ctx1)
			break;
	}

	if (!wjtagsj_ctx1)
		return ERROR_JTAG_INIT_FAILED;


	if (swd_mode) {

		wjtagsj_enable_swd(wjtagsj_ctx1,true);
	}

	freq = wjtagsj_set_frequency(wjtagsj_ctx1, jtag_get_speed_khz() * 1000);

	return wjtagsj_flush(wjtagsj_ctx1);
}

static int wjtagsj_quit(void)
{
	wjtagsj_close(wjtagsj_ctx1);

	free(swd_cmd_queue);

	return ERROR_OK;
}


static int wjtagsj_swd_init(void)
{
	LOG_INFO("interface SWD mode enabled");
	swd_mode = true;

	swd_cmd_queue_alloced = 1280;
	swd_cmd_queue = malloc(swd_cmd_queue_alloced * sizeof(*swd_cmd_queue));

	return swd_cmd_queue != NULL ? ERROR_OK : ERROR_FAIL;
}

static void wjtagsj_swd_swdio_en(bool enable)
{
	wjtagsj_enable_swdio(wjtagsj_ctx1, enable);
}

/**
 * Flush the queue and process the SWD transaction queue
 * @param dap
 * @return
 */
static int wjtagsj_swd_run_queue(void)
{
	LOG_DEBUG("Executing %zu queued transactions", swd_cmd_queue_length);
	int retval;


	if (queued_retval != ERROR_OK) {
		LOG_DEBUG("Skipping due to previous errors: %d", queued_retval);
		goto skip;
	}

	/* A transaction must be followed by another transaction or at least 8 idle cycles to
	 * ensure that data is clocked through the AP. */
	wjtagsj_clock_data_out(wjtagsj_ctx1, NULL, 0, 8, false);


	queued_retval = wjtagsj_flush(wjtagsj_ctx1);
	if (queued_retval != ERROR_OK) {
		LOG_ERROR("flush queue failed");
		goto skip;
	}

	for (size_t i = 0; i < swd_cmd_queue_length; i++) {
		int ack = buf_get_u32(swd_cmd_queue[i].trn_ack_data_parity_trn, 1, 3);

		LOG_DEBUG("%s %s %s reg %X = %08"PRIx32,
		          ack == SWD_ACK_OK ? "OK" : ack == SWD_ACK_WAIT ? "WAIT" : ack == SWD_ACK_FAULT ? "FAULT" : "JUNK",
		          swd_cmd_queue[i].cmd & SWD_CMD_APnDP ? "AP" : "DP",
		          swd_cmd_queue[i].cmd & SWD_CMD_RnW ? "read" : "write",
		          (swd_cmd_queue[i].cmd & SWD_CMD_A32) >> 1,
		          buf_get_u32(swd_cmd_queue[i].trn_ack_data_parity_trn,
		                      1 + 3 + (swd_cmd_queue[i].cmd & SWD_CMD_RnW ? 0 : 1), 32));

		if (ack != SWD_ACK_OK) {
			queued_retval = ack == SWD_ACK_WAIT ? ERROR_WAIT : ERROR_FAIL;
			goto skip;

		} else if (swd_cmd_queue[i].cmd & SWD_CMD_RnW) {
			uint32_t data = buf_get_u32(swd_cmd_queue[i].trn_ack_data_parity_trn, 1 + 3, 32);
			int parity = buf_get_u32(swd_cmd_queue[i].trn_ack_data_parity_trn, 1 + 3 + 32, 1);

			if (parity != parity_u32(data)) {
				LOG_ERROR("SWD Read data parity mismatch");
				queued_retval = ERROR_FAIL;
				goto skip;
			}

			if (swd_cmd_queue[i].dst != NULL)
				*swd_cmd_queue[i].dst = data;
		}
	}

skip:
	swd_cmd_queue_length = 0;
	retval = queued_retval;
	queued_retval = ERROR_OK;

	return retval;
}

static void wjtagsj_swd_queue_cmd(uint8_t cmd, uint32_t *dst, uint32_t data, uint32_t ap_delay_clk)
{
	if (swd_cmd_queue_length >= swd_cmd_queue_alloced) {
		/* Not enough room in the queue. Run the queue and increase its size for next time.*/
		queued_retval = wjtagsj_swd_run_queue();
		struct swd_cmd_queue_entry *q = realloc(swd_cmd_queue, swd_cmd_queue_alloced * 2 * sizeof(*swd_cmd_queue));
		if (q != NULL) {
			swd_cmd_queue = q;
			swd_cmd_queue_alloced *= 2;
			LOG_DEBUG("Increased SWD command queue to %zu elements", swd_cmd_queue_alloced);
		}
	}

	if (queued_retval != ERROR_OK)
		return;

	size_t i = swd_cmd_queue_length++;
	swd_cmd_queue[i].cmd = cmd | SWD_CMD_START | SWD_CMD_PARK;

	wjtagsj_clock_data_out(wjtagsj_ctx1, &swd_cmd_queue[i].cmd, 0, 8, false);

	if (swd_cmd_queue[i].cmd & SWD_CMD_RnW) {
		/* Queue a read transaction */
		swd_cmd_queue[i].dst = dst;

		wjtagsj_swd_swdio_en(false);
		wjtagsj_clock_data_in(wjtagsj_ctx1, swd_cmd_queue[i].trn_ack_data_parity_trn,
		                      0, 1 + 3 + 32 + 1 + 1, false);
		wjtagsj_swd_swdio_en(true);
	} else {
		/* Queue a write transaction */
		wjtagsj_swd_swdio_en(false);

		wjtagsj_clock_data_in(wjtagsj_ctx1, swd_cmd_queue[i].trn_ack_data_parity_trn,
		                      0, 1 + 3 + 1, false);

		wjtagsj_swd_swdio_en(true);

		buf_set_u32(swd_cmd_queue[i].trn_ack_data_parity_trn, 1 + 3 + 1, 32, data);
		buf_set_u32(swd_cmd_queue[i].trn_ack_data_parity_trn, 1 + 3 + 1 + 32, 1, parity_u32(data));

		wjtagsj_clock_data_out(wjtagsj_ctx1, swd_cmd_queue[i].trn_ack_data_parity_trn,
		                       1 + 3 + 1, 32 + 1, false);
	}

	/* Insert idle cycles after AP accesses to avoid WAIT */
	if (cmd & SWD_CMD_APnDP)
		wjtagsj_clock_data_out(wjtagsj_ctx1, NULL, 0, ap_delay_clk, false);

}

static void wjtagsj_swd_read_reg(uint8_t cmd, uint32_t *value, uint32_t ap_delay_clk)
{
	assert(cmd & SWD_CMD_RnW);
	wjtagsj_swd_queue_cmd(cmd, value, 0, ap_delay_clk);
}

static void wjtagsj_swd_write_reg(uint8_t cmd, uint32_t value, uint32_t ap_delay_clk)
{
	assert(!(cmd & SWD_CMD_RnW));
	wjtagsj_swd_queue_cmd(cmd, NULL, value, ap_delay_clk);
}

static int_least32_t wjtagsj_swd_frequency(int_least32_t hz)
{
	if (hz > 0)
		freq = wjtagsj_set_frequency(wjtagsj_ctx1, hz);

	return freq;
}

static int wjtagsj_swd_switch_seq(enum swd_special_seq seq)
{
	switch (seq) {
	case LINE_RESET:
		LOG_DEBUG("SWD line reset");
		wjtagsj_clock_data_out(wjtagsj_ctx1, swd_seq_line_reset, 0, swd_seq_line_reset_len, false);
		break;
	case JTAG_TO_SWD:
		LOG_DEBUG("JTAG-to-SWD");
		wjtagsj_clock_data_out(wjtagsj_ctx1, swd_seq_jtag_to_swd, 0, swd_seq_jtag_to_swd_len, false);
		break;
	case SWD_TO_JTAG:
		LOG_DEBUG("SWD-to-JTAG");
		wjtagsj_clock_data_out(wjtagsj_ctx1, swd_seq_swd_to_jtag, 0, swd_seq_swd_to_jtag_len, false);
		break;
	default:
		LOG_ERROR("Sequence %d not supported", seq);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static const struct swd_driver wjtagsj_swd = {
	.init = wjtagsj_swd_init,
	.frequency = wjtagsj_swd_frequency,
	.switch_seq = wjtagsj_swd_switch_seq,
	.read_reg = wjtagsj_swd_read_reg,
	.write_reg = wjtagsj_swd_write_reg,
	.run = wjtagsj_swd_run_queue,
};

static const char * const wjtagsj_transports[] = { "jtag", "swd", NULL };
static const char * const wjtagst_transports[] = { "jtag", NULL, NULL };

struct jtag_interface w_jtag_sj_interface = {
	.name = "w_jtag_sj",
	.supported = DEBUG_CAP_TMS_SEQ,
	.commands = wjtagsj_command_handlers,
	.transports = wjtagsj_transports,
	.swd = &wjtagsj_swd,

	.init = wjtagsj_initialize,
	.quit = wjtagsj_quit,
	.speed = wjtagsj_speed,
	.speed_div = wjtagsj_speed_div,
	.khz = wjtagsj_khz,
	.execute_queue = wjtagsj_execute_queue,
	.config_trace = wjtagsj_config_trace,
	.poll_trace = wjtagsj_poll_trace,
};

COMMAND_HANDLER(wjtagsj_handle_device_desc_command)
{
	if (CMD_ARGC == 1) {
		if (wjtagsj_device_desc)
			free(wjtagsj_device_desc);
		wjtagsj_device_desc = strdup(CMD_ARGV[0]);
	} else {
		LOG_ERROR("expected exactly one argument to wjtagsj_device_desc <description>");
	}

	if(strncmp(wjtagsj_device_desc, "W-JTAG-SJ", 9 ) == 0)
		w_jtag_sj_interface.transports =wjtagsj_transports;
	else if(strncmp(wjtagsj_device_desc, "W-JTAG-ST", 9 ) == 0)
		w_jtag_sj_interface.transports =wjtagst_transports;
	else {
		LOG_ERROR("expected W-JTAG-SJ | W-JTAG-ST only");
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

COMMAND_HANDLER(wjtagsj_handle_serial_command)
{
	if (CMD_ARGC == 1) {
		if (wjtagsj_serial)
			free(wjtagsj_serial);
		wjtagsj_serial = strdup(CMD_ARGV[0]);
	} else {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	return ERROR_OK;
}


COMMAND_HANDLER(wjtagsj_handle_vid_pid_command)
{
	if (CMD_ARGC > MAX_USB_IDS * 2) {
		LOG_WARNING("ignoring extra IDs in wjtagsj_vid_pid "
		            "(maximum is %d pairs)", MAX_USB_IDS);
		CMD_ARGC = MAX_USB_IDS * 2;
	}
	if (CMD_ARGC < 2 || (CMD_ARGC & 1)) {
		LOG_WARNING("incomplete wjtagsj_vid_pid configuration directive");
		if (CMD_ARGC < 2)
			return ERROR_COMMAND_SYNTAX_ERROR;
		/* remove the incomplete trailing id */
		CMD_ARGC -= 1;
	}

	unsigned i;
	for (i = 0; i < CMD_ARGC; i += 2) {
		COMMAND_PARSE_NUMBER(u16, CMD_ARGV[i], wjtagsj_vid[i >> 1]);
		COMMAND_PARSE_NUMBER(u16, CMD_ARGV[i + 1], wjtagsj_pid[i >> 1]);
	}

	/*
	 * Explicitly terminate, in case there are multiples instances of
	 * wjtagsj_vid_pid.
	 */
	wjtagsj_vid[i >> 1] = wjtagsj_pid[i >> 1] = 0;

	return ERROR_OK;
}

COMMAND_HANDLER(wjtagsj_handle_swo_raw_data)
{
	if (CMD_ARGC == 1) {
		bool enable;
		COMMAND_PARSE_ON_OFF(CMD_ARGV[0], enable);
		swo_raw_data =enable;
	} else {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	return ERROR_OK;
}

COMMAND_HANDLER(wjtagsj_swo_max_baudrate)
{
	if (CMD_ARGC == 1) {
		unsigned int  swo_freq;
		COMMAND_PARSE_NUMBER(u32,CMD_ARGV[0],swo_freq);
        if((swo_freq == SWO_MAX_FREQ_FS) ||(swo_freq == SWO_MAX_FREQ_HS) )
		    swo_max_freq=swo_freq;
        else
            {
            LOG_WARNING("invalid swo baudrate:2250000 for FS;1875000 for HS");
        }
	} else {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	return ERROR_OK;
}


static const struct command_registration wjtagsj_command_handlers[] = {
	{
		.name = "wjtagsj_device_desc",
		.handler = &wjtagsj_handle_device_desc_command,
		.mode = COMMAND_CONFIG,
		.help = "set the USB device description of interface",
		.usage = "description_string",
	},
	{
		.name = "wjtagsj_serial",
		.handler = &wjtagsj_handle_serial_command,
		.mode = COMMAND_CONFIG,
		.help = "set the serial number of interface",
		.usage = "serial_string",
	},
	{
		.name = "wjtagsj_vid_pid",
		.handler = &wjtagsj_handle_vid_pid_command,
		.mode = COMMAND_CONFIG,
		.help = "the vendor ID and product ID of interface",
		.usage = "(vid pid)* ",
	},
	{
		.name = "wjtagsj_swo_raw_data",
		.handler = &wjtagsj_handle_swo_raw_data,
		.mode = COMMAND_ANY,
		.help = "raw data or processed",
		.usage = "(0|1|on|off)",
	},
	{
		.name = "wjtagsj_swo_max_baudrate",
		.handler = &wjtagsj_swo_max_baudrate,
		.mode = COMMAND_ANY,
		.help = "set swo max baudrate for HS/FS adapter type:HS-1875000;FS-2250000",
		.usage = "swo_max_baudrate",
	},
	COMMAND_REGISTRATION_DONE
};
