/* ULP Example: pulse counting

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.

   This file contains assembly code which runs on the ULP.

   ULP wakes up to run this code at a certain period, determined by the values
   in SENS_ULP_CP_SLEEP_CYCx_REG registers. On each wake up, the program checks
   the input on GPIO0. If the value is different from the previous one, the
   program "debounces" the input: on the next debounce_max_count wake ups,
   it expects to see the same value of input.
   If this condition holds true, the program increments edge_count and starts
   waiting for input signal polarity to change again.
   When the edge counter reaches certain value (set by the main program),
   this program running triggers a wake up from deep sleep.
*/

/* ULP assembly files are passed through C preprocessor first, so include directives
   and C macros may be used in these files
 */
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"

	/* Define variables, which go into .bss section (zero-initialized data) */
	.bss
	.global is_ulp_wakeup
is_ulp_wakeup:
	.long 0
	.global dect_io
dect_io:
	.long 0
	.global wake_up_io
wake_up_io:
	.long 0
	.global wake_up_index
wake_up_index:
	.long 0
	.global io_numbers_active_low
io_numbers_active_low:
	.long 0

	/* RTC IO number used to sample the input signal.
	   Set by main program. */
	.global io_number_active_low
io_number_active_low:
	.long 0
	.long 0
	.long 0
	.long 0
	.long 0
	.long 0
	.long 0
	.long 0
	.long 0
	.long 0
	.long 0
	.long 0
	.long 0
	.long 0
	.long 0
	.long 0
	
	.global io_number_active_high
io_number_active_high:
	.long 0
	.long 0
	/* Code goes into .text section */
	.text
	.global entry
entry:
	move r0,dect_io
	ld r0,r0,0
	jumpr continue,0,eq
	move r0,0
	move r2,0
	move r3,0
	move r1,io_number_active_low//r1=io_number

	.global read_io_loop_active_low
read_io_loop_active_low:	
	ld r3,r1,0//r3=*io_number
	move r0,r3//r0=r3
	jumpr read_io_high_active_low, 16, ge//jump if r0>=16

	/* Read the value of lower 16 RTC IOs into R0 */
	READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S, 16)
	rsh r0, r0, r3
	jump read_done_active_low

		/* Read the value of RTC IOs 16-17, into R0 */
	.global read_io_high_active_low
read_io_high_active_low:
	READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S + 16, 2)
	sub r3, r3, 16
	rsh r0, r0, r3

	.global read_done_active_low
read_done_active_low:
	and r0, r0, 1//io_state & 0
	jumpr io_low_active,0,eq//jump if (io_state & 0) == 0
	ADD r2,r2,1
	move r0,r2
	ADD r1,r1,1//io_number++

	jumpr read_io_loop_active_low,5,LT
	move r0,0
	move r2,0
	move r3,0
	move r1,io_number_active_low//r1=io_number
	//jump read_io_loop_active_high_start
	halt

	.global io_low_active
io_low_active:
	move r0,wake_up_io
	ld r3,r1,0
	st r3,r0,0
	move r0,wake_up_index
	move r3,r2
	st r3,r0,0
	jump wake_up
	halt

	.global continue
continue:
	halt


// read_io_loop_active_high_start:
// 	move r0,0
// 	move r3,0
// 	move r1,io_number_active_high//r1=io_number
// read_io_loop_active_high:
// 	ld r3,r1,0//r3=*io_number
// 	move r0,r3//r0=r3
// 	jumpr read_io_high_active_high, 16, ge//jump if r0>=16

// 	/* Read the value of lower 16 RTC IOs into R0 */
// 	READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S, 16)
// 	rsh r0, r0, r3
// 	jump read_done_active_high
	
// 		/* Read the value of RTC IOs 16-17, into R0 */
// read_io_high_active_high:
// 	READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S + 16, 2)
// 	sub r3, r3, 16
// 	rsh r0, r0, r3

// read_done_active_high:
// 	and r0, r0, 1//io_state & 0
// 	jump io_high_active,eq//jump if (io_state & 0) == 0
// 	ADD r2,r2,1
// 	move r0,r2
// 	ADD r1,r1,2//io_number++
// 	jumpr read_io_loop_active_high,0,LT
// 	halt

// io_high_active:
// 	//r2=io_number[low_active]
// 	move r0,wake_up_io
// 	ld r3,r1,0
// 	st r3,r0,0
// 	jump wake_up

