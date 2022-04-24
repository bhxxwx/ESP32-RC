       
       
       
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
 .text
 .global entry
entry:
 move r0,dect_io
 ld r0,r0,0
 jumpr continue,0,eq
 move r0,0
 move r2,0
 move r3,0
 move r1,io_number_active_low
 .global read_io_loop_active_low
read_io_loop_active_low:
 ld r3,r1,0
 move r0,r3
 jumpr read_io_high_active_low, 16, ge
 REG_RD ((((0x3ff48400 + 0x24)) - 0x3ff48000) / 4), ((14) + (16) - 1), (14)
 rsh r0, r0, r3
 jump read_done_active_low
 .global read_io_high_active_low
read_io_high_active_low:
 REG_RD ((((0x3ff48400 + 0x24)) - 0x3ff48000) / 4), ((14 + 16) + (2) - 1), (14 + 16)
 sub r3, r3, 16
 rsh r0, r0, r3
 .global read_done_active_low
read_done_active_low:
 and r0, r0, 1
 jumpr io_low_active,0,eq
 ADD r2,r2,1
 move r0,r2
 ADD r1,r1,1
 jumpr read_io_loop_active_low,7,LT
 move r0,0
 move r2,0
 move r3,0
 move r1,io_number_active_low
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
