; Telemetry Control Box Baseboard Program
; ---------------------------------------

; ------------------------------------------------------------
;                        CONSTANTS
; ------------------------------------------------------------

; Receiver type code, to which we add the firmware version 
; to generate the receiver version. 
const receiver_type 120 ; For A3042 receivers.

; Control Register Addresses.
const irq_bits_addr 0x1E00 ; Interrupt Request Bits (Read)
const irq_mask_addr 0x1E01 ; Interrupt Mask Bits (Read/Write)
const irq_reset_addr 0x1E02 ; Interrupt Reset Bits (Write)
const irq_set_addr 0x1E03 ; Interrupt Set Bits (Write)
const irq_tmr1_max_addr 0x1E04 ; Interrupt Timer Period Minus One (Read/Write)
const cpu_rst_addr 0x1E05 ; Program Reset (Write)
const test_point_addr 0x1E06 ; Test Point Register (Read/Write)
const msg_write_addr 0x1E07 ; Message Write Data (Write)
const dm_reset_addr 0x1E08 ; Detector Module Reset (Write)
const dm_complete_addr 0x1E09 ; Detector Read Complete (Write)
const dm_strobe_addr 0x1E0A ; Data Strobe Upstream (Write)
const dm_data_addr 0x1E0B ; Detector Module Data (Read)
const dm_count_addr 0x1E0C ; Number of Queued Messages (Read)
const irq_tmr1_addr 0x1E0D ; The interrupt timer value (Read)
const relay_djr_addr 0x1E0E ; Relay Device Job Register (Read)
const relay_crhi_addr 0x1E0F ; Relay Command Register HI (Read)
const relay_crlo_addr 0x1E10 ; Relay Command Register LO (Read)
const relay_djr_rst_addr 0x1E11 ; Reset Relay Device Job Register (Write)
const relay_rc3_addr 0x1E12 ; Repeat Counter Byte 3 (Read)
const relay_rc2_addr 0x1E13 ; Repeat Counter Byte 2 (Read)
const relay_rc1_addr 0x1E14 ; Repeat Counter Byte 1 (Read)
const relay_rc0_addr 0x1E15 ; Repeat Counter Byte 0 (Read)
const fifo_cnt2_addr 0x1E16 ; Fifo Message Count Byte 2 (Read)
const fifo_cnt1_addr 0x1E17 ; Fifo Message Count Byte 1 (Read)
const fifo_cnt0_addr 0x1E18 ; Fifo Message Count Byte 0 (Read)
const incoming_addr 0x1E19 ; Detector Module Incoming (Read/Write)
const received_addr 0x1E1A ; Detector Module Received (Read/Write)
const irq_tmr2_max_addr 0x1E1B ; Interrupt Timer Two Period Minus One (Read/Write)
const irq_tmr2_addr 0x1E1C ; Interrupt Counter Value (Read)
const fv_addr 0x1E1D ; Firmware Version number (Read)
const zero_indicator_addr 0x1E20 ; Zero channel indicator (Write)

; Controller job numbers.
const read_job 3
const command_job 10

; Command bit masks.
const reset_bit_mask 0x01
const sel_bit_mask 0x04
const select_all_code 0xFF
const select_none_code 0x00

; Hardware Constants.
const num_indicators 15
const num_detectors 16

; Detector coil to daisy chain mapping. We give the location in the
; daisy chain of each antenna coil. We will write the antenna coil
; powers into a buffer in order they are listed on the tracker geometry
; drawing. The number of antennas whos powers are written to memory
; is not necessarily equal the number of detector modules. The power
; measurement from an auxilliary antenna will be written last or not 
; at all.
const antenna_1 8
const antenna_2 7
const antenna_3 6
const antenna_4 9
const antenna_5 5
const antenna_6 4
const antenna_7 11
const antenna_8 10
const antenna_9 3
const antenna_10 12
const antenna_11 13
const antenna_12 2
const antenna_13 15
const antenna_14 14
const antenna_15 1
const antenna_16 16

; Interrupt bit masks.
const int_ts_mask 0x01
const int_rcv_mask 0x02
const int_lamps_mask 0x08
const int_dmerr_mask 0x04
const int_unused_mask 0xF0

; Software Constants
const sp_initial 0x1700 ; Bottom of the stack in RAM.
const init_bg_pwr_value 15 ; Initial value for background powers

; Timinig Constants. RCK is 32.768 kHz, PCK is 20 MHz.
const rck_per_ts 255 ; RCK periods per timestamp interrupt minus one
const rck_per_lamps 15 ; RCK periods per lamp interrupt minus one
const ts_per_bgpwr 64; Timestamp interrupts per background measurement
const activity_linger 4 ; Timer periods of light per message received
const flash_linger 200 ; Timer periods for reset flash
const daisy_chain_delay 10 ; PCK periods for daisy-chain round trip
const dmrst_length 10 ; PCK periods for reset pulse.

; Variable Locations.
const msg_id 0x0000 ; Message ID
const msg_hi 0x0001 ; HI byte of message contents
const msg_lo 0x0002 ; LO byte of message contents
const clock_hi 0x0003 ; HI byte of clock
const clock_lo 0x0004 ; LO byte of clock
const ts_cntr 0x0005 ; Counts timer interrupts
const dm_pwr 0x0100 ; Power array, size is num_detectors
const zero_channel_select 0x0200 ; Base of channel select array
const zero_channel_timer 0x0300 ; Base of channel timer array
const dm_bg_pwr 0x0400 ; Base of background power array

; ------------------------------------------------------------
;                         START
; ------------------------------------------------------------
start: 

; The CPU reserves two locations 0x0000 for the start of program
; execution, and 0x0003 for interrupt execution. We put jumps at
; both locations. A jump takes exactly three bytes.
jp initialize
jp interrupt

; ------------------------------------------------------------
;                       INITIALIZE
; ------------------------------------------------------------
initialize:

; Disable interrupts, in case we are running this routine from
; a jump instruction as a way of re-booting quickly.
seti

; Initialize the stack pointer.
ld HL,sp_initial
ld SP,HL

; Rising edge on tp_reg(2).
ld A,(test_point_addr)
or A,0x04                   
ld (test_point_addr),A   

; Disable interrupts, cancel interrupt requests.
ld A,0x00            
ld (irq_mask_addr),A  
ld A,0xFF              
ld (irq_reset_addr),A  

; Select all channels for reception by writing ones to their select bits.
ld A,0xFF
push A
pop B
ld A,0x01
ld IX,zero_channel_select
init_channel_select_loop:
ld (IX),A
inc IX
dec B
jp nz,init_channel_select_loop

; Flash the indicators. We set all the indicator counters to the flash_linger value
; and so turn on the indicators for flash_linger timer interrupt periods. The flash
; will start only after we enable interrupts.
ld IX,zero_channel_timer
ld A,num_indicators
push A
pop B
ld A,flash_linger
flash_indicators:
inc IX
ld (IX),A
dec B
jp nz,flash_indicators

; Reset the clock.
ld A,0x00
ld (clock_hi),A
ld (clock_lo),A

; If we are initializing after RESET, the detector modules will already be
; in their reset state because DMRST will be asserted. If we are re-initializing
; in software, we must set DMRST now to reset the detectors. We wait for some
; time to allow the reset to complete.
ld A,0x01
ld (dm_reset_addr),A
ld A,dmrst_length
dly A

; Clear detector modulr reset, DMRST, which allows the detectors to start receiving.
ld A,0x00
ld (dm_reset_addr),A

; Write a zero clock message into the message buffer. We use "nop" instructions
; as a delay between writes to the message buffer so as to allow the buffer time
; to store the messages.
ld A,0x00
ld (msg_write_addr),A
nop
nop
nop
nop
ld (msg_write_addr),A
nop
nop
nop
nop
ld (msg_write_addr),A
nop
nop
nop
nop
ld A,(fv_addr)
add A,receiver_type
ld (msg_write_addr),A

; Store the initial values of background powers as the payload to the 
; clock message.
ld IX,dm_bg_pwr
ld A,num_detectors
push A
pop B
ld A,init_bg_pwr_value
init_bg_pwr:
inc IX
ld (IX),A
dec B
jp nz,init_bg_pwr
ld HL,dm_bg_pwr
call store_powers

; Set the timestamp interrupt counter to one, so we will record 
; background powers on the next timestamp interrupt.
ld A,1
ld (ts_cntr),A

; Set up a interrupt timer.
configure_interrupts:
ld A,rck_per_ts
ld (irq_tmr1_max_addr),A 
ld A,rck_per_lamps
ld (irq_tmr2_max_addr),A 

; Reset all interrupts.
ld A,select_all_code    
ld (irq_reset_addr),A  

; Turn on timer interrupt, receive message interrupt, and lamp interrupts. 
; Note that interrupts are currently disabled by the I flag.
ld A,int_ts_mask
or A,int_rcv_mask
or A,int_lamps_mask            
ld (irq_mask_addr),A  

; Reset the device job register, which tells the Relay that the Controller
; is ready for further commands. Any write to the reset location will do.
ld (relay_djr_rst_addr),A

; Falling edge on tp_reg(2).
ld A,(test_point_addr)
and A,0xFB                  
ld (test_point_addr),A   

; The last thing we do is enable interrupts
clri

; Go to the main program.
jp main

; ------------------------------------------------------------
;                           MAIN
; ------------------------------------------------------------
main:

; ---------------------------------------------------------------
; Generate a rising edge on tp_reg(0) to indicate the main loop is
; starting.
; ---------------------------------------------------------------
ld A,(test_point_addr)
or A,0x01                    
ld (test_point_addr),A   

; ---------------------------------------------------------------
; Check the Device Job Register. If it's not zero, take action and 
; if requested action is complete, clear the register to indicate 
; the job is done.
; ----------------------------------------------------------------
ld A,(relay_djr_addr)
add A,0x00
jp z,djr_done

; Check for a command transmit job.
push A
sub A,command_job
pop A
jp nz,not_command_job

; Generate a rising edge of tp_reg(2).
ld A,(test_point_addr)
or A,0x04                    
ld (test_point_addr),A   

; ----------------------------------------------------------------------
; Read the command and see if it matches the reset command. If so, we
; reboot the controller. The reboot does not reset the device job register.
; Instead, the register will be cleared after re-boot by the initialization
; routine, to indicate that the reset is complete.
; ----------------------------------------------------------------------
ld A,(relay_crlo_addr)
and A,reset_bit_mask
jp z,msg_select
ld (cpu_rst_addr),A
wait

msg_select:
ld A,(relay_crlo_addr)
and A,sel_bit_mask
jp z,other_commands

; ----------------------------------------------------------------------
; Message Select Command. We interpret the command and modify the message
; selection array accordingly.
; ----------------------------------------------------------------------
ld A,(relay_crhi_addr)
sub A,select_none_code
jp z,select_none
ld A,(relay_crhi_addr)
sub A,select_all_code
jp z,select_all

; Set the select bit for this particular channel.
ld HL,zero_channel_select
push H
ld A,(relay_crhi_addr)
push A
pop IX
ld A,0x01
ld (IX),A
jp done_cmd_xmit

; For select none, we write a zero to all select bits.
select_none:
ld A,0xFF
push A
pop B
ld A,0x00
ld IX,zero_channel_select
select_none_loop:
ld (IX),A
inc IX
dec B
jp nz,select_none_loop
jp done_cmd_xmit

; For select all, we write a one to all select bits.
select_all:
ld IX,zero_channel_select
ld A,0xFF
push A
pop B
ld A,0x01
select_all_loop:
ld (IX),A
inc IX
dec B
jp nz,select_all_loop
jp done_cmd_xmit

;---------------------------------------------------------------------
; Other commands we ignore.
;---------------------------------------------------------------------
other_commands:
jp done_cmd_xmit

;---------------------------------------------------------------------
; Done with all supported command jobs.  Generate a falling, edge on 
; tp_reg(2). 
;----------------------------------------------------------------------
done_cmd_xmit:
ld A,(test_point_addr)
and A,0xFB                    
ld (test_point_addr),A   
jp djr_reset

;---------------------------------------------------------------------
; Other jobs we ignore.
;---------------------------------------------------------------------
not_command_job:
jp djr_reset

; ------------------------------------------------------------------
; Reset the device job register. Any write to the DJR reset location 
; clears DJR to zero.
; ------------------------------------------------------------------
djr_reset:
ld A,0x01
ld (relay_djr_rst_addr),A

; ---------------------------------------------------------------
; Generate a falling edge on tp_reg(0) to indicate the main loop is
; done.
; ---------------------------------------------------------------
djr_done:   
ld A,(test_point_addr)
and A,0xFE                
ld (test_point_addr),A  

jp main 


; ------------------------------------------------------------
;                        INTERRUPT
; ------------------------------------------------------------
interrupt:

; -----------------------------------------------------------
; Push all the flags and registers onto the stack so we don't
; have to worry about which ones we use in the interrupt.
;------------------------------------------------------------
push F 
push A
push B
push C
push D
push E
push H
push L
push IX
push IY

; ------------------------------------------------------------
; Generate a rising edge on tp_reg(1) to indicate start of 
; interrupt routine.
; ------------------------------------------------------------
ld A,(test_point_addr) 
or A,0x02       
ld (test_point_addr),A 

; ------------------------------------------------------------------
; Determine which interrupt request bit is set, and act upon it in
; order of highest to lowest priority. If we don't recognise the 
; interrupt, we reset all the unknown interrupts.
;-------------------------------------------------------------------
ld A,(irq_bits_addr)
and A,int_ts_mask
jp nz,int_ts
ld A,(irq_bits_addr)
and A,int_rcv_mask
jp nz,int_rcv
ld A,(irq_bits_addr)
and A,int_lamps_mask
jp nz,int_lamps
ld A,(irq_bits_addr)
and A,int_dmerr_mask
jp nz,int_dmerr
ld A,int_unused_mask
ld (irq_reset_addr),A  
jp int_done

; ---------------------------------------------------------
; Generate a timestamp message, store in message buffer along
; with background powers. Sometimes, we initiate a background
; power measurement.
;----------------------------------------------------------
int_ts:

; Increment the sixteen-bit bit clock.
ld A,(clock_lo)
inc A
ld (clock_lo),A
jp nz,store_clock
ld A,(clock_hi)
inc A
ld (clock_hi),A

; Write the clock identifier, which is zero, and the clock high
; and low bytes to the message buffer, followed by the receiver
; version. These four bytes are the clock message without any
; payload. We add "nop" instructions to give the message buffer
; writes time to complete.
store_clock:
ld A,0x00
ld (msg_write_addr),A
nop
nop
ld A,(clock_hi)
ld (msg_write_addr),A
nop
nop
ld A,(clock_lo)
ld (msg_write_addr),A
nop
nop
ld A,(fv_addr)
add A,receiver_type
ld (msg_write_addr),A

; Store the background powers as the payload to the clock message.
ld HL,dm_bg_pwr
call store_powers

; Every ts_per_bgpwr timestamp interrupts, we refresh our measurement
; of background power.
ld A,(ts_cntr)
dec A
ld (ts_cntr),A
jp nz,int_ts_done
ld A,ts_per_bgpwr
ld (ts_cntr),A

; Check to see if INCOMING or RECEIVED are unasserted. If not, then
; jump to end of timestamp interrupt.
ld A,(incoming_addr)
and A,0x01
jp nz,int_ts_done
ld A,(received_addr)
and A,0x01
jp nz,int_ts_done

; Assert INCOMING then RECEIVED so as to get the detector modules
; to store a background power measurement. We start by asserting 
; INCOMING, wait 1.5 us, assert RECEIVED, wait 1.5 us, unassert both.
ld A,0x01
ld (incoming_addr),A
ld A,30
dly A
ld A,0x01
ld (received_addr),A
ld A,30
dly A
ld A,0x00
ld (incoming_addr),A
ld (received_addr),A

; Reset the timer interrupt.
int_ts_done:
ld A,int_ts_mask             
ld (irq_reset_addr),A  
jp int_done

; --------------------------------------------------------------
; Read out the detector modules using the daisy chain bus. If the
; channel is selected for storage, read the entire power array, store
; the message id, contents, and power measurements in RAM, the channel
; activity lamp that corresponds to the message identifier, and store
; the message in the message buffer. 
; --------------------------------------------------------------
int_rcv:

; First pulse on DSU allows detectors to determine the best
; receiver and so establish the channel ID. We give the detector
; modules a few clock cycles to settle down.
ld A,0x01               
ld (dm_strobe_addr),A
ld A,daisy_chain_delay
dly A
ld A,0x00
ld (dm_strobe_addr),A
ld A,daisy_chain_delay
dly A

; Read the channel ID.
ld A,0x01               
ld (dm_strobe_addr),A
ld A,daisy_chain_delay
dly A
ld A,(dm_data_addr)
ld (msg_id),A
ld A,0x00
ld (dm_strobe_addr),A
ld A,daisy_chain_delay
dly A

; Check that this channel is among those enabled by the channel select
; array. If not, set the message_id to zero and terminate the readout.
; A message with message_id zero is one we always read out, because it
; contains background powers. We will store the background powers in 
; an array, but we will not write a zero-channel message to memory. We 
; write zero-channel messages only on timestamp interrupts.
ld A,(msg_id)
add A,0x00
jp z,int_continue_read
ld HL,zero_channel_select
push H
ld A,(msg_id)
push A
pop IX
ld A,(IX)
sub A,0x00
jp nz,int_continue_read
ld A,0x00
ld (msg_id),A
jp dm_drc

; Read the HI data byte.
int_continue_read:
ld A,0x01               
ld (dm_strobe_addr),A
ld A,daisy_chain_delay
dly A
ld A,(dm_data_addr)
ld (msg_hi),A
ld A,0x00
ld (dm_strobe_addr),A
ld A,daisy_chain_delay
dly A

; Read the LO data byte.
ld A,0x01               
ld (dm_strobe_addr),A
ld A,daisy_chain_delay
dly A
ld A,(dm_data_addr)
ld (msg_lo),A
ld A,0x00
ld (dm_strobe_addr),A
ld A,daisy_chain_delay
dly A

; Read the detector powers, storing them in locations dm_pwr + daisy chain 
; location when the message_id is non-zero, storing them in the dm_bg_pwr + 
; daisy chain location when the message_id is zer. So the first detector 
; power will be at location dm_pwr + 1 or dm_bg_pwr + 1. First we load IX 
; with the base of the correct array.
ld A,(msg_id)
add A,0x00
jp z,sel_dm_bg
ld IX,dm_pwr
jp rd_all_pwrs
sel_dm_bg:
ld IX,dm_bg_pwr

; Read the daisy chain to get the power from each detector module.
rd_all_pwrs:
ld A,num_detectors
push A
pop B
inc IX
rd_pwr:
ld A,0x01               
ld (dm_strobe_addr),A
ld A,daisy_chain_delay
dly A
ld A,(dm_data_addr)
ld (IX),A
ld A,0x00
ld (dm_strobe_addr),A
ld A,daisy_chain_delay
dly A
inc IX
dec B
jp nz,rd_pwr

; Generate a pulse on Detector Read Complete
dm_drc:
ld A,0x01               
ld (dm_complete_addr),A
ld A,daisy_chain_delay
dly A
ld A,0x00               
ld (dm_complete_addr),A

; Check again to see if the message id is zero, and if so, we are done.
; We don't store a zero-id message in the buffer now, we will do so at
; the next timestamp interrupt, and at that time we will use the background
; powers stored in this receive interrupt.
ld A,(msg_id)
add A,0x00
jp z,int_rcv_done

; Set the channel indicator timer. We use the lower four bits of
; the channel ID to select one of the fifteen indicator lamps.
ld HL,zero_channel_timer
push H
ld A,(msg_id)
and A,0x0F
push A
pop IX
ld A,activity_linger
ld (IX),A

; Store the message in the buffer. We pay particular attention to the
; value of the timestamp. While we were reading out the detector 
; modules, the timestamp may have incremented from 255 to 0 or 1, without
; any opportunity to store a clock message in the buffer. The timer
; interrupt bit will be set if and only if the timestamp has incremented
; without a clock message being stored, so we check this bit, and if it
; is set, we store 255 for the timestamp rather than the current value of
; the interrupt timer.
ld A,(msg_id)
ld (msg_write_addr),A
ld A,(msg_hi)
ld (msg_write_addr),A
ld A,(msg_lo)
ld (msg_write_addr),A
ld A,(irq_bits_addr)
and A,0x01
jp z,int_rcv_ts
ld A,255
jp int_rcv_stts
int_rcv_ts:
ld A,(irq_tmr1_addr)
int_rcv_stts:
ld (msg_write_addr),A

; Store the power measurements in the buffer. We use the
; store powers routine to transfer the powers we have just
; recorded in the dm_pwr array.
ld HL,dm_pwr
call store_powers

; Reset the receiver interrupt.
int_rcv_done:
ld A,int_rcv_mask             
ld (irq_reset_addr),A  
jp int_done

; ------------------------------------------------------------------
; On a lamp interrupt, we attend to the activity lamps, we decrement
; the interrupt counter, and if the counter reaches zero, we store
; a timestamp message and set the counter to its maximum value.
;-------------------------------------------------------------------
int_lamps:

; Go through the indicator timers, decrementing by one, or leaving
; them at one. When they reach one, we turn off the lamp, otherwise
; we turn on the lamp. We use B to count through the channels, IX
; and IY to point to the timers and indicators.
ld A,num_indicators       
push A
pop B
ld IX,zero_channel_timer   
ld IY,zero_indicator_addr  

; Read the timer value. If zero, leave it, if greater  than zero, 
; decrement it. If zero, jump to lamp off.
ch_tmr_start:
inc IX
inc IY
ld A,(IX)                   
add A,0x00                 
jp z,ch_lamp_off
dec A
jp z,ch_lamp_off

; Load timer with decremented value and turn on the lamp
ld (IX),A                  
ld A,0x01                  
ld (IY),A
jp ch_tmr_end

; A is zero and the lamp should be off, so set timer to zero and 
; lamp control to zero.
ch_lamp_off:                
ld (IX),A  
ld (IY),A

; Repeat until all fifteen timers have been handled.
ch_tmr_end:                
dec B
jp nz,ch_tmr_start

; We are done, so reset the lamp timer interrupt.
int_lamps_done:
ld A,int_lamps_mask             
ld (irq_reset_addr),A  
jp int_done

; --------------------------------------------------------
; If we are acting on the detector module error interrupt,
; we reset the detector modules and the available message 
; counter by writing to the detectcor module reset location.
; --------------------------------------------------------
int_dmerr:
ld A,0x01              ; Set bit zero
ld (dm_reset_addr),A   ; and write to DMRST.
ld A,dmrst_length      ; Load reset pulse length
dly A                  ; and wait.
ld A,0x00              ; Clear bit zero
ld (dm_reset_addr),A   ; and write to DMRST.
ld A,int_dmerr_mask    ; Load the dmerr mask and use
ld (irq_reset_addr),A  ; to reset the dmerr interrupt bit.
jp int_done

; --------------------------------------------------------
; Generate a falling edge on tp_reg(1) to indicate the end
; of the interrupt. Pop the registers and flags off the 
; stack and return from interrupt.
; --------------------------------------------------------
int_done:
ld A,(test_point_addr) 
and A,0xFD 
ld (test_point_addr),A 
pop IY
pop IX
pop L
pop H
pop E
pop D
pop C
pop B
pop A
pop F
rti

; -------------------------------------------------------------
;                        SUBROUTINES
; -------------------------------------------------------------

; Calling convention: calling process pushes anything it wants
; protected onto the stack before calling, and pops upon return.

; Store Detector Powers in Message Buffer. We un-scramble the daisy 
; chain so as to present the measurements in the order of the base 
; board coil numbering, rather than the daisy chain order. Stores 
; a total of sixteen bytes to the message buffer for sixteen power
; detector values. The power values must be available in an array 
; pointed to by HL when the routine is called.
store_powers:

; Detector Coil Number 1.
ld A,antenna_1
push H
push A
pop IX
ld A,(IX)
ld (msg_write_addr),A

; Detector Coil Number 2.
ld A,antenna_2
push H
push A
pop IX
ld A,(IX)
ld (msg_write_addr),A

; Detector Coil Number 3.
ld A,antenna_3
push H
push A
pop IX
ld A,(IX)
ld (msg_write_addr),A

; Detector Coil Number 4.
ld A,antenna_4
push H
push A
pop IX
ld A,(IX)
ld (msg_write_addr),A

; Detector Coil Number 5.
ld A,antenna_5
push H
push A
pop IX
ld A,(IX)
ld (msg_write_addr),A

; Detector Coil Number 6.
ld A,antenna_6
push H
push A
pop IX
ld A,(IX)
ld (msg_write_addr),A

; Detector Coil Number 7.
ld A,antenna_7
push H
push A
pop IX
ld A,(IX)
ld (msg_write_addr),A

; Detector Coil Number 8.
ld A,antenna_8
push H
push A
pop IX
ld A,(IX)
ld (msg_write_addr),A

; Detector Coil Number 9.
ld A,antenna_9
push H
push A
pop IX
ld A,(IX)
ld (msg_write_addr),A

; Detector Coil Number 10.
ld A,antenna_10
push H
push A
pop IX
ld A,(IX)
ld (msg_write_addr),A

; Detector Coil Number 11.
ld A,antenna_11
push H
push A
pop IX
ld A,(IX)
ld (msg_write_addr),A

; Detector Coil Number 12.
ld A,antenna_12
push H
push A
pop IX
ld A,(IX)
ld (msg_write_addr),A

; Detector Coil Number 13.
ld A,antenna_13
push H
push A
pop IX
ld A,(IX)
ld (msg_write_addr),A

; Detector Coil Number 14.
ld A,antenna_14
push H
push A
pop IX
ld A,(IX)
ld (msg_write_addr),A

; Detector Coil Number 15.
ld A,antenna_15
push H
push A
pop IX
ld A,(IX)
ld (msg_write_addr),A

; Detector Coil Number 16.
ld A,antenna_16
push H
push A
pop IX
ld A,(IX)
ld (msg_write_addr),A

done_store_powers:
ret

; ------------------------------------------------------------
;                       END
; ------------------------------------------------------------
