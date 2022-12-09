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
const irq_tmr1_addr 0x1E0D ; The interrupt timer value (Read)
const relay_djr_addr 0x1E0E ; Relay Device Job Register (Read)
const relay_crhi_addr 0x1E0F ; Relay Command Register HI (Read)
const relay_crlo_addr 0x1E10 ; Relay Command Register LO (Read)
const relay_djr_rst_addr 0x1E11 ; Reset Relay Device Job Register (Write)
const relay_rc3_addr 0x1E12 ; Repeat Counter Byte 3 (Read)
const relay_rc2_addr 0x1E13 ; Repeat Counter Byte 2 (Read)
const relay_rc1_addr 0x1E14 ; Repeat Counter Byte 1 (Read)
const relay_rc0_addr 0x1E15 ; Repeat Counter Byte 0 (Read)
const comm_status_addr 0x1E16 ; Communication Status Register (Read)
const dpcr_addr 0x1E17 ; Display Panel Configuration Request (Write)
const dpod_addr 0x1E18 ; Display Panel Output Data (Write)
const dpid_addr 0x1E19 ; Display Panel Input Data (Read)
const dmb_read_addr 0x1E1A ; Detector Module Buffer Read (Write)
const irq_tmr2_max_addr 0x1E1B ; Interrupt Timer Two Period Minus One (Read/Write)
const irq_tmr2_addr 0x1E1C ; Interrupt Counter Value (Read)
const fv_addr 0x1E1D ; Firmware Version number (Read)
const zero_indicator_addr 0x1E20 ; Zero channel indicator (Write)
const dmb_id_addr 0x1E30 ; Detector Module ID (Read)
const dmb_hi_addr 0x1E31 ; Detector Module HI Data Byte (Read)
const dmb_lo_addr 0x1E32 ; Detector Module LO Data Byte (Read)
const dmb_pwr_addr 0x1E33 ; Detector Module Power (Read)
const dmb_an_addr 0x1E34 ; Detector Module Antenna Number (Read)

; Controller job numbers.
const read_job 3
const command_job 10

; Bit masks.
const reset_bit_mask 0x01
const sel_bit_mask 0x04
const select_all_code 0xFF
const select_none_code 0x00
const upload_bit_mask 0x01
const empty_bit_mask 0x02
const ethernet_bit_mask 0x04
const int_ts_mask 0x01
const int_mrdy_mask 0x02
const int_lamps_mask 0x08
const int_dmerr_mask 0x04
const int_unused_mask 0xF0
const valid_id_mask 0x0F
const config_bit_mask 0x01
const dpirdy_bit_mask 0x10
const dmbrdy_bit_mask 0x20
const dmbfull_bit_mask 0x40
const mrdy_mask 0x80

; Display panel opcodes
const dp_opcode_msg 0x10
const dp_opcode_comm 0x20
const dp_opcode_sw 0x60

; Hardware Constants.
const num_indicators 15
const num_detectors 16

; Daisy chain index to antenna input map. The constant index_0, for
; example, contains the antenna input number connected to the first
; detector module in the daisy chain.
const index_0  13
const index_1  11
const index_2  9
const index_3  6
const index_4  4
const index_5  3
const index_6  2
const index_7  1
const index_8  5
const index_9  8
const index_10 7
const index_11 10
const index_12 12
const index_13 14
const index_14 15
const index_15 16

; Timinig Constants. RCK is 32.768 kHz, PCK is 20 MHz.
const rck_per_ts 255 ; RCK periods per timestamp interrupt minus one
const rck_per_lamps 15 ; RCK periods per lamp interrupt minus one
const activity_linger 4 ; Timer periods of light per message received
const flash_linger 200 ; Timer periods for reset flash
const daisy_chain_delay 5 ; PCK periods for daisy-chain round trip
const dmrst_length 10 ; PCK periods for reset pulse

; Message Identifiers
const clock_id 0x00 ; Clock message identifier.
const invalid_id 0xF0 ; Invalid message identifier

; Variable, Constant, Array, and Stack Locations.
const msg_id      0x0000 ; Message Identifier
const msg_hi      0x0001 ; Message Data, HI
const msg_lo      0x0002 ; Message Data, LO
const msg_pwr     0x0003 ; Message Power
const msg_an      0x0004 ; Message Antenna Number
const msg_id_prv  0x0005 ; Previous Message Identifier
const msg_hi_prv  0x0006 ; Previous Message Data, HI
const msg_lo_prv  0x0007 ; Previous Message Data, LO
const msg_pwr_prv 0x0008 ; Previous Message Power
const msg_an_prv  0x0009 ; Previous Message Antenna Number
const clock_hi    0x0020 ; Clock HI
const clock_lo    0x0021 ; Clock LO
const main_cntr   0x0022 ; Main Loop Counter
const zero_channel_select 0x0200 ; Base of channel select array
const zero_channel_timer 0x0300 ; Base of channel timer array
const zero_index_antenna 0x0400 ; Base of antenna mapping table
const sp_initial 0x1700 ; Bottom of the stack in RAM.

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

; Set up the detector module index to antenna input mapping. 
ld IX,zero_index_antenna
ld A,index_0
ld (IX),A
inc IX
ld A,index_1
ld (IX),A
inc IX
ld A,index_2
ld (IX),A
inc IX
ld A,index_3
ld (IX),A
inc IX
ld A,index_4
ld (IX),A
inc IX
ld A,index_5
ld (IX),A
inc IX
ld A,index_6
ld (IX),A
inc IX
ld A,index_7
ld (IX),A
inc IX
ld A,index_8
ld (IX),A
inc IX
ld A,index_9
ld (IX),A
inc IX
ld A,index_10
ld (IX),A
inc IX
ld A,index_11
ld (IX),A
inc IX
ld A,index_12
ld (IX),A
inc IX
ld A,index_13
ld (IX),A
inc IX
ld A,index_14
ld (IX),A
inc IX
ld A,index_15
ld (IX),A

; Clear the previous message, setting its ID to invalid_id and its other
; records to zero.
ld A,invalid_id
ld (msg_id_prv),A
ld A,0x00
ld (msg_hi_prv),A
ld (msg_lo_prv),A
ld (msg_pwr_prv),A
ld (msg_an_prv),A

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

; If we are initializing after a Top-Level or Mid-Level reset, the detector modules will
; already be reset, but if we are re-booting the Controller by jumping to the initialize
; routine, we need to assert DMRST to reset the detector modules and the display panel,
; so we do that now.
ld A,0x01
ld (dm_reset_addr),A
ld A,dmrst_length
dly A

; We just set DMRST in our code, but even if we had not done so, we would still have to
; clear it now, because a Top-Level or Mid-Level reset will set DMRST and leave it set
; after the reset is over.
ld A,0x00
ld (dm_reset_addr),A

; Write a zero clock message into the message buffer. We use "nop" instructions
; as a delay between writes to the message buffer so as to allow the buffer time
; to store the messages.
ld A,clock_id
ld (msg_write_addr),A
nop
nop
ld A,0
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
nop
nop
ld A,0
ld (msg_write_addr),A
nop
nop
nop
nop
ld (msg_write_addr),A

; Configure interrupt timers.
configure_interrupts:
ld A,rck_per_ts
ld (irq_tmr1_max_addr),A 
ld A,rck_per_lamps
ld (irq_tmr2_max_addr),A 

; Reset all interrupts.
ld A,select_all_code    
ld (irq_reset_addr),A  

; Unmask the timer and lamp interrupts. Interrupts are currently disabled 
; by the I flag.
ld A,int_ts_mask
or A,int_lamps_mask            
ld (irq_mask_addr),A  

; Reset the device job register, which tells the Relay that the Controller
; is ready for further commands. Any write to the reset location will do.
ld (relay_djr_rst_addr),A

; Falling edge on tp_reg(2) indicates the end of initialization.
ld A,(test_point_addr)
and A,0xFB                  
ld (test_point_addr),A   

; Enable interrupts to start the data acquisition.
clri

; Go to the main program.
jp main

; ------------------------------------------------------------
;                           MAIN
; ------------------------------------------------------------
main:

; ---------------------------------------------------------------
; Generate a rising edge on tp_reg(0) to indicate the main loop is
; starting. 9CK
; ---------------------------------------------------------------
ld A,(test_point_addr)
or A,0x01                    
ld (test_point_addr),A   

; ---------------------------------------------------------------
; Increment the main loop counter, which we can use to manage
; tasks that require less frequency attention. 8CK
; ---------------------------------------------------------------
ld A,(main_cntr)
inc A
ld (main_cntr),A

; ---------------------------------------------------------------
; Manage communication with the display panel. We alternate between
; transmitting a serial instruction to the display panel and 
; reading an instruction from the display panel.
; ---------------------------------------------------------------

; Check timer to see if we should transmit to display panel. 9CK
main_dp_comms:
ld A,(main_cntr)
sub A,0x40
jp nz,main_dpi

; Transmit the lower four bits of the communication status register
; to the display panel. We combine these four bits with the communication
; op-code and write to dpod_addr, which sets the data bits and initiates 
; transmission. 
ld A,(comm_status_addr)
and A,0x0F
or A,dp_opcode_comm
ld (dpod_addr),A

; Check to see if there is a new byte waiting from the display panel. If
; not, jump to next task. 9CK
main_dpi:
ld A,(comm_status_addr)
and A,dpirdy_bit_mask
jp z,main_message_handler

; Read the new byte received from the display panel and store in C.
ld A,(dpid_addr)
push A
pop C

; Check the opcode. If not one we recognise, go to message handler.
and A,0xF0
sub A,dp_opcode_sw 
jp nz,main_message_handler

; Check configuration bit state and update the configuration flag
; in the controller.
push C
pop A
and A,config_bit_mask
jp z,main_dp_config
ld A,0x01
main_dp_config:
ld (dpcr_addr),A
jp main_message_handler

; ---------------------------------------------------------------
; The main loop handles message readout from the detector modules,
; elimination of duplicates, and storage of these messages in the
; message buffer. Each time we run through the main loop, we will
; either do nothing, if no message is waiting, or read a message
; and set it aside, or write a previous message to the buffer. See
; diagram Message_Handler.jpg in A3042 documentation for states.
; ---------------------------------------------------------------
main_message_handler:

; Check the message ready (MRDY) flag. If it's not set, jump to 
; main_no_mrdy. 9CK
ld A,(comm_status_addr)
and A,dmbrdy_bit_mask
jp z,main_no_mrdy

; Read the new message. We assert Detector Module Buffer Read (DMBRD) to 
; read the message out of the Detector Module Buffer. This message is 
; guaranteed by the Detector Module Interface to have a valid ID. 38CK
ld (dmb_read_addr),A
ld A,(dmb_id_addr)
ld (msg_id),A
ld A,(dmb_hi_addr)
ld (msg_hi),A
ld A,(dmb_lo_addr)
ld (msg_lo),A
ld A,(dmb_pwr_addr)
ld (msg_pwr),A
ld A,(dmb_an_addr)
ld (msg_an),A

; Check if the new message has the same ID as our previous message. If 
; not, we jump to main_new_id. 14CK
ld A,(msg_id)
push A
pop B
ld A,(msg_id_prv)
sub A,B
jp nz,main_new_id

; Check to see if the new message has greater power. If not, we ignore
; this message. We are rejecting duplicates in our effort to fine the
; top antenna. 12CK
ld A,(msg_pwr_prv)
push A
pop B
ld A,(msg_pwr)
sub A,B
jp c,main_done_messages

; With the new message having greater power, we jump to main_overwrite_prv.
jp main_overwrite_prv

; Check to see if the previous message is valid. If not, jump to 
; main_overwrite_prv.
main_new_id:
ld A,(msg_id_prv)
and A,valid_id_mask
jp z,main_overwrite_prv

; Our previous message ID is valid and different from our new message ID, so
; we store the previous message before jumping to main_overwrite_prv. The 
; previous message may not be the top antenna message for its channel. We may
; have several transmitters colliding, with messages coming in from all of 
; them, alternating between channels as we read out the detector modules.
; Such collisions are rare, but they will result in multiple messages from
; the same channel being written to the message buffer. These duplicates
; will be removed later, when the Neuroplayer applies our lwdaq_receiver 
; routine to the data.
call save_msg_prv

; Overwrite the previous message with the new message. If we have not yet
; saved the previous message, it will never be saved. Our new message has
; become the previous message, so we jump to main_done_message.
main_overwrite_prv:
ld A,(msg_id)
ld (msg_id_prv),A
ld A,(msg_hi)
ld (msg_hi_prv),A
ld A,(msg_lo)
ld (msg_lo_prv),A
ld A,(msg_pwr)
ld (msg_pwr_prv),A
ld A,(msg_an)
ld (msg_an_prv),A
jp main_done_messages

; Now new message is ready. We check to see if the previous message is 
; valid. If not, jump to main_done_messages.
main_no_mrdy:
ld A,(msg_id_prv)
and A,valid_id_mask
jp z,main_done_messages

; No new message is ready. There is a lull in reception and our previous
; message is valid. There is no need to wait for any further messages: this
; message is most likely the top antenna message for its sample, and is
; certainly the last message for the sample.
call save_msg_prv

; Done with dealing with messages.
main_done_messages:

; ---------------------------------------------------------------
; Check the Device Job Register. If it's not zero, take action and 
; if requested action is complete, clear the register to indicate 
; the job is done. 9CK
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

; ----------------------------------------------------------------------
; Message Select Command. We interpret the command and modify the message
; selection array accordingly.
; ----------------------------------------------------------------------

msg_select:
ld A,(relay_crlo_addr)
and A,sel_bit_mask
jp z,other_commands

; Check the message select code.
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
; done. 9CK
; ---------------------------------------------------------------
djr_done:   
ld A,(test_point_addr)
and A,0xFE                
ld (test_point_addr),A  

; Return to top of main loop. 3CK
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
and A,int_lamps_mask
jp nz,int_lamps
ld A,(irq_bits_addr)
and A,int_dmerr_mask
jp nz,int_dmerr
ld A,int_unused_mask
ld (irq_reset_addr),A  
jp int_done

; ---------------------------------------------------------
; Generate a timestamp message, store in message buffer.
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
; writes time to complete. We need four clock cycles between
; writes to the message buffer. The "ld A,(nn)" instruction itself
; takes four cycles, and is therefore sufficient. We add two zeros 
; for the power and antenna number of the clock message.
store_clock:
ld A,clock_id
ld (msg_write_addr),A
ld A,(clock_hi)
ld (msg_write_addr),A
ld A,(clock_lo)
ld (msg_write_addr),A
ld A,(fv_addr)
add A,receiver_type
ld (msg_write_addr),A
nop
nop
ld A,0
ld (msg_write_addr),A
nop
nop
nop
nop
ld (msg_write_addr),A

; Reset the timestamp interrupt.
int_ts_done:
ld A,int_ts_mask             
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


; --------------------------------------------------------------
; Store the previous message in the message buffer, enable an
; activity lamp on the base board, transmit a lamp activity 
; notification to the display panel. Note that this routine will
; store any previous message, valid or not.
; --------------------------------------------------------------
save_msg_prv:

; Push all the flags and registers onto the stack so we don't
; have to worry about which ones we use in the interrupt.
push F 
push A
push H
push IX

; Check that this channel is among those enabled by the channel select
; array. If not, set the message identifier to the invalid identifier
; and terminate the readout. 
ld HL,zero_channel_select
push H
ld A,(msg_id)
push A
pop IX
ld A,(IX)
sub A,0
jp z,save_prv_done

; Transmit a message to the display panel. The eight-bit message consists
; of an operation code in the top four bits and the lower four bits of the 
; message identifier. The transmission will take 9 us, which is 180 
; instructions at 20 MHz. This routine will conclude in roughly 60
; instructions.
ld A,(msg_id_prv)
and A,valid_id_mask
or A,dp_opcode_msg
ld (dpod_addr),A

; Set the channel indicator timer. We use the lower four bits of
; the channel ID to select one of the fifteen indicator lamps.
ld HL,zero_channel_timer
push H
ld A,(msg_id_prv)
and A,valid_id_mask
push A
pop IX
ld A,activity_linger
ld (IX),A

; Swap the daisy chain index, which we have so far been using as
; our antenna number, for the antenna input number on the TC
; enclosure. During initialization, we set up a table specifying
; the mapping from index to antenna. We use that table now.
ld HL,zero_index_antenna
push H
ld A,(msg_an_prv)
push A
pop IX
ld A,(IX)
ld (msg_an_prv),A

; Store the message in the buffer. We disable interrupts during the write
; so that we do not conflict with the timestamp interrupt's writing of 
; clock messages to the same buffer. We pay particular attention to the
; value of the timestamp. While we were reading out the detector modules, 
; the timestamp may have incremented from 255 to 0 or 1, without any 
; opportunity to store a clock message in the buffer. The timer interrupt 
; bit will be set if and only if the timestamp has incremented without a 
; clock message being stored, so we check this bit, and if it is set, we 
; store 255 for the timestamp rather than the current value of the interrupt 
; timer. We do not need to add any "nop" instructions between writes to
; the message buffer because we have at least one "ld A,(nn)" between each
; write, and these take four clock cycles.
seti
ld A,(msg_id_prv)
ld (msg_write_addr),A
ld A,(msg_hi_prv)
ld (msg_write_addr),A
ld A,(msg_lo_prv)
ld (msg_write_addr),A
ld A,(irq_bits_addr)
and A,0x01
jp z,save_prv_ts
ld A,255
jp save_prv_stts
save_prv_ts:
ld A,(irq_tmr1_addr)
save_prv_stts:
ld (msg_write_addr),A
ld A,(msg_pwr_prv)
ld (msg_write_addr),A
ld A,(msg_an_prv)
ld (msg_write_addr),A
clri

; Set the previous message ID to invalid_id to mark it as written. Pop 
; the registers and flags off the stack.
save_prv_done:
ld A,invalid_id
ld (msg_id_prv),A
pop IX
pop H
pop A
pop F
ret

; ------------------------------------------------------------
;                       END
; ------------------------------------------------------------
