-- <pre> Telemetry Control Box (TCB) Controller Firmware. 

-- V1.1, 15-SEP-22: Based upon P3038BB v8.3.

-- V1.2, 16-SEP-22: Use DC5 and DC6 for communication with display
-- panel. We use DC6 for Serial Data Out (SDO) and DC5 for Serial
-- Data In (SDI). These signals were SHOW and HIDE on the ALT
-- detector modules. Add reset value high-impedance for RECEIVED_pin 
-- and INCOMING_pin. Prior ommission of explicit reset values was 
-- causing RECEIVED_pin to be driven HI by the base board, stopping 
-- all detector readout. 

-- V2.1, 20-NOV-22: Eliminate INCOMING, change RECEIVED into MRDY. 
-- We make MRDY available in the CPU memory. The logic is now designed 
-- to support the TCB readout of all messages received by individual, 
-- independent detector modules. Update Reset Arbitrator to cooperate 
-- with the Display Panel.

-- V2.2, 23-NOV-22: Add support for display panel configuration switch.

-- V3.2, 07-DEC-22: Add SDO transmit Buffer. Add SDI receiver Buffer.

-- V3.3, 09-DEC-22: Add Detector Module Buffer. Reduce program memory to 
-- 2 KByte so code will ultimately fit into a 4000ZE chip. Detector
-- Module Interface reads out the daisy chain and rejects invalide
-- message before storing in the buffer. Message reading greatly 
-- simplified in the CPU. Correct bug in implementation of channel
-- simplified in the CPU. Correct bug in implementation of channel
-- selection in CPU code. Main loop when eliminating duplicates
-- takes 38-87 PCK cycles. Running with PCK at 20 MHz in the 7000HC 
-- chip, that's 3 us, or in the 4000ZE chip, 6 us.

-- V3.4, 10-DEC-22: Fix bug in duplicate elimination.

-- V4.1, 15-DEC-22: Fix message handler bug. First attempt at production code.

-- V4.2, 25-DEC-22: Reassign DC4 as Detector Module Configure (DMCFG) under
-- control of the CPU, for use in conjunction with Detector Module Read
-- Control (DMRC) to let the detector modules figure out their own location
-- in the daisy chain. Fix bug whereby CPU was reading out the input of the
-- detector module buffer instead of the output. 

-- V4.3, 05-JAN-23: Swap UPLOAD and EMPTY in communication status register.
-- Improve comments. Increase fifo_near_empty from 15 to 128 to improve the
-- buffer empty indicator behavior. Adjust the detector module index map to match 
-- latest detector module firmware 2.4. Add status flags to timestamp message as 
-- payload.

-- V5.1, 29-JUN-24: Add serial interface for Transmitting Feedthrough (A3042TF).
-- Take over TP3 and TP4 for TX and RX. Outgoing data is sixteen-bit words-- that the CPU writes into a buffer for transmission. Incoming data is eight bits 
-- updated each time an eight-bit transmission is received from the feedthrough.
-- Remove unused thirty-two bit repeat counter. The TX transmitter asserts TX during
-- RESET, which in turn resets the TF. Re-work the Reset Arbiter so it holds down
-- the base board !RESET after pressing base board or display panel reset switches.
-- Make mid-level RESET pulse long enough to cause TF to reset. Add read location in
-- control space for the digital input values received from the TF. 

-- V5.2, 11-JUL-24: In software, we refine duplicate rejection to check the contents 
-- of the messages as well. 

-- V5.3, 15-AUG-24: Increase drive current of DMCK to 24 mA. Increment hardware version
-- to 2 to indicate presence of the DMCK Termination Modificaiton. 

-- V6.1, 14-APR-25: Accelerate detector module readout, now takes 3 us per message. 
-- Add Duplicate Selector process, which reads from the Detector Module buffer and
-- selects the most powerful of consecutive duplicate messages, then stores them in
-- a Selected Message Buffer for the CPU to read out and store.

-- Global constants and types.  
library ieee;  
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity main is 
	port (
		MCK : in std_logic; -- Master (10 MHz) Clock
		RCK_in : in std_logic; -- Reference (32.768 kHz) Clock
		
		cont_data : inout std_logic_vector(7 downto 0);
		cont_addr : in std_logic_vector(5 downto 0);
		CWR_in : in std_logic;
		CDS_in : in std_logic;
		ETH : in std_logic;
		EGRN, EYLW : out std_logic;
		
		buff_addr : out std_logic_vector(20 downto 0);
		buff_data : inout std_logic_vector(7 downto 0);
		REN_not : out std_logic;
		RWE_not : out std_logic;
		
		RESET_not : inout std_logic;
		
		indicators : out std_logic_vector(1 to 19);
		config_lamp : out std_logic;
		
		switches : in std_logic_vector(2 to 4);
		
		DSU : out std_logic; -- Data Strobe Upstream
		dub : in std_logic_vector(7 downto 0); -- Data Upstream Bus
		
		DMRST_pin : inout std_logic; -- Detector Module Reset (DC0)
		DMRC : out std_logic; -- Detector Module Read Control (DC1)
		DMERR_in : in std_logic; -- Detecor Module Error (DC2)
		MRDY_in : in std_logic; -- Message Ready Flag (DC3)
		DMCFG : inout std_logic; -- Detector Module Show Lamps (DC4)
		SDI_in : in std_logic; -- Serial Data In (DC5)
		SDO : out std_logic; -- Serial Data Out (DC6)
		DMCK : out std_logic; -- Demodulator Clock (DC7)
		
		TX : out std_logic; -- Transmitting Feedthrough Transmit
		RX : in std_logic; -- Transmitting Feedthrough Receive
		
		TP1, TP2 : out std_logic -- Test Points
	);	

-- Version numbers.
	constant hardware_id : integer := 42;
	constant hardware_version : integer := 2;
	constant firmware_version : integer := 6;

-- Configuration of OSR8.
	constant prog_addr_len : integer := 11;
	constant cpu_addr_len : integer := 12;
	constant ram_addr_len : integer := 12;
	constant start_pc : integer := 0;
	constant interrupt_pc : integer := 3;
end;

architecture behavior of main is

-- Attributes to guide the compiler.
	attribute syn_keep : boolean;
	attribute nomerge : string;

-- Reset signals.
	signal RST_BY_CPU : boolean := false; -- Reset by CPU
	signal RST_BY_RELAY : boolean := false; -- Reset by Relay
	signal DMRST_BY_CPU : boolean := false; -- Reset Detector Modules by CPU
	signal BBRST : boolean; -- Reset by Base Board Switch
	signal BBRSTD : boolean; -- BBRST Delayed
	signal DMRST : boolean; -- Reset from Detector Module Bus

-- Synchronized, delayed, and inverted inputs.
	signal RCK, RESET, MRDY, DMERR : std_logic;
	signal CDS, CWR : boolean;
	signal DMRC_DEL : std_logic;
	
-- Clock and Timing Signals.
	signal LOCK : std_logic; -- Phase Locked Loop Has Locked
	signal FCK : std_logic; -- Fast Clock (80 MHz)
	signal CK : std_logic; -- State Machine Clock (40 MHz)
	signal PCK : std_logic; -- Processor Clock (20 MHz)
	signal SCK : std_logic; -- Serial Clock (2 MHz)
	
-- Indicator Signals
	signal UPLOAD : boolean; -- Uploading To Relay
	signal EMPTY : boolean; -- Message Buffer Empty
	signal BBCONFIG : boolean; -- Base Board Requesting Reconfiguration
	signal DPCONFIG : boolean; -- Display Panel Requesting Reconfiguration
	signal CONFIG : boolean; -- Reconfiguration Requested
	
-- Message Buffer
	signal MWRS : boolean; -- Message Write Strobe
	signal MWRACK : boolean; -- Message Write Acknowledge
	signal mwr_data : std_logic_vector(7 downto 0); -- Message Write Data
	signal MRDS : boolean; -- Message Read Strobe
	signal MRDACK : boolean; -- Message Read Acknowledge
	signal mrd_data : std_logic_vector(7 downto 0); -- Message Read Data
	signal fifo_byte_count : std_logic_vector(20 downto 0);
	constant fifo_near_empty : integer := 128; 

-- CPU-Writeable Test Points
	signal tp_reg : std_logic_vector(7 downto 0) := "00000000";
	signal indicator_control : std_logic_vector(1 to 15);

-- CPU memory access signals.
	signal prog_data : std_logic_vector(7 downto 0); -- Program memory data bus
	signal prog_addr : std_logic_vector(prog_addr_len-1 downto 0); -- Program memory address bus.
	signal cpu_ram_addr : std_logic_vector(ram_addr_len-1 downto 0); -- RAM Address
	signal cpu_ram_out, cpu_ram_in : std_logic_vector(7 downto 0); -- RAM Data In and Out
	signal CPURWR : std_logic; -- CPU Ram Write
	signal cpu_data_out, cpu_data_in : std_logic_vector(7 downto 0); 
	signal cpu_addr : std_logic_vector(cpu_addr_len-1 downto 0);
	attribute syn_keep of cpu_addr : signal is true;
	attribute nomerge of cpu_addr : signal is "";  
	signal CPUWR : boolean; -- Write (Not Read)
	signal CPUDS : boolean; -- Data Strobe

-- CPU control, interrupt, and error signals.
	signal CPUIRQ : boolean; -- Interrupt Request
	signal CPUSIG : std_logic_vector(2 downto 0); -- Signals for debugging.
	signal irq_mask : std_logic_vector(7 downto 0); -- Interrupt Mask
	signal irq_tmr1_max : std_logic_vector(7 downto 0); -- Interupt Request Period
	signal irq_tmr1 : std_logic_vector(7 downto 0); -- Interrupt Request Counter
	signal irq_tmr2_max : std_logic_vector(7 downto 0); -- Interupt Request Period
	signal irq_tmr2 : std_logic_vector(7 downto 0); -- Interrupt Request Counter
	signal irq_bits : std_logic_vector(7 downto 0); -- Interrupt Request Bits
	signal irq_rst : std_logic_vector(7 downto 0); -- Interrupt Reset Bits
	signal irq_set : std_logic_vector(7 downto 0); -- Interrupt Set Bits
	signal INTCTRZ1,INTCTRZ2 : boolean; -- Interrupt Counter Zero Flags
	signal DJRRST : boolean; -- Reset Device Job Register
	attribute syn_keep of DJRRST : signal is true;
	attribute nomerge of DJRRST : signal is "";  
		
	-- CPU Memory Map Constants, sizes and base addresses in blocks of 512 bytes.
	constant cpu_ram_base : integer := 0;
	constant cpu_ram_range : integer := 6;
	constant cpu_ctrl_base : integer := 7;
	constant cpu_ctrl_range : integer := 1;
	
	-- CPU Memory Map Constants, low byte of control register space.
	constant irq_bits_addr : integer := 0; -- Interrupt Request Bits (Read)
	constant irq_mask_addr : integer := 1; -- Interrupt Mask Bits (Read/Write)
	constant irq_reset_addr : integer := 2; -- Interrupt Reset Bits (Write)
	constant irq_set_addr : integer := 3; -- Interrupt Set Bits (Write)
	constant irq_tmr1_max_addr : integer := 4; -- Timer One Period Minus One (Read/Write)
	constant cpu_rst_addr : integer := 5; -- Program Reset (Write)
	constant test_point_addr : integer := 6; -- Test Point Register (Read/Write)
	constant msg_write_addr : integer := 7; -- Message Write Data (Write)
	constant dm_reset_addr : integer := 8; -- Detector Module Reset (Write)
	constant dm_config_addr : integer := 9; -- Detector Module Configure (Write)
	constant errors_addr : integer := 10; -- Error Flags (Read)
	constant irq_tmr1_addr : integer := 13; -- Timer One value (Read)
	constant relay_djr_addr : integer := 14; -- Relay Device Job Register (Read)
	constant relay_crhi_addr : integer := 15; -- Relay Command Register HI (Read)
	constant relay_crlo_addr : integer := 16; -- Relay Command Register LO (Read)
	constant relay_djr_rst_addr : integer := 17; -- Reset Device Job Register (Write)
	constant relay_der_addr : integer := 18; -- Relay Device Element Register (Read)
	constant comm_status_addr: integer := 22; -- Communication Status Register (Read)
	constant dpcr_addr : integer := 23; -- Display Panel Configuration Request (Write)
	constant dpod_addr : integer := 24; -- Display Panel Output Data (Write)
	constant dpid_addr : integer := 25; -- Display Panel Input Data (Read)
	constant msb_read_addr : integer := 26; -- Message Selector Buffer Read (Write)
	constant irq_tmr2_max_addr : integer := 27; -- Timer Two Period Minus One (Read/Write)
	constant irq_tmr2_addr : integer := 28; -- Timer Two value (Read)
	constant fv_addr : integer := 29; -- Firmware Version number (Read)
	constant indicators_addr : integer := 32; -- Indicator lamp array (Write)
	constant indicator_low : integer := 1; -- Low index of CPU-controlled indicators
	constant indicator_hi : integer := 15; -- High index of CPU-controlled indicators
	constant msb_id_addr : integer := 48; -- Selected Message ID (Read)
	constant msb_hi_addr : integer := 49; -- Selected Message HI Data Byte (Read)
	constant msb_lo_addr : integer := 50; -- Selected Message LO Data Byte (Read)
	constant msb_pwr_addr : integer := 51; -- Selected Message Power (Read)
	constant msb_an_addr : integer := 52; -- Selected Message Antenna Number (Read)
	constant tf_sr_addr : integer := 53; -- Transmitting Feedthrough Status Register (Read)
	constant tf_op_addr : integer := 54; -- Transmitting Feedthrough Opcode (Write)
	constant tf_n_addr : integer := 55; -- Transmitting Feedthrough Operand (Write)
	
	-- Relay Interface Registers.
	signal cont_djr : std_logic_vector(7 downto 0); -- Device Job Register
	signal cont_cr : std_logic_vector(15 downto 0); -- Command Register
	signal cont_der : std_logic_vector(7 downto 0); -- Device Element Register
	
	-- Relay Interface Memory Map Constants with Read and Write as seen by the
	-- LWDAQ Relay that is master of the interface. We respect the existing
	-- allocation of controller addresses given in the A2071 manual. 
	constant cont_id_addr : integer := 0; -- Hardware Identifier (Read)
	constant cont_sr_addr : integer := 1; -- Status Register (Read)
	constant cont_djr_addr : integer := 3; -- Device Job Register (Read/Write)
	constant cont_der_addr : integer := 15; -- Device Element Register (Write)
	constant cont_hv_addr : integer := 18; -- Hardware Version (Read)
	constant cont_fv_addr : integer := 19; -- Firmware Version (Read)
	constant cont_crhi_addr : integer := 32; -- Command Register HI (Write)
	constant cont_crlo_addr : integer := 33; -- Command Register LO (Write)
	constant cont_di_lo : integer := 38; -- Digital Inputs LO (Read)
	constant cont_cfsw_addr : integer := 40; -- Configuration Switch (Read)
	constant cont_srst_addr : integer := 41; -- Software Reset of Controller (Write)
	constant cont_fifo_av_addr : integer := 61; -- Fifo Blocks Available (Read)
	constant cont_fifo_ds_addr : integer := 62; -- Fifo Data Strobe (Read/Write)
	constant cont_fifo_rd_addr : integer := 63; -- Fifo Read Portal (Read)
			
-- Indicator signals.
	signal HIDE : boolean := false;
	signal SHOW : boolean := false;
	
-- Display Panel Interface
	signal DPXWR : std_logic; -- Display Panel Transmit Write
	signal DPXRD : std_logic; -- Display Panel Transmit Read
	signal DPXEMPTY : std_logic; -- Display Panel Transmit Buffer Empty
	signal DPXFULL : std_logic; -- Display Panel Transmit Buffer Full
	signal DPRWR : std_logic; -- Display Panel Receiver Write
	signal DPRRD : std_logic; -- Display Panel Receiver Read
	signal DPREMPTY : std_logic; -- Display Panel Receiver Buffer Empty
	signal DPRFULL : std_logic; -- Display Panel Receiver Buffer Full
	signal dp_in, dp_in_waiting, dp_out: std_logic_vector(7 downto 0);
	
-- Transmitting Feedthrough Interface;
	signal TFXWR : std_logic; -- Transmitting Feedthrough Transmit Write
	signal TFXRD : std_logic; -- Transmitting Feedthrough Transmit Read
	signal TFXEMPTY : std_logic; -- Transmitting Feedthrough Transmit Buffer Empty
	signal TFXFULL : std_logic; -- Transmitting Feedthrough Transmit Buffer Full
	signal TFRA : std_logic; -- Transmitting Feedthrough Receive Active
	signal tf_out_opcode, tf_in, tf_in_sr : std_logic_vector(7 downto 0);
	signal tf_out : std_logic_vector(15 downto 0);
	
-- Detector Module Buffer Interface
	signal DMBWR : std_logic; -- Detector Module Buffer Write
	signal DMBRD : std_logic; -- Detector Module Buffer Read
	signal DMBEMPTY : std_logic; -- Detector Module Buffer Empty
	signal DMBFULL : std_logic; -- Detector Module Buffer Full
	signal DMIBSY : std_logic; -- Detector Module Interface Busy
	signal dmb_in, dmb_out : std_logic_vector(39 downto 0);
	
-- Message Selector Buffer Interface
	signal MSBWR : std_logic; -- Message Selector Buffer Write
	signal MSBRD : std_logic; -- Message Selector Buffer Read
	signal MSBEMPTY : std_logic; -- Message Selector Buffer Empty
	signal MSBFULL : std_logic; -- Message Selector Buffer Full
	signal MSBSY : std_logic; -- Message Selector Busy
	signal msb_in, msb_out : std_logic_vector(39 downto 0);

-- General-Purpose Constant
	constant max_data_byte : std_logic_vector(7 downto 0) := "11111111";
	constant high_z_byte : std_logic_vector(7 downto 0) := "ZZZZZZZZ";
	constant zero_data_byte : std_logic_vector(7 downto 0) := "00000000";
	constant one_data_byte : std_logic_vector(7 downto 0) := "00000001";

-- Functions and Procedures	
	function to_std_logic (v: boolean) return std_ulogic is
	begin if v then return('1'); else return('0'); end if; end function;
	
begin
	-- We generate FCK of 80 MHz from MCK of 10 MHz using a PLL
	Clock : entity X8PLL port map (
		CLKI => MCK,
		CLKOP => FCK,
		LOCK => LOCK
	);

	-- Our fast clock (FCK) is 80 MHz. We divide FCK down to 40 MHz for our state
	-- machines (CK), and 20 MHz for the CPU (PCK). We divide down to 8 MHz for the 
	-- demodulator clock (DMCK), which the demodulators multiply to 40 MHz with 
	-- their phase locked loops to run their detectors. We divide FCK down to 2 MHz
	-- for our serial receiver and delays (SCK). Note that the clocks do not stop 
	-- when RESET is asserted. We use the clocks to control our reset arbitration.
	Divider : process (FCK) is
	variable p_count : integer range 0 to 3;
	variable d_count : integer range 0 to 15;
	variable s_count : integer range 0 to 63;
	begin
		if rising_edge(FCK) then
			CK <= to_std_logic(CK = '0');
			if (p_count <= 1) then
				PCK <= '0';
			else
				PCK <= '1';
			end if;
			if (p_count = 3) then
				p_count := 0;
			else
				p_count := p_count + 1;
			end if;
		end if;
		
		if rising_edge(FCK) then
			if (d_count <= 4) then
				DMCK <= '0';
			else
				DMCK <= '1';
			end if;
			if (d_count = 9) then
				d_count := 0;
			else
				d_count := d_count + 1;
			end if;
		end if;
		
		if rising_edge(FCK) then
			if (s_count <= 20) then
				SCK <= '0';
			else
				SCK <= '1';
			end if;
			if (s_count = 39) then
				s_count := 0;
			else
				s_count := s_count + 1;
			end if;
		end if;
	end process;
	
	-- The Input Processor provides synchronized versions of incoming 
	-- signals and positive-polarity versions too.
	Input_Processor : process (CK,SCK) is
	constant max_state : integer := 15;
	variable rcv_state, next_rcv_state : integer range 0 to max_state;
	begin
		if rising_edge(CK) then
			RCK <= RCK_in;
			CDS <= (CDS_in = '0');
			MRDY <= MRDY_in;
			DMERR <= DMERR_in;
			DMRST <= (DMRST_pin = '1');
		end if;
		
		if rising_edge(SCK) then
			BBRST <= (RESET_not = '0');
			BBRSTD <= BBRST;		
		end if;
		
		CWR <= (CWR_in = '0');
	end process;
	
	-- The Reset Arbitrator manages the three levels of reset. The Top-Level 
	-- Reset is when we press either the base board reset switch or the display 
	-- panel reset switch. Pressing either switch causes a 100-ms LO on the base
	-- board !RESET signal. The Relay, Controller, Detector Modules, Transmitting
	-- Feedthrough and Display Panel all reset in response to !RESET. A Mid-Level 
	-- Reset is one in which the Relay remains active, but everything else resets. 
	-- The !RESET signal on the base board is not driven LO. The RESET lamp on 
	-- the display panel will flash. A Low-Level Reset is one in which only the 
	-- Detector Modules, and Display Panel reset. The Low-Level reset we handle 
	-- directly from the Controller's CPU using DMRST_BY_CPU, which we use to drive 
	-- DMRST_pin HI. In a low-level reset, the lamp on the display panel will 
	-- flash briefly.
	Reset_Arbitrator : process (CK,SCK) is
	constant dp_init_len : integer := 63;
	variable dp_count : integer range 0 to dp_init_len;
	constant mid_rst_len : integer := 65535;
	variable mid_state, mid_next_state : integer range 0 to mid_rst_len;
	constant bb_rst_len : integer := 262143;
	variable bb_count : integer range 0 to bb_rst_len;
	variable mid_init_rst, dp_init_rst : boolean;
	begin
		
		-- If the Display Panel drives DMRST HI, we drive RESET_not LO, which
		-- in turn asserts BBRST. All controller state machines reset and the
		-- CPU reboots. We decide that the Display Panel is driving DMRST HI 
		-- when we have had DMRST_BY_CPU unasserted for some time, and yet 
		-- DMRST remains asserted. We use SCK, 2 MHz, to generate a sufficient
		-- waiting period to be sure DMRST comes from the display panel. We 
		-- then generate a 500-ns dp_init_rst flag, which will provoke the
		-- BBRST state machine.
		if rising_edge(SCK) then
			if (DMRST_pin = '1') and (not DMRST_BY_CPU) then
				if dp_count < dp_init_len then
					dp_count := dp_count + 1;
					dp_init_rst := false;
				else
					dp_count := dp_init_len;
					dp_init_rst := true;
				end if;
			else
				dp_count := 0;
				dp_init_rst := false;
			end if;
		end if;
		
		-- If we have dp_init_rst pulse from the Display Panel reset switch,
		-- or BBRST is asserted by some other process, we will drive !RESET 
		-- LO for 131 ms.
		if rising_edge(SCK) then
			if (BBRST and (not BBRSTD)) or dp_init_rst then 
				bb_count := 0;
				RESET_not <= '0';
			else
				if bb_count < bb_rst_len then
					bb_count := bb_count + 1;
					RESET_not <= '0';
				else 
					bb_count := bb_rst_len;
					RESET_not <= 'Z';
				end if;
			end if;
		end if;
		
		-- When DMRST_BY_CPU, we drive DMRST_pin HI immediately.
		if rising_edge(CK) then
			if DMRST_BY_CPU then
				DMRST_pin <= '1';
			else
				DMRST_pin <= 'Z';
			end if;
		end if;

		-- We generate a mid-level reset if we have the CPU or Relay trying
		-- to reset the Controller. We also generate a mid-level reset whenever
		-- we have a high-level reset caused by BBRST. The mid-level reset is
		-- provoked by the local RESET signal. This in turn asserts DMRST to
		-- reset the detector modules. We run this state machine of CK because 
		-- we want to detect the PCK pulse on RST_BY_CPU generated by a CPU 
		-- write to its own reset register. The mid-level pulse we generate on
		-- RESET is long enough to cause the Transmitting Feedthrough to reset
		-- by the assertion of a HI on TX.
		if rising_edge(CK) then
			mid_init_rst := RST_BY_CPU or RST_BY_RELAY;
			if mid_state = 0 then
				if mid_init_rst then 
					mid_next_state := 1;
				else 
					mid_next_state := 0;
				end if;
			elsif mid_state = mid_rst_len then
				if mid_init_rst then
					mid_next_state := mid_rst_len;
				else
					mid_next_state := 0;
				end if;
			else
				mid_next_state := mid_state + 1;
			end if;
			RESET <= to_std_logic((mid_state > 1) or BBRST);
			mid_state := mid_next_state;
		end if;
	end process;
	
	-- The Message Buffer is a FIFO made out of the external SRAM, two pointers,
	-- and the Message Buffer Controller. Writes to the buffer occur when the 
	-- Message Write Strobe (MWRS) is asserted, and are complete when Message
	-- Write Acknowledge (MWRACK) is asserted. The data written is the byte 
	-- held in the global mwr_data register. The MWRACK flag remains asserted
	-- until MWRS is unasserted. The process requesting that mwr_data be 
	-- written to the buffer should see MWRACK and unassert MWRS, thus indicating
	-- that the write is complete. We follow the same handshake process for
	-- read cycles. The process requesting that mrd_data be loaded with the 
	-- byte at the front of the FIFO asserts MRDS and waits for MRDACK. When
	-- it sees MRDACK, it copies mrd_data and unasserts MRDS to complete the
	-- read cycle. The global integer fifo_byte_count keeps count of how many
	-- bytes are available for readout in the FIFO. We increment the counter
	-- on each write. We do not allow a read from the buffer unless the counter
	-- is greater than zero, so the process requesting mrd_data must wait for
	-- MRDACK. Given that clock messages will be stored by the CPU at 128 Hz,
	-- this wait can be as long as 8 ms. While the reading process is waiting,
	-- the writing process is free to write the next byte, thanks to the 
	-- buffer ignoring the read request until a byte is available. When a
	-- read and a write request arrive at the same time, the controller gives
	-- priority to the write cycle.
	Message_Buffer_Controller : process (CK,RESET)
	variable write_ptr, next_write_ptr : std_logic_vector(20 downto 0);
	variable read_ptr, next_read_ptr : std_logic_vector(20 downto 0);
	variable state, next_state : integer range 0 to 7;
	variable buff_enable, buff_write : boolean;
	begin
		if (RESET = '1') then
			write_ptr := std_logic_vector(to_unsigned(0,21));
			read_ptr := std_logic_vector(to_unsigned(0,21));
			fifo_byte_count <= std_logic_vector(to_unsigned(0,21));
			buff_data <= high_z_byte;
			mrd_data <= max_data_byte;
			buff_enable := false;
			buff_write := false;
			MWRACK <= false;
			MRDACK <= false;
		elsif rising_edge(CK) then
			next_write_ptr := write_ptr;
			next_read_ptr := read_ptr;
			next_state := state;
			case state is
				when 0 =>
					if not MWRS then MWRACK <= false; end if;
					if not MRDS then MRDACK <= false; end if;
					if MWRS and (not MWRACK) then 
						buff_addr <= write_ptr;
						buff_data <= mwr_data;
						buff_enable := false;
						buff_write := true;
						next_state := 1; 
					elsif MRDS and (not MRDACK) then
						if to_integer(unsigned(fifo_byte_count)) /= 0 then
							buff_addr <= read_ptr;
							buff_data <= high_z_byte;
							buff_enable := true;
							buff_write := false;
							next_state := 4;
						else
							buff_addr <= read_ptr;
							buff_data <= high_z_byte;
							buff_enable := false;
							buff_write := false;
							next_state := 0;
						end if;
					else
						buff_addr <= read_ptr;
						buff_data <= high_z_byte;
						buff_enable := false;
						buff_write := false;
						next_state := 0;					
					end if;
				when 1 =>
					buff_addr <= write_ptr;
					buff_data <= mwr_data;
					buff_enable := true;
					buff_write := true;
					next_state := 2;
				when 2 =>
					buff_addr <= write_ptr;
					buff_data <= mwr_data;
					buff_enable := false;
					buff_write := true;
					next_state := 3;
				when 3 =>
					buff_addr <= write_ptr;
					buff_data <= mwr_data;
					buff_enable := false;
					buff_write := false;
					MWRACK <= true;
					next_write_ptr := std_logic_vector(unsigned(write_ptr) + 1);
					fifo_byte_count <= std_logic_vector(unsigned(fifo_byte_count) + 1);
					next_state := 0;
				when 4 =>
					buff_addr <= read_ptr;
					buff_data <= high_z_byte;
					mrd_data <= buff_data;
					buff_enable := true;
					buff_write := false;
					next_state := 5;
				when 5 =>
					buff_addr <= read_ptr;
					buff_data <= high_z_byte;
					buff_enable := false;
					buff_write := false;
					MRDACK <= true;
					next_read_ptr := std_logic_vector(unsigned(read_ptr) + 1);
					fifo_byte_count <= std_logic_vector(unsigned(fifo_byte_count) - 1);
					next_state := 0;
				when others =>
					buff_addr <= read_ptr;
					buff_data <= high_z_byte;
					buff_enable := false;
					buff_write := false;
					next_state := 0;
			end case;
			state := next_state;
			read_ptr := next_read_ptr;
			write_ptr := next_write_ptr;
		end if;
		
		REN_not <= to_std_logic(not buff_enable);
		RWE_not <= to_std_logic(not buff_write);
	end process;
	
-- User memory and configuration code for the CPU. This RAM will be initialized at
-- start-up with a configuration file, and so may be read after power up to configure
-- sensor. The configuration data will begin at address zero.
	RAM : entity CPU_RAM port map (
		Clock => not PCK,
		ClockEn => '1',
        Reset => RESET,
		WE => CPURWR,
		Address => cpu_ram_addr, 
		Data => cpu_ram_in,
		Q => cpu_ram_out);

-- Instruction Memory for CPU. This read-only memory will be initialized with the
-- CPU program, the first instruction of the program being stored at address zero.
-- The CPU reads the instruction memory with a separate address bus, which we call
-- the program counter.
	ROM : entity CPU_ROM port map (
		Address => prog_addr,
        OutClock => not PCK,
        OutClockEn => '1',
        Reset => RESET,	
        Q => prog_data);

-- The processor itself, and eight-bit microprocessor with thirteen-bit address bus.
	CPU : entity OSR8_CPU 
		generic map (
			prog_addr_len => prog_addr_len,
			cpu_addr_len => cpu_addr_len,
			start_pc => start_pc,
			interrupt_pc => interrupt_pc
		)
		port map (
			prog_data => prog_data,
			prog_addr => prog_addr,
			cpu_data_out => cpu_data_out,
			cpu_data_in => cpu_data_in,
			cpu_addr => cpu_addr,
			WR => CPUWR,
			DS => CPUDS,
			IRQ => CPUIRQ,
			SIG => CPUSIG,
			RESET => RESET,
			CK => PCK
		);
		
-- The Display Panel Output Buffer (DPO Buffer) buffer is where the 
-- CPU writes bytes it wants the Display Panel Transmitter to serialize 
-- for the Display Panel.
	DPO_Buffer : entity FIFO8
		port map (
			Data => cpu_data_out,
			WrClock => not PCK,
			RDClock => not SCK,
			WrEn => DPXWR,
			RdEn => DPXRD,
			Reset => RESET,
			RPReset => RESET,
			Q => dp_out,
			Empty => DPXEMPTY,
			Full => DPXFULL
		);

-- The Display Panel Input Buffer (DPI Buffer) is where the Display 
-- Panel Receiver writes bytes for the CPU to read out. The CPU checks 
-- the DPREMPTY flag to see if a byte is waiting, then reads it out.
	DPI_Buffer : entity FIFO8
		port map (
			Data => dp_in,
			WrClock => not SCK,
			RDClock => not PCK,
			WrEn => DPRWR,
			RdEn => DPRRD,
			Reset => RESET,
			RPReset => RESET,
			Q => dp_in_waiting,
			Empty => DPREMPTY,
			Full => DPRFULL
		);
		
-- The Transmitting Feedthrough Output Buffer (TFO Buffer) buffer is 
-- where the CPU writes sixteen-bit words it wants the Transmitting-- Feedthrough Transmitter to serialize. The top eight bits are an
-- operation code, the bottom eight bits are an operand.
	TFO_Buffer : entity FIFO16
		port map (
			Data(15 downto 8) => cpu_data_out,
			Data(7 downto 0) => tf_out_opcode,
			WrClock => not PCK,
			RDClock => not SCK,
			WrEn => TFXWR,
			RdEn => TFXRD,
			Reset => RESET,
			RPReset => RESET,
			Q => tf_out,
			Empty => TFXEMPTY,
			Full => TFXFULL
		);

-- The Detector Module Buffer is where the Detector Module Reader puts
-- messages for the Message Selector to read out and sort through to
-- reject duplicates that are less powerful than the top antenna message.
-- It is forty bits wide. When it's Empty flag is not set, a message is 
-- ready to read.
	DM_Buffer : entity FIFO40
		port map (
			Data => dmb_in,
			WrClock => not SCK,
			RdClock => not SCK,
			WrEn => DMBWR,
			RdEn => DMBRD,
			Reset=> RESET,
			RPReset => RESET,
			Q => dmb_out,
			Empty => DMBEMPTY,
			Full => DMBFULL
		);
		
-- The Message Selector Buffer is where the Message Selector puts its
-- selected messages for the CPU to read out and subsequently write into 
-- the Message Buffer. It is forty bits wide. When it's Empty flag is 
-- not set, a message is ready to read.
	MS_Buffer : entity FIFO40
		port map (
			Data => msb_in,
			WrClock => not SCK,
			RdClock => not PCK,
			WrEn => MSBWR,
			RdEn => MSBRD,
			Reset=> RESET,
			RPReset => RESET,
			Q => msb_out,
			Empty => MSBEMPTY,
			Full => MSBFULL
		);

-- The Memory Manager maps eight-bit read and write access to Detector Module 
-- daisy chain bus, and the registers of the relay interface, as well as the
-- Random Access Memory, and Interrupt Handler. Byte ordering is big-endian. 
	MMU : process (PCK,RESET) is
		variable top_bits : integer range 0 to 7;
		variable bottom_bits : integer range 0 to 127;
	begin
	
		-- Some variables for brevity.
		top_bits := to_integer(unsigned(cpu_addr(cpu_addr_len-1 downto 9)));
		bottom_bits := to_integer(unsigned(cpu_addr(7 downto 0)));
		
		-- The RAM data in, its address, and its write strobe are all 
		-- combinatorial functions of the CPU outputs. They will be ready 
		-- well before the falling edge of PCK.
		cpu_ram_in <= cpu_data_out;
		cpu_ram_addr <= cpu_addr(ram_addr_len-1 downto 0);
		CPURWR <= to_std_logic(
			(top_bits >= cpu_ram_base) 
			and (top_bits < cpu_ram_base+cpu_ram_range)
			and CPUWR);
		
		-- The CPU data in we select with the following combinatorial 
		-- multiplexer.
		case top_bits is
		when cpu_ram_base to (cpu_ram_base+cpu_ram_range-1) => 
			cpu_data_in <= cpu_ram_out;
		when cpu_ctrl_base to (cpu_ctrl_base+cpu_ctrl_range-1) =>
			case bottom_bits is
				when irq_bits_addr => cpu_data_in <= irq_bits;
				when irq_mask_addr => cpu_data_in <= irq_mask;
				when irq_tmr1_max_addr => cpu_data_in <= irq_tmr1_max;
				when irq_tmr1_addr => cpu_data_in <= irq_tmr1;
				when irq_tmr2_max_addr => cpu_data_in <= irq_tmr2_max;
				when irq_tmr2_addr => cpu_data_in <= irq_tmr2;
				when fv_addr => cpu_data_in <= 
					std_logic_vector(to_unsigned(firmware_version,8));
				when test_point_addr => cpu_data_in <= tp_reg;
				when relay_djr_addr => cpu_data_in <= cont_djr;
				when relay_crhi_addr => cpu_data_in <= cont_cr(15 downto 8);
				when relay_crlo_addr => cpu_data_in <= cont_cr(7 downto 0);
				when relay_der_addr => cpu_data_in <= cont_der;
				when errors_addr =>
					cpu_data_in <= (others => '0');
					cpu_data_in(0) <= DMERR;
				when comm_status_addr => 
					cpu_data_in(0) <= to_std_logic(EMPTY);
					cpu_data_in(1) <= to_std_logic(UPLOAD);
					cpu_data_in(2) <= ETH;
					cpu_data_in(3) <= to_std_logic(CONFIG);
					cpu_data_in(4) <= not DPREMPTY;
					cpu_data_in(5) <= not MSBEMPTY;
					cpu_data_in(6) <= DMIBSY;
					cpu_data_in(7) <= MRDY;
				when dpid_addr => 
					cpu_data_in <= dp_in_waiting;
				when tf_sr_addr => cpu_data_in <= tf_in;
				when msb_id_addr => cpu_data_in <= msb_out(39 downto 32);
				when msb_hi_addr => cpu_data_in <= msb_out(31 downto 24);
				when msb_lo_addr => cpu_data_in <= msb_out(23 downto 16);
				when msb_pwr_addr => cpu_data_in <= msb_out(15 downto 8);
				when msb_an_addr => cpu_data_in <= msb_out(7 downto 0);
				when others => cpu_data_in <= max_data_byte;
			end case;
		when others =>
			cpu_data_in <= max_data_byte;
		end case;
		
		-- We use the falling edge of PCK to write to registers and to initiate sensor 
		-- and transmit activity. We have RST_BY_CPU set upon a write to cpu_rst_addr,
		-- which provokes a RESET pulse starting on the next rising edge of CK, which in
		-- turn causes an asynchronous reset of RST_BY_CPU. But the RESET pulse endures,
		-- thanks to the state machine in the Reset Arbitrator. The Message Write Strobe
		-- (MWRS) is set on a write to msg_write_addr, and mwr_data is written for the
		-- use of the Message Buffer Controller. We continue to assert MWRS until we 
		-- see MRACK is asserted, at which point we clear MWRS on the falling edge of PCK.
		-- If it so happens that MRACK becomes asserted just when we are once again writing
		-- to msg_write_addr, the clearing of MWRS will fail, and the new byte will not be
		-- written to the message buffer because the Message Buffer Controller needs MWRS
		-- to be unasserted before it writes another byte.
		if (RESET = '1') then 
			irq_tmr1_max <= max_data_byte;
			irq_tmr2_max <= max_data_byte;
			irq_mask <= zero_data_byte;
			irq_rst <= zero_data_byte;
			irq_set <= zero_data_byte;
			MWRS <= false;
			DJRRST <= false;
			RST_BY_CPU <= false;
			DMRST_BY_CPU <= true;
			DPXWR <= '0';
			DPRRD <= '0';
			TFXWR <= '0';
			MSBRD <= '0';
			DPCONFIG <= false;
			DMCFG <= '0';
			for i in 1 to 15 loop indicator_control(i) <= '0'; end loop;
		elsif falling_edge(PCK) then
			irq_rst <= zero_data_byte;
			irq_set <= zero_data_byte;
			DJRRST <= false;
			DPXWR <= '0';
			DPRRD <= '0';
			TFXWR <= '0';
			MSBRD <= '0';
			if MWRACK then MWRS <= false; end if;
			if CPUDS then 
				if (top_bits >= cpu_ctrl_base) 
						and (top_bits <= cpu_ctrl_base+cpu_ctrl_range-1) then
					if CPUWR then
						case bottom_bits is
							when irq_mask_addr => irq_mask <= cpu_data_out;
							when irq_reset_addr => irq_rst <= cpu_data_out;
							when irq_set_addr => irq_set <= cpu_data_out;
							when irq_tmr1_max_addr => irq_tmr1_max <= cpu_data_out;
							when irq_tmr2_max_addr => irq_tmr2_max <= cpu_data_out;
							when cpu_rst_addr => RST_BY_CPU <= true;
							when dpcr_addr => DPCONFIG <= (cpu_data_out(0) = '1');
							when test_point_addr => tp_reg <= cpu_data_out;
							when msg_write_addr => 
								mwr_data <= cpu_data_out;
								MWRS <= true;
							when dm_reset_addr => DMRST_BY_CPU <= (cpu_data_out(0) = '1');
							when dm_config_addr => DMCFG <= cpu_data_out(0);
							when relay_djr_rst_addr => DJRRST <= true;
							when indicators_addr + 1 to indicators_addr + 15 =>
								indicator_control(to_integer(unsigned(cpu_addr(3 downto 0)))) 
									<= cpu_data_out(0);
							when dpod_addr => if (DPXFULL = '0') then DPXWR <= '1'; end if;
							when tf_op_addr => tf_out_opcode <= cpu_data_out;
							when tf_n_addr => if (TFXFULL = '0') then TFXWR <= '1'; end if;
							when msb_read_addr => MSBRD <= '1';
						end case;
					end if;
					if not CPUWR then
						-- This implementation of DPRRD results in the new byte being
						-- available next time we read the buffer, which is okay if the
						-- display panel keeps sending bytes, but not okay if it sends
						-- only one. 
						if (bottom_bits = dpid_addr) then 
							DPRRD <= '1'; 
						end if;
					end if;
				end if;
			end if;
		end if;
	end process;
	
	-- The Interrupt_Controller provides the interrupt signal to the CPU in response to
	-- sensor and timer events. By default, at power-up, all interrupts are maske.
	Interrupt_Controller : process (RCK,PCK,RESET) is
	begin
		-- Interrupt Timer One, counting up from zero to irq_tmr1_max with period RCK. It 
		-- never stops, so we can generate regular, periodic interrupts. 
		if (RESET = '1') then
			irq_tmr1 <= zero_data_byte;
		elsif rising_edge(RCK) then
			if (irq_tmr1 = irq_tmr1_max) then
				irq_tmr1 <= zero_data_byte;
			else
				irq_tmr1 <= std_logic_vector(unsigned(irq_tmr1)+1);
			end if;
		end if;

		-- Interrupt Timer Two, counting up from zero to irq_tmrs_max with period RCK. It 
		-- never stops, so we can generate regular, periodic interrupts. 
		if (RESET = '1') then
			irq_tmr2 <= zero_data_byte;
		elsif rising_edge(RCK) then
			if (irq_tmr2 = irq_tmr2_max) then
				irq_tmr2 <= zero_data_byte;
			else
				irq_tmr2 <= std_logic_vector(unsigned(irq_tmr2)+1);
			end if;
		end if;

		-- The interrupt management runs of PCK.
		if (RESET = '1') then
			CPUIRQ <= false;
			irq_bits <= zero_data_byte;
			INTCTRZ1 <= false;
			INTCTRZ2 <= false;
		elsif rising_edge(PCK) then
		
			-- The Timer One Interrupt is set when the counter is zero
			-- and reset when we write of 1 to irq_rst(0). We detect
			-- the first PCK clock period in which the counter becomes
			-- zero with the help of the INTCTRZ1 flag.
			INTCTRZ1 <= (irq_tmr1 = zero_data_byte);
			if (irq_rst(0) = '1') then
				irq_bits(0) <= '0';
			elsif ((irq_tmr1 = zero_data_byte) and (not INTCTRZ1)) 
					or (irq_set(0) = '1') then
				irq_bits(0) <= '1';
			end if;

			-- The detector module receive interrupt is set when we see
			-- that there are messages in the Detector Module Buffer.
			if (irq_rst(1) = '1') then
				irq_bits(1) <= '0';
			elsif (DMBEMPTY = '0')
					or (irq_set(1) = '1') then
				irq_bits(1) <= '1';
			end if;
			
			-- The detector module error interrupt is set when we see
			-- a rising edge on the DMERR input. 
			if (irq_rst(2) = '1') then
				irq_bits(2) <= '0';
			elsif (DMERR = '1')
					or (irq_set(2) = '1') then
				irq_bits(2) <= '1';
			end if;

			-- The Timer Two Interrupt is set when the counter is zero
			-- and reset when we write of 1 to irq_rst(3). We detect
			-- the first PCK clock period in which the counter becomes
			-- zero with the help of the INTCTRZ1 flag.
			INTCTRZ2 <= (irq_tmr2 = zero_data_byte);
			if (irq_rst(3) = '1') then
				irq_bits(3) <= '0';
			elsif ((irq_tmr2 = zero_data_byte) and (not INTCTRZ2)) 
					or (irq_set(3) = '1') then
				irq_bits(3) <= '1';
			end if;
												
			-- The CPU software interrupts that the CPU sets and resets
			-- itself through the irq_rst and irq_set control registers.
			for i in 4 to 7 loop
				if (irq_rst(i) = '1') then
					irq_bits(i) <= '0';
				elsif (irq_set(i) = '1') then
					irq_bits(i) <= '1';
				end if;
			end loop;
		end if;

		-- We generate an interrupt if any one interrupt bit is 
		-- set and unmasked.
		CPUIRQ <= (irq_bits and irq_mask) /= zero_data_byte;
	end process;
	
	-- The LWDAQ Relay Interface provides the relay with read and write access to
	-- the sixty-four-byte controller address space. We can write to eight-bit 
	-- registers with LWDAQ TCPIP messages and read them back. In some cases, the 
	-- mere act of reading or writing from a location causes a flag to be set or 
	-- cleared. Most registers appear in the controller CPU's address space as well,
	-- and are the means by which the relay and controller communicate. In the case
	-- of reads from the message buffer, the relay must make sure the byte is 
	-- available before it reads, and does so by writing any value to the FIFO Data 
	-- Strobe location and then polling the same location until it is non-zero. The
	-- initial write sets the Message Read Strobe (MRDS), which instructs the 
	-- Message Buffer Controller to read a byte from the message FIFO and make it
	-- available to the relay at the FIFO read address. When the byte is ready,
	-- the Message Buffer Controller sets Message Read Acknowledge (MRACK) for
	-- readback at the strobe location. When the Relay reads the byte from the read
	-- location, the Relay Interface clears MRDS, and the Message Buffer Controller
	-- clears MRDACK a clock period later. The RST_BY_RELAY signal initiates a
	-- RESET pulse, which is produced by the Reset Arbitrator. This pulse endures
	-- for many clock cycles, regardless of when RST_BY_RELAY is unasserted, so
	-- we are free to clear RST_BY_RELAY with an asynchronous reset using the
	-- RESET pulse. We note that the RESET pulse causes a pulse on Detector 
	-- Module Reset (DMRST) and Device Job Register Reset (DJRRST), and so resets
	-- not only the controller, but the demodulators and the job register. But
	-- the RESET does not affect the relay itself, which we must reset with 
	-- a separate LWDAQ reboot instruction.
	Relay_Interface : process (CK,RESET) is
	variable integer_addr : integer range 0 to 63;
	begin
		integer_addr := to_integer(unsigned(cont_addr));
		BBCONFIG <= (switches(4) = '1');
		CONFIG <=  BBCONFIG or DPCONFIG; 
		
		if RESET = '1' then
			cont_data <= high_z_byte;
			cont_djr <= zero_data_byte;
			MRDS <= false;
			RST_BY_RELAY <= false;
		elsif rising_edge(CK) then		
			if CWR then cont_data <= high_z_byte; end if;			
			
			if CDS and CWR then
				case integer_addr is
				when cont_djr_addr => cont_djr <= cont_data;
				when cont_crhi_addr => cont_cr(15 downto 8) <= cont_data;
				when cont_crlo_addr => cont_cr(7 downto 0) <= cont_data;
				when cont_der_addr => cont_der <= cont_data;
				when cont_srst_addr => RST_BY_RELAY <= true;
				when cont_fifo_ds_addr => MRDS <= true;
				end case;
			elsif CDS and (not CWR) then
				case integer_addr is
				when cont_id_addr => 
					cont_data <= std_logic_vector(to_unsigned(hardware_id,8));
				when cont_sr_addr => 
					cont_data(0) <= to_std_logic(cont_djr /= "00000000");
					cont_data(1) <= DMERR;
					cont_data(2) <= to_std_logic(MRDS);
					cont_data(3) <= to_std_logic(MWRS);
					cont_data(4) <= to_std_logic(CPUIRQ);
					cont_data(5) <= tp_reg(5);
					cont_data(6) <= tp_reg(6);
					cont_data(7) <= tp_reg(7);
				when cont_djr_addr => 
					cont_data <= cont_djr;
				when cont_hv_addr =>
					cont_data <= std_logic_vector(to_unsigned(hardware_version,8));
				when cont_fv_addr =>
					cont_data <= std_logic_vector(to_unsigned(firmware_version,8));
				when cont_di_lo => 
					cont_data <= tf_in;
				when cont_cfsw_addr => -- The Relay looks for a zero to configure.
					cont_data(0) <= to_std_logic(not CONFIG);
				when cont_fifo_av_addr =>
					if to_integer(unsigned(fifo_byte_count(20 downto 17))) = 0 then
						cont_data <= fifo_byte_count(16 downto 9);
					else
						cont_data <= max_data_byte;
					end if;
				when cont_fifo_ds_addr =>
					cont_data <= (others => '0');
					cont_data(0) <= to_std_logic(MRDACK);
				when cont_fifo_rd_addr =>
					cont_data <= mrd_data;
					MRDS <= false;
				when others =>
					cont_data <= max_data_byte;
				end case;
			end if;
			
			if DJRRST then cont_djr <= zero_data_byte; end if;
			
		end if;
	end process;
	
	-- The Detector Module Interface watches for MRDY, which indicates
	-- that one or more detector modules has a message ready for readout
	-- When it sees MRDY, it asserts Detector Module Read Control (DMRC).
	-- It reads the first byte of the five-byte message from the daisy
	-- chain bus. If the lower four bits of this byte are zero, the 
	-- interface abandons the read. The detector module must discard the
	-- message. Once the interface has acquired five bytes, it stores them 
	-- as a forty-bit record in the Detector Module Buffer (DMB). If one 
	-- of the detector modules fails, breaking the daisy-chain, the modules 
	-- upstream of the failure will assert MRDY continuously. Most likely, 
	-- we will be reading messages with ID byte zero, and these we will 
	-- discard. If, however, the corrupted message read from the faulty 
	-- module presents a valid ID, we will keep reading the same corrupted 
	-- message and our buffer will fill up. We stop writing to the buffer 
	-- when it is full and keep the detector modules waiting until the buffer 
	-- is no longer full. The interface sets a flag DMIBSY when it is not in 
	-- its rest state. This flag is available to the CPU in the communications 
	-- DMBFULL. Whe the interface sees Detector Module Configure (DMCFG) 
	-- asserted, it starts a configuration access, asserting DMRC until DMCFG 
	-- is unasserted. During this cycle, the detector modules will be 
	-- calculating their position in the daisy chain.
	Detector_Module_Interface : process (SCK,RESET) is
	variable state, next_state : integer range 0 to 7;
	constant dmi_idle : integer := 0;
	constant dmi_id : integer := 1;
	constant dmi_hi : integer := 2;
	constant dmi_lo : integer := 3;
	constant dmi_pwr : integer := 4;
	constant dmi_an : integer := 5;
	constant dmi_config : integer := 6;	
	begin
		if (RESET = '1') then
			DMBWR <= '0';
			dmb_in <= (others => '0');
			state := dmi_idle;
			DMRC <= '0';
			DSU <= '0';
			DMIBSY <= '0';
		elsif rising_edge(SCK) then
			dmb_in <= dmb_in;
			DMBWR <= '0';
			DMIBSY <= '1';

			next_state := state;
			case state is
				when dmi_idle => 
					if (DMCFG = '1') then
						DMIBSY <= '0';
						DMRC <= '0'; 
						DSU <= '0';
						next_state := dmi_config;
					elsif (MRDY = '0') or (DMBFULL = '1') then 
						DMIBSY <= '0';
						DMRC <= '0'; 
						DSU <= '0';
						next_state := dmi_idle;
					else 
						DMRC <= '1'; 
						DSU <= '1';
						next_state := dmi_id;
					end if;
				when dmi_id => 
					DMRC <= '1'; 
					DSU <= '0'; 
					dmb_in(39 downto 32) <= dub;
					next_state := 2;
				when dmi_hi =>
					if (dmb_in(35 downto 32) = "0000") then
						DMRC <= '0';
						DSU <= '0';
						next_state := dmi_idle;
					else
						DMRC <= '1'; 
						DSU <= '1'; 
						dmb_in(31 downto 24) <= dub;
						next_state := dmi_lo;
					end if;
				when dmi_lo =>
					DMRC <= '1'; 
					DSU <= '0'; 
					dmb_in(23 downto 16) <= dub;
					next_state := dmi_pwr;
				when dmi_pwr => 
					DMRC <= '1'; 
					DSU <= '1'; 
					dmb_in(15 downto 8) <= dub;
					next_state := dmi_an;
				when dmi_an => 
					DMRC <= '0'; 
					DSU <= '0'; 
					dmb_in(7 downto 0) <= dub;
					DMBWR <= '1';
					next_state := dmi_idle;
				when dmi_config =>
					DMRC <= '1'; 
					DSU <= '0';
					if (DMCFG = '1') then
						next_state := dmi_config;
					else
						next_state := dmi_idle;
					end if;
				when others =>
					DMRC <= '0'; 
					DSU <= '0';
					next_state := 0;
			end case;
			state := next_state;
		end if;
	end process;
	
	-- The Message Selector reads messages out of the Detector Module
	-- Buffer, eliminates consecutive duplicates, and stores the top
	-- antenna messages in the Message Selector Buffer. 
	Message_Selector : process (SCK,RESET) is
	variable state, next_state : integer range 0 to 7;
	variable msg_prv : std_logic_vector(39 downto 0);
	constant mss_idle : integer := 0;
	constant mss_check : integer := 1;
	constant mss_wrclr : integer := 2;
	constant mss_wrrpl : integer := 3;
	begin
		if (RESET = '1') then
			MSBWR <= '0';
			DMBRD <= '0';
			msb_in <= (others => '0');
			msg_prv := (others => '0');
			state := mss_idle;
		elsif rising_edge(SCK) then
			msb_in <= msb_in;
			msg_prv := msg_prv;
			MSBWR <= '0';
			DMBRD <= '0';
			
			next_state := state;
			case state is
				when mss_idle =>
					if (DMBEMPTY = '0') then
						DMBRD <= '1';
						next_state := mss_check;
					elsif (msg_prv(35 downto 32) /= "0000") 
						and (MRDY = '0') 
						and (DMIBSY = '0') then
						msb_in <= msg_prv;
						next_state := mss_wrclr;
					else
						next_state := mss_idle;
					end if;
				when mss_check =>
					if (msg_prv(35 downto 32) = "0000") then
						msg_prv := dmb_out;
						next_state := mss_idle;
					elsif (dmb_out(39 downto 16) /= msg_prv(39 downto 16)) then
						msb_in <= msg_prv;
						next_state := mss_wrrpl;
					elsif (to_integer(unsigned(dmb_out(15 downto 8))) <= 
						to_integer(unsigned(msg_prv(15 downto 8)))) then
						next_state := mss_idle;
					else
						msg_prv := dmb_out;
						next_state := mss_idle;
					end if;
				when mss_wrclr =>
					MSBWR <= '1';
					msg_prv := (others => '0');
					next_state := mss_idle;
				when mss_wrrpl => 
					MSBWR <= '1';
					msg_prv := dmb_out;
					next_state := mss_idle;
				when others =>
					next_state := mss_idle;
			end case;
			state := next_state;
		end if;
		
		MSBSY <= to_std_logic(state /= mss_idle);
	end process;
	
	-- The Lamp Inhibitor looks at the HIDE switch and turns on or off
	-- the front edge flashing lamps and the detector module lamps. It
	-- does not reset with the RESET signal, but only on power-up. The
	-- HIDE and SHOW buttons work so long as RCK is running and we have
	-- power. No malfunction of the CPU program can stop them. No failure
	-- of the logic chip's PLL or of the CK oscillator will stop them.
	Lamp_Inhibitor : process (RCK_in,RESET) is
	constant wait_count : integer := 4095;
	variable debounce_state : integer range 0 to wait_count := 0;
	begin
		if rising_edge(RCK_in) then
			HIDE <= (switches(3) = '1');
			indicators(19) <= to_std_logic(not HIDE);
			SHOW <= (switches(2) = '1');
			indicators(18) <= to_std_logic(not SHOW); 
		end if;
		
		-- Set the indicator lamps according to the indicator control
		-- and the HIDE and SHOW signals. The indicator is on when the
		-- indicator output is LO.
		for i in 1 to 15 loop
			if SHOW then
				indicators(i) <= '0';
			elsif (indicator_control(i) = '1') and (not HIDE) then
				indicators(i) <= '0';
			else 
				indicators(i) <= '1';
			end if;
		end loop;
	end process;
	
	-- The Display Panel Transmitter reads message sfrom the SDO Buffer 
	-- and serializes them for tranmission over SDO to the Display Panel. 
	-- Transmission takes place at 1 MBPS. The SDO signal is usually
	-- LO, so we begin with 2 us of guaranteed LO for set-up, then a 1-us 
	-- HI for a start bit, and the eight data bits, 1 us each. 
	DP_Transmitter : process (SCK,RESET) is 
	variable state : integer range 0 to 255;
	begin
		if (RESET = '1') then
			state := 0;
		elsif rising_edge(SCK) then
			case state is
				when 4 => SDO <= '1';
				when 5 => SDO <= '1';
				when 6 => SDO <= dp_out(7);
				when 7 => SDO <= dp_out(7);
				when 8 => SDO <= dp_out(6);
				when 9 => SDO <= dp_out(6);
				when 10 => SDO <= dp_out(5);
				when 11 => SDO <= dp_out(5);
				when 12 => SDO <= dp_out(4);
				when 13 => SDO <= dp_out(4);
				when 14 => SDO <= dp_out(3);
				when 15 => SDO <= dp_out(3);
				when 16 => SDO <= dp_out(2);
				when 17 => SDO <= dp_out(2);
				when 18 => SDO <= dp_out(1);
				when 19 => SDO <= dp_out(1);
				when 20 => SDO <= dp_out(0);
				when 21 => SDO <= dp_out(0);
				when others => SDO <= '0';
			end case;
			
			DPXRD <= '0';
			case state is
				when 0 => 
					if (DPXEMPTY = '0') then 
						state := 1; 
					else 
						state := 0;
					end if;
				when 1 => 
					DPXRD <= '1';
					state := 2;
				when 31 =>
					state := 0;
				when others =>
					state := state + 1;
			end case;
		end if;
	end process;
	
	-- The Display Panel Receiver receives eight-bit messages from the
	-- display panel and writes them into the SDI Buffer.
	DP_Receiver : process (SCK,RESET) is
	variable state : integer range 0 to 255;
	variable RSDI, FSDI : std_logic;
	begin
		if falling_edge(SCK) then
			FSDI := SDI_in;
		end if;
		if rising_edge(SCK) then
			RSDI := SDI_in;
		end if;
		
		if (RESET = '1') then
			state := 0;
			dp_in <= (others => '0');
		elsif rising_edge(SCK) then
		
			case state is
				when 1 =>
					dp_in <= (others => '0');
				when 4 | 6 | 8 | 10 | 12 | 14 | 16 | 18 => 
					dp_in(7 downto 1) <= dp_in(6 downto 0);
					dp_in(0) <= FSDI; 
				when others => dp_in <= dp_in;
			end case;
			
			DPRWR <= '0';
			case state is 
				when 0 => 
					if (RSDI = '0') and (DPRFULL = '0') then
						state := 1;
					else 
						state := 0;
					end if;
				when 1 =>
					if (RSDI = '0') then
						state := 2;
					else 
						state := 0;
					end if;
				when 2 =>
					if (RSDI = '1') then 
						state := 3; 
					else
						state := 2;
					end if;
				when 20 =>
					DPRWR <= '1';
					state := 0;
				when others =>
					state := state + 1;
			end case;
		end if;
	end process;
	
	-- The Transmitting Feedthrough Transmitter transmits sixteen bit 
	-- serial messages to the Transmitting Feedthrough, should it exist,
	-- using the TX output. Transmission takes place at 1 MBPS. The TX 
	-- signal is usually LO, so we begin with 2 us of guaranteed LO for 
	-- set-up, then a 1-us HI for a start bit, and the eight data bits, 
	-- 1 us each.
	TF_Transmitter : process (SCK,RESET) is 
	variable state : integer range 0 to 255;
	begin
		if (RESET = '1') then
			state := 0;
			TX <= '1';
		elsif rising_edge(SCK) then
			case state is
				when 4 => TX <= '1';
				when 5 => TX <= '1';
				when 6 => TX <= tf_out(15);
				when 7 => TX <= tf_out(15);
				when 8 => TX <= tf_out(14);
				when 9 => TX <= tf_out(14);
				when 10 => TX <= tf_out(13);
				when 11 => TX <= tf_out(13);
				when 12 => TX <= tf_out(12);
				when 13 => TX <= tf_out(12);
				when 14 => TX <= tf_out(11);
				when 15 => TX <= tf_out(11);
				when 16 => TX <= tf_out(10);
				when 17 => TX <= tf_out(10);
				when 18 => TX <= tf_out(9);
				when 19 => TX <= tf_out(9);
				when 20 => TX <= tf_out(8);
				when 21 => TX <= tf_out(8);
				when 22 => TX <= tf_out(7);
				when 23 => TX <= tf_out(7);
				when 24 => TX <= tf_out(6);
				when 25 => TX <= tf_out(6);
				when 26 => TX <= tf_out(5);
				when 27 => TX <= tf_out(5);
				when 28 => TX <= tf_out(4);
				when 29 => TX <= tf_out(4);
				when 30 => TX <= tf_out(3);
				when 31 => TX <= tf_out(3);
				when 32 => TX <= tf_out(2);
				when 33 => TX <= tf_out(2);
				when 34 => TX <= tf_out(1);
				when 35 => TX <= tf_out(1);
				when 36 => TX <= tf_out(0);
				when 37 => TX <= tf_out(0);
				when others => TX <= '0';
			end case;
			
			TFXRD <= '0';
			case state is
				when 0 => 
					if (TFXEMPTY = '0') then 
						state := 1; 
					else 
						state := 0;
					end if;
				when 1 => 
					TFXRD <= '1';
					state := 2;
				when 45 =>
					state := 0;
				when others =>
					state := state + 1;
			end case;
		end if;
	end process;
	
	-- The Transmitting Feedthrough Receiver receives eight-bit messages 
	-- from the Transmitting Feedthrough and writes them into the tf_in
	-- register, which we can read with the CPU. Each new byte received
	-- updates the tf_in byte. The RX signal is usually low. We wait for
	-- a 1-us HI followed by eight bits at 1 MBPS.
	TF_Receiver : process (SCK,RESET) is
	variable state : integer range 0 to 255;
	variable RRX, FRX : std_logic;
	begin
		if falling_edge(SCK) then
			FRX := RX;
		end if;
		if rising_edge(SCK) then
			RRX := RX;
		end if;
		
		if (RESET = '1') then
			state := 0;
			tf_in <= (others => '0');
			tf_in_sr <= (others => '0');
		elsif rising_edge(SCK) then
		
			case state is
				when 1 =>
					tf_in_sr <= (others => '0');
				when 4 | 6 | 8 | 10 | 12 | 14 | 16 | 18 => 
					tf_in_sr(7 downto 1) <= tf_in_sr(6 downto 0);
					tf_in_sr(0) <= FRX; 
				when others => tf_in_sr <= tf_in_sr;
			end case;
			
			case state is 
				when 0 => 
					if (RRX = '0') then
						state := 1;
					else 
						state := 0;
					end if;
				when 1 =>
					if (RRX = '0') then
						state := 2;
					else 
						state := 0;
					end if;
				when 2 =>
					if (RRX = '1') then 
						state := 3; 
					else
						state := 2;
					end if;
				when 20 =>
					tf_in <= tf_in_sr;
					state := 0;
				when others =>
					state := state + 1;
			end case;
		end if;
		
		TFRA <= to_std_logic(state >= 3);
	end process;		

	-- The UPLOAD flag is set when the Relay is reading from the message buffer.
	-- The EMPTY flag is set when the Message Buffer is nearly empty.
	Fifo_Indicators : process (CK,RESET) is
	constant end_count : integer := 255;
	variable upload_state, empty_state : integer range 0 to end_count;
	begin
		if (RESET = '1') then
			UPLOAD <= false;
			upload_state := 0;
			EMPTY <= false;
			empty_state := 0;
		elsif rising_edge(CK) then
			if upload_state = 0 then
				if MRDS then
					upload_state := 1;
				else 
					upload_state := 0;
				end if;
			else
				upload_state := upload_state + 1;
			end if;
			UPLOAD <= (upload_state > 0);
			
			if empty_state = 0 then
				if (to_integer(unsigned(fifo_byte_count)) < fifo_near_empty) then
					empty_state := 1;
				else
					empty_state := 0;
				end if;
			else
				empty_state := empty_state + 1;
			end if;
			EMPTY <= (empty_state > 0);
		end if;
	end process;
	
	-- Upload and Empty indicators are 16 and 17 respectively, negative true.
	indicators(16) <= to_std_logic(not UPLOAD);
	indicators(17) <= to_std_logic(not EMPTY);
	
	-- Configure lamp, negative true. Turns on when we press either the base
	-- board configuration switch or the Display Panel sends a CONFIG bit.
	config_lamp <= to_std_logic(not CONFIG); 
	
	-- Ethernet activity and lamps. These turn on when their outputs are '1'.
	EGRN <= '1';
	EYLW <= to_std_logic(ETH = '0');
	
	-- Test points. We change these so much that we don't bother putting 
	-- their functions in the top comments.	
	Test_Points : process (CONFIG, HIDE, SHOW) is
	begin
		if SHOW then
			if HIDE then
				TP1 <= DMBWR;
				TP2 <= DMIBSY;
			else
				TP1 <= MSBWR;
				TP2 <= MSBSY;
			end if;
		else 
			if HIDE then
				TP1 <= DMBRD;
				TP2 <= MSBRD;
			else
				TP1 <= tp_reg(0);
				TP2 <= tp_reg(1);
			end if;
		end if;
	end process;
	
end behavior;