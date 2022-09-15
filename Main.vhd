-- <pre> ALT Base Board (A3038BB) Controller Firmware, Toplevel Unit

-- Version A01, 22-APR-21. We have all inputs and outputs defined and draft code for all 
-- functions. We test CPU with lamps by running a program that makes the white lamps wax 
-- and wane.  We can read the test point register, which makes it possible to change only 
-- one bit at a time.

-- Version A02, 30-APR-21. We make the interrupt counter a global signal and allow the 
-- CPU to read it back, for use as our message timestamp. Add relay interface with locations
-- for job, command, status, and other standard locations. These map into both the CPU
-- and the relay address spaces.

-- Version A03, 05-MAY-21. We debug the message counter and implement an interrupt for
-- message counter non-zero.

-- Version A04, [13-MAY-21] Fix RESET arbitration so the CPU does not get stuck in reset
-- after it writes to its self-reset register. Fix Message Buffer Controller handshake
-- implementation. Add CPU stack overflow flag to reset conditions. The resulting code
-- runs perfectly with the Recorder Instrument, providing SCT messages with data and clocks,
-- although our CPU code does not yet include the power measurement payload. Remaining
-- problems in the code are timing violations that we have to overlook, and if we run
-- the CPU RAM off !PCK rather than !CK, the code asserts RESET every 10 ms for reasons we 
-- are unable to determine. So we leave CPU RAM on !CK for the final version of A04.

-- Version A05, [01-JUN-21] Disable output registers on CPU RAM and our firmware now
-- acts as we expect, with the MMU, RAM and ROM running of !PCK. Add reset of irq_set
-- and irq_rst bits on RESET. Provide DMCK of 8 MHz for detector modules. Increment
-- available message counter on falling edge of DMRCV.

-- Version A06, [18-JUN-21] Add PLL to generate 80 MHz from 10 MHz. Add debounce to 
-- falling edge of RECEIVER_pin.

-- Version A07, [19-AUG-21] Add locations in CPU memory where we can assert INCOMING
-- and RECEIVED and read their current values. Add second interrupt timer for use
-- by display lamp intensity interrupt.

-- Version A08, [14-SEP-22] Remove STOF so we can compile with OSR8V3. We also edit the
-- OSR8V3 code so program and cpu addresses are 13 bits by default, to match our existin
-- code.


-- Global constants and types.  
library ieee;  
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity main is 
	port (
		MCK : in std_logic; -- Master (10 MHz) Clock
		RCK_pin : in std_logic; -- Reference (32.768 kHz) Clock
		
		cont_data : inout std_logic_vector(7 downto 0);
		cont_addr : in std_logic_vector(5 downto 0);
		CWR_pin : in std_logic;
		CDS_pin : in std_logic;
		ETH : in std_logic;
		EGRN, EYLW : out std_logic;
		
		buff_addr : out std_logic_vector(20 downto 0);
		buff_data : inout std_logic_vector(7 downto 0);
		REN_not : out std_logic;
		RWE_not : out std_logic;
		
		HWRST_pin : in std_logic;
		
		indicators : out std_logic_vector(1 to 19);
		config_lamp : out std_logic;
		
		switches : in std_logic_vector(2 to 4);
		
		DSU : out std_logic; -- Data Strobe Upstream
		dub : in std_logic_vector(7 downto 0); -- Data Upstream Bus
		
		DMRST : inout std_logic; -- Detector Module Reset (DC0)
		DRC : inout std_logic; -- Detector Readout Complete (DC1)
		DMERR_pin : in std_logic; -- Detecor Module Error (DC2)
		INCOMING_pin : inout std_logic; -- Incoming Message Flag (DC3)
		RECEIVED_pin : inout std_logic; -- Message Received Flag (DC4)
		HIDEDM : out std_logic; -- Hide Detector Module Lamps (DC5)
		SHOWDM : out std_logic; -- Show Detector Module Lamps (DC6)
		DMCK : out std_logic; -- Demodulator Clock (DC7)
		
		TP1, TP2, TP3, TP4 : out std_logic -- Test Point Register
	);	

-- Version numbers.
	constant hardware_id : integer := 38;
	constant hardware_version : integer := 2;
	constant firmware_version : integer := 7;

-- Configuration of OSR8 CPU, RAM, and ROM.
	constant prog_addr_len : integer := 13;
	constant cpu_addr_len : integer := 13;
	constant ram_addr_len : integer := 13;
	constant start_pc : integer := 0;
	constant interrupt_pc : integer := 3;
end;

architecture behavior of main is

-- Attributes to guide the compiler.
	attribute syn_keep : boolean;
	attribute nomerge : string;

-- Reset signals.
	signal RCV_RST_CPU : boolean := false; -- Receiver logic reset by CPU
	signal RCV_RST_RELAY : boolean := false; -- Receiver logic reset by relay
	signal HWRST : boolean := false; -- Hardware reset from switch.

-- Synchronized, delayed, and inverted inputs.
	signal RCK, INCOMING, RESET, RECEIVED, DMERR : std_logic;
	signal CDS, CWR : boolean;
	signal RECEIVED_DEL, DRC_DEL : std_logic;
	
-- Clock and Timing Signals.
	signal LOCK : std_logic; -- PLL Lock
	signal FCK : std_logic; -- Fast Clock (80 MHz)
	signal CK : std_logic; -- State machine clock (40 MHz)
	signal PCK : std_logic; -- Processor clock (20 MHz)

-- Detector Modules
	constant dm_buff_len : integer := 64;
	signal dm_msg_count : integer range 0 to dm_buff_len-1;
	
-- Message Buffer
	signal MWRS : boolean; -- Message Write Strobe
	signal MWRACK : boolean; -- Message Write Acknowledge
	signal mwr_data : std_logic_vector(7 downto 0); -- Message Write Data
	signal MRDS : boolean; -- Message Read Strobe
	signal MRDACK : boolean; -- Message Read Acknowledge
	signal mrd_data : std_logic_vector(7 downto 0); -- Message Read Data
	signal fifo_byte_count : std_logic_vector(20 downto 0);

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
		
	-- CPU Memory Map Constants, sizes and base addresses in units of 512 bytes.
	constant cpu_ram_base : integer := 0;
	constant cpu_ram_range : integer := 12;
	constant cpu_ctrl_base : integer := 15;
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
	constant dm_complete_addr : integer := 9; -- Detector Read Complete (Write)
	constant dm_strobe_addr : integer := 10; -- Data Strobe Upstream (Write)
	constant dm_data_addr : integer := 11; -- Detector Module Data (Read)
	constant dm_count_addr : integer := 12; -- Number of Queued Messages (Read)
	constant irq_tmr1_addr : integer := 13; -- Timer One value (Read)
	constant relay_djr_addr : integer := 14; -- Relay Device Job Register (Read)
	constant relay_crhi_addr : integer := 15; -- Relay Command Register HI (Read)
	constant relay_crlo_addr : integer := 16; -- Relay Command Register LO (Read)
	constant relay_djr_rst_addr : integer := 17; -- Reset Relay Device Job Register (Write)
	constant relay_rc3_addr : integer := 18; -- Repeat Counter Byte 3 (Read)
	constant relay_rc2_addr : integer := 19; -- Repeat Counter Byte 2 (Read)
	constant relay_rc1_addr : integer := 20; -- Repeat Counter Byte 1 (Read)
	constant relay_rc0_addr : integer := 21; -- Repeat Counter Byte 0 (Read)
	constant fifo_cnt2_addr : integer := 22; -- Fifo Message Count Byte 2 (Read)
	constant fifo_cnt1_addr : integer := 23; -- Fifo Message Count Byte 1 (Read)
	constant fifo_cnt0_addr : integer := 24; -- Fifo Message Count Byte 0 (Read)
	constant incoming_addr : integer := 25; -- Assert INCOMING (Read/Write)
	constant received_addr : integer := 26; -- Assert RECEIVED (Read/Write)
	constant irq_tmr2_max_addr : integer := 27; -- Timer Two Period Minus One (Read/Write)
	constant irq_tmr2_addr : integer := 28; -- Timer Two value (Read)
	constant fv_addr : integer := 29; -- Firmware Version number (Read)
	constant indicators_addr : integer := 32; -- Indicator lamp array (Write)
	constant indicator_low : integer := 1; -- Low index of CPU-controlled indicators
	constant indicator_hi : integer := 15; -- High index of CPU-controlled indicators
	
	-- Relay Interface Registers.
	signal cont_djr : std_logic_vector(7 downto 0); -- Device Job Register
	signal cont_cr : std_logic_vector(15 downto 0); -- Command Register
	signal cont_rc : std_logic_vector(31 downto 0); -- Repeat Counter
	
	-- Relay Interface Memory Map Constants with Read and Write as seen by the
	-- LWDAQ Relay that is master of the interface. We respect the existing
	-- allocation of controller addresses given in the A2071 manual. 
	constant cont_id_addr : integer := 0; -- Hardware Identifier (Read)
	constant cont_sr_addr : integer := 1; -- Status Register (Read)
	constant cont_djr_addr : integer := 3; -- Device Job Register (Read/Write)
	constant cont_hv_addr : integer := 18; -- Hardware Version (Read)
	constant cont_fv_addr : integer := 19; -- Firmware Version (Read)
	constant cont_crhi_addr : integer := 32; -- Command Ragister HI (Write)
	constant cont_crlo_addr : integer := 33; -- Command Ragister LO (Write)
	constant cont_rc3_addr : integer := 34; -- Repeat Counter Byte 3 (Write)
	constant cont_rc2_addr : integer := 35; -- Repeat Counter Byte 2 (Write)
	constant cont_rc1_addr : integer := 36; -- Repeat Counter Byte 1 (Write)
	constant cont_rc0_addr : integer := 37; -- Repeat Counter Byte 0 (Write)
	constant cont_cfsw_addr : integer := 40; -- Configuration Switch (Read)
	constant cont_srst_addr : integer := 41; -- Software Reset of Controller (Write)
	constant cont_fifo_av_addr : integer := 61; -- Fifo Blocks Available (Read)
	constant cont_fifo_ds_addr : integer := 62; -- Fifo Data Strobe (Read/Write)
	constant cont_fifo_rd_addr : integer := 63; -- Fifo Read Portal (Read)
			
-- Indicator signals.
	signal HIDE : boolean := false;
	signal SHOW : boolean := false;
	
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
	-- demodulator clock (DMCK), which the demodulators multiply to 40 MHz for their 
	-- detectors.
	Divider : process (FCK) is
	variable p_count : integer range 0 to 3;
	variable d_count : integer range 0 to 15;
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
	end process;
	
	-- The Synchronizer provides positive-polarity versions of incoming signals,
	-- synchronized with CK. We take particular care with RECEIVED_pin, which is a
	-- slow-falling global signal that we must debounce on all falling edges.
	Synchronizer : process (CK,RESET) is
	constant max_state : integer := 15;
	variable rcv_state, next_rcv_state : integer range 0 to max_state;
	begin
		-- Synchronizing with CK.
		if rising_edge(CK) then
			RCK <= RCK_pin;
			CDS <= (CDS_pin = '0');
			INCOMING <= INCOMING_pin;
			DMERR <= DMERR_pin;
			HWRST <= (HWRST_pin = '0');
		end if;
		
		-- Synchronize RECEIVED_pin, debounce falling edge.
		if RESET = '1' then
			rcv_state := 0;
			RECEIVED <= '0';
		elsif rising_edge(CK) then
			if rcv_state = 0 then
				if RECEIVED_pin = '0' then
					RECEIVED <= '0';
					next_rcv_state := 0;
				else
					RECEIVED <= '1';
					next_rcv_state := 1;
				end if;
			elsif rcv_state = 1 then
				RECEIVED <= '1';
				if RECEIVED_pin = '0' then
					next_rcv_state := 2;
				else
					next_rcv_state := 1;
				end if;
			elsif rcv_state = max_state then
				RECEIVED <= '0';
				next_rcv_state := 0;
			else
				RECEIVED <= '0';
				next_rcv_state := rcv_state + 1;
			end if;
			rcv_state := next_rcv_state;
		end if;
		
		-- Signals delayed by one CK period.
		if rising_edge(CK) then
			RECEIVED_DEL <= RECEIVED;
			DRC_DEL <= DRC;
		end if;
	
		-- Inversions for negative-true signals.
		CWR <= CWR_pin = '0';
	end process;
	
	-- The Reset Arbitrator manages the various sources of reset signals.
	Reset_Arbitrator : process (CK)
	constant reset_len : integer := 63;
	variable state, next_state : integer range 0 to reset_len;
	variable initiate : boolean;
	begin
		if rising_edge(CK) then
			initiate := HWRST or RCV_RST_CPU or RCV_RST_RELAY;
			if state = 0 then
				if initiate then 
					next_state := 1;
				else 
					next_state := 0;
				end if;
			elsif state = reset_len then
				if initiate then
					next_state := reset_len;
				else
					next_state := 0;
				end if;
			else
				next_state := state + 1;
			end if;
			RESET <= to_std_logic(state >= 2);
			state := next_state;
		end if;
	end process;
	
	-- The Available Message Counter counts up on each falling edge of RECEIVED 
	-- and counts down on each falling edge of Detector Read Complete (DRC), so
	-- as to keep count of the number of messages stored in the detector module
	-- message buffers.
	Available_Message_Counter : process (CK,DMRST)
	begin
		if (DMRST = '1') then
			dm_msg_count <= 0;
		elsif rising_edge(CK) then
			if (RECEIVED_DEL = '1') and (RECEIVED = '0') then
				if (DRC = '0') and (DRC_DEL = '1') then
					dm_msg_count <= dm_msg_count;
				else
					dm_msg_count <= dm_msg_count + 1;
				end if;
			elsif (DRC = '0') and (DRC_DEL = '1') then
				dm_msg_count <= dm_msg_count - 1;
			else
				dm_msg_count <= dm_msg_count;
			end if;
		end if;
	end process;
	
	-- The Message Buffer is a FIFO made out of the external SRAM, two pointers,
	-- and the Message Buffer Controller. Writes to the FIFO occur when the 
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
	variable REN, RWE : boolean;
	begin
		if (RESET = '1') then
			write_ptr := std_logic_vector(to_unsigned(0,21));
			read_ptr := std_logic_vector(to_unsigned(0,21));
			fifo_byte_count <= std_logic_vector(to_unsigned(0,21));
			buff_data <= high_z_byte;
			mrd_data <= max_data_byte;
			REN := false;
			RWE := false;
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
						REN := false;
						RWE := true;
						next_state := 1; 
					elsif MRDS and (not MRDACK) then
						if to_integer(unsigned(fifo_byte_count)) /= 0 then
							buff_addr <= read_ptr;
							buff_data <= high_z_byte;
							REN := true;
							RWE := false;
							next_state := 4;
						else
							buff_addr <= read_ptr;
							buff_data <= high_z_byte;
							REN := false;
							RWE := false;
							next_state := 0;
						end if;
					else
						buff_addr <= read_ptr;
						buff_data <= high_z_byte;
						REN := false;
						RWE := false;
						next_state := 0;					
					end if;
				when 1 =>
					buff_addr <= write_ptr;
					buff_data <= mwr_data;
					REN := true;
					RWE := true;
					next_state := 2;
				when 2 =>
					buff_addr <= write_ptr;
					buff_data <= mwr_data;
					REN := false;
					RWE := true;
					next_state := 3;
				when 3 =>
					buff_addr <= write_ptr;
					buff_data <= mwr_data;
					REN := false;
					RWE := false;
					MWRACK <= true;
					next_write_ptr := std_logic_vector(unsigned(write_ptr) + 1);
					fifo_byte_count <= std_logic_vector(unsigned(fifo_byte_count) + 1);
					next_state := 0;
				when 4 =>
					buff_addr <= read_ptr;
					buff_data <= high_z_byte;
					mrd_data <= buff_data;
					REN := true;
					RWE := false;
					next_state := 5;
				when 5 =>
					buff_addr <= read_ptr;
					buff_data <= high_z_byte;
					REN := false;
					RWE := false;
					MRDACK <= true;
					next_read_ptr := std_logic_vector(unsigned(read_ptr) + 1);
					fifo_byte_count <= std_logic_vector(unsigned(fifo_byte_count) - 1);
					next_state := 0;
				when others =>
					buff_addr <= read_ptr;
					buff_data <= high_z_byte;
					REN := false;
					RWE := false;
					next_state := 0;
			end case;
			state := next_state;
			read_ptr := next_read_ptr;
			write_ptr := next_write_ptr;
		end if;
		
		REN_not <= to_std_logic(not REN);
		RWE_not <= to_std_logic(not RWE);
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
	
-- The Memory Manager maps eight-bit read and write access to Detector Module 
-- daisy chain bus, and the registers of the relay interface, as well as the
-- Random Access Memory, and Interrupt Handler. Byte ordering is big-endian 
-- (most significant byte at lower address). 
	MMU : process (PCK,RESET) is
		variable top_bits : integer range 0 to 15;
		variable bottom_bits : integer range 0 to 127;
	begin
	
		-- Some variables for brevity.
		top_bits := to_integer(unsigned(cpu_addr(12 downto 9)));
		bottom_bits := to_integer(unsigned(cpu_addr(7 downto 0)));
		
		-- The RAM data in, its address, and its write strobe are all 
		-- combinatorial functions of the CPU outputs. They will be ready 
		-- well before the falling edge of PCK.
		cpu_ram_in <= cpu_data_out;
		cpu_ram_addr <= cpu_addr(12 downto 0);
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
			when fv_addr => cpu_data_in <= std_logic_vector(
				to_unsigned(firmware_version,8));
			when test_point_addr => cpu_data_in <= tp_reg;
			when dm_data_addr => cpu_data_in <= dub;
			when dm_count_addr => 
				cpu_data_in <= std_logic_vector(to_unsigned(dm_msg_count,8));
			when relay_djr_addr => cpu_data_in <= cont_djr;
			when relay_crhi_addr => cpu_data_in <= cont_cr(15 downto 8);
			when relay_crlo_addr => cpu_data_in <= cont_cr(7 downto 0);
			when relay_rc3_addr => cpu_data_in <= cont_rc(31 downto 24);
			when relay_rc2_addr => cpu_data_in <= cont_rc(23 downto 16);
			when relay_rc1_addr => cpu_data_in <= cont_rc(15 downto 8);
			when relay_rc0_addr => cpu_data_in <= cont_rc(7 downto 0);
			when incoming_addr => 
				if (INCOMING = '1') then
					cpu_data_in <= one_data_byte;
				else
					cpu_data_in <= zero_data_byte;
				end if;
			when received_addr => 
				if (RECEIVED = '1') then
					cpu_data_in <= one_data_byte;
				else
					cpu_data_in <= zero_data_byte;
				end if;
			when fifo_cnt2_addr => 
				cpu_data_in(7 downto 5) <= "000";
				cpu_data_in(4 downto 0) <= fifo_byte_count(20 downto 16);
			when fifo_cnt1_addr => cpu_data_in <= fifo_byte_count(15 downto 8);
			when fifo_cnt0_addr => cpu_data_in <= fifo_byte_count(7 downto 0);
			when others => cpu_data_in <= max_data_byte;
			end case;
		when others =>
			cpu_data_in <= max_data_byte;
		end case;
		
		-- We use the falling edge of PCK to write to registers and to initiate sensor 
		-- and transmit activity. We have RCV_RST_CPU set upon a write to cpu_rst_addr,
		-- which provokes a RESET pulse starting on the next rising edge of CK, which in
		-- turn causes an asynchronous reset of RCV_RST_CPU. But the RESET pulse endures,
		-- thanks to the state machine in the Reset Arbitrator. The Message Write Strobe
		-- (MWRS) is set on a write to msg_write_addr, and mwr_data is written for the
		-- use of the Message Buffer Controller. On the next falling edge of PCK, if 
		-- MWRACK is asserted, we clear MWRS, unless the very next rising edge carries
		-- another write to msg_write_addr, in which case this clear of MWRS will be
		-- over-ridden. 
		if (RESET = '1') then 
			irq_tmr1_max <= max_data_byte;
			irq_tmr2_max <= max_data_byte;
			irq_mask <= zero_data_byte;
			irq_rst <= zero_data_byte;
			irq_set <= zero_data_byte;
			MWRS <= false;
			DMRST <= '1';
			DJRRST <= false;
			RCV_RST_CPU <= false;
			for i in 1 to 15 loop indicator_control(i) <= '0'; end loop;
		elsif falling_edge(PCK) then
			irq_rst <= zero_data_byte;
			irq_set <= zero_data_byte;
			DMRST <= '0';
			DJRRST <= false;
			if MWRACK then MWRS <= false; end if;
			if CPUDS and CPUWR then 
				if (top_bits >= cpu_ctrl_base) 
						and (top_bits <= cpu_ctrl_base+cpu_ctrl_range-1) then
					case bottom_bits is
						when irq_mask_addr => irq_mask <= cpu_data_out;
						when irq_reset_addr => irq_rst <= cpu_data_out;
						when irq_set_addr => irq_set <= cpu_data_out;
						when irq_tmr1_max_addr => irq_tmr1_max <= cpu_data_out;
						when irq_tmr2_max_addr => irq_tmr2_max <= cpu_data_out;
						when cpu_rst_addr => RCV_RST_CPU <= true;
						when test_point_addr => tp_reg <= cpu_data_out;
						when msg_write_addr => 
							mwr_data <= cpu_data_out;
							MWRS <= true;
						when dm_reset_addr => DMRST <= '1';
						when dm_complete_addr => DRC <= cpu_data_out(0);
						when dm_strobe_addr => DSU <= cpu_data_out(0);
						when relay_djr_rst_addr => DJRRST <= true;
						when incoming_addr => 
							if (cpu_data_out(0) = '1') then
								INCOMING_pin <= '1';
							else
								INCOMING_pin <= 'Z';
							end if;
						when received_addr => 
							if (cpu_data_out(0) = '1') then
								RECEIVED_pin <= '1';
							else
								RECEIVED_pin <= 'Z';
							end if;
						when indicators_addr + 1 to indicators_addr + 15 =>
							indicator_control(to_integer(unsigned(cpu_addr(3 downto 0)))) 
								<= cpu_data_out(0);
					end case;
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

			-- The detector module receive interrupt is set when we see
			-- that there are messages in the incoming message buffer.
			if (irq_rst(1) = '1') then
				irq_bits(1) <= '0';
			elsif (dm_msg_count /= 0)
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
	-- clears MRDACK a clock period later. The RCV_RST_RELAY signal initiates a
	-- RESET pulse, which is produced by the Reset Arbitrator. This pulse endures
	-- for many clock cycles, regardless of when RCV_RST_RELAY is unasserted, so
	-- we are free to clear RCV_RST_RELAY with an asynchronous reset using the
	-- RESET pulse. We note that the RESET pulse causes a pulse on Detector 
	-- Module Reset (DMRST) and Device Job Register Reset (DJRRST), and so resets
	-- not only the controller, but the demodulators and the job register. But
	-- the RESET does not affect the relay itself, which we must reset with 
	-- a separate LWDAQ reboot instruction.
	Relay_Interface : process (CK,RESET) is
	variable integer_addr : integer range 0 to 63;
	begin
		integer_addr := to_integer(unsigned(cont_addr));
		
		if RESET = '1' then
			cont_data <= high_z_byte;
			cont_djr <= zero_data_byte;
			MRDS <= false;
			RCV_RST_RELAY <= false;
		elsif rising_edge(CK) then		
			if CWR then cont_data <= high_z_byte; end if;
			
			if CDS and CWR then
				case integer_addr is
				when cont_djr_addr => cont_djr <= cont_data;
				when cont_crhi_addr => cont_cr(15 downto 8) <= cont_data;
				when cont_crlo_addr => cont_cr(7 downto 0) <= cont_data;
				when cont_rc3_addr => cont_rc(31 downto 24) <= cont_data;
				when cont_rc2_addr => cont_rc(23 downto 16) <= cont_data;
				when cont_rc1_addr => cont_rc(15 downto 8) <= cont_data;
				when cont_rc0_addr => cont_rc(7 downto 0) <= cont_data;
				when cont_srst_addr => RCV_RST_RELAY <= true;
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
					cont_data(5) <= to_std_logic(dm_msg_count /= 0);
					cont_data(6) <= tp_reg(6);
					cont_data(7) <= tp_reg(7);
				when cont_djr_addr => 
					cont_data <= cont_djr;
				when cont_hv_addr =>
					cont_data <= std_logic_vector(to_unsigned(hardware_version,8));
				when cont_fv_addr =>
					cont_data <= std_logic_vector(to_unsigned(firmware_version,8));
				when cont_cfsw_addr =>
					cont_data(0) <= to_std_logic(switches(4) = '0');
				when cont_fifo_av_addr =>
					if to_integer(unsigned(fifo_byte_count(20 downto 17))) = 0 then
						cont_data <= fifo_byte_count(16 downto 9);
					else
						cont_data <= max_data_byte;
					end if;
				when cont_fifo_ds_addr =>
					cont_data(0) <= to_std_logic(MRDACK);
					cont_data(7 downto 1) <= "0000000";
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
		
	-- The Lamp Inhibitor looks at the HIDE switch and turns on or off
	-- the front edge flashing lamps and the detector module lamps. It
	-- does not reset with the RESET signal, but only on power-up. The
	-- HIDE and SHOW buttons work so long as RCK is running and we have
	-- power. No malfunction of the CPU program can stop them.
	Lamp_Inhibitor : process (RCK_pin,RESET) is
	constant wait_count : integer := 4095;
	variable debounce_state : integer range 0 to wait_count := 0;
	begin
		if rising_edge(RCK_pin) then
			if debounce_state = 0 then
				if switches(3) = '1' then
					HIDE <= not HIDE;
					debounce_state := 1;
				end if;
			elsif debounce_state = wait_count then
				if switches(3) = '0' then
					debounce_state := 0;
				else
					debounce_state := wait_count;
				end if;
			else
				debounce_state := debounce_state + 1;
			end if;
			indicators(19) <= to_std_logic(not HIDE);
			
			if switches(2) = '1' then
				SHOW <= true;
			else
				SHOW <= false;
			end if;
			indicators(18) <= to_std_logic(not SHOW); 
		end if;
		
		HIDEDM <= to_std_logic(HIDE);
		SHOWDM <= to_std_logic(SHOW);

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
	
	-- Ethernet activity and lamps.
	EGRN <= '1';
	EYLW <= to_std_logic(ETH = '0');
	
	-- Upload indicator, shines when Relay is reading from message FIFO.
	-- Empty indicator, shines when the FIFO is nearly empty. Both indicators
	-- turn on when their logic signal is LO.
	Fifo_Indicators : process (CK,RESET) is
	begin
		if (RESET = '1') then
			indicators(16) <= '1';
			indicators(17) <= '1';
		elsif rising_edge(CK) then
			-- Indicator 16 is UPLOAD.
			if (MRDS and (not HIDE)) or SHOW then
				indicators(16) <= '0';
			else
				indicators(16) <= '1';
			end if;
			
			-- Indicator 17 is EMPTY.
			if ((to_integer(unsigned(fifo_byte_count)) = 0) 
					and (not HIDE)) or SHOW then
				indicators(17) <= '0';
			else
				indicators(17) <= '1';
			end if;
		end if;
	end process;
	
	-- Configure lamp. Turns on when the config lamp output goes low.
	config_lamp <= to_std_logic(switches(4) = '0'); -- Configuration lamp and switch
	
	-- Test points. We have TP1..TP3 showing the microprocessor-controlled registers 
	-- zero through two. We have TP4 showing us any changes in the upstream data bus.
	TP1 <= tp_reg(0);
	TP2 <= tp_reg(1);
	TP3 <= tp_reg(2);
	TP4 <= dub(0) xor dub(1) xor dub(2) xor dub(3) xor dub(4) xor dub(5) xor dub(6) xor dub(7);
end behavior;