-- VHDL netlist generated by SCUBA Diamond (64-bit) 3.12.1.454
-- Module  Version: 7.5
--C:\lscc\diamond\3.12\ispfpga\bin\nt64\scuba.exe -w -n CPU_RAM -lang vhdl -synth synplify -bus_exp 7 -bb -arch xo2c00 -type bram -wp 10 -rp 1000 -addr_width 12 -data_width 8 -num_rows 3072 -cascade -1 -mem_init0 -writemode NORMAL 

-- Fri Dec 09 13:55:09 2022

library IEEE;
use IEEE.std_logic_1164.all;
-- synopsys translate_off
library MACHXO2;
use MACHXO2.components.all;
-- synopsys translate_on

entity CPU_RAM is
    port (
        Clock: in  std_logic; 
        ClockEn: in  std_logic; 
        Reset: in  std_logic; 
        WE: in  std_logic; 
        Address: in  std_logic_vector(11 downto 0); 
        Data: in  std_logic_vector(7 downto 0); 
        Q: out  std_logic_vector(7 downto 0));
end CPU_RAM;

architecture Structure of CPU_RAM is

    -- internal signal declarations
    signal wren_inv: std_logic;
    signal scuba_vhi: std_logic;
    signal scuba_vlo: std_logic;
    signal wren_inv_g: std_logic;
    signal mdout0_1_0: std_logic;
    signal mdout0_0_0: std_logic;
    signal mdout0_1_1: std_logic;
    signal mdout0_0_1: std_logic;
    signal mdout0_1_2: std_logic;
    signal mdout0_0_2: std_logic;
    signal mdout0_1_3: std_logic;
    signal mdout0_0_3: std_logic;
    signal mdout0_1_4: std_logic;
    signal mdout0_0_4: std_logic;
    signal mdout0_1_5: std_logic;
    signal mdout0_0_5: std_logic;
    signal mdout0_1_6: std_logic;
    signal mdout0_0_6: std_logic;
    signal addr11_ff: std_logic;
    signal mdout0_1_7: std_logic;
    signal mdout0_0_7: std_logic;

    -- local component declarations
    component AND2
        port (A: in  std_logic; B: in  std_logic; Z: out  std_logic);
    end component;
    component FD1P3DX
        port (D: in  std_logic; SP: in  std_logic; CK: in  std_logic; 
            CD: in  std_logic; Q: out  std_logic);
    end component;
    component INV
        port (A: in  std_logic; Z: out  std_logic);
    end component;
    component MUX21
        port (D0: in  std_logic; D1: in  std_logic; SD: in  std_logic; 
            Z: out  std_logic);
    end component;
    component VHI
        port (Z: out  std_logic);
    end component;
    component VLO
        port (Z: out  std_logic);
    end component;
    component DP8KC
        generic (INIT_DATA : in String; INITVAL_1F : in String; 
                INITVAL_1E : in String; INITVAL_1D : in String; 
                INITVAL_1C : in String; INITVAL_1B : in String; 
                INITVAL_1A : in String; INITVAL_19 : in String; 
                INITVAL_18 : in String; INITVAL_17 : in String; 
                INITVAL_16 : in String; INITVAL_15 : in String; 
                INITVAL_14 : in String; INITVAL_13 : in String; 
                INITVAL_12 : in String; INITVAL_11 : in String; 
                INITVAL_10 : in String; INITVAL_0F : in String; 
                INITVAL_0E : in String; INITVAL_0D : in String; 
                INITVAL_0C : in String; INITVAL_0B : in String; 
                INITVAL_0A : in String; INITVAL_09 : in String; 
                INITVAL_08 : in String; INITVAL_07 : in String; 
                INITVAL_06 : in String; INITVAL_05 : in String; 
                INITVAL_04 : in String; INITVAL_03 : in String; 
                INITVAL_02 : in String; INITVAL_01 : in String; 
                INITVAL_00 : in String; ASYNC_RESET_RELEASE : in String; 
                RESETMODE : in String; GSR : in String; 
                WRITEMODE_B : in String; WRITEMODE_A : in String; 
                CSDECODE_B : in String; CSDECODE_A : in String; 
                REGMODE_B : in String; REGMODE_A : in String; 
                DATA_WIDTH_B : in Integer; DATA_WIDTH_A : in Integer);
        port (DIA8: in  std_logic; DIA7: in  std_logic; 
            DIA6: in  std_logic; DIA5: in  std_logic; 
            DIA4: in  std_logic; DIA3: in  std_logic; 
            DIA2: in  std_logic; DIA1: in  std_logic; 
            DIA0: in  std_logic; ADA12: in  std_logic; 
            ADA11: in  std_logic; ADA10: in  std_logic; 
            ADA9: in  std_logic; ADA8: in  std_logic; 
            ADA7: in  std_logic; ADA6: in  std_logic; 
            ADA5: in  std_logic; ADA4: in  std_logic; 
            ADA3: in  std_logic; ADA2: in  std_logic; 
            ADA1: in  std_logic; ADA0: in  std_logic; CEA: in  std_logic; 
            OCEA: in  std_logic; CLKA: in  std_logic; WEA: in  std_logic; 
            CSA2: in  std_logic; CSA1: in  std_logic; 
            CSA0: in  std_logic; RSTA: in  std_logic; 
            DIB8: in  std_logic; DIB7: in  std_logic; 
            DIB6: in  std_logic; DIB5: in  std_logic; 
            DIB4: in  std_logic; DIB3: in  std_logic; 
            DIB2: in  std_logic; DIB1: in  std_logic; 
            DIB0: in  std_logic; ADB12: in  std_logic; 
            ADB11: in  std_logic; ADB10: in  std_logic; 
            ADB9: in  std_logic; ADB8: in  std_logic; 
            ADB7: in  std_logic; ADB6: in  std_logic; 
            ADB5: in  std_logic; ADB4: in  std_logic; 
            ADB3: in  std_logic; ADB2: in  std_logic; 
            ADB1: in  std_logic; ADB0: in  std_logic; CEB: in  std_logic; 
            OCEB: in  std_logic; CLKB: in  std_logic; WEB: in  std_logic; 
            CSB2: in  std_logic; CSB1: in  std_logic; 
            CSB0: in  std_logic; RSTB: in  std_logic; 
            DOA8: out  std_logic; DOA7: out  std_logic; 
            DOA6: out  std_logic; DOA5: out  std_logic; 
            DOA4: out  std_logic; DOA3: out  std_logic; 
            DOA2: out  std_logic; DOA1: out  std_logic; 
            DOA0: out  std_logic; DOB8: out  std_logic; 
            DOB7: out  std_logic; DOB6: out  std_logic; 
            DOB5: out  std_logic; DOB4: out  std_logic; 
            DOB3: out  std_logic; DOB2: out  std_logic; 
            DOB1: out  std_logic; DOB0: out  std_logic);
    end component;
    attribute MEM_LPC_FILE : string; 
    attribute MEM_INIT_FILE : string; 
    attribute GSR : string; 
    attribute MEM_LPC_FILE of CPU_RAM_0_0_2 : label is "CPU_RAM.lpc";
    attribute MEM_INIT_FILE of CPU_RAM_0_0_2 : label is "INIT_ALL_0s";
    attribute MEM_LPC_FILE of CPU_RAM_0_1_1 : label is "CPU_RAM.lpc";
    attribute MEM_INIT_FILE of CPU_RAM_0_1_1 : label is "INIT_ALL_0s";
    attribute MEM_LPC_FILE of CPU_RAM_1_0_0 : label is "CPU_RAM.lpc";
    attribute MEM_INIT_FILE of CPU_RAM_1_0_0 : label is "INIT_ALL_0s";
    attribute GSR of FF_0 : label is "ENABLED";
    attribute NGD_DRC_MASK : integer;
    attribute NGD_DRC_MASK of Structure : architecture is 1;

begin
    -- component instantiation statements
    INV_0: INV
        port map (A=>WE, Z=>wren_inv);

    AND2_t0: AND2
        port map (A=>wren_inv, B=>ClockEn, Z=>wren_inv_g);

    CPU_RAM_0_0_2: DP8KC
        generic map (INIT_DATA=> "STATIC", ASYNC_RESET_RELEASE=> "SYNC", 
        INITVAL_1F=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_1E=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_1D=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_1C=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_1B=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_1A=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_19=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_18=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_17=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_16=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_15=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_14=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_13=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_12=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_11=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_10=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_0F=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_0E=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_0D=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_0C=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_0B=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_0A=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_09=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_08=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_07=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_06=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_05=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_04=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_03=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_02=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_01=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_00=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        CSDECODE_B=> "0b111", CSDECODE_A=> "0b000", WRITEMODE_B=> "NORMAL", 
        WRITEMODE_A=> "NORMAL", GSR=> "ENABLED", RESETMODE=> "ASYNC", 
        REGMODE_B=> "NOREG", REGMODE_A=> "NOREG", DATA_WIDTH_B=>  4, 
        DATA_WIDTH_A=>  4)
        port map (DIA8=>scuba_vlo, DIA7=>scuba_vlo, DIA6=>scuba_vlo, 
            DIA5=>scuba_vlo, DIA4=>scuba_vlo, DIA3=>Data(3), 
            DIA2=>Data(2), DIA1=>Data(1), DIA0=>Data(0), 
            ADA12=>Address(10), ADA11=>Address(9), ADA10=>Address(8), 
            ADA9=>Address(7), ADA8=>Address(6), ADA7=>Address(5), 
            ADA6=>Address(4), ADA5=>Address(3), ADA4=>Address(2), 
            ADA3=>Address(1), ADA2=>Address(0), ADA1=>scuba_vlo, 
            ADA0=>scuba_vlo, CEA=>ClockEn, OCEA=>ClockEn, CLKA=>Clock, 
            WEA=>WE, CSA2=>scuba_vlo, CSA1=>scuba_vlo, CSA0=>Address(11), 
            RSTA=>Reset, DIB8=>scuba_vlo, DIB7=>scuba_vlo, 
            DIB6=>scuba_vlo, DIB5=>scuba_vlo, DIB4=>scuba_vlo, 
            DIB3=>scuba_vlo, DIB2=>scuba_vlo, DIB1=>scuba_vlo, 
            DIB0=>scuba_vlo, ADB12=>scuba_vlo, ADB11=>scuba_vlo, 
            ADB10=>scuba_vlo, ADB9=>scuba_vlo, ADB8=>scuba_vlo, 
            ADB7=>scuba_vlo, ADB6=>scuba_vlo, ADB5=>scuba_vlo, 
            ADB4=>scuba_vlo, ADB3=>scuba_vlo, ADB2=>scuba_vlo, 
            ADB1=>scuba_vlo, ADB0=>scuba_vlo, CEB=>scuba_vhi, 
            OCEB=>scuba_vhi, CLKB=>scuba_vlo, WEB=>scuba_vlo, 
            CSB2=>scuba_vlo, CSB1=>scuba_vlo, CSB0=>scuba_vlo, 
            RSTB=>scuba_vlo, DOA8=>open, DOA7=>open, DOA6=>open, 
            DOA5=>open, DOA4=>open, DOA3=>mdout0_0_3, DOA2=>mdout0_0_2, 
            DOA1=>mdout0_0_1, DOA0=>mdout0_0_0, DOB8=>open, DOB7=>open, 
            DOB6=>open, DOB5=>open, DOB4=>open, DOB3=>open, DOB2=>open, 
            DOB1=>open, DOB0=>open);

    CPU_RAM_0_1_1: DP8KC
        generic map (INIT_DATA=> "STATIC", ASYNC_RESET_RELEASE=> "SYNC", 
        INITVAL_1F=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_1E=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_1D=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_1C=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_1B=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_1A=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_19=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_18=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_17=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_16=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_15=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_14=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_13=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_12=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_11=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_10=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_0F=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_0E=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_0D=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_0C=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_0B=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_0A=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_09=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_08=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_07=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_06=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_05=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_04=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_03=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_02=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_01=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_00=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        CSDECODE_B=> "0b111", CSDECODE_A=> "0b000", WRITEMODE_B=> "NORMAL", 
        WRITEMODE_A=> "NORMAL", GSR=> "ENABLED", RESETMODE=> "ASYNC", 
        REGMODE_B=> "NOREG", REGMODE_A=> "NOREG", DATA_WIDTH_B=>  4, 
        DATA_WIDTH_A=>  4)
        port map (DIA8=>scuba_vlo, DIA7=>scuba_vlo, DIA6=>scuba_vlo, 
            DIA5=>scuba_vlo, DIA4=>scuba_vlo, DIA3=>Data(7), 
            DIA2=>Data(6), DIA1=>Data(5), DIA0=>Data(4), 
            ADA12=>Address(10), ADA11=>Address(9), ADA10=>Address(8), 
            ADA9=>Address(7), ADA8=>Address(6), ADA7=>Address(5), 
            ADA6=>Address(4), ADA5=>Address(3), ADA4=>Address(2), 
            ADA3=>Address(1), ADA2=>Address(0), ADA1=>scuba_vlo, 
            ADA0=>scuba_vlo, CEA=>ClockEn, OCEA=>ClockEn, CLKA=>Clock, 
            WEA=>WE, CSA2=>scuba_vlo, CSA1=>scuba_vlo, CSA0=>Address(11), 
            RSTA=>Reset, DIB8=>scuba_vlo, DIB7=>scuba_vlo, 
            DIB6=>scuba_vlo, DIB5=>scuba_vlo, DIB4=>scuba_vlo, 
            DIB3=>scuba_vlo, DIB2=>scuba_vlo, DIB1=>scuba_vlo, 
            DIB0=>scuba_vlo, ADB12=>scuba_vlo, ADB11=>scuba_vlo, 
            ADB10=>scuba_vlo, ADB9=>scuba_vlo, ADB8=>scuba_vlo, 
            ADB7=>scuba_vlo, ADB6=>scuba_vlo, ADB5=>scuba_vlo, 
            ADB4=>scuba_vlo, ADB3=>scuba_vlo, ADB2=>scuba_vlo, 
            ADB1=>scuba_vlo, ADB0=>scuba_vlo, CEB=>scuba_vhi, 
            OCEB=>scuba_vhi, CLKB=>scuba_vlo, WEB=>scuba_vlo, 
            CSB2=>scuba_vlo, CSB1=>scuba_vlo, CSB0=>scuba_vlo, 
            RSTB=>scuba_vlo, DOA8=>open, DOA7=>open, DOA6=>open, 
            DOA5=>open, DOA4=>open, DOA3=>mdout0_0_7, DOA2=>mdout0_0_6, 
            DOA1=>mdout0_0_5, DOA0=>mdout0_0_4, DOB8=>open, DOB7=>open, 
            DOB6=>open, DOB5=>open, DOB4=>open, DOB3=>open, DOB2=>open, 
            DOB1=>open, DOB0=>open);

    scuba_vhi_inst: VHI
        port map (Z=>scuba_vhi);

    CPU_RAM_1_0_0: DP8KC
        generic map (INIT_DATA=> "STATIC", ASYNC_RESET_RELEASE=> "SYNC", 
        INITVAL_1F=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_1E=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_1D=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_1C=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_1B=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_1A=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_19=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_18=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_17=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_16=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_15=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_14=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_13=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_12=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_11=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_10=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_0F=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_0E=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_0D=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_0C=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_0B=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_0A=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_09=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_08=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_07=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_06=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_05=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_04=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_03=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_02=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_01=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_00=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        CSDECODE_B=> "0b111", CSDECODE_A=> "0b010", WRITEMODE_B=> "NORMAL", 
        WRITEMODE_A=> "NORMAL", GSR=> "ENABLED", RESETMODE=> "ASYNC", 
        REGMODE_B=> "NOREG", REGMODE_A=> "NOREG", DATA_WIDTH_B=>  9, 
        DATA_WIDTH_A=>  9)
        port map (DIA8=>scuba_vlo, DIA7=>Data(7), DIA6=>Data(6), 
            DIA5=>Data(5), DIA4=>Data(4), DIA3=>Data(3), DIA2=>Data(2), 
            DIA1=>Data(1), DIA0=>Data(0), ADA12=>Address(9), 
            ADA11=>Address(8), ADA10=>Address(7), ADA9=>Address(6), 
            ADA8=>Address(5), ADA7=>Address(4), ADA6=>Address(3), 
            ADA5=>Address(2), ADA4=>Address(1), ADA3=>Address(0), 
            ADA2=>scuba_vlo, ADA1=>scuba_vlo, ADA0=>scuba_vhi, 
            CEA=>ClockEn, OCEA=>ClockEn, CLKA=>Clock, WEA=>WE, 
            CSA2=>scuba_vlo, CSA1=>Address(11), CSA0=>Address(10), 
            RSTA=>Reset, DIB8=>scuba_vlo, DIB7=>scuba_vlo, 
            DIB6=>scuba_vlo, DIB5=>scuba_vlo, DIB4=>scuba_vlo, 
            DIB3=>scuba_vlo, DIB2=>scuba_vlo, DIB1=>scuba_vlo, 
            DIB0=>scuba_vlo, ADB12=>scuba_vlo, ADB11=>scuba_vlo, 
            ADB10=>scuba_vlo, ADB9=>scuba_vlo, ADB8=>scuba_vlo, 
            ADB7=>scuba_vlo, ADB6=>scuba_vlo, ADB5=>scuba_vlo, 
            ADB4=>scuba_vlo, ADB3=>scuba_vlo, ADB2=>scuba_vlo, 
            ADB1=>scuba_vlo, ADB0=>scuba_vlo, CEB=>scuba_vhi, 
            OCEB=>scuba_vhi, CLKB=>scuba_vlo, WEB=>scuba_vlo, 
            CSB2=>scuba_vlo, CSB1=>scuba_vlo, CSB0=>scuba_vlo, 
            RSTB=>scuba_vlo, DOA8=>open, DOA7=>mdout0_1_7, 
            DOA6=>mdout0_1_6, DOA5=>mdout0_1_5, DOA4=>mdout0_1_4, 
            DOA3=>mdout0_1_3, DOA2=>mdout0_1_2, DOA1=>mdout0_1_1, 
            DOA0=>mdout0_1_0, DOB8=>open, DOB7=>open, DOB6=>open, 
            DOB5=>open, DOB4=>open, DOB3=>open, DOB2=>open, DOB1=>open, 
            DOB0=>open);

    scuba_vlo_inst: VLO
        port map (Z=>scuba_vlo);

    FF_0: FD1P3DX
        port map (D=>Address(11), SP=>wren_inv_g, CK=>Clock, 
            CD=>scuba_vlo, Q=>addr11_ff);

    mux_7: MUX21
        port map (D0=>mdout0_0_0, D1=>mdout0_1_0, SD=>addr11_ff, Z=>Q(0));

    mux_6: MUX21
        port map (D0=>mdout0_0_1, D1=>mdout0_1_1, SD=>addr11_ff, Z=>Q(1));

    mux_5: MUX21
        port map (D0=>mdout0_0_2, D1=>mdout0_1_2, SD=>addr11_ff, Z=>Q(2));

    mux_4: MUX21
        port map (D0=>mdout0_0_3, D1=>mdout0_1_3, SD=>addr11_ff, Z=>Q(3));

    mux_3: MUX21
        port map (D0=>mdout0_0_4, D1=>mdout0_1_4, SD=>addr11_ff, Z=>Q(4));

    mux_2: MUX21
        port map (D0=>mdout0_0_5, D1=>mdout0_1_5, SD=>addr11_ff, Z=>Q(5));

    mux_1: MUX21
        port map (D0=>mdout0_0_6, D1=>mdout0_1_6, SD=>addr11_ff, Z=>Q(6));

    mux_0: MUX21
        port map (D0=>mdout0_0_7, D1=>mdout0_1_7, SD=>addr11_ff, Z=>Q(7));

end Structure;

-- synopsys translate_off
library MACHXO2;
configuration Structure_CON of CPU_RAM is
    for Structure
        for all:AND2 use entity MACHXO2.AND2(V); end for;
        for all:FD1P3DX use entity MACHXO2.FD1P3DX(V); end for;
        for all:INV use entity MACHXO2.INV(V); end for;
        for all:MUX21 use entity MACHXO2.MUX21(V); end for;
        for all:VHI use entity MACHXO2.VHI(V); end for;
        for all:VLO use entity MACHXO2.VLO(V); end for;
        for all:DP8KC use entity MACHXO2.DP8KC(V); end for;
    end for;
end Structure_CON;

-- synopsys translate_on
