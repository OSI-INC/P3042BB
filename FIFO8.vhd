-- VHDL netlist generated by SCUBA Diamond (64-bit) 3.12.1.454
-- Module  Version: 5.8
--C:\lscc\diamond\3.12\ispfpga\bin\nt64\scuba.exe -w -n FIFO8 -lang vhdl -synth synplify -bus_exp 7 -bb -arch xo2c00 -type ebfifo -depth 32 -width 8 -rwidth 8 -pfu_fifo -no_enable -pe -1 -pf -1 

-- Wed Jan 04 18:39:08 2023

library IEEE;
use IEEE.std_logic_1164.all;
-- synopsys translate_off
library MACHXO2;
use MACHXO2.components.all;
-- synopsys translate_on

entity FIFO8 is
    port (
        Data: in  std_logic_vector(7 downto 0); 
        WrClock: in  std_logic; 
        RdClock: in  std_logic; 
        WrEn: in  std_logic; 
        RdEn: in  std_logic; 
        Reset: in  std_logic; 
        RPReset: in  std_logic; 
        Q: out  std_logic_vector(7 downto 0); 
        Empty: out  std_logic; 
        Full: out  std_logic);
end FIFO8;

architecture Structure of FIFO8 is

    -- internal signal declarations
    signal invout_1: std_logic;
    signal invout_0: std_logic;
    signal wptr_4_inv: std_logic;
    signal w_gdata_0: std_logic;
    signal w_gdata_1: std_logic;
    signal w_gdata_2: std_logic;
    signal w_gdata_3: std_logic;
    signal w_gdata_4: std_logic;
    signal wptr_4: std_logic;
    signal wptr_5: std_logic;
    signal r_gdata_0: std_logic;
    signal r_gdata_1: std_logic;
    signal r_gdata_2: std_logic;
    signal r_gdata_3: std_logic;
    signal r_gdata_4: std_logic;
    signal rptr_5: std_logic;
    signal w_gcount_0: std_logic;
    signal w_gcount_1: std_logic;
    signal w_gcount_2: std_logic;
    signal w_gcount_3: std_logic;
    signal w_gcount_4: std_logic;
    signal w_gcount_5: std_logic;
    signal r_gcount_0: std_logic;
    signal r_gcount_1: std_logic;
    signal r_gcount_2: std_logic;
    signal r_gcount_3: std_logic;
    signal r_gcount_4: std_logic;
    signal r_gcount_5: std_logic;
    signal w_gcount_r20: std_logic;
    signal w_gcount_r0: std_logic;
    signal w_gcount_r21: std_logic;
    signal w_gcount_r1: std_logic;
    signal w_gcount_r22: std_logic;
    signal w_gcount_r2: std_logic;
    signal w_gcount_r23: std_logic;
    signal w_gcount_r3: std_logic;
    signal w_gcount_r24: std_logic;
    signal w_gcount_r4: std_logic;
    signal w_gcount_r25: std_logic;
    signal w_gcount_r5: std_logic;
    signal r_gcount_w20: std_logic;
    signal r_gcount_w0: std_logic;
    signal r_gcount_w21: std_logic;
    signal r_gcount_w1: std_logic;
    signal r_gcount_w22: std_logic;
    signal r_gcount_w2: std_logic;
    signal r_gcount_w23: std_logic;
    signal r_gcount_w3: std_logic;
    signal r_gcount_w24: std_logic;
    signal r_gcount_w4: std_logic;
    signal r_gcount_w25: std_logic;
    signal r_gcount_w5: std_logic;
    signal empty_i: std_logic;
    signal rRst: std_logic;
    signal full_i: std_logic;
    signal iwcount_0: std_logic;
    signal iwcount_1: std_logic;
    signal w_gctr_ci: std_logic;
    signal iwcount_2: std_logic;
    signal iwcount_3: std_logic;
    signal co0: std_logic;
    signal iwcount_4: std_logic;
    signal iwcount_5: std_logic;
    signal co2: std_logic;
    signal co1: std_logic;
    signal wcount_5: std_logic;
    signal scuba_vhi: std_logic;
    signal ircount_0: std_logic;
    signal ircount_1: std_logic;
    signal r_gctr_ci: std_logic;
    signal ircount_2: std_logic;
    signal ircount_3: std_logic;
    signal co0_1: std_logic;
    signal ircount_4: std_logic;
    signal ircount_5: std_logic;
    signal co2_1: std_logic;
    signal co1_1: std_logic;
    signal rcount_5: std_logic;
    signal rdataout7: std_logic;
    signal rdataout6: std_logic;
    signal rdataout5: std_logic;
    signal rdataout4: std_logic;
    signal rdataout3: std_logic;
    signal rdataout2: std_logic;
    signal rdataout1: std_logic;
    signal rdataout0: std_logic;
    signal rptr_4: std_logic;
    signal rden_i: std_logic;
    signal cmp_ci: std_logic;
    signal wcount_r0: std_logic;
    signal wcount_r1: std_logic;
    signal rcount_0: std_logic;
    signal rcount_1: std_logic;
    signal co0_2: std_logic;
    signal w_g2b_xor_cluster_0: std_logic;
    signal wcount_r3: std_logic;
    signal rcount_2: std_logic;
    signal rcount_3: std_logic;
    signal co1_2: std_logic;
    signal wcount_r4: std_logic;
    signal empty_cmp_clr: std_logic;
    signal rcount_4: std_logic;
    signal empty_cmp_set: std_logic;
    signal empty_d: std_logic;
    signal empty_d_c: std_logic;
    signal wren_i: std_logic;
    signal cmp_ci_1: std_logic;
    signal rcount_w0: std_logic;
    signal rcount_w1: std_logic;
    signal wcount_0: std_logic;
    signal wcount_1: std_logic;
    signal co0_3: std_logic;
    signal r_g2b_xor_cluster_0: std_logic;
    signal rcount_w3: std_logic;
    signal wcount_2: std_logic;
    signal wcount_3: std_logic;
    signal co1_3: std_logic;
    signal rcount_w4: std_logic;
    signal full_cmp_clr: std_logic;
    signal wcount_4: std_logic;
    signal full_cmp_set: std_logic;
    signal full_d: std_logic;
    signal full_d_c: std_logic;
    signal scuba_vlo: std_logic;
    signal mdL0_0_7: std_logic;
    signal mdL0_0_6: std_logic;
    signal mdL0_0_5: std_logic;
    signal mdL0_0_4: std_logic;
    signal mdL0_0_3: std_logic;
    signal mdL0_0_2: std_logic;
    signal mdL0_0_1: std_logic;
    signal mdL0_0_0: std_logic;
    signal dec0_wre3: std_logic;
    signal mdL0_1_7: std_logic;
    signal mdL0_1_6: std_logic;
    signal mdL0_1_5: std_logic;
    signal mdL0_1_4: std_logic;
    signal mdL0_1_3: std_logic;
    signal mdL0_1_2: std_logic;
    signal mdL0_1_1: std_logic;
    signal mdL0_1_0: std_logic;
    signal rptr_3: std_logic;
    signal rptr_2: std_logic;
    signal rptr_1: std_logic;
    signal rptr_0: std_logic;
    signal dec1_wre7: std_logic;
    signal wptr_3: std_logic;
    signal wptr_2: std_logic;
    signal wptr_1: std_logic;
    signal wptr_0: std_logic;

    -- local component declarations
    component AGEB2
        port (A0: in  std_logic; A1: in  std_logic; B0: in  std_logic; 
            B1: in  std_logic; CI: in  std_logic; GE: out  std_logic);
    end component;
    component AND2
        port (A: in  std_logic; B: in  std_logic; Z: out  std_logic);
    end component;
    component CU2
        port (CI: in  std_logic; PC0: in  std_logic; PC1: in  std_logic; 
            CO: out  std_logic; NC0: out  std_logic; NC1: out  std_logic);
    end component;
    component FADD2B
        port (A0: in  std_logic; A1: in  std_logic; B0: in  std_logic; 
            B1: in  std_logic; CI: in  std_logic; COUT: out  std_logic; 
            S0: out  std_logic; S1: out  std_logic);
    end component;
    component FD1P3BX
        port (D: in  std_logic; SP: in  std_logic; CK: in  std_logic; 
            PD: in  std_logic; Q: out  std_logic);
    end component;
    component FD1P3DX
        port (D: in  std_logic; SP: in  std_logic; CK: in  std_logic; 
            CD: in  std_logic; Q: out  std_logic);
    end component;
    component FD1S3BX
        port (D: in  std_logic; CK: in  std_logic; PD: in  std_logic; 
            Q: out  std_logic);
    end component;
    component FD1S3DX
        port (D: in  std_logic; CK: in  std_logic; CD: in  std_logic; 
            Q: out  std_logic);
    end component;
    component INV
        port (A: in  std_logic; Z: out  std_logic);
    end component;
    component MUX21
        port (D0: in  std_logic; D1: in  std_logic; SD: in  std_logic; 
            Z: out  std_logic);
    end component;
    component OR2
        port (A: in  std_logic; B: in  std_logic; Z: out  std_logic);
    end component;
    component ROM16X1A
        generic (INITVAL : in std_logic_vector(15 downto 0));
        port (AD3: in  std_logic; AD2: in  std_logic; AD1: in  std_logic; 
            AD0: in  std_logic; DO0: out  std_logic);
    end component;
    component DPR16X4C
        generic (INITVAL : in String);
        port (DI0: in  std_logic; DI1: in  std_logic; DI2: in  std_logic; 
            DI3: in  std_logic; WCK: in  std_logic; WRE: in  std_logic; 
            RAD0: in  std_logic; RAD1: in  std_logic; 
            RAD2: in  std_logic; RAD3: in  std_logic; 
            WAD0: in  std_logic; WAD1: in  std_logic; 
            WAD2: in  std_logic; WAD3: in  std_logic; 
            DO0: out  std_logic; DO1: out  std_logic; 
            DO2: out  std_logic; DO3: out  std_logic);
    end component;
    component VHI
        port (Z: out  std_logic);
    end component;
    component VLO
        port (Z: out  std_logic);
    end component;
    component XOR2
        port (A: in  std_logic; B: in  std_logic; Z: out  std_logic);
    end component;
    attribute GSR : string; 
    attribute MEM_INIT_FILE : string; 
    attribute MEM_LPC_FILE : string; 
    attribute COMP : string; 
    attribute GSR of FF_69 : label is "ENABLED";
    attribute GSR of FF_68 : label is "ENABLED";
    attribute GSR of FF_67 : label is "ENABLED";
    attribute GSR of FF_66 : label is "ENABLED";
    attribute GSR of FF_65 : label is "ENABLED";
    attribute GSR of FF_64 : label is "ENABLED";
    attribute GSR of FF_63 : label is "ENABLED";
    attribute GSR of FF_62 : label is "ENABLED";
    attribute GSR of FF_61 : label is "ENABLED";
    attribute GSR of FF_60 : label is "ENABLED";
    attribute GSR of FF_59 : label is "ENABLED";
    attribute GSR of FF_58 : label is "ENABLED";
    attribute GSR of FF_57 : label is "ENABLED";
    attribute GSR of FF_56 : label is "ENABLED";
    attribute GSR of FF_55 : label is "ENABLED";
    attribute GSR of FF_54 : label is "ENABLED";
    attribute GSR of FF_53 : label is "ENABLED";
    attribute GSR of FF_52 : label is "ENABLED";
    attribute GSR of FF_51 : label is "ENABLED";
    attribute GSR of FF_50 : label is "ENABLED";
    attribute GSR of FF_49 : label is "ENABLED";
    attribute GSR of FF_48 : label is "ENABLED";
    attribute GSR of FF_47 : label is "ENABLED";
    attribute GSR of FF_46 : label is "ENABLED";
    attribute GSR of FF_45 : label is "ENABLED";
    attribute GSR of FF_44 : label is "ENABLED";
    attribute GSR of FF_43 : label is "ENABLED";
    attribute GSR of FF_42 : label is "ENABLED";
    attribute GSR of FF_41 : label is "ENABLED";
    attribute GSR of FF_40 : label is "ENABLED";
    attribute GSR of FF_39 : label is "ENABLED";
    attribute GSR of FF_38 : label is "ENABLED";
    attribute GSR of FF_37 : label is "ENABLED";
    attribute GSR of FF_36 : label is "ENABLED";
    attribute GSR of FF_35 : label is "ENABLED";
    attribute GSR of FF_34 : label is "ENABLED";
    attribute GSR of FF_33 : label is "ENABLED";
    attribute GSR of FF_32 : label is "ENABLED";
    attribute GSR of FF_31 : label is "ENABLED";
    attribute GSR of FF_30 : label is "ENABLED";
    attribute GSR of FF_29 : label is "ENABLED";
    attribute GSR of FF_28 : label is "ENABLED";
    attribute GSR of FF_27 : label is "ENABLED";
    attribute GSR of FF_26 : label is "ENABLED";
    attribute GSR of FF_25 : label is "ENABLED";
    attribute GSR of FF_24 : label is "ENABLED";
    attribute GSR of FF_23 : label is "ENABLED";
    attribute GSR of FF_22 : label is "ENABLED";
    attribute GSR of FF_21 : label is "ENABLED";
    attribute GSR of FF_20 : label is "ENABLED";
    attribute GSR of FF_19 : label is "ENABLED";
    attribute GSR of FF_18 : label is "ENABLED";
    attribute GSR of FF_17 : label is "ENABLED";
    attribute GSR of FF_16 : label is "ENABLED";
    attribute GSR of FF_15 : label is "ENABLED";
    attribute GSR of FF_14 : label is "ENABLED";
    attribute GSR of FF_13 : label is "ENABLED";
    attribute GSR of FF_12 : label is "ENABLED";
    attribute GSR of FF_11 : label is "ENABLED";
    attribute GSR of FF_10 : label is "ENABLED";
    attribute GSR of FF_9 : label is "ENABLED";
    attribute GSR of FF_8 : label is "ENABLED";
    attribute GSR of FF_7 : label is "ENABLED";
    attribute GSR of FF_6 : label is "ENABLED";
    attribute GSR of FF_5 : label is "ENABLED";
    attribute GSR of FF_4 : label is "ENABLED";
    attribute GSR of FF_3 : label is "ENABLED";
    attribute GSR of FF_2 : label is "ENABLED";
    attribute GSR of FF_1 : label is "ENABLED";
    attribute GSR of FF_0 : label is "ENABLED";
    attribute MEM_INIT_FILE of fifo_pfu_0_0 : label is "(0-15)(0-3)";
    attribute MEM_LPC_FILE of fifo_pfu_0_0 : label is "FIFO8.lpc";
    attribute COMP of fifo_pfu_0_0 : label is "fifo_pfu_0_0";
    attribute MEM_INIT_FILE of fifo_pfu_0_1 : label is "(0-15)(4-7)";
    attribute MEM_LPC_FILE of fifo_pfu_0_1 : label is "FIFO8.lpc";
    attribute COMP of fifo_pfu_0_1 : label is "fifo_pfu_0_1";
    attribute MEM_INIT_FILE of fifo_pfu_1_0 : label is "(16-31)(0-3)";
    attribute MEM_LPC_FILE of fifo_pfu_1_0 : label is "FIFO8.lpc";
    attribute COMP of fifo_pfu_1_0 : label is "fifo_pfu_1_0";
    attribute MEM_INIT_FILE of fifo_pfu_1_1 : label is "(16-31)(4-7)";
    attribute MEM_LPC_FILE of fifo_pfu_1_1 : label is "FIFO8.lpc";
    attribute COMP of fifo_pfu_1_1 : label is "fifo_pfu_1_1";
    attribute syn_keep : boolean;
    attribute NGD_DRC_MASK : integer;
    attribute NGD_DRC_MASK of Structure : architecture is 1;

begin
    -- component instantiation statements
    AND2_t12: AND2
        port map (A=>WrEn, B=>invout_1, Z=>wren_i);

    INV_2: INV
        port map (A=>full_i, Z=>invout_1);

    AND2_t11: AND2
        port map (A=>RdEn, B=>invout_0, Z=>rden_i);

    INV_1: INV
        port map (A=>empty_i, Z=>invout_0);

    OR2_t10: OR2
        port map (A=>Reset, B=>RPReset, Z=>rRst);

    XOR2_t9: XOR2
        port map (A=>wcount_0, B=>wcount_1, Z=>w_gdata_0);

    XOR2_t8: XOR2
        port map (A=>wcount_1, B=>wcount_2, Z=>w_gdata_1);

    XOR2_t7: XOR2
        port map (A=>wcount_2, B=>wcount_3, Z=>w_gdata_2);

    XOR2_t6: XOR2
        port map (A=>wcount_3, B=>wcount_4, Z=>w_gdata_3);

    XOR2_t5: XOR2
        port map (A=>wcount_4, B=>wcount_5, Z=>w_gdata_4);

    XOR2_t4: XOR2
        port map (A=>rcount_0, B=>rcount_1, Z=>r_gdata_0);

    XOR2_t3: XOR2
        port map (A=>rcount_1, B=>rcount_2, Z=>r_gdata_1);

    XOR2_t2: XOR2
        port map (A=>rcount_2, B=>rcount_3, Z=>r_gdata_2);

    XOR2_t1: XOR2
        port map (A=>rcount_3, B=>rcount_4, Z=>r_gdata_3);

    XOR2_t0: XOR2
        port map (A=>rcount_4, B=>rcount_5, Z=>r_gdata_4);

    INV_0: INV
        port map (A=>wptr_4, Z=>wptr_4_inv);

    LUT4_15: ROM16X1A
        generic map (initval=> X"8000")
        port map (AD3=>scuba_vhi, AD2=>wren_i, AD1=>wptr_4_inv, 
            AD0=>scuba_vhi, DO0=>dec0_wre3);

    LUT4_14: ROM16X1A
        generic map (initval=> X"8000")
        port map (AD3=>scuba_vhi, AD2=>wren_i, AD1=>wptr_4, 
            AD0=>scuba_vhi, DO0=>dec1_wre7);

    LUT4_13: ROM16X1A
        generic map (initval=> X"6996")
        port map (AD3=>w_gcount_r22, AD2=>w_gcount_r23, 
            AD1=>w_gcount_r24, AD0=>w_gcount_r25, 
            DO0=>w_g2b_xor_cluster_0);

    LUT4_12: ROM16X1A
        generic map (initval=> X"6996")
        port map (AD3=>w_gcount_r24, AD2=>w_gcount_r25, AD1=>scuba_vlo, 
            AD0=>scuba_vlo, DO0=>wcount_r4);

    LUT4_11: ROM16X1A
        generic map (initval=> X"6996")
        port map (AD3=>w_gcount_r23, AD2=>w_gcount_r24, 
            AD1=>w_gcount_r25, AD0=>scuba_vlo, DO0=>wcount_r3);

    LUT4_10: ROM16X1A
        generic map (initval=> X"6996")
        port map (AD3=>w_gcount_r21, AD2=>w_gcount_r22, 
            AD1=>w_gcount_r23, AD0=>wcount_r4, DO0=>wcount_r1);

    LUT4_9: ROM16X1A
        generic map (initval=> X"6996")
        port map (AD3=>w_gcount_r20, AD2=>w_gcount_r21, 
            AD1=>w_gcount_r22, AD0=>wcount_r3, DO0=>wcount_r0);

    LUT4_8: ROM16X1A
        generic map (initval=> X"6996")
        port map (AD3=>r_gcount_w22, AD2=>r_gcount_w23, 
            AD1=>r_gcount_w24, AD0=>r_gcount_w25, 
            DO0=>r_g2b_xor_cluster_0);

    LUT4_7: ROM16X1A
        generic map (initval=> X"6996")
        port map (AD3=>r_gcount_w24, AD2=>r_gcount_w25, AD1=>scuba_vlo, 
            AD0=>scuba_vlo, DO0=>rcount_w4);

    LUT4_6: ROM16X1A
        generic map (initval=> X"6996")
        port map (AD3=>r_gcount_w23, AD2=>r_gcount_w24, 
            AD1=>r_gcount_w25, AD0=>scuba_vlo, DO0=>rcount_w3);

    LUT4_5: ROM16X1A
        generic map (initval=> X"6996")
        port map (AD3=>r_gcount_w21, AD2=>r_gcount_w22, 
            AD1=>r_gcount_w23, AD0=>rcount_w4, DO0=>rcount_w1);

    LUT4_4: ROM16X1A
        generic map (initval=> X"6996")
        port map (AD3=>r_gcount_w20, AD2=>r_gcount_w21, 
            AD1=>r_gcount_w22, AD0=>rcount_w3, DO0=>rcount_w0);

    LUT4_3: ROM16X1A
        generic map (initval=> X"0410")
        port map (AD3=>rptr_5, AD2=>rcount_5, AD1=>w_gcount_r25, 
            AD0=>scuba_vlo, DO0=>empty_cmp_set);

    LUT4_2: ROM16X1A
        generic map (initval=> X"1004")
        port map (AD3=>rptr_5, AD2=>rcount_5, AD1=>w_gcount_r25, 
            AD0=>scuba_vlo, DO0=>empty_cmp_clr);

    LUT4_1: ROM16X1A
        generic map (initval=> X"0140")
        port map (AD3=>wptr_5, AD2=>wcount_5, AD1=>r_gcount_w25, 
            AD0=>scuba_vlo, DO0=>full_cmp_set);

    LUT4_0: ROM16X1A
        generic map (initval=> X"4001")
        port map (AD3=>wptr_5, AD2=>wcount_5, AD1=>r_gcount_w25, 
            AD0=>scuba_vlo, DO0=>full_cmp_clr);

    FF_69: FD1P3BX
        port map (D=>iwcount_0, SP=>wren_i, CK=>WrClock, PD=>Reset, 
            Q=>wcount_0);

    FF_68: FD1P3DX
        port map (D=>iwcount_1, SP=>wren_i, CK=>WrClock, CD=>Reset, 
            Q=>wcount_1);

    FF_67: FD1P3DX
        port map (D=>iwcount_2, SP=>wren_i, CK=>WrClock, CD=>Reset, 
            Q=>wcount_2);

    FF_66: FD1P3DX
        port map (D=>iwcount_3, SP=>wren_i, CK=>WrClock, CD=>Reset, 
            Q=>wcount_3);

    FF_65: FD1P3DX
        port map (D=>iwcount_4, SP=>wren_i, CK=>WrClock, CD=>Reset, 
            Q=>wcount_4);

    FF_64: FD1P3DX
        port map (D=>iwcount_5, SP=>wren_i, CK=>WrClock, CD=>Reset, 
            Q=>wcount_5);

    FF_63: FD1P3DX
        port map (D=>w_gdata_0, SP=>wren_i, CK=>WrClock, CD=>Reset, 
            Q=>w_gcount_0);

    FF_62: FD1P3DX
        port map (D=>w_gdata_1, SP=>wren_i, CK=>WrClock, CD=>Reset, 
            Q=>w_gcount_1);

    FF_61: FD1P3DX
        port map (D=>w_gdata_2, SP=>wren_i, CK=>WrClock, CD=>Reset, 
            Q=>w_gcount_2);

    FF_60: FD1P3DX
        port map (D=>w_gdata_3, SP=>wren_i, CK=>WrClock, CD=>Reset, 
            Q=>w_gcount_3);

    FF_59: FD1P3DX
        port map (D=>w_gdata_4, SP=>wren_i, CK=>WrClock, CD=>Reset, 
            Q=>w_gcount_4);

    FF_58: FD1P3DX
        port map (D=>wcount_5, SP=>wren_i, CK=>WrClock, CD=>Reset, 
            Q=>w_gcount_5);

    FF_57: FD1P3DX
        port map (D=>wcount_0, SP=>wren_i, CK=>WrClock, CD=>Reset, 
            Q=>wptr_0);

    FF_56: FD1P3DX
        port map (D=>wcount_1, SP=>wren_i, CK=>WrClock, CD=>Reset, 
            Q=>wptr_1);

    FF_55: FD1P3DX
        port map (D=>wcount_2, SP=>wren_i, CK=>WrClock, CD=>Reset, 
            Q=>wptr_2);

    FF_54: FD1P3DX
        port map (D=>wcount_3, SP=>wren_i, CK=>WrClock, CD=>Reset, 
            Q=>wptr_3);

    FF_53: FD1P3DX
        port map (D=>wcount_4, SP=>wren_i, CK=>WrClock, CD=>Reset, 
            Q=>wptr_4);

    FF_52: FD1P3DX
        port map (D=>wcount_5, SP=>wren_i, CK=>WrClock, CD=>Reset, 
            Q=>wptr_5);

    FF_51: FD1P3BX
        port map (D=>ircount_0, SP=>rden_i, CK=>RdClock, PD=>rRst, 
            Q=>rcount_0);

    FF_50: FD1P3DX
        port map (D=>ircount_1, SP=>rden_i, CK=>RdClock, CD=>rRst, 
            Q=>rcount_1);

    FF_49: FD1P3DX
        port map (D=>ircount_2, SP=>rden_i, CK=>RdClock, CD=>rRst, 
            Q=>rcount_2);

    FF_48: FD1P3DX
        port map (D=>ircount_3, SP=>rden_i, CK=>RdClock, CD=>rRst, 
            Q=>rcount_3);

    FF_47: FD1P3DX
        port map (D=>ircount_4, SP=>rden_i, CK=>RdClock, CD=>rRst, 
            Q=>rcount_4);

    FF_46: FD1P3DX
        port map (D=>ircount_5, SP=>rden_i, CK=>RdClock, CD=>rRst, 
            Q=>rcount_5);

    FF_45: FD1P3DX
        port map (D=>r_gdata_0, SP=>rden_i, CK=>RdClock, CD=>rRst, 
            Q=>r_gcount_0);

    FF_44: FD1P3DX
        port map (D=>r_gdata_1, SP=>rden_i, CK=>RdClock, CD=>rRst, 
            Q=>r_gcount_1);

    FF_43: FD1P3DX
        port map (D=>r_gdata_2, SP=>rden_i, CK=>RdClock, CD=>rRst, 
            Q=>r_gcount_2);

    FF_42: FD1P3DX
        port map (D=>r_gdata_3, SP=>rden_i, CK=>RdClock, CD=>rRst, 
            Q=>r_gcount_3);

    FF_41: FD1P3DX
        port map (D=>r_gdata_4, SP=>rden_i, CK=>RdClock, CD=>rRst, 
            Q=>r_gcount_4);

    FF_40: FD1P3DX
        port map (D=>rcount_5, SP=>rden_i, CK=>RdClock, CD=>rRst, 
            Q=>r_gcount_5);

    FF_39: FD1P3DX
        port map (D=>rcount_0, SP=>rden_i, CK=>RdClock, CD=>rRst, 
            Q=>rptr_0);

    FF_38: FD1P3DX
        port map (D=>rcount_1, SP=>rden_i, CK=>RdClock, CD=>rRst, 
            Q=>rptr_1);

    FF_37: FD1P3DX
        port map (D=>rcount_2, SP=>rden_i, CK=>RdClock, CD=>rRst, 
            Q=>rptr_2);

    FF_36: FD1P3DX
        port map (D=>rcount_3, SP=>rden_i, CK=>RdClock, CD=>rRst, 
            Q=>rptr_3);

    FF_35: FD1P3DX
        port map (D=>rcount_4, SP=>rden_i, CK=>RdClock, CD=>rRst, 
            Q=>rptr_4);

    FF_34: FD1P3DX
        port map (D=>rcount_5, SP=>rden_i, CK=>RdClock, CD=>rRst, 
            Q=>rptr_5);

    FF_33: FD1P3DX
        port map (D=>rdataout0, SP=>rden_i, CK=>RdClock, CD=>rRst, 
            Q=>Q(0));

    FF_32: FD1P3DX
        port map (D=>rdataout1, SP=>rden_i, CK=>RdClock, CD=>rRst, 
            Q=>Q(1));

    FF_31: FD1P3DX
        port map (D=>rdataout2, SP=>rden_i, CK=>RdClock, CD=>rRst, 
            Q=>Q(2));

    FF_30: FD1P3DX
        port map (D=>rdataout3, SP=>rden_i, CK=>RdClock, CD=>rRst, 
            Q=>Q(3));

    FF_29: FD1P3DX
        port map (D=>rdataout4, SP=>rden_i, CK=>RdClock, CD=>rRst, 
            Q=>Q(4));

    FF_28: FD1P3DX
        port map (D=>rdataout5, SP=>rden_i, CK=>RdClock, CD=>rRst, 
            Q=>Q(5));

    FF_27: FD1P3DX
        port map (D=>rdataout6, SP=>rden_i, CK=>RdClock, CD=>rRst, 
            Q=>Q(6));

    FF_26: FD1P3DX
        port map (D=>rdataout7, SP=>rden_i, CK=>RdClock, CD=>rRst, 
            Q=>Q(7));

    FF_25: FD1S3DX
        port map (D=>w_gcount_0, CK=>RdClock, CD=>Reset, Q=>w_gcount_r0);

    FF_24: FD1S3DX
        port map (D=>w_gcount_1, CK=>RdClock, CD=>Reset, Q=>w_gcount_r1);

    FF_23: FD1S3DX
        port map (D=>w_gcount_2, CK=>RdClock, CD=>Reset, Q=>w_gcount_r2);

    FF_22: FD1S3DX
        port map (D=>w_gcount_3, CK=>RdClock, CD=>Reset, Q=>w_gcount_r3);

    FF_21: FD1S3DX
        port map (D=>w_gcount_4, CK=>RdClock, CD=>Reset, Q=>w_gcount_r4);

    FF_20: FD1S3DX
        port map (D=>w_gcount_5, CK=>RdClock, CD=>Reset, Q=>w_gcount_r5);

    FF_19: FD1S3DX
        port map (D=>r_gcount_0, CK=>WrClock, CD=>rRst, Q=>r_gcount_w0);

    FF_18: FD1S3DX
        port map (D=>r_gcount_1, CK=>WrClock, CD=>rRst, Q=>r_gcount_w1);

    FF_17: FD1S3DX
        port map (D=>r_gcount_2, CK=>WrClock, CD=>rRst, Q=>r_gcount_w2);

    FF_16: FD1S3DX
        port map (D=>r_gcount_3, CK=>WrClock, CD=>rRst, Q=>r_gcount_w3);

    FF_15: FD1S3DX
        port map (D=>r_gcount_4, CK=>WrClock, CD=>rRst, Q=>r_gcount_w4);

    FF_14: FD1S3DX
        port map (D=>r_gcount_5, CK=>WrClock, CD=>rRst, Q=>r_gcount_w5);

    FF_13: FD1S3DX
        port map (D=>w_gcount_r0, CK=>RdClock, CD=>Reset, 
            Q=>w_gcount_r20);

    FF_12: FD1S3DX
        port map (D=>w_gcount_r1, CK=>RdClock, CD=>Reset, 
            Q=>w_gcount_r21);

    FF_11: FD1S3DX
        port map (D=>w_gcount_r2, CK=>RdClock, CD=>Reset, 
            Q=>w_gcount_r22);

    FF_10: FD1S3DX
        port map (D=>w_gcount_r3, CK=>RdClock, CD=>Reset, 
            Q=>w_gcount_r23);

    FF_9: FD1S3DX
        port map (D=>w_gcount_r4, CK=>RdClock, CD=>Reset, 
            Q=>w_gcount_r24);

    FF_8: FD1S3DX
        port map (D=>w_gcount_r5, CK=>RdClock, CD=>Reset, 
            Q=>w_gcount_r25);

    FF_7: FD1S3DX
        port map (D=>r_gcount_w0, CK=>WrClock, CD=>rRst, Q=>r_gcount_w20);

    FF_6: FD1S3DX
        port map (D=>r_gcount_w1, CK=>WrClock, CD=>rRst, Q=>r_gcount_w21);

    FF_5: FD1S3DX
        port map (D=>r_gcount_w2, CK=>WrClock, CD=>rRst, Q=>r_gcount_w22);

    FF_4: FD1S3DX
        port map (D=>r_gcount_w3, CK=>WrClock, CD=>rRst, Q=>r_gcount_w23);

    FF_3: FD1S3DX
        port map (D=>r_gcount_w4, CK=>WrClock, CD=>rRst, Q=>r_gcount_w24);

    FF_2: FD1S3DX
        port map (D=>r_gcount_w5, CK=>WrClock, CD=>rRst, Q=>r_gcount_w25);

    FF_1: FD1S3BX
        port map (D=>empty_d, CK=>RdClock, PD=>rRst, Q=>empty_i);

    FF_0: FD1S3DX
        port map (D=>full_d, CK=>WrClock, CD=>Reset, Q=>full_i);

    w_gctr_cia: FADD2B
        port map (A0=>scuba_vlo, A1=>scuba_vhi, B0=>scuba_vlo, 
            B1=>scuba_vhi, CI=>scuba_vlo, COUT=>w_gctr_ci, S0=>open, 
            S1=>open);

    w_gctr_0: CU2
        port map (CI=>w_gctr_ci, PC0=>wcount_0, PC1=>wcount_1, CO=>co0, 
            NC0=>iwcount_0, NC1=>iwcount_1);

    w_gctr_1: CU2
        port map (CI=>co0, PC0=>wcount_2, PC1=>wcount_3, CO=>co1, 
            NC0=>iwcount_2, NC1=>iwcount_3);

    w_gctr_2: CU2
        port map (CI=>co1, PC0=>wcount_4, PC1=>wcount_5, CO=>co2, 
            NC0=>iwcount_4, NC1=>iwcount_5);

    scuba_vhi_inst: VHI
        port map (Z=>scuba_vhi);

    r_gctr_cia: FADD2B
        port map (A0=>scuba_vlo, A1=>scuba_vhi, B0=>scuba_vlo, 
            B1=>scuba_vhi, CI=>scuba_vlo, COUT=>r_gctr_ci, S0=>open, 
            S1=>open);

    r_gctr_0: CU2
        port map (CI=>r_gctr_ci, PC0=>rcount_0, PC1=>rcount_1, CO=>co0_1, 
            NC0=>ircount_0, NC1=>ircount_1);

    r_gctr_1: CU2
        port map (CI=>co0_1, PC0=>rcount_2, PC1=>rcount_3, CO=>co1_1, 
            NC0=>ircount_2, NC1=>ircount_3);

    r_gctr_2: CU2
        port map (CI=>co1_1, PC0=>rcount_4, PC1=>rcount_5, CO=>co2_1, 
            NC0=>ircount_4, NC1=>ircount_5);

    mux_7: MUX21
        port map (D0=>mdL0_0_7, D1=>mdL0_1_7, SD=>rptr_4, Z=>rdataout7);

    mux_6: MUX21
        port map (D0=>mdL0_0_6, D1=>mdL0_1_6, SD=>rptr_4, Z=>rdataout6);

    mux_5: MUX21
        port map (D0=>mdL0_0_5, D1=>mdL0_1_5, SD=>rptr_4, Z=>rdataout5);

    mux_4: MUX21
        port map (D0=>mdL0_0_4, D1=>mdL0_1_4, SD=>rptr_4, Z=>rdataout4);

    mux_3: MUX21
        port map (D0=>mdL0_0_3, D1=>mdL0_1_3, SD=>rptr_4, Z=>rdataout3);

    mux_2: MUX21
        port map (D0=>mdL0_0_2, D1=>mdL0_1_2, SD=>rptr_4, Z=>rdataout2);

    mux_1: MUX21
        port map (D0=>mdL0_0_1, D1=>mdL0_1_1, SD=>rptr_4, Z=>rdataout1);

    mux_0: MUX21
        port map (D0=>mdL0_0_0, D1=>mdL0_1_0, SD=>rptr_4, Z=>rdataout0);

    empty_cmp_ci_a: FADD2B
        port map (A0=>scuba_vlo, A1=>rden_i, B0=>scuba_vlo, B1=>rden_i, 
            CI=>scuba_vlo, COUT=>cmp_ci, S0=>open, S1=>open);

    empty_cmp_0: AGEB2
        port map (A0=>rcount_0, A1=>rcount_1, B0=>wcount_r0, 
            B1=>wcount_r1, CI=>cmp_ci, GE=>co0_2);

    empty_cmp_1: AGEB2
        port map (A0=>rcount_2, A1=>rcount_3, B0=>w_g2b_xor_cluster_0, 
            B1=>wcount_r3, CI=>co0_2, GE=>co1_2);

    empty_cmp_2: AGEB2
        port map (A0=>rcount_4, A1=>empty_cmp_set, B0=>wcount_r4, 
            B1=>empty_cmp_clr, CI=>co1_2, GE=>empty_d_c);

    a0: FADD2B
        port map (A0=>scuba_vlo, A1=>scuba_vlo, B0=>scuba_vlo, 
            B1=>scuba_vlo, CI=>empty_d_c, COUT=>open, S0=>empty_d, 
            S1=>open);

    full_cmp_ci_a: FADD2B
        port map (A0=>scuba_vlo, A1=>wren_i, B0=>scuba_vlo, B1=>wren_i, 
            CI=>scuba_vlo, COUT=>cmp_ci_1, S0=>open, S1=>open);

    full_cmp_0: AGEB2
        port map (A0=>wcount_0, A1=>wcount_1, B0=>rcount_w0, 
            B1=>rcount_w1, CI=>cmp_ci_1, GE=>co0_3);

    full_cmp_1: AGEB2
        port map (A0=>wcount_2, A1=>wcount_3, B0=>r_g2b_xor_cluster_0, 
            B1=>rcount_w3, CI=>co0_3, GE=>co1_3);

    full_cmp_2: AGEB2
        port map (A0=>wcount_4, A1=>full_cmp_set, B0=>rcount_w4, 
            B1=>full_cmp_clr, CI=>co1_3, GE=>full_d_c);

    scuba_vlo_inst: VLO
        port map (Z=>scuba_vlo);

    a1: FADD2B
        port map (A0=>scuba_vlo, A1=>scuba_vlo, B0=>scuba_vlo, 
            B1=>scuba_vlo, CI=>full_d_c, COUT=>open, S0=>full_d, 
            S1=>open);

    fifo_pfu_0_0: DPR16X4C
        generic map (initval=> "0x0000000000000000")
        port map (DI0=>Data(4), DI1=>Data(5), DI2=>Data(6), DI3=>Data(7), 
            WCK=>WrClock, WRE=>dec0_wre3, RAD0=>rptr_0, RAD1=>rptr_1, 
            RAD2=>rptr_2, RAD3=>rptr_3, WAD0=>wptr_0, WAD1=>wptr_1, 
            WAD2=>wptr_2, WAD3=>wptr_3, DO0=>mdL0_0_4, DO1=>mdL0_0_5, 
            DO2=>mdL0_0_6, DO3=>mdL0_0_7);

    fifo_pfu_0_1: DPR16X4C
        generic map (initval=> "0x0000000000000000")
        port map (DI0=>Data(0), DI1=>Data(1), DI2=>Data(2), DI3=>Data(3), 
            WCK=>WrClock, WRE=>dec0_wre3, RAD0=>rptr_0, RAD1=>rptr_1, 
            RAD2=>rptr_2, RAD3=>rptr_3, WAD0=>wptr_0, WAD1=>wptr_1, 
            WAD2=>wptr_2, WAD3=>wptr_3, DO0=>mdL0_0_0, DO1=>mdL0_0_1, 
            DO2=>mdL0_0_2, DO3=>mdL0_0_3);

    fifo_pfu_1_0: DPR16X4C
        generic map (initval=> "0x0000000000000000")
        port map (DI0=>Data(4), DI1=>Data(5), DI2=>Data(6), DI3=>Data(7), 
            WCK=>WrClock, WRE=>dec1_wre7, RAD0=>rptr_0, RAD1=>rptr_1, 
            RAD2=>rptr_2, RAD3=>rptr_3, WAD0=>wptr_0, WAD1=>wptr_1, 
            WAD2=>wptr_2, WAD3=>wptr_3, DO0=>mdL0_1_4, DO1=>mdL0_1_5, 
            DO2=>mdL0_1_6, DO3=>mdL0_1_7);

    fifo_pfu_1_1: DPR16X4C
        generic map (initval=> "0x0000000000000000")
        port map (DI0=>Data(0), DI1=>Data(1), DI2=>Data(2), DI3=>Data(3), 
            WCK=>WrClock, WRE=>dec1_wre7, RAD0=>rptr_0, RAD1=>rptr_1, 
            RAD2=>rptr_2, RAD3=>rptr_3, WAD0=>wptr_0, WAD1=>wptr_1, 
            WAD2=>wptr_2, WAD3=>wptr_3, DO0=>mdL0_1_0, DO1=>mdL0_1_1, 
            DO2=>mdL0_1_2, DO3=>mdL0_1_3);

    Empty <= empty_i;
    Full <= full_i;
end Structure;

-- synopsys translate_off
library MACHXO2;
configuration Structure_CON of FIFO8 is
    for Structure
        for all:AGEB2 use entity MACHXO2.AGEB2(V); end for;
        for all:AND2 use entity MACHXO2.AND2(V); end for;
        for all:CU2 use entity MACHXO2.CU2(V); end for;
        for all:FADD2B use entity MACHXO2.FADD2B(V); end for;
        for all:FD1P3BX use entity MACHXO2.FD1P3BX(V); end for;
        for all:FD1P3DX use entity MACHXO2.FD1P3DX(V); end for;
        for all:FD1S3BX use entity MACHXO2.FD1S3BX(V); end for;
        for all:FD1S3DX use entity MACHXO2.FD1S3DX(V); end for;
        for all:INV use entity MACHXO2.INV(V); end for;
        for all:MUX21 use entity MACHXO2.MUX21(V); end for;
        for all:OR2 use entity MACHXO2.OR2(V); end for;
        for all:ROM16X1A use entity MACHXO2.ROM16X1A(V); end for;
        for all:DPR16X4C use entity MACHXO2.DPR16X4C(V); end for;
        for all:VHI use entity MACHXO2.VHI(V); end for;
        for all:VLO use entity MACHXO2.VLO(V); end for;
        for all:XOR2 use entity MACHXO2.XOR2(V); end for;
    end for;
end Structure_CON;

-- synopsys translate_on
