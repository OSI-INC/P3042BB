-- VHDL module instantiation generated by SCUBA Diamond (64-bit) 3.12.1.454
-- Module  Version: 5.4
-- Mon Nov 21 17:34:47 2022

-- parameterized module component declaration
component CPU_ROM
    port (Address: in  std_logic_vector(12 downto 0); 
        OutClock: in  std_logic; OutClockEn: in  std_logic; 
        Reset: in  std_logic; Q: out  std_logic_vector(7 downto 0));
end component;

-- parameterized module component instance
__ : CPU_ROM
    port map (Address(12 downto 0)=>__, OutClock=>__, OutClockEn=>__, 
        Reset=>__, Q(7 downto 0)=>__);
