--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   16:08:24 05/31/2022
-- Design Name:   
-- Module Name:   /media/dmishins/1AD0BEB7D0BE9909/Users/Daniel/Desktop/Research/DAPHNE/CONTROLLER/Controller_FPGA1_OTtest/sim/ControllerFPGA_1_tb.vhd
-- Project Name:  CTRL_iseproj
-- Target Device:  
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: ControllerFPGA_1
-- 
-- Dependencies:
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
--
-- Notes: 
-- This testbench has been automatically generated using types std_logic and
-- std_logic_vector for the ports of the unit under test.  Xilinx recommends
-- that these types always be used for the top-level I/O of a design in order
-- to guarantee that the testbench will bind correctly to the post-implementation 
-- simulation model.
--------------------------------------------------------------------------------
LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
 
-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--USE ieee.numeric_std.ALL;
 
ENTITY ControllerFPGA_1_tb IS
END ControllerFPGA_1_tb;
 
ARCHITECTURE behavior OF ControllerFPGA_1_tb IS 
 
    -- Component Declaration for the Unit Under Test (UUT)
 
    COMPONENT ControllerFPGA_1
    PORT(
         GigEx_Data : INOUT  std_logic_vector(15 downto 0);
         GigEx_Addr : OUT  std_logic_vector(8 downto 0);
         GigEx_nCS : OUT  std_logic;
         GigEx_nWE : OUT  std_logic;
         GigEx_Clk : OUT  std_logic;
         GigEx_nBE : OUT  std_logic_vector(1 downto 0);
         GigEx_EOF : IN  std_logic_vector(1 downto 0);
         GigEx_Length : IN  std_logic;
         ClkB_P : IN  std_logic;
         ClkB_N : IN  std_logic;
         CpldRst : IN  std_logic;
         CpldCS : IN  std_logic;
         uCRd : IN  std_logic;
         uCWr : IN  std_logic;
         Debug : OUT  std_logic_vector(10 downto 1)
        );
    END COMPONENT;
    

   --Inputs
   signal GigEx_EOF : std_logic_vector(1 downto 0) := (others => '0');
   signal GigEx_Length : std_logic := '0';
   signal ClkB_P : std_logic := '0';
   signal ClkB_N : std_logic := '0';
   signal CpldRst : std_logic := '0';
   signal CpldCS : std_logic := '0';
   signal uCRd : std_logic := '0';
   signal uCWr : std_logic := '0';

	--BiDirs
   signal GigEx_Data : std_logic_vector(15 downto 0);

 	--Outputs
   signal GigEx_Addr : std_logic_vector(8 downto 0);
   signal GigEx_nCS : std_logic;
   signal GigEx_nWE : std_logic;
   signal GigEx_Clk : std_logic;
   signal GigEx_nBE : std_logic_vector(1 downto 0);
   signal Debug : std_logic_vector(10 downto 1);

   -- Clock period definitions
   constant ClkB_P_period : time := 9.4 ns;
   constant ClkB_N_period : time := 9.4 ns;
 
BEGIN
 
	-- Instantiate the Unit Under Test (UUT)
   uut: ControllerFPGA_1 PORT MAP (
          GigEx_Data => GigEx_Data,
          GigEx_Addr => GigEx_Addr,
          GigEx_nCS => GigEx_nCS,
          GigEx_nWE => GigEx_nWE,
          GigEx_Clk => GigEx_Clk,
          GigEx_nBE => GigEx_nBE,
          GigEx_EOF => GigEx_EOF,
          GigEx_Length => GigEx_Length,
          ClkB_P => ClkB_P,
          ClkB_N => ClkB_N,
          CpldRst => CpldRst,
          CpldCS => CpldCS,
          uCRd => uCRd,
          uCWr => uCWr,
          Debug => Debug
        );

   -- Clock process definitions
 
   ClkB_P_process :process
   begin
		ClkB_P <= '0';
		wait for ClkB_P_period/2;
		ClkB_P <= '1';
		wait for ClkB_P_period/2;
   end process;
 
   ClkB_N_process :process
   begin
		ClkB_N <= '0';
		wait for ClkB_N_period/2;
		ClkB_N <= '1';
		wait for ClkB_N_period/2;
   end process;
 

   -- Stimulus process
   stim_proc: process
   begin		
      -- hold reset state for 100 ns.
		CpldRst <= '0';
		
      wait for 100 ns;	
		CpldRst <= '1';


      -- insert stimulus here 

      wait;
   end process;

END;
