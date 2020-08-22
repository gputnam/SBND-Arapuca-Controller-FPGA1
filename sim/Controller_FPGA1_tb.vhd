--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   17:11:02 10/23/2015
-- Design Name:   
-- Module Name:   C:/Experiments/mu2e/Readout_Controller/Controller_FPGA1/Controller_FPGA1_tb.vhd
-- Project Name:  Controller_FPGA1
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
use work.Project_defs.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;

USE ieee.numeric_std.ALL;
library UNISIM;  

use UNISIM.Vcomponents.all;
-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--USE ieee.numeric_std.ALL;

ENTITY Controller_FPGA1_tb IS
END Controller_FPGA1_tb;

ARCHITECTURE behavior OF Controller_FPGA1_tb IS 

-- Component Declaration for the Unit Under Test (UUT)

 COMPONENT ControllerFPGA_1
 PORT(
	ClkB_P,ClkB_N : in std_logic;
	DQ : inout std_logic_vector(15 downto 0);
	ZEthA : buffer std_logic_vector(8 downto 0);
	ZEthCS,ZEthWE,ZEthClk : buffer std_logic;
	ZEthBE : buffer std_logic_vector(1 downto 0);
	ZEthEOF : in std_logic_vector(1 downto 0);
	ZEthLen : in std_logic;
-- Debug port
	Debug : buffer std_logic_vector(10 downto 1));
	
 END COMPONENT;
begin
end behavior;