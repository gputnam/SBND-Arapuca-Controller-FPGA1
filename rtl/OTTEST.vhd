-- Firmware for apex FPGA

-- Sten Hansen Fermilab 10/15/2015

-- FPGA responsible for collecting data from three front end FPGAs
-- Microcontroller interface, GBT transceiver interface to the DAQ

-- 10/15/15 microcontoller interface
-- 12/22/15 serial data receivers for data coming from the PHY FPGAs
-- 03/15/16 serializers for the front panel LEDs, PLL chip
-- 03/15/16 serializers for the front panel LEDs, PLL chip
-- 05/16/16 minimal GTP loop back demonstrated
-- 04/02/18 Setup Beam On/Beam Off Microbunch generator

----------------------------- Main Body of design -------------------------

LIBRARY ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;

Library UNISIM;
use UNISIM.vcomponents.all;

use work.Project_defs.all;

entity ControllerFPGA_1 is port(
	CpldRst, CpldCS, uCRd, uCWr, EthCS : in std_logic;
-- Orange Tree Ethernet daughter card lines
	DQ : inout std_logic_vector(15 downto 0);
	ZEthA : out std_logic_vector(8 downto 0);
	ZEthCS,ZEthWE,ZEthClk : out std_logic;
	ZEthBE : out std_logic_vector(1 downto 0);
	ZEthEOF : in std_logic_vector(1 downto 0);
	ZEthLen : in std_logic;

	ClkB_P,ClkB_N : in std_logic;
-- Debug port
	Debug : out std_logic_vector(10 downto 1)
);

end ControllerFPGA_1;

architecture arch of ControllerFPGA_1 is
component SysPll
port
 (-- Clock in ports
  CLK_IN1_P         : in     std_logic;
  CLK_IN1_N         : in     std_logic;
  -- Clock out ports
  CLK_OUT1          : out    std_logic;
  --CLK_OUT2          : out    std_logic;
  --CLK_OUT3          : out    std_logic;
  -- Status and control signals
  RESET             : in     std_logic;
  LOCKED            : out    std_logic
 );
end component;

    --------------------------------------------------------------------------
    -- Declare constants
    
    -- GigExpedite register addresses
    constant LOCAL_IP_ADDR_HIGH         : std_logic_vector(9 downto 0) := "1000000000";
    constant LOCAL_IP_ADDR_LOW          : std_logic_vector(9 downto 0) := "1000000010";
    constant LINK_STATUS                : std_logic_vector(9 downto 0) := "1000000100";
    
    -- Per channel registers
    constant LOCAL_PORT               : std_logic_vector(4 downto 0) := "00000";
    constant REMOTE_IP_ADDR_HIGH      : std_logic_vector(4 downto 0) := "00010";
    constant REMOTE_IP_ADDR_LOW       : std_logic_vector(4 downto 0) := "00100";
    constant REMOTE_PORT              : std_logic_vector(4 downto 0) := "00110";
    constant MTU_TTL                  : std_logic_vector(4 downto 0) := "01000";
    constant INTERRUPT_ENABLE_STATUS  : std_logic_vector(4 downto 0) := "01010";
    constant CONNECTION_STATE         : std_logic_vector(4 downto 0) := "01100";
    constant FRAME_LENGTH             : std_logic_vector(4 downto 0) := "01110";
    constant DATA_FIFO                : std_logic_vector(4 downto 0) := "10000";
    
    -- Connection states (for CONNECTION_STATE reg)
    constant CLOSED       : std_logic_vector(15 downto 0) := X"0000";
    constant LISTEN       : std_logic_vector(15 downto 0) := X"0001";
    constant CONNECT      : std_logic_vector(15 downto 0) := X"0002";
    constant ESTABLISHED  : std_logic_vector(15 downto 0) := X"0003";
    constant CONN_TCP     : std_logic_vector(15 downto 0) := X"0010";
    constant CONN_ENABLE  : std_logic_vector(15 downto 0) := X"0020";
    
    -- Interrupt enable and status bits (for INTERRUPT_ENABLE_STATUS reg)
    constant IE_INCOMING          : std_logic_vector(15 downto 0) := X"0001";
    constant IE_OUTGOING_EMPTY    : std_logic_vector(15 downto 0) := X"0002";
    constant IE_OUTGOING_NOT_FULL : std_logic_vector(15 downto 0) := X"0004";
    constant IE_STATE             : std_logic_vector(15 downto 0) := X"0008";

    -- Test transfer length
    constant WRITE_LENGTH : std_logic_vector(31 downto 0) := conv_std_logic_vector(500*1024*1024, 32);

    --------------------------------------------------------------------------
    -- Declare signals
    signal RefClk : std_logic;
    signal EthClk : std_logic;
    signal RST : std_logic;
	
	signal nullbit: std_logic;
	
    signal UserWE : std_logic;
    signal UserRE : std_logic;
    signal UserAddr : std_logic_vector(9 downto 0);
    signal UserBE : std_logic_vector(1 downto 0);
    signal UserWriteData : std_logic_vector(15 downto 0);
    signal UserReadData : std_logic_vector(15 downto 0);
    signal UserReadDataValid : std_logic;
    signal UserInterrupt : std_logic;

    type STATE_TYPE is (
        USER_FPGA_DELAY_INIT,
        USER_FPGA_CLEAN,
        USER_FPGA_CLEAN_CHECK,
        USER_FPGA_INIT,
        USER_FPGA_IDLE,
        USER_FPGA_CHECK_STATE,
        USER_FPGA_CHECK_SPACE,
        USER_FPGA_WRITE_DATA,
        USER_FPGA_WRITE_MAIL1,
        USER_FPGA_WRITE_MAIL2,
        USER_FPGA_READ_LENGTH,
		  USER_FPGA_READ_DATA,
		  USER_FPGA_READ_MAIL1,
		  USER_FPGA_WAIT1,
		  USER_FPGA_WAIT2,
		  USER_FPGA_WAIT3,
		  USER_FPGA_READ_MAIL2
    );
    signal UserFPGAState : STATE_TYPE;
    signal Delay : std_logic_vector(23 downto 0);
    signal UserFPGASubState : std_logic_vector(15 downto 0);
    signal UserValidCount : std_logic_vector(15 downto 0);
    signal ConnectionState : std_logic_vector(5 downto 0);
    signal InterruptStatus : std_logic_vector(15 downto 0);
    signal FrameLength : std_logic_vector(15 downto 0);
    signal Clean : std_logic;
    signal RampData : std_logic_vector(15 downto 0);
    signal WriteCount : std_logic_vector(31 downto 0);
    signal nRefClk, ClkDiv2 : std_logic := '0';
	 signal ResetHi, Pll_Locked : std_logic;
begin
--Debug (1) <= ZEthClk;
--Debug (2) <= ZEthCS;
--Debug (3) <= ZEthWE;
	--ZEthBE : buffer std_logic_vector(1 downto 0);
	--ZEthEOF : in std_logic_vector(1 downto 0);
	--ZEthLen : in std_logic;
nullbit <= '0';
--Debug <= (others => '0');

Sys_Pll : SysPll
  port map(
 -- Clock in ports
    CLK_IN1_P => ClkB_P,
    CLK_IN1_N => ClkB_N,
 -- Clock out ports
    CLK_OUT1 => RefClk,   -- 125 MHz
    --CLK_OUT2 =>  open,--nEthClk,  -- 160 MHz 180 deg. phase
	 --CLK_OUT3 =>  open, --EthClk,   -- 160 MHz used for Orange Tree I/O
 -- Status and control signals
    RESET  => ResetHi,
    LOCKED => Pll_Locked
);
	 
	 ODDR2_inst : ODDR2
generic map(
DDR_ALIGNMENT => "NONE", -- Sets output alignment to "NONE", "C0", "C1"
INIT => '0', -- Sets initial state of the Q output to '0' or '1'
SRTYPE => "SYNC") -- Specifies "SYNC" or "ASYNC" set/reset
port map (
Q => Debug(5), -- 1-bit output data
C0 => RefClk, -- 1-bit clock input
C1 => nRefClk, -- 1-bit clock input
CE => '1', -- 1-bit clock enable input
D0 => '0', -- 1-bit data input (associated with C0)
D1 => '1', -- 1-bit data input (associated with C1)
R => '0', -- 1-bit reset input
S => '0' -- 1-bit set input
);
nRefClk <= not RefClk;
EthClk <= RefClk;
--Debug(1) <= SysClk;
--Debug(2) <= EthClk;
--Debug(3) <= nEthClk;
--Debug(10 downto 7) <= (others => '0');
Debug(6) <= ClkDiv2;
Debug(4 downto 1) <= UserReadData(3 downto 0);
    -- State machine to read/write Ethernet
    process (CpldRst, EthClk) begin
        if (CpldRst='0') then
            UserFPGAState <= USER_FPGA_DELAY_INIT;
            Delay <= (others=>'0');
            UserFPGASubState <= (others=>'0');
            UserValidCount <= (others=>'0');
            ConnectionState <= (others=>'0');
            InterruptStatus <= (others=>'0');
            FrameLength <= (others=>'0');
            Clean <= '0';
            RampData <= X"0001";
            WriteCount <= (others=>'0');
				ClkDiv2 <= '0';
        elsif (EthClk'event and EthClk='1') then
				ClkDiv2 <= not ClkDiv2;
            -- Counter of completed register reads
            if (UserReadDataValid='1') then
                UserValidCount <= UserValidCount + 1;
            end if;

            case UserFPGAState is
                -- Power on startup delay until GigExpedite reports a valid IP address
                when USER_FPGA_DELAY_INIT =>
						  Debug(10 downto 7) <= "0000";
						  --Debug(6 downto 5) <= Delay(1 downto 0);
						  --Debug(4) <= UserReadDataValid;
						  --Debug(3 downto 2) <= UserReadData(1 downto 0);
                    if (Delay=X"ffffff") then
                        if (UserReadDataValid='1' and UserReadData/=X"0000" and UserReadData/=X"ffff") then
                            UserFPGAState <= USER_FPGA_CLEAN;
                        end if;
                    else
                        Delay <= Delay + 1;
                    end if;
                
                -- Reset all connections in case the previous
                -- application closed unexpectedly and left some open
                -- Write 0 to each connection state then
                -- read the status for each connection in turn
                -- Loop round until all statuses are zero.
                when USER_FPGA_CLEAN =>
						  Debug(10 downto 7) <= "0001";
                    UserFPGASubState <= UserFPGASubState + 1;
                    if (UserFPGASubState=X"000f") then
                        UserFPGASubState <= (others=>'0');
                        UserFPGAState <= USER_FPGA_CLEAN_CHECK;
                        Clean <= '1';
                    end if;
                when USER_FPGA_CLEAN_CHECK =>
						  Debug(10 downto 7) <= "0010";
                    UserFPGASubState <= UserFPGASubState + 1;
                    if (UserReadDataValid='1' and UserReadData/=X"0000") then
                        Clean <= '0';
                    end if;
                    if (UserValidCount=X"0010") then
                        if (Clean='1') then
                            UserFPGASubState <= (others=>'0');
                            UserValidCount <= (others=>'0');
                            UserFPGAState <= USER_FPGA_INIT;
                        else
                            Clean <= '1';
                            UserFPGASubState <= (others=>'0');
                            UserValidCount <= (others=>'0');
                        end if;
                    end  if;
                
                -- Initialise register set
                -- Configures one connection as a TCP server waiting
                -- for a connection from a client (i.e. the host program)
                when USER_FPGA_INIT => 
						  Debug(10 downto 7) <= "0011";
                    UserFPGASubState <= UserFPGASubState + 1;
                    if (UserFPGASubState=X"0003") then
                        UserFPGAState <= USER_FPGA_IDLE;
                    end if;
                
                -- Wait for interrupt from GigExpedite device
                when USER_FPGA_IDLE =>
						  Debug(10 downto 7) <= "0100";
                    UserFPGASubState <= (others=>'0');
                    UserValidCount <= (others=>'0');
                    if (UserInterrupt='1') then
                        -- Interrupt has been received
                        UserFPGAState <= USER_FPGA_CHECK_STATE;
                    end if;

                -- Check if the connection state has changed
                when USER_FPGA_CHECK_STATE =>
						  Debug(10 downto 7) <= "0101";

                    UserFPGASubState <= UserFPGASubState + 1;
                    if (UserReadDataValid='1' and UserValidCount=X"0000") then
                        -- Store the interrupt status bits
                        InterruptStatus <= UserReadData;
                    end if;
                    if (UserReadDataValid='1' and UserValidCount=X"0001") then
                        -- Store the new connection state
                        ConnectionState <= UserReadData(5 downto 0);
                        if (UserReadData(3 downto 0)=X"0") then
                            WriteCount <= (others=>'0');
                        end if;

                        -- Next, check if there is incoming data available
                        UserFPGASubState <= (others=>'0');
                        UserValidCount <= (others=>'0');
                        if (InterruptStatus(0)='0') then
                            -- There is no data available
                            -- Next, check if there is outgoing data to send
                            UserFPGAState <= USER_FPGA_CHECK_SPACE;
                        else
                            -- Read frame length
                            UserFPGAState <= USER_FPGA_READ_LENGTH;
                        end if;
                    end if;

                -- Check if there is incoming data
                when USER_FPGA_READ_LENGTH =>
						  Debug(10 downto 7) <= "0110";

                    UserFPGASubState <= UserFPGASubState + 1;
                    if (UserReadDataValid='1' and UserValidCount=X"0000") then
                        -- Read frame length from GigExpedite
                        -- Round the number of bytes up to the total number
                        -- of 16 bit reads we will need to do
                        UserFPGASubState <= (others=>'0');
                        UserValidCount <= (others=>'0');
                        FrameLength <= ('0' & UserReadData(15 downto 1)) + ("000000000000000" & UserReadData(0));
                        if (UserReadData=X"0000") then
                            -- Length was zero - skip the read
                            UserFPGAState <= USER_FPGA_IDLE;
                        else
                            -- Got a valid frame length - read the data
                            UserFPGAState <= USER_FPGA_READ_DATA;
                        end if;
                    end if;
                
                -- Read data from GigExpedite device
                when USER_FPGA_READ_DATA =>
						  Debug(10 downto 7) <= "0111";
                    UserFPGASubState <= UserFPGASubState + 1;
                    if (UserValidCount>=FrameLength) then
                        -- End of frame reached
                        FrameLength <= (others=>'0');
                        UserFPGASubState <= (others=>'0');
                        UserValidCount <= (others=>'0');
                        UserFPGAState <= USER_FPGA_CHECK_SPACE;
                    end if;
                
                -- Check if there is space in the outgoing GigExpedite buffer
                -- and we have data to send back to the host
                when USER_FPGA_CHECK_SPACE =>
						  Debug(10 downto 7) <= "1000";
                    UserFPGASubState <= UserFPGASubState + 1;
                    if (WriteCount=WRITE_LENGTH or
                        ConnectionState(3 downto 0)/=ESTABLISHED or
                        InterruptStatus(2)='0') then
                        -- Already sent all our data or the 
                        -- connection from the host has not been made
                        -- or there is no space in the GigExpedite buffer
                        UserFPGASubState <= (others=>'0');
                        UserValidCount <= (others=>'0');
                        UserFPGAState <= USER_FPGA_IDLE;
                    else
                        -- OK to write data to GigExpedite FIFO
                        UserFPGASubState <= (others=>'0');
                        UserValidCount <= (others=>'0');
                        UserFPGAState <= USER_FPGA_WRITE_DATA;
                    end if;

                -- Write 1kbyte of data to outgoing FIFO
                when USER_FPGA_WRITE_DATA =>
						  Debug(10 downto 7) <= "1001";

                    UserFPGASubState <= UserFPGASubState + 1;
                    RampData(7 downto 0) <= RampData(7 downto 0)+2;
                    RampData(15 downto 8) <= RampData(15 downto 8)+2;
                    WriteCount <= WriteCount + 2;
                    if (WriteCount=WRITE_LENGTH-2 or
                        UserFPGASubState=X"01ff") then
                        UserFPGASubState <= (others=>'0');
                        UserValidCount <= (others=>'0');
                        UserFPGAState <= USER_FPGA_READ_MAIL1;
                    end if;
					  when USER_FPGA_READ_MAIL1 =>
						  Debug(10 downto 7) <= "1010";

								UserFPGAState <= USER_FPGA_WAIT1;
					  when USER_FPGA_WAIT1 =>
						  Debug(10 downto 7) <= "1011";
								UserFPGAState <= USER_FPGA_WAIT3;
					  when USER_FPGA_WAIT3 =>
						  Debug(10 downto 7) <= "1111";
								UserFPGAState <= USER_FPGA_WRITE_MAIL1;
					  when USER_FPGA_WRITE_MAIL1 =>
						  Debug(10 downto 7) <= "1100";
								UserFPGAState <= USER_FPGA_WAIT2;
					  when USER_FPGA_WAIT2 =>
						  Debug(10 downto 7) <= "1101";
						  if(UserReadDataValid='1') then

								UserFPGAState <= USER_FPGA_WRITE_MAIL2;
								end if;

					  when USER_FPGA_WRITE_MAIL2 =>
						  Debug(10 downto 7) <= "1110";
								UserFPGAState <= USER_FPGA_IDLE;
                when others => 
						  Debug(10 downto 7) <= "1111";
            end case;
        end if;
    end process;
    
    -- Map states to Ethernet register accesses
    process (UserFPGAState, Delay, UserFPGASubState, UserReadDataValid, UserReadData,
             UserValidCount, ConnectionState, FrameLength, RampData) begin
        case UserFPGAState is
            when USER_FPGA_DELAY_INIT =>
                -- Read IP address
                if (Delay=X"ffffff") then
                    UserRE <= '1';
                else
                    UserRE <= '0';
                end if;
                UserWE <= '0';
                UserBE <= "11";
                UserAddr <= LOCAL_IP_ADDR_LOW;
                UserWriteData <= (others=>'0');
            
            when USER_FPGA_CLEAN => 
                -- Reset all connections
                UserRE <= '0';
                UserWE <= '1';
                UserBE <= "11";
                UserAddr <= '0' & UserFPGASubState(3 downto 0) & CONNECTION_STATE;
                UserWriteData <= (others=>'0');
            
            when USER_FPGA_CLEAN_CHECK =>
                -- Check all connections have been reset
                if (UserFPGASubState(4)='0') then
                    UserRE <= '1';
                else
                    UserRE <= '0';
                end if;
                UserWE <= '0';
                UserBE <= "11";
                UserAddr <= '0' & UserFPGASubState(3 downto 0) & CONNECTION_STATE;
                UserWriteData <= (others=>'0');
            
            when USER_FPGA_INIT =>
                -- Set up registers to make one connection listen on
                -- port 0x5002 for TCP connections
                UserRE <= '0';
                UserWE <= '1';
                UserBE <= "11";
                case UserFPGASubState(3 downto 0) is
                    when X"0" =>
                        UserAddr <= '0' & X"0" & LOCAL_PORT;
                        UserWriteData <= X"5002";
                    when X"1" =>
                        UserAddr <= '0' & X"0" & MTU_TTL;
                        UserWriteData <= X"8080";
                    when X"2" =>
                        UserAddr <= '0' & X"0" & INTERRUPT_ENABLE_STATUS;
                        UserWriteData <= (IE_OUTGOING_NOT_FULL or IE_INCOMING or IE_STATE);
                    when others =>
                        UserAddr <= '0' & X"0" & CONNECTION_STATE;
                        UserWriteData <= (CONN_TCP or CONN_ENABLE or LISTEN);
                end case;
                
            when USER_FPGA_CHECK_STATE =>
                -- Read connection state then update accordingly
                -- The only important state change is to ESTABLISHED which
                -- we must acknowledge by changing our state to ESTABLISHED.
                -- All other state changes result in us returning to LISTEN
                -- to wait for another connection from a new client.
                if (UserFPGASubState(3 downto 1)="000") then
                    UserRE <= '1';
                else
                    UserRE <= '0';
                end if;
                if (UserReadDataValid='1' and UserValidCount(0)='1' and
                    UserReadData(5 downto 0)/=ConnectionState) then
                    UserWE <= '1';
                else
                    UserWE <= '0';
                end if;
                UserBE <= "11";
                if (UserFPGASubState(3 downto 0)="0000") then
                    UserAddr <= '0' & X"0" & INTERRUPT_ENABLE_STATUS;
                else
                    UserAddr <= '0' & X"0" & CONNECTION_STATE;
                end if;
                if (UserReadData(3 downto 0)=ESTABLISHED) then
                    UserWriteData <= (CONN_TCP or CONN_ENABLE or ESTABLISHED);
                else
                    UserWriteData <= (CONN_TCP or CONN_ENABLE or LISTEN);
                end if;
            
            when USER_FPGA_READ_LENGTH =>
                -- Read from interrupt status and then frame length register
                if (UserFPGASubState(3 downto 0)=X"0") then
                    UserRE <= '1'; 
                else
                    UserRE <= '0';
                end if;
                UserWE <= '0';
                UserBE <= "11";
                UserAddr <= '0' & X"0" & FRAME_LENGTH;
                UserWriteData <= (others=>'0');
                
            when USER_FPGA_READ_DATA =>
                -- Read from connection FIFO
                if (UserFPGASubState<FrameLength) then
                    UserRE <= '1';
                else
                    UserRE <= '0';
                end if;
                UserWE <= '0';
                UserBE <= "11";
                UserAddr <= '0' & X"0" & DATA_FIFO;
                UserWriteData <= (others=>'0');

            when USER_FPGA_CHECK_SPACE =>
                -- No access necessary
                UserRE <= '0';
                UserWE <= '0';
                UserBE <= "11";
                UserAddr <= (others=>'0');
                UserWriteData <= (others=>'0');
                
            when USER_FPGA_WRITE_DATA =>
                -- Write test data to connection FIFO
                UserRE <= '0';
                UserWE <= '1';
                UserBE <= "11";
                UserAddr <= '0' & X"0" & DATA_FIFO;
                UserWriteData <= RampData;
					 
            when USER_FPGA_READ_MAIL1 =>
                -- Read from connection FIFO
					 UserRE <= '1';
                UserWE <= '0';
                UserBE <= "11";
                UserAddr <=  "1100000000";
                UserWriteData <= (others=>'0');
					 
            when USER_FPGA_WRITE_MAIL1 =>
					 UserRE <= '0';
                UserWE <= '1';
                UserBE <= "11";
                UserAddr <=  "1100000010";
                UserWriteData <= X"1234";       
				 when USER_FPGA_WRITE_MAIL2 =>
					 UserRE <= '0';
                UserWE <= '1';
                UserBE <= "11";
                UserAddr <=  "1100000010";
                UserWriteData <= UserReadData;
					 
                
            when others =>
                -- Don't do anything
                UserRE <= '0';
                UserWE <= '0';
                UserAddr <= (others=>'0');
                UserBE <= "11";
                UserWriteData <= (others=>'0');

        end case;
    end process;

    --------------------------------------------------------------------------
    -- Instantiate GigEx interface physical layer
    GigExPhyInst : entity work.GigExPhy16 
        generic map (
            CLOCK_RATE => 104000000
        )
        port map (
            CLK => EthClk,
				Debug => open,

            GigEx_Clk => ZEthClk,
            GigEx_nCS => ZEthCS,
            GigEx_nWE => ZEthWE,
            GigEx_Addr(9 downto 1) => ZEthA,
				GigEx_Addr(0) => open,

            GigEx_nBE => ZEthBE,
            GigEx_Data => DQ,
            GigEx_EOF => ZEthEOF,
            GigEx_Length => ZEthLen,
            GigEx_Header => nullbit,
            GigEx_nInt => nullbit,

            UserWE => UserWE,
            UserRE => UserRE,
            UserAddr => UserAddr,
            UserBE => UserBE,
            UserWriteData => UserWriteData,
            UserOwner => X"00",
            UserReadData => UserReadData,
            UserReadDataValid => UserReadDataValid,
            UserInterrupt => UserInterrupt
        );
    
end arch;