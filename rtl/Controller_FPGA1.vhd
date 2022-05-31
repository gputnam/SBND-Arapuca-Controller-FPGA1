-- Firmware to test OT connection on main FPGA on ROC
-- Modified from OT example code by Daniel Mishins, with parts copied from code by Sten Hansen



LIBRARY ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;

Library UNISIM;
use UNISIM.vcomponents.all;

use work.Project_defs.all;

entity ControllerFPGA_1 is port(

	-- Orange Tree Ethernet daughter card lines
	GigEx_Data : inout std_logic_vector(15 downto 0);
	GigEx_Addr : out std_logic_vector(8 downto 0);
	GigEx_nCS,GigEx_nWE,GigEx_Clk : out std_logic;
	GigEx_nBE : out std_logic_vector(1 downto 0);
	GigEx_EOF : in std_logic_vector(1 downto 0);
	GigEx_Length : in std_logic;

	-- Physical input clock for OT
	ClkB_P,ClkB_N : in std_logic;
	
	
	--uC strobes and reset
	CpldRst, CpldCS, uCRd, uCWr : in std_logic;
	-- Debug port
	Debug : out std_logic_vector(10 downto 1)
	
	
);

end ControllerFPGA_1;

architecture arch of ControllerFPGA_1 is
	--PLL for generating OT clock
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
    signal EthClk : std_logic;

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
        USER_FPGA_READ_LENGTH,
        USER_FPGA_READ_DATA
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
	 
	 -- active high reset for xilinx stuff
	 signal ResetHi : std_logic;
	 signal Pll_Locked : std_logic;
	 
    
begin
	--Debug (1) <= ZEthClk;
	--Debug (2) <= ZEthCS;
	--Debug (3) <= ZEthWE;
		--ZEthBE : buffer std_logic_vector(1 downto 0);
		--ZEthEOF : in std_logic_vector(1 downto 0);
		--ZEthLen : in std_logic;
	--Debug <= (others => '0');
	--Debug(1) <= SysClk;
	--Debug(2) <= EthClk;
	--Debug(3) <= nEthClk;
	Debug(10 downto 1) <= (others => '0');
	--Debug(6) <= ClkDiv2;
	--Debug(4 downto 1) <= UserReadData(3 downto 0);
	
	ResetHi <= not CpldRst;

	--Generate reference clock for OT with PLL
	Sys_Pll : SysPll
	  port map(
	 -- Clock in ports
		 CLK_IN1_P => ClkB_P,
		 CLK_IN1_N => ClkB_N,
	 -- Clock out ports
		 CLK_OUT1 => EthClk,   -- 125 MHz
		 --CLK_OUT2 =>  open,--nEthClk,  -- 160 MHz 180 deg. phase
		 --CLK_OUT3 =>  open, --EthClk,   -- 160 MHz used for Orange Tree I/O
	 -- Status and control signals
		 RESET  => ResetHi,
		 LOCKED => Pll_Locked
	);
		 

    -- State machine to read/write Ethernet
    process (ResetHi, EthClk) begin
        if (ResetHi='1') then
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
        elsif (EthClk'event and EthClk='1') then
            -- Counter of completed register reads
            if (UserReadDataValid='1') then
                UserValidCount <= UserValidCount + 1;
            end if;

            case UserFPGAState is
                -- Power on startup delay until GigExpedite reports a valid IP address
                when USER_FPGA_DELAY_INIT =>
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
					 -- (un) Modified to only clean Channel 0. Other channels controlled by uC.
                when USER_FPGA_CLEAN =>
                    UserFPGASubState <= UserFPGASubState + 1;
                    if (UserFPGASubState=X"000f") then
                        UserFPGASubState <= (others=>'0');
                        UserFPGAState <= USER_FPGA_CLEAN_CHECK;
                        Clean <= '1';
                    end if;
                when USER_FPGA_CLEAN_CHECK =>
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
                    UserFPGASubState <= UserFPGASubState + 1;
                    if (UserFPGASubState=X"0003") then
                        UserFPGAState <= USER_FPGA_IDLE;
                    end if;
                
                -- Wait for interrupt from GigExpedite device
                when USER_FPGA_IDLE =>
                    UserFPGASubState <= (others=>'0');
                    UserValidCount <= (others=>'0');
                    --if (UserInterrupt='1') then
                        -- Interrupt has been received
								-- Since we have no interrupt pin, just always check status
                        UserFPGAState <= USER_FPGA_CHECK_STATE;
                    --end if;

                -- Check if the connection state has changed
                when USER_FPGA_CHECK_STATE =>
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
                        --if (InterruptStatus(0)='0') then
                            -- There is no data available
                            -- Next, check if there is outgoing data to send
                            UserFPGAState <= USER_FPGA_CHECK_SPACE;
                        --else
                            -- Read frame length
                            --UserFPGAState <= USER_FPGA_READ_LENGTH;
                        --end if;
                    end if;

                -- Check if there is space in the outgoing GigExpedite buffer
                -- and we have data to send back to the host
                when USER_FPGA_CHECK_SPACE =>
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
                    UserFPGASubState <= UserFPGASubState + 1;
                    RampData(7 downto 0) <= RampData(7 downto 0)+2;
                    RampData(15 downto 8) <= RampData(15 downto 8)+2;
                    WriteCount <= WriteCount + 2;
                    if (WriteCount=WRITE_LENGTH-2 or
                        UserFPGASubState=X"01ff") then
                        UserFPGASubState <= (others=>'0');
                        UserValidCount <= (others=>'0');
                        UserFPGAState <= USER_FPGA_IDLE;
                    end if;

                when others => null;
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
            CLOCK_RATE => 125000000
        )
        port map (
            CLK => EthClk,
            
            GigEx_Clk => GigEx_Clk,
            GigEx_nCS => GigEx_nCS,
            GigEx_nWE => GigEx_nWE,
            GigEx_Addr (9 downto 1) => GigEx_Addr,
					GigEx_Addr(0) => open,
            GigEx_nBE => GigEx_nBE,
            GigEx_Data => GigEx_Data,
            GigEx_EOF => GigEx_EOF,
            GigEx_Length => GigEx_Length,
            GigEx_Header => '0',
            GigEx_nInt => '0',

            UserWE => UserWE,
            UserRE => UserRE,
            UserAddr => UserAddr,
            UserBE => UserBE,
            UserWriteData => UserWriteData,
            UserOwner => X"00",
            UserReadData => UserReadData,
            UserReadDataValid => UserReadDataValid,
            UserInterrupt => open
        );
    
end arch;