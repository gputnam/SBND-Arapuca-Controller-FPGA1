--
--   ZestETM1 GigExpedite Physical Layer for 16 bit Interface
--   File name: GigExPhy16.vhd
--   Version: 1.00
--   Date: 13/8/2013
--   
--   ZestETM1 low level physical layer for interface to GigExpedite.
--   Instantiates IO primitives to ensure timing to/from GigEx is met.
--
--   Copyright (C) 2013 Orange Tree Technologies Ltd. All rights reserved.
--   Orange Tree Technologies grants the purchaser of a ZestETM1 the right to use and 
--   modify this logic core in any form such as VHDL source code or EDIF netlist in 
--   FPGA designs that target the ZestETM1.
--   Orange Tree Technologies prohibits the use of this logic core in any form such 
--   as VHDL source code or EDIF netlist in FPGA designs that target any other
--   hardware unless the purchaser of the ZestETM1 has purchased the appropriate 
--   licence from Orange Tree Technologies. Contact Orange Tree Technologies if you 
--   want to purchase such a licence.
--
--  *****************************************************************************************
--  **
--  **  Disclaimer: LIMITED WARRANTY AND DISCLAIMER. These designs are
--  **              provided to you "as is". Orange Tree Technologies and its licensors 
--  **              make and you receive no warranties or conditions, express, implied, 
--  **              statutory or otherwise, and Orange Tree Technologies specifically 
--  **              disclaims any implied warranties of merchantability, non-infringement,
--  **              or fitness for a particular purpose. Orange Tree Technologies does not
--  **              warrant that the functions contained in these designs will meet your 
--  **              requirements, or that the operation of these designs will be 
--  **              uninterrupted or error free, or that defects in the Designs will be 
--  **              corrected. Furthermore, Orange Tree Technologies does not warrant or 
--  **              make any representations regarding use or the results of the use of the 
--  **              designs in terms of correctness, accuracy, reliability, or otherwise.                                               
--  **
--  **              LIMITATION OF LIABILITY. In no event will Orange Tree Technologies 
--  **              or its licensors be liable for any loss of data, lost profits, cost or 
--  **              procurement of substitute goods or services, or for any special, 
--  **              incidental, consequential, or indirect damages arising from the use or 
--  **              operation of the designs or accompanying documentation, however caused 
--  **              and on any theory of liability. This limitation will apply even if 
--  **              Orange Tree Technologies has been advised of the possibility of such 
--  **              damage. This limitation shall apply notwithstanding the failure of the 
--  **              essential purpose of any limited remedies herein.
--  **
--  *****************************************************************************************

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;

library UNISIM;
use UNISIM.VComponents.all;

entity GigExPhy16 is
    generic (
        CLOCK_RATE : integer := 125000000
    );
    port (
        CLK : in std_logic;
        
        -- Interface to GigExpedite
        GigEx_Clk : out std_logic;
        GigEx_nCS : out std_logic;
        GigEx_nWE : out std_logic;
        GigEx_Addr : out std_logic_vector(9 downto 0);
        GigEx_nBE : out std_logic_vector(1 downto 0);
        GigEx_Data : inout std_logic_vector(15 downto 0);
        GigEx_EOF : in std_logic_vector(1 downto 0);
        GigEx_Length : in std_logic;
        GigEx_Header : in std_logic;
        GigEx_nInt : in std_logic;

        -- Application interface
        UserWE : in std_logic;
        UserRE : in std_logic;
        UserAddr : in std_logic_vector(9 downto 0);
        UserBE : in std_logic_vector(1 downto 0);
        UserWriteData : in std_logic_vector(15 downto 0);
        UserOwner : in std_logic_vector(7 downto 0);
        UserReadData : out std_logic_vector(15 downto 0);
        UserReadDataValid : out std_logic;
        UserValidOwner : out std_logic_vector(7 downto 0);
        UserValidEOF : out std_logic_vector(1 downto 0);
        UserValidLength : out std_logic;
        UserValidHeader : out std_logic;
        UserInterrupt : out std_logic
    );
end GigExPhy16;

architecture arch of GigExPhy16 is

    -- Declare signals
    signal UserDataDelay3 : std_logic_vector(15 downto 0);
    signal UserDataDelay2 : std_logic_vector(15 downto 0);
    signal UserDataDelay1 : std_logic_vector(15 downto 0);
    signal UserWEDelay3 : std_logic;
    signal UserWEDelay2 : std_logic;
    signal UserWEDelay1 : std_logic;

    signal RegUser_Addr : std_logic_vector(9 downto 0);
    signal RegUser_nCS : std_logic;
    signal RegUser_nWE : std_logic;
    signal RegUser_nBE : std_logic_vector(1 downto 0);
    
    signal GigEx_DVal : std_logic_vector(15 downto 0);
    signal GigEx_TSVal : std_logic_vector(15 downto 0);

    signal GigEx_nCSVal : std_logic;
    signal GigEx_nWEVal : std_logic;
    signal GigEx_AVal : std_logic_vector(9 downto 0);
    signal GigEx_nBEVal : std_logic_vector(1 downto 0);

    signal nCLK : std_logic;
    signal RegGigEx_DP : std_logic_vector(15 downto 0);
    signal RegGigEx_DN : std_logic_vector(15 downto 0);
    signal RegGigEx_EOFP : std_logic_vector(1 downto 0);
    signal RegGigEx_EOFN : std_logic_vector(1 downto 0);
    signal RegGigEx_LengthP : std_logic;
    signal RegGigEx_LengthN : std_logic;
    signal RegGigEx_HeaderP : std_logic;
    signal RegGigEx_HeaderN : std_logic;
    signal User_ReadDataP : std_logic_vector(15 downto 0);
    signal User_ReadDataN : std_logic_vector(15 downto 0);
    signal User_ReadEOFP : std_logic_vector(1 downto 0);
    signal User_ReadEOFN : std_logic_vector(1 downto 0);
    signal User_ReadLengthP : std_logic;
    signal User_ReadLengthN : std_logic;
    signal User_ReadHeaderP : std_logic;
    signal User_ReadHeaderN : std_logic;

    signal ReadDelay : std_logic_vector(5 downto 0);
    type OWNER_TYPE is array(0 to 5) of std_logic_vector(7 downto 0);
    signal Owner : OWNER_TYPE;

    attribute IOB : string;
    attribute IOB of nCSInst : label is "FORCE";
    attribute IOB of nWEInst : label is "FORCE";

begin

    -- Output clock with DDR FF to give predictable timing
    nCLK <= not CLK;
    ClockOutInst : ODDR2
        generic map (
            DDR_ALIGNMENT => "NONE",
            INIT => '0',
            SRTYPE => "SYNC"
        )
        port map (
            Q => GigEx_Clk,
            C0 => CLK,
            C1 => nCLK,
            CE => '1',
            D0 => '0',
            D1 => '1',
            R => '0',
            S => '0'
        );

    -- Data outputs
    -- This makes the data/control change on the falling edge of the clock
    -- at the GigExpedite to ensure setup/hold times
    process (CLK) begin
        if (CLK'event and CLK='1') then
            UserDataDelay3 <= UserDataDelay2;
            UserDataDelay2 <= UserDataDelay1;
            UserDataDelay1 <= UserWriteData;
            UserWEDelay3 <= UserWEDelay2;
            UserWEDelay2 <= UserWEDelay1;
            UserWEDelay1 <= not UserWE;
        end if;
    end process;

    DataOutCopy:
        for g in 0 to 15 generate
            attribute IOB of DataOutCopyInst : label is "FORCE";
        begin
		  
            DataOutCopyInst : ODDR2
                generic map (
                    DDR_ALIGNMENT => "C0",
                    INIT => '0',
                    SRTYPE => "ASYNC"
                )
                port map (
                    Q => GigEx_DVal(g),
                    C0 => CLK,
                    C1 => nCLK,
                    CE => '1',
                    D0 => UserDataDelay3(g),
                    D1 => UserDataDelay3(g),
                    R => '0',
                    S => '0'
                );
        end generate;

    DataTSCopy:
        for g in 0 to 15 generate
            attribute IOB of DataTSCopyInst : label is "FORCE";
        begin
            DataTSCopyInst : ODDR2
                generic map (
                    DDR_ALIGNMENT => "C0",
                    INIT => '0',
                    SRTYPE => "ASYNC"
                )
                port map (
                    Q => GigEx_TSVal(g),
                    C0 => CLK,
                    C1 => nCLK,
                    CE => '1',
                    D0 => UserWEDelay3,
                    D1 => UserWEDelay3,
                    R => '0',
                    S => '0'
                );
        end generate;

    -- Tristate buffers on data output
    DataTS:
        for g in 0 to 15 generate
        begin
            GigEx_Data(g) <= GigEx_DVal(g) when GigEx_TSVal(g)='0' else 'Z';
        end generate;
    
    -- Address and control outputs
    -- This makes the data/control change on the falling edge of the clock
    -- at the GigExpedite to ensure setup/hold times
    process (CLK) begin
        if (CLK'event and CLK='1') then
            RegUser_nCS <= not (UserWE or UserRE);
            RegUser_nBE <= not UserBE;
            RegUser_nWE <= not UserWE;
            RegUser_Addr <= UserAddr;
        end if;
    end process;
    
    nCSInst : FDCE
        generic map (
            INIT => '1'
        )
        port map (
            Q => GigEx_nCSVal,
            C => CLK,
            CE => '1',
            D => RegUser_nCS,
            CLR => '0'
        );

    nWEInst : FDCE
        generic map (
            INIT => '1'
        )
        port map (
            Q => GigEx_nWEVal,
            C => CLK,
            CE => '1',
            D => RegUser_nWE,
            CLR => '0'
        );

    AddrCopy:
        for g in 0 to 9 generate
            attribute IOB of AddrCopyInst : label is "FORCE";
        begin
            AddrCopyInst : FDCE
                generic map (
                    INIT => '0'
                )
                port map (
                    Q => GigEx_AVal(g),
                    C => CLK,
                    CE => '1',
                    D => RegUser_Addr(g),
                    CLR => '0'
                );
        end generate;

    nBECopy:
        for g in 0 to 1 generate
            attribute IOB of nBECopyInst : label is "FORCE";
        begin
            nBECopyInst : FDCE
                generic map (
                    INIT => '1'
                )
                port map (
                    Q => GigEx_nBEVal(g),
                    C => CLK,
                    CE => '1',
                    D => RegUser_nBE(g),
                    CLR => '0'
                );
        end generate;

    GigEx_nCS <= GigEx_nCSVal;
    GigEx_nWE <= GigEx_nWEVal;
    GigEx_Addr <= GigEx_AVal;
    GigEx_nBE <= GigEx_nBEVal;
    
    -- Read path
    -- Use DDR FF to get rising and falling edge registers
    DataInCopy:
        for g in 0 to 15 generate
        begin
            DataInCopyInst : IDDR2
                generic map (
                    DDR_ALIGNMENT => "NONE",
                    INIT_Q0 => '0',
                    INIT_Q1 => '0',
                    SRTYPE => "SYNC"
                )
                port map (
                    Q0 => RegGigEx_DP(g),
                    Q1 => RegGigEx_DN(g),
                    C0 => CLK,
                    C1 => nCLK,
                    CE => '1',
                    D => GigEx_Data(g),
                    R => '0',
                    S => '0'
                );
        end generate;
        
    EOFInCopy:
        for g in 0 to 1 generate
        begin
            EOFInCopyInst : IDDR2
                generic map (
                    DDR_ALIGNMENT => "NONE",
                    INIT_Q0 => '0',
                    INIT_Q1 => '0',
                    SRTYPE => "SYNC"
                )
                port map (
                    Q0 => RegGigEx_EOFP(g),
                    Q1 => RegGigEx_EOFN(g),
                    C0 => CLK,
                    C1 => nCLK,
                    CE => '1',
                    D => GigEx_EOF(g),
                    R => '0',
                    S => '0'
                );
        end generate;
        
    LengthInInst : IDDR2
        generic map (
            DDR_ALIGNMENT => "NONE",
            INIT_Q0 => '0',
            INIT_Q1 => '0',
            SRTYPE => "SYNC"
        )
        port map (
            Q0 => RegGigEx_LengthP,
            Q1 => RegGigEx_LengthN,
            C0 => CLK,
            C1 => nCLK,
            CE => '1',
            D => GigEx_Length,
            R => '0',
            S => '0'
        );
        
    HeaderInInst : IDDR2
        generic map (
            DDR_ALIGNMENT => "NONE",
            INIT_Q0 => '0',
            INIT_Q1 => '0',
            SRTYPE => "SYNC"
        )
        port map (
            Q0 => RegGigEx_HeaderP,
            Q1 => RegGigEx_HeaderN,
            C0 => CLK,
            C1 => nCLK,
            CE => '1',
            D => GigEx_Header,
            R => '0',
            S => '0'
        );

    -- Re-register to user clock
    DataInPCopy:
        for g in 0 to 15 generate
        begin
            DataInPCopyInst : FD
                port map (
                    Q => User_ReadDataP(g),
                    C => CLK,
                    D => RegGigEx_DP(g)
                );
        end generate;
    DataInNCopy:
        for g in 0 to 15 generate
        begin
            DataInNCopyInst : FD
                port map (
                    Q => User_ReadDataN(g),
                    C => CLK,
                    D => RegGigEx_DN(g)
                );
        end generate;
    EOFInPCopy:
        for g in 0 to 1 generate
        begin
            EOFInPCopyInst : FD
                port map (
                    Q => User_ReadEOFP(g),
                    C => CLK,
                    D => RegGigEx_EOFP(g)
                );
        end generate;
    EOFInNCopy:
        for g in 0 to 1 generate
        begin
            EOFInNCopyInst : FD
                port map (
                    Q => User_ReadEOFN(g),
                    C => CLK,
                    D => RegGigEx_EOFN(g)
                );
        end generate;
    LengthInPCopyInst : FD
        port map (
            Q => User_ReadLengthP,
            C => CLK,
            D => RegGigEx_LengthP
        );
    LengthInNCopyInst : FD
        port map (
            Q => User_ReadLengthN,
            C => CLK,
            D => RegGigEx_LengthN
        );
    HeaderInPCopyInst : FD
        port map (
            Q => User_ReadHeaderP,
            C => CLK,
            D => RegGigEx_HeaderP
        );
    HeaderInNCopyInst : FD
        port map (
            Q => User_ReadHeaderN,
            C => CLK,
            D => RegGigEx_HeaderN
        );

    -- Generate delayed valid and owner signals to match read data
    process (CLK) begin
        if (CLK'event and CLK='1') then
            ReadDelay <= ReadDelay(4 downto 0) & UserRE;
            Owner(5) <= Owner(4);
            Owner(4) <= Owner(3);
            Owner(3) <= Owner(2);
            Owner(2) <= Owner(1);
            Owner(1) <= Owner(0);
            Owner(0) <= UserOwner;
        end if;
    end process;

    -- Select read data path on clock rate
    -- Since the delays to/from the GigExpedite are fixed, the read sample
    -- clock edge will change with the clock rate changing
    process (User_ReadDataN, User_ReadDataP, User_ReadEOFN, User_ReadEOFP, User_ReadHeaderN, User_ReadHeaderP,
             User_ReadLengthN, User_ReadLengthP, ReadDelay) begin
        if (CLOCK_RATE<60000000) then
            UserReadData <= User_ReadDataN;
            UserValidEOF <= User_ReadEOFN;
            UserValidHeader <= User_ReadHeaderN;
            UserValidLength <= User_ReadLengthN;
            UserReadDataValid <= ReadDelay(4);
            UserValidOwner <= Owner(4);
        elsif (CLOCK_RATE<130000000) then
            UserReadData <= User_ReadDataP;
            UserValidEOF <= User_ReadEOFP;
            UserValidHeader <= User_ReadHeaderP;
            UserValidLength <= User_ReadLengthP;
            UserReadDataValid <= ReadDelay(5);
            UserValidOwner <= Owner(5);
        else
            UserReadData <= User_ReadDataN;
            UserValidEOF <= User_ReadEOFN;
            UserValidHeader <= User_ReadHeaderN;
            UserValidLength <= User_ReadLengthN;
            UserReadDataValid <= ReadDelay(5);
            UserValidOwner <= Owner(5);
        end if;
    end process;
		

end arch;
