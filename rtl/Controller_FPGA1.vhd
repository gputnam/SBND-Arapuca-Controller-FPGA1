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
-- 08/10/18 Added event buffer FIFO flag outputs



-- Modified for SBND in 2022 by Daniel Mishins
-- Must use updated uC code for new Data Request Scheme

----------------------------- Main Body of design -------------------------

LIBRARY ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;

Library UNISIM;
use UNISIM.vcomponents.all;

use work.Project_defs.all;

entity ControllerFPGA_1 is port(

-- 100 MHz VXO clock, 50MHz Phy clock
	VXO_P,VXO_N,ClkB_P,ClkB_N,Clk50MHz,BnchClk : in std_logic;
-- 156.25 MHz GTP Reference clock, Gigabit data lines
	GTPClk_P,GTPClk_N,GTPRx_P,GTPRx_N : in std_logic_vector(1 downto 0);
	GTPTx_P,GTPTx_N : out std_logic_vector(1 downto 0);
-- Optical transcever slow control lines
	TDisA,TDisB : buffer std_logic;
-- The partucular optical transceivers we bought don't come with I2C control
-- thses lines don't go anywhere for now.
	SD_A,SD_B : in std_logic;
-- microcontroller strobes
	CpldRst, CpldCS, uCRd, uCWr, EthCS : in std_logic;
-- microcontroller data, address buses
	uCA : in std_logic_vector(11 downto 0);
	uCD : inout std_logic_vector(15 downto 0);
-- Geographic address pins
	GA : in std_logic_vector(1 downto 0);
-- Serial inter-chip link clock, framing lines
	LINKClk_P,LINKClk_N,LinkFR_P,LinkFR_N  : in std_logic_vector(2 downto 0);
-- Serial inter-chip link Data lines
	LinkSDat_P,LinkSDat_N : in std_logic_vector(5 downto 0);
-- FM Transmitters for uBunch and Triggers
	HeartBeatFM,TrigFM,uBunchLED,TrigLED,
-- Pll control lines
	PllSClk,PllSDat,PllLd,PllPDn : buffer std_logic;
	PllStat : in std_logic;
-- Serial control lines for the RJ-45 LEDs
	LEDSClk,LEDSDat : out std_logic_vector(2 downto 0);
	LEDLd : out std_logic_vector(5 downto 0);
	LEDRst : buffer std_logic;
-- Orange Tree Ethernet daughter card lines
	DQ : inout std_logic_vector(15 downto 0);
	ZEthA : buffer std_logic_vector(8 downto 0);
	ZEthCS,ZEthWE,ZEthClk : buffer std_logic;
	ZEthBE : buffer std_logic_vector(1 downto 0);
	ZEthEOF : in std_logic_vector(1 downto 0);
	ZEthLen : in std_logic;
-- Back panel LEMOs
	GPO : buffer std_logic_vector(1 downto 0);
	GPI,NimTrig : in std_logic;
-- Debug port
	Debug : buffer std_logic_vector(10 downto 1)
);

end ControllerFPGA_1;

architecture behavioural of ControllerFPGA_1 is

---------------------- Signal declarations -----------------------

-- Name Arrays according to their size
Type Array_2x2 is Array(0 to 1) of std_logic_vector (1 downto 0);
Type Array_2x3 is Array(0 to 1) of std_logic_vector (2 downto 0);
Type Array_2x10 is Array(0 to 1) of std_logic_vector(9 downto 0);
Type Array_2x13 is Array(0 to 1) of std_logic_vector (12 downto 0);
Type Array_2x16 is Array(0 to 1) of std_logic_vector (15 downto 0);

Type Array_3x3 is Array(0 to 2) of std_logic_vector(2 downto 0);
Type Array_3x4 is Array(0 to 2) of std_logic_vector(3 downto 0);
Type Array_3x5 is Array(0 to 2) of std_logic_vector(4 downto 0);
Type Array_3x8 is Array(0 to 2) of std_logic_vector(7 downto 0);
Type Array_3x13 is Array(0 to 2) of std_logic_vector (12 downto 0);
Type Array_3x14 is Array (0 to 2) of std_logic_vector (13 downto 0);
Type Array_3x16 is Array(0 to 2) of std_logic_vector (15 downto 0);

Type Array_3x2x10 is Array (0 to 2) of Array_2x10;

-- Synchronous edge detectors of uC read and write strobes
Signal RDDL,WRDL : std_logic_vector (1 downto 0);
Signal FMRDDL,FMWRDL : std_logic_vector (1 downto 0);
signal EthWRDL,EthRDDL : std_logic_vector (4 downto 0);

-- Clock and reset signals
signal Buff_Rst,SysClk,Clk80MHz,FMGenClk,ResetHi,Pll_Locked,nEthClk,
		 EthClk,SerdesRst,LinkBuffRst,GTPRst, Seq_Rst : std_logic;

-- Counter that determines the trig out pulse width
signal GPOCount : std_logic_vector(2 downto 0);

-- Orange tree signals
signal iDQ : std_logic_vector (15 downto 0);
Signal DQWrtDly : Array_3x16;
signal DQEn : std_logic;

-- uC data bus
signal iCD : std_logic_vector(15 downto 0);
signal AddrReg : std_logic_vector(11 downto 0);
-- FM transmit enable
signal EnTx1,TrgSrc : std_logic;
signal Counter1s : std_logic_vector (27 downto 0);

-- Count the number of triggers
signal DReq_Count : std_logic_vector (31 downto 0);
-- Make a test counter that increments with each read
signal TestCount : std_logic_vector (31 downto 0);
-- Uptime counter to check for un-anticipated resets
signal UpTimeCount,UpTimeStage : std_logic_vector (31 downto 0);

-- Spill counter, event word cout, spill word count
signal SpillCount,EventWdCnt : std_logic_vector (15 downto 0);

signal GPIDL,TrigDL,iWrtDL  : Array_2x2;
signal GateWidth0,GateWidth1,PedWidth0,PedWidth1 : std_logic_vector (7 downto 0);
-- Test Pulse generator signals
-- DDS frequency registers
signal FreqReg,PhaseAcc : std_logic_vector (31 downto 0);
signal PhaseAccD : std_logic;

-- Link receive FIFO signals
signal LinkFIFOEn,LinkFIFOEnd,LinkFIFORdReq,LinkFIFOWrReq,
		 LinkFIFOEmpty,LinkFIFOFull : std_logic_vector (2 downto 0);
signal LinkFIFORdCnt : Array_3x13;
signal LinkRDDL : std_logic_vector (1 downto 0);
signal LinkFIFOOut : Array_3x16;

-- Event buffer signals
signal EventBuff_WrtEn,EventBuff_RdEn,
		 EventBuff_Full,EventBuff_Empty,EvBuffWrtGate : std_logic;
signal EventBuff_Dat,EventBuff_Out,EventSum : std_logic_vector (15 downto 0);
signal FIFOCount : Array_3x16;
--signal to stop the event builder from running.
signal FormHold, FormRst : std_logic;

Type Event_Builder_Seq is (Idle,RdInWdCnt0,RdInWdCnt1,RdInWdCnt2,SumWdCnt,WrtWdCnt,RdStat0,
								   RdStat1,RdStat2,WrtStat,WaitEvent,ReadFIFO0,ReadFIFO1,ReadFIFO2);
signal Event_Builder : Event_Builder_Seq;

-- Front panel LED Shifter signals
signal CMDwr_en,CMDrd_en,CMD_Full,CMD_Empty : std_logic;
signal ClkDiv : std_logic_vector (2 downto 0);
signal CMDBitCount : std_logic_vector (3 downto 0);
signal LEDShiftReg : std_logic_vector (15 downto 0);
signal CMD_Out : std_logic_vector (18 downto 0);
Type LEDSerializer_FSM is (Idle,Load,Shift,RdFIFO,SendRst,WaitRst,WaitPClk,SendPClk);
Signal LED_Shift : LEDSerializer_FSM;

-- Pll Chip Shifter signals
signal PLLBuffwr_en,PLLBuffrd_en,PLLBuff_full,PLLBuff_empty : std_logic;
signal PllStage : std_logic_vector (7 downto 0);
signal PLLBuff_Out,PllShiftReg : std_logic_vector (23 downto 0);
signal PllBitCount : std_logic_vector (4 downto 0);
Type PllSerializer_FSM is (Idle,Load,Shift,WaitLd,SendLd);
Signal Pll_Shift : PllSerializer_FSM;

-- Each channel produces two deserialized bit streams with ten bits total
Signal LinkPDat : Array_3x2x10;

signal WrtWdCount0,WrtWdCount1 : std_logic_vector(11 downto 0);
signal Buff_Wrt,Buff_Rd,Buff_Empty : std_logic_vector(2 downto 0);

signal SerDesInP,SerDesInN : Array_3x3;

-- Deserialize frame along with the 8 data lines. Use the deserialized 
-- frame signal as an input to the bitslip state machine
signal LinkFRDat : Array_3x5;
signal SlipReq : std_logic_vector(2 downto 0);
signal Slippause : Array_3x4;

-- Signal names used by SERDES see: XAP1024
signal rxioclkp : std_logic_vector(2 downto 0);
signal rxioclkn : std_logic_vector(2 downto 0);
signal rx_serdesstrobe	: std_logic_vector(2 downto 0);

signal RxOutClk : std_logic_vector (2 downto 0);

-- Signals used by the microbunch,trigger FM transmitters
signal HrtBtTxOuts : TxOutRec;
signal HrtBtDone,HrtBtTxReq,HrtBtTxAck,HrtBtFMTxEn,TxEnReq,LinkBusy : std_logic;
signal HrtBtData : std_logic_vector (23 downto 0);

signal uBunchCount : std_logic_vector(47 downto 0);
signal Beam_On : std_logic;

--Signals used by the trigger/heartbeat/dr logic
signal NimTrigBuf, PPSBuf : std_logic_vector(2 downto 0);
signal ManTrig, TriggerReq : std_logic;
signal TriggerHoldoff, TriggerHoldoffCounter: std_logic_vector(15 downto 0);
signal ResetTrgCntOnPPS, ResetTimestampOnPPS : std_logic;

signal TriggerTimestampCounter : std_logic_vector(31 downto 0);

-- Trigger request packet buffer, FIFO status bits
signal DReqBuff_Out, DReqBuff_In : std_logic_vector (15 downto 0);
signal DReqBuff_wr_en,DReqBuff_rd_en,DReqBuff_uCRd, DCSPktBuff_uCRd,
		 DReqBuff_Full,TrigTx_Sel,DReqBuff_Emtpy,
		 Dreq_Tx_Req,Dreq_Tx_ReqD,DReq_Tx_Ack,BmOnTrigReq,Stat_DReq : std_logic;
		 
signal TrgPktCnt : std_logic_vector (10 downto 0);
signal LinkFIFOStatReg,Lower_FM_Bits : std_logic_vector (2 downto 0);  

signal Trig_Tx_Req : std_logic;

Type Trig_Tx_State is (Idle,SenduBunch0,SenduBunch1,
								SendHdr);
signal IntTrigSeq : Trig_Tx_State;

-- Time stamp FIFO
signal TStmpBuff_Out : std_logic_vector (31 downto 0);
signal TStmpBuff_wr_en,TStmpBuff_rd_en,TStmpBuff_Full,TStmpBuff_Emtpy : std_logic;
signal TStmpWds : std_logic_vector (10 downto 0);

-- FEB active register
signal ActiveReg : std_logic_vector (23 downto 0);
signal FPGA234_Active : Array_3x8;
signal ActiveCE : std_logic_vector(2 downto 0);
-- Controller ID register
signal IDReg : std_logic_vector (3 downto 0); 
-- FEB ID DPRam signals 
signal FEBID_addra,FEBID_addrb : std_logic_vector (4 downto 0); 
signal FEBID_doutb : std_logic_vector (15 downto 0); 
signal FEBID_wea : std_logic_vector (0 downto 0); 
-- Register that is the "OR" of all the status words from the FEBs
signal StatOr, Stat0 : std_logic_vector (7 downto 0); 


-- Link counters

begin

Sys_Pll : SysPll
  port map(
 -- Clock in ports
    CLK_IN1_P => ClkB_P,
    CLK_IN1_N => ClkB_N,
-- Clock out ports
    CLK_OUT1 => SysClk,   -- 100 MHz
    CLK_OUT2 => EthClk,   -- 125 MHz used for Orange Tree I/O
	 CLK_OUT3 => nEthClk,  -- 125 MHz 180 deg. phase fro DDR In
	 CLK_OUT4 => Clk80MHz, -- 80 MHz for 20mbit FM transmitter
-- Status and control signals
    RESET  => ResetHi,
    LOCKED => Pll_Locked);

HrtBtData(19 downto 0) <= uBunchCount(19 downto 0);
HrtBtData(20) <= Beam_On;
HrtBtData(21) <= '0' when uBunchCount(31 downto 20) = 0 else '1';
HrtBtData(23 downto 22) <= "00";
-- FM transmitter for boadcasting microbunch numbers to the FEBs
HeartBeatTx : FM_Tx 
	generic map (Pwidth => 24)
		 port map(clock => Clk80MHz, 
					 reset => ResetHi,
					 Enable => HrtBtFMTxEn,
					 Data => HrtBtData, 
					 Tx_Out => HrtBtTxOuts);
HeartBeatFM <= HrtBtTxOuts.FM;


-- FIFO for buffering broadcast trigger requests, 
DReqBuff : FIFO_SC_1kx16
  PORT MAP (
    clk => Clk80MHz,
    rst => ResetHI,
    din => DReqBuff_In,
    wr_en => DReqBuff_wr_en,
    rd_en => DReqBuff_rd_en,
    dout => DReqBuff_Out,
    full => DReqBuff_Full,
    empty => DReqBuff_Emtpy,
    data_count => TrgPktCnt
  );


-- Queue up time stamps for later checking
TimeStampBuff : TrigPktBuff
  PORT MAP (rst => ResetHI,
    clk => Clk80MHz,
    din => TriggerTimestampCounter,
    wr_en => TStmpBuff_wr_en,
    rd_en => TStmpBuff_rd_en,
    dout => TStmpBuff_Out,
    full => TStmpBuff_Full,
    empty => TStmpBuff_Emtpy,
	 data_count => TStmpWds);

-- DP Ram for storing FEB addresses
FEBIDList : FEBIDListRam
  PORT MAP (clka => SysClk,
    wea => FEBID_wea,
    addra => FEBID_addra,
    dina => uCD,
    clkb => SysClk,
    rstb => ResetHi,
    addrb => FEBID_addrb,
    doutb => FEBID_doutb);

EventBuff: FIFO_SC_4Kx16
  port map (clk => SysClk,
		rst => ResetHi,
		wr_en => EventBuff_WrtEn,
		rd_en => EventBuff_RdEn,
      din => EventBuff_Dat,
      dout => EventBuff_Out,
      full => EventBuff_Full,
	   empty => EventBuff_Empty);

-- GTP event handling process
TrigReqTx : process (SysClk, CpldRst)

begin

 if CpldRst = '0' then 

	Event_Builder <= Idle;
	LinkFIFORdReq <= (others =>'0'); StatOr <= X"00";  Stat0 <= X"00";
	EventBuff_RdEn <= '0';
	FIFOCount <= (others => (others => '0')); EventBuff_WrtEn <= '0';
	ActiveReg <= X"000000"; LinkFIFOStatReg <= "000";
	AddrReg <= (others =>'0');
	FormHold <= '1'; -- in this version of the firmware, event builder won't run.
	FormRst <= '0';

elsif rising_edge (SysClk) then

	if (uCWR = '0' or uCRD = '0') and CpldCS = '0'
	then
		AddrReg <= uCA;
	else
		AddrReg <= AddrReg;
	end if;

	LinkRDDL(0) <= not CpldCS and not uCRD;
	LinkRDDL(1) <= LinkRDDL(0);


---------------------------------------------------------------------------
-- Idle,RdInWdCnt0,RdInWdCnt1,RdInWdCnt2,SumWdCnt,WrtWdCnt,RdStat0,
-- RdStat1,RdStat2,WrtStat,WaitEvent,ReadFIFO0,ReadFIFO1,ReaddFIFO2
---------------------------------------------------------------------------
Case Event_Builder is
	when Idle => --Debug(10 downto 7) <= X"0";
		if LinkFIFOEmpty /= 7 and FormHold = '0' and TStmpWds >= 3 
		 then Event_Builder <= WaitEvent;
		else Event_Builder <= Idle;
		end if;
	when WaitEvent => --Debug(10 downto 7) <= X"1";
			-- Wait for a complete event to be in all link FIFOs from active ports
	    if ((LinkFIFOOut(0)(12 downto 0) <= LinkFIFORdCnt(0) and LinkFIFOEmpty(0) = '0') or ActiveReg(7 downto 0) = 0)
	   and ((LinkFIFOOut(1)(12 downto 0) <= LinkFIFORdCnt(1) and LinkFIFOEmpty(1) = '0') or ActiveReg(15 downto 8) = 0)
	   and ((LinkFIFOOut(2)(12 downto 0) <= LinkFIFORdCnt(2) and LinkFIFOEmpty(2) = '0') or ActiveReg(23 downto 16) = 0) 
	    then
		 if ActiveReg(15 downto 0) = 0 then Event_Builder <= RdInWdCnt2;
	  elsif ActiveReg(7 downto 0) = 0 then Event_Builder <= RdInWdCnt1;
	  else Event_Builder <= RdInWdCnt0;
	  end if;
	  elsif FormRst = '1' then Event_Builder <= Idle; 
	  else Event_Builder <= WaitEvent;
	end if;
 -- Read in three word counts in order to sum into a controller word count
	when RdInWdCnt0 => --Debug(10 downto 7) <= X"2"; 
		  if ActiveReg(23 downto 8) = 0 then Event_Builder <= SumWdCnt;
	  elsif ActiveReg(15 downto 8) = 0 then Event_Builder <= RdInWdCnt2;
	  else Event_Builder <= RdInWdCnt1;
	  end if;
	when RdInWdCnt1 => --Debug(10 downto 7) <= X"3";
		if ActiveReg(23 downto 16) = 0 then Event_Builder <= SumWdCnt;
			else Event_Builder <= RdInWdCnt2;
		end if;
	when RdInWdCnt2 => --Debug(10 downto 7) <= X"4";
			Event_Builder <= SumWdCnt;
-- Subtract 2 from each link word count FIFO to account for the word count and status words
	when SumWdCnt => --Debug(10 downto 7) <= X"5"; 
			Event_Builder <= WrtWdCnt;
-- Write the controller word count	
	when WrtWdCnt => --Debug(10 downto 7) <= X"6"; 
		if ActiveReg(15 downto 0) = 0 then Event_Builder <= RdStat2;
	elsif ActiveReg(7 downto 0) = 0 then Event_Builder <= RdStat1;
	  else Event_Builder <= RdStat0;
	end if;  
-- Read the status from the link FIFOs
	when RdStat0 => --Debug(10 downto 7) <= X"7";
		if ActiveReg(23 downto 8) = 0 then Event_Builder <= WrtStat;
	elsif ActiveReg(15 downto 8) = 0 then Event_Builder <= RdStat2;
	   else Event_Builder <= RdStat1;
		end if;
	when RdStat1 => --Debug(10 downto 7) <= X"8"; 
			if ActiveReg(23 downto 16) = 0 then Event_Builder <= WrtStat;
			else Event_Builder <= RdStat2;
			end if;
	when RdStat2 => --Debug(10 downto 7) <= X"9"; 
		Event_Builder <= WrtStat;
-- Write the "OR" of the status as the controller status word
	when WrtStat => --Debug(10 downto 7) <= X"A";
-- Skip over any Link that has no data
			if FIFOCount(0) /= 0 and ActiveReg(7 downto 0) /= 0 
				then Event_Builder <= ReadFIFO0;
	   elsif FIFOCount(0) = 0 and FIFOCount(1) /= 0 
				and ActiveReg(15 downto 8) /= 0 
				then Event_Builder <= ReadFIFO1; 
	   elsif FIFOCount(0) = 0 and FIFOCount(1) = 0 
				and FIFOCount(2) /= 0 and ActiveReg(23 downto 16) /= 0  
				then Event_Builder <= ReadFIFO2; 
	   else Event_Builder <= Idle;
		end if;
-- Read the data words from the three link FIFOs in succession
	 when ReadFIFO0 => --Debug(10 downto 7) <= X"B";
		if FIFOCount(0) = 1 or FIFOCount(0) = 0 then  
-- Skip over any Link that has no data
				if FIFOCount(1) /= 0 and ActiveReg(15 downto 8) /= 0  
				  then Event_Builder <= ReadFIFO1; 
				 elsif FIFOCount(1) = 0 and FIFOCount(2) /= 0 and ActiveReg(23 downto 16) /= 0 
				  then Event_Builder <= ReadFIFO2;
		       else Event_Builder <= Idle;
		      end if;
		  elsif FormRst = '1' then Event_Builder <= Idle;
		else Event_Builder <= ReadFIFO0;
		end if;
	 when ReadFIFO1 => --Debug(10 downto 7) <= X"C";
		if FIFOCount(1) = 1 or FIFOCount(1) = 0 then
-- Skip over any Link that has no data
			 if FIFOCount(2) /= 0  and ActiveReg(23 downto 16) /= 0  
			   then Event_Builder <= ReadFIFO2;
		     else Event_Builder <= Idle;
			 end if;
		 elsif FormRst = '1' then Event_Builder <= Idle; 
		else Event_Builder <= ReadFIFO1;
		end if;
	 when ReadFIFO2 => --Debug(10 downto 7) <= X"D";
		if FIFOCount(2) = 1 or FIFOCount(2) = 0 
			then Event_Builder <= Idle;
		 elsif FormRst = '1' then Event_Builder <= Idle; 
		else Event_Builder <= ReadFIFO2;
		end if;
	 when others => --Debug(10 downto 7) <= X"E";
	   Event_Builder <= Idle;
  end case;

-- Sum the word counts from the three Link FIFOs.
		if Event_Builder = Idle then EventSum <= (others => '0');
-- Account for removing the word count and status words from the data
	elsif Event_Builder = RdInWdCnt0 then EventSum <= LinkFIFOOut(0) - 2;
	elsif Event_Builder = RdInWdCnt1 then EventSum <= EventSum + (LinkFIFOOut(1) - 2);
	elsif Event_Builder = RdInWdCnt2 then EventSum <= EventSum + (LinkFIFOOut(2) - 2);
	else EventSum <= EventSum;
	end if;

-- Select the data source for the event buffer FIFO
	   if Event_Builder = WrtWdCnt then EventBuff_Dat <= EventSum;
	elsif Event_Builder = WrtStat	then EventBuff_Dat <= Stat0 & StatOR;
	elsif LinkFIFORdReq(0) = '1' then EventBuff_Dat <= LinkFIFOOut(0);
	elsif LinkFIFORdReq(1) = '1' then EventBuff_Dat <= LinkFIFOOut(1);
	elsif LinkFIFORdReq(2) = '1' then EventBuff_Dat <= LinkFIFOOut(2);
	else EventBuff_Dat <= EventBuff_Dat;
	end if;

-- Do an "or" of the FEB error words for the cotroller error word
   if Event_Builder = RdStat0 then 
				StatOr <= StatOr or LinkFIFOOut(0)(7 downto 0);
				Stat0  <=           LinkFIFOOut(0)(7 downto 0);
elsif Event_Builder = RdStat1 then 
				StatOr <= StatOr or LinkFIFOOut(1)(7 downto 0);
				Stat0 <= Stat0;
elsif Event_Builder = RdStat2 then 
				StatOr <= StatOr or LinkFIFOOut(2)(7 downto 0);
				Stat0 <= Stat0;
else StatOr <= StatOr; Stat0 <= Stat0;
end if;

--Copy port activity bits from the other FPGAs to this register
if TrigTx_Sel = '1' 
   then 
		if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = ActvRegAddrHi 
		  then ActiveReg <= uCD(7 downto 0) & ActiveReg(15 downto 0);
	 elsif WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = ActvRegAddrLo 
		then ActiveReg <= ActiveReg(23 downto 16) & uCD;
	  else ActiveReg <= ActiveReg;
	 end if;
   else ActiveReg <= FPGA234_Active(2) & FPGA234_Active(1) & FPGA234_Active(0);
end if;

-- Count down the words read from each of the link FIFOs
	if Event_Builder = RdInWdCnt0 then FIFOCount(0) <= LinkFIFOOut(0) - 2;
	elsif Event_Builder = ReadFIFO0 and FIFOCount(0) /= 0 
						then FIFOCount(0) <= FIFOCount(0) - 1;
	else FIFOCount(0) <= FIFOCount(0);
	end if;

	if Event_Builder = RdInWdCnt1 then FIFOCount(1) <= LinkFIFOOut(1) - 2;
	elsif Event_Builder = ReadFIFO1 and FIFOCount(1) /= 0 
						then FIFOCount(1) <= FIFOCount(1) - 1;
	else FIFOCount(1) <= FIFOCount(1);
	end if;

	if Event_Builder = RdInWdCnt2 then FIFOCount(2) <= LinkFIFOOut(2) - 2;
	elsif Event_Builder = ReadFIFO2 and FIFOCount(2) /= 0 
						then FIFOCount(2) <= FIFOCount(2) - 1;
	else FIFOCount(2) <= FIFOCount(2);
	end if;

-- Link FIFO reads
-- Microcontroller read
   if (LinkRDDL = 2 and AddrReg(11 downto 10) = GA and AddrReg(9 downto 0) = LinkRdAddr(0))
-- Read of header words, read of data words
   or Event_Builder = RdInWdCnt0 or Event_Builder = RdStat0 or Event_Builder = ReadFIFO0
 	then LinkFIFORdReq(0) <= '1'; 
	else LinkFIFORdReq(0) <= '0'; 
	end if;

 if (LinkRDDL = 2 and AddrReg(11 downto 10) = GA and AddrReg(9 downto 0) = LinkRdAddr(1))
-- Read of header words, read of data words
   or Event_Builder = RdInWdCnt1 or Event_Builder = RdStat1 or Event_Builder = ReadFIFO1
	then LinkFIFORdReq(1) <= '1'; 
	else LinkFIFORdReq(1) <= '0'; 
	end if;

 if (LinkRDDL = 2 and AddrReg(11 downto 10) = GA and AddrReg(9 downto 0) = LinkRdAddr(2))
-- Read of header words, read of data words
   or Event_Builder = RdInWdCnt2 or Event_Builder = RdStat2 or Event_Builder = ReadFIFO2
	then LinkFIFORdReq(2) <= '1'; 
	else LinkFIFORdReq(2) <= '0'; 
	end if;

 if Event_Builder = Idle then EvBuffWrtGate <= '0';
 elsif Event_Builder = WrtStat then EvBuffWrtGate <= '1';
 else EvBuffWrtGate <= EvBuffWrtGate;
 end if;

 if Event_Builder = WrtWdCnt or Event_Builder = WrtStat 
	or (LinkFIFORdReq /= 0 and EvBuffWrtGate = '1')
   then EventBuff_WrtEn <= '1'; --Debug(6) <= '1';
  else EventBuff_WrtEn <= '0';  --Debug(6) <= '0';
 end if;

if (1 = 0)
then EventBuff_RdEn <= '1';  --Debug(9) <= '1';
else EventBuff_RdEn <= '0';  --Debug(9) <= '0';
end if;

--Debug(4) <= EventBuff_Empty;

end if; -- CpldRst

end process;

-- DG: new process -- handles generation of Data Request from external LEMO-NIM trigger
-- buffers data request in register so uC can readout and send request to FEB's
gen_datareq : process (Clk80MHz, CpldRst)
begin
	if CpldRst = '0'
	then 
		IntTrigSeq <= Idle; 
		DReqBuff_wr_en <= '0'; 

	elsif rising_edge (Clk80MHz)
	then

		-- SBND Internal Data Request Generator
		-- Simplified from mu2e version by removing all fiber logic.
		-- Requires Modified mode in uC code
		-- (Now: Idle,SenduBunch0,SenduBunch1,SendHdr)
		-- (Was: Idle,SendTrigHdr,SendPktType,SendPad0,SenduBunch0,SenduBunch1,
		--		SenduBunch2,SendPad1,SendPad2,SendPad3,SendCRC)
		Case IntTrigSeq is
			when Idle =>
			  if Trig_Tx_Req = '1' then IntTrigSeq <= SendHdr;
			  else IntTrigSeq <= Idle;
			  end if;
			when  SendHdr=> IntTrigSeq <= SenduBunch0;
			when SenduBunch0 => IntTrigSeq <= SenduBunch1;
			when SenduBunch1 => IntTrigSeq <= Idle;
			when others => IntTrigSeq <= Idle;  
		end Case;


		if IntTrigSeq = SenduBunch0  
		then
			DReqBuff_In <= uBunchCount(15 downto 0);
			DReqBuff_wr_en <= '1';

		elsif IntTrigSeq = SenduBunch1  
		then
			DReqBuff_In <= uBunchCount(31 downto 16);
			DReqBuff_wr_en <= '1';

		elsif IntTrigSeq = SendHdr  
		then
			DReqBuff_In <= x"F0F0";
			DReqBuff_wr_en <= '1';
		else
				DReqBuff_wr_en <= '0';
		end if; --fsm
		
		--	Read of the trigger request FIFO
		if FMRDDL = 2 and AddrReg(11 downto 10) = GA and AddrReg(9 downto 0) = TRigReqBuffAd 
		then
			DReqBuff_rd_en <= '1';
		else
			DReqBuff_rd_en <= '0';
		end if;

	end if; --clk

end process;

-- Fifo for buffering microcontoller parallel data prior to serializing
-- Used for controlling the 96 harmonica jack LEDs
CMDFifo : CMD_Fifo
  PORT MAP (clk => SysClk,
    rst => ResetHi,
    din(18 downto 16) => uCA(2 downto 0),
	 din(15 downto 0) => uCD,
    wr_en => CMDwr_en,
    rd_en => CMDrd_en,
    dout => CMD_Out,
    full => CMD_Full,empty => CMD_Empty);

-- Fifo for buffering microcontoller parallel data prior to serializing
-- Used for the stup registers on the ADF4001
PLLBuff: PLL_Buff
  PORT MAP (clk => SysClk,
    rst => ResetHi,
    din(23 downto 16) => PllStage,
	 din(15 downto 0) => uCD,
    wr_en => PLLBuffwr_en,
    rd_en => PLLBuffrd_en,
    dout => PLLBuff_Out,
    full => PLLBuff_full,
    empty => PLLBuff_empty);

---------------------------------------------------------------------------
-- Logic for the serial inputs from the three FPGAs attached to the FEBs --
---------------------------------------------------------------------------
GenSerdes : for i in 0 to 2 generate

-- Collect two data lanes and a frame signal into a three bit vector
-- Deserialize x 5
SerDesInP(i) <= (LinkFR_P(i) & LinkSDat_P(2*i+1) & LinkSDat_P(2*i));
SerDesInN(i) <= (LinkFR_N(i) & LinkSDat_N(2*i+1) & LinkSDat_N(2*i));

-- Deserializer macro refer to XAPP1064
LVDSInClk0 : serdes_1_to_n_clk_ddr_s8_diff generic map(
      	S			=> S) 		
port map (
	clkin_p   		=> LINKClk_P(i),
	clkin_n   		=> LINKClk_N(i),
	rxioclkp    	=> rxioclkp(i),
	rxioclkn   		=> rxioclkn(i),
	rx_serdesstrobe => rx_serdesstrobe(i),
	rx_bufg_x1		=> RxOutClk(i));

-- Data Inputs
LVDSInDat0 : serdes_1_to_n_data_ddr_s8_diff generic map(
      	S		=> S,			
      	D		=> D)
port map (                   
	use_phase_detector 	=> '1',	-- '1' enables the phase detector logic
	datain_p     	=> SerDesInP(i),
	datain_n     	=> SerDesInN(i),
	rxioclkp    	=> rxioclkp(i),
	rxioclkn   		=> rxioclkn(i),
	rxserdesstrobe => rx_serdesstrobe(i),
	gclk    		=> RxOutClk(i), -- this clock is assymmetric beacuse of the odd serialization factor
	bitslip   	=> SlipReq(i),
	reset   		=> SerdesRst,
  data_out(14 downto 10)  => LinkFRDat(i),
  data_out(9) => LinkPDat(i)(1)(0), -- the serial data goes out msb first, comes in lsb first
  data_out(8) => LinkPDat(i)(1)(1), -- so bit order needs to be reversed
  data_out(7) => LinkPDat(i)(1)(2),
  data_out(6) => LinkPDat(i)(1)(3),
  data_out(5) => LinkPDat(i)(1)(4),
  data_out(4) => LinkPDat(i)(0)(0),
  data_out(3) => LinkPDat(i)(0)(1),
  data_out(2) => LinkPDat(i)(0)(2),
  data_out(1) => LinkPDat(i)(0)(3),
  data_out(0) => LinkPDat(i)(0)(4),
  debug_in  	=> "00",
  debug    		=> open);

-- Extract the eight payload bits from the 10 bit parallel data fron the deserializer
-- Three lower bits from lane 1 and 5 bits from lane 0
LinkBuff : LinkFIFO
  port map (rst => LinkBuffRst, wr_clk => RxOutClk(i), rd_clk => SysClk, 
    wr_en => LinkFIFOWrReq(i),rd_en => LinkFIFORdReq(i),
    din(15 downto 13) => LinkPDat(i)(1)(7 downto 5),
    din(12 downto 8) => LinkPDat(i)(0)(9 downto 5),
    din( 7 downto 5) => LinkPDat(i)(1)(2 downto 0),
    din( 4 downto 0) => LinkPDat(i)(0)(4 downto 0),
    dout => LinkFIFOOut(i), empty => LinkFIFOEmpty(i),
	 full => LinkFIFOFull(i),
	 rd_data_count => LinkFIFORdCnt(i));

end generate;

--------------- Logic clocked with Serdes receive clocks ---------------

-- Three links from 3 FPGAs. Two lane serial with a 50MHz frame and 250MHz 
-- double data rate clock. V-Valid flag, D-high byte d-low byte

-- Clk0    -_-_-_-_-_-_-_-_-_-_
-- Frame0  -----_____-----_____
-- Lane 01 V1DDDV1dddV1DDDV1ddd
-- Lane 00 DDDDDdddddDDDDDddddd

-- Clk1    -_-_-_-_-_-_-_-_-_-_
-- Frame1  _____-----_____-----
-- Lane 11 V1DDDV1dddV1DDDV1ddd
-- Lane 10 DDDDDdddddDDDDDddddd

-- Clk2    -_-_-_-_-_-_-_-_-_-_
-- Frame2  -----_____-----_____
-- Lane 21 V1DDDV1dddV1DDDV1ddd
-- Lane 20 DDDDDdddddDDDDDddddd

GenLinkBuff : for i in 0 to 2 generate

LinkBuff : process (RxOutClk(i), CpldRst)

begin

 if CpldRst = '0' then 
 
	LinkPDat(i)(1)(9 downto 5) <= (others => '0'); 
	LinkPDat(i)(0)(9 downto 5) <= (others => '0'); 
	LinkFIFOWrReq(i) <= '0'; FPGA234_Active(i) <= (others => '0'); 
	SlipReq(i) <= '0'; Slippause(i) <= X"0";  ActiveCE(i) <= '0';

elsif rising_edge (RxOutClk(i)) then

-- Engage bit slip if shifted in framing signal isn't all 1's or all 0's
	if LinkFRDat(i) /= 0 and LinkFRDat(i) /= 31 and Slippause(i) = 0
	then Slippause(i) <= X"F";
	elsif Slippause(i) /= 0
	then  Slippause(i) <= Slippause(i) - 1;
	else Slippause(i) <= Slippause(i);
	end if;

-- Allow time between requests for bit slip to take effect
	if Slippause(i) = X"F" then SlipReq(i) <= '1';	
	else SlipReq(i) <= '0';
	end if;

-- Copy five bit shift result to five bit register to form a 10 bit result
	LinkPDat(i)(1)(9 downto 5) <= LinkPDat(i)(1)(4 downto 0);
	LinkPDat(i)(0)(9 downto 5) <= LinkPDat(i)(0)(4 downto 0);

-- Link Frame1 is reversed. Deal with that here.
if i = 1 then
-- Write to the Link FIFO when the frame signal indicates data is word aligned
	if LinkFRDat(i) = 0 and LinkPDat(i)(1)(4 downto 3) = "11"
			then LinkFIFOWrReq(i) <= '1';
	      else LinkFIFOWrReq(i) <= '0';
	end if;
-- Use the spare link receive bit to retrieve activity bits from the fron FPGAs
	if LinkFRDat(i) = 0 and LinkPDat(i)(1)(4 downto 3) = "10"
	  then ActiveCE(i) <= '1'; 
	 else  ActiveCE(i) <= '0';
	end if;
else
	if LinkFRDat(i) = 31 and LinkPDat(i)(1)(4 downto 3) = "11"
		then LinkFIFOWrReq(i) <= '1';
	   else LinkFIFOWrReq(i) <= '0';
	end if;
	if LinkFRDat(i) = 31 and LinkPDat(i)(1)(4 downto 3) = "10"
	  then ActiveCE(i) <= '1'; 
	 else  ActiveCE(i) <= '0';
	end if;
end if;

if ActiveCE(i) = '1' then 
     FPGA234_Active(i)(7 downto 5) <= LinkPDat(i)(1)(2 downto 0);
	  FPGA234_Active(i)(4 downto 0) <= LinkPDat(i)(0)(4 downto 0);
else 
	  FPGA234_Active(i) <= FPGA234_Active(i);
end if;
--    din( 7 downto 5) => LinkPDat(i)(1)(2 downto 0),
--    din( 4 downto 0) => LinkPDat(i)(0)(4 downto 0),

end if; -- CpldRst = '0'

end process;

end generate;

-- Reset for the input deserializer
SerdesRst <= '1' when CpldRst = '0' 
  	                or (CpldCS = '0' and uCWR = '0' and uCA = LinkCSRAddr and uCD(8) = '1') else '0';
LinkBuffRst <= '1' when CpldRst = '0' 
  	                or (CpldCS = '0' and uCWR = '0' and uCA = LinkCSRAddr and uCD(9) = '1') else '0';

ResetHi <= not CpldRst;  -- Generate and active high reset for the Xilinx macros

----------------------- Orange tree interface logic -----------------------------

 DQ <= DQWrtDly(2) when DQEn = '1' else (others => 'Z'); 
iDQ <= DQ when EthRDDL(4 downto 3) = 1 else iDQ;

EthProc : process(EthClk, CpldRst)

 begin 

-- asynchronous reset/preset
 if CpldRst = '0' then

	ZEthClk <= '0'; EthWRDL <= (others => '0');
	DQWrtDly <= (others => (others => '0'));
	ZEthA <= (others => '0'); DQEn <= '0';
	ZEthCS <= '1'; ZEthWE <= '1'; 
	ZEthBE <= "11"; EthRDDL <= (others => '0');
	GPO(0) <= '0'; 

 elsif rising_edge (EthClk) then 


	ZEthClk <= not ZEthClk;

-- Strobe timing delay chains
-- Write strobe timer
	if ZEthClk = '1' then
	 EthWRDL(0) <= not EthCS and not uCWR;
	 EthWRDL(1) <= EthWRDL(0);
	 EthWRDL(2) <= EthWRDL(1);
	 EthWRDL(3) <= EthWRDL(2);
	 EthWRDL(4) <= EthWRDL(3);
	else EthWRDL <= EthWRDL;
	end if; 

-- Read strobe timer
	if ZEthClk = '1' then
	 EthRDDL(0) <= not EthCS and not uCRD;
	 EthRDDL(1) <= EthRDDL(0);
	 EthRDDL(2) <= EthRDDL(1);
	 EthRDDL(3) <= EthRDDL(2);
	 EthRDDL(4) <= EthRDDL(3);
	else EthRDDL <= EthRDDL;
	end if; 

 -- Write data pipeline
if EthCS = '0' and uCWR = '0' then DQWrtDly(0) <= uCD;
 else DQWrtDly(0) <= DQWrtDly(0);
 end if;
 
 if ZEthClk = '1' then 
		DQWrtDly(1) <= DQWrtDly(0);
		DQWrtDly(2) <= DQWrtDly(1);
 else DQWrtDly(1) <= DQWrtDly(1);
		DQWrtDly(2) <= DQWrtDly(2);
  end if;

-- Tri state enable for read data
 if ZEthClk = '1' and DQEn = '0' and EthWRDL(2 downto 1) = 1 then DQEn <= '1'; 
  elsif ZEthClk = '1' and DQEn = '1' and EthWRDL(4 downto 3) = 1 then DQEn <= '0';
   else DQEn <= DQEn; 
 end if;

-- Chip enable and byte select
	if ZEthClk = '1' and ZEthCS = '1' and (EthWRDL(1 downto 0) = 1 or EthRDDL(1 downto 0) = 1) then 
		ZEthCS <= '0'; 
-- use a specific address to access any trailing bytes from a series of word accesses
		if uCA(8) = '0' and uCA(3 downto 0) = "1001" then 
				ZEthBE <= "01";
			else 
				ZEthBE <= "00";
			end if;
	elsif ZEthClk = '1' and ZEthCS = '0' and (EthWRDL(2 downto 1) = 1 or EthRDDL(2 downto 1) = 1) then
		ZEthCS <= '1';
		ZEthBE <= "11";
	else ZEthCS <= ZEthCS;
		 ZEthBE <= ZEthBE;
	end if;

-- Latch the address
	if EthCS = '0' and (uCWR = '0' or uCRd = '0') and uCA(8) = '0' and uCA(3 downto 0) = "1001"  
		then ZEthA <= uCA(8 downto 1) & '0'; 
	elsif EthCS = '0' and (uCWR = '0' or uCRd = '0') and (uCA(8) = '1' or uCA(3 downto 0) /= "1001")
		then ZEthA <= uCA(8 downto 0); 
	else ZEthA <= ZEthA; 
	end if;

-- Write strobe
		if ZEthClk = '1' and ZEthWE = '1' and EthWRDL(1 downto 0) = 1 
			then ZEthWE <= '0'; 
	elsif (ZEthClk = '1' and ZEthWE = '0' and EthWRDL(2 downto 1) = 1) or uCRd = '0'
			then ZEthWE <= '1'; 
	else ZEthWE <= ZEthWE;
	end if;

end if;

end process;

HrtBtFMTxReq : process(Clk80MHz, CpldRst)

begin 
-- asynchronous reset/preset
	if CpldRst = '0' then
		HrtBtFMTxEn <= '0'; 
		uBunchCount <= (others => '0');
		NimTrigBUF <= "000";
		ManTrig <= '0';
		PPSBuf <= "000";
		TriggerTimestampCounter <= (others => '0');
		TriggerReq <= '0';
		TriggerHoldoffCounter <= x"0000";
		TriggerHoldoff <= x"1000";
		ResetTrgCntOnPPS <= '0'; ResetTimestampOnPPS <= '0';
		Beam_On <= '0'; 
		FMRDDL <= "00";
		FMWRDL <= "00";
		
		TStmpBuff_wr_en <= '0';
		TStmpBuff_rd_en <= '0';
		Trig_Tx_Req <= '0';



	elsif rising_edge(Clk80MHz) then


		FMRDDL(0) <= not uCRD and not CpldCS;
		FMRDDL(1) <= FMRDDL(0);

		FMWRDL(0) <= not uCWR and not CpldCS;
		FMWRDL(1) <= FMWRDL(0);

		-- Write to TriggerControl D0 to manually trigger
		if FMWRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = TriggerHoldoffAddress
		then
			TriggerHoldoff <= uCD;
		else
			TriggerHoldoff <= TriggerHoldoff;
		end if;

		-- Write to TriggerControl D0 to manually trigger
		if FMWRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = TriggerControlAddress and uCD(0)='1'
		then 
			ManTrig <='1';
		else
			ManTrig <= '0';
		end if;


		-- Write to TriggerControl D2 to manually reset the trigger timestamp. A PPS may reset this every second in the future
		if (FMWRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = TriggerControlAddress and uCD(2)='1')
				or (ResetTimestampOnPPS = '1' and PPSBuf(2 downto 1) = 1)
		then
			TriggerTimestampCounter <= (others => '0');
		else
			TriggerTimestampCounter <= TriggerTimestampCounter + 1;
		end if;

		-- Buffer the external (active lo) NimTrig signal to align to clock signal
		NimTrigBUF(0) <= not NimTrig;
		NimTrigBUF(1) <= NimTrigBUF(0);
		NimTrigBUF(2) <= NimTrigBUF(1);

		PPSBuf(0) <= GPI;
		PPSBuf(1) <= PPSBuf(0);
		PPSBuf(2) <= PPSBuf(1);

		-- Trigger goes high on rising edge of either Nim or Man trig bufs, as long as there isnt a recent trigger
		if (NimTrigBuf(2 downto 1) = 1 or ManTrig = '1') and (TriggerHoldoffCounter = 0)
		then
			TriggerReq <= '1';
		else
			TriggerReq <= '0';
		end if;
		
		if TriggerReq = '1'		
		then
			TriggerHoldoffCounter <= TriggerHoldoff;
		elsif TriggerHoldoffCounter > 0 
		then
			TriggerHoldoffCounter <= TriggerHoldoffCounter - 1;
		else
			TriggerHoldoffCounter <= x"0000";
		end if;


		if (FMWRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = TriggerControlAddress and uCD(1)='1')
				or (ResetTrgCntOnPPS = '1' and PPSBuf(2 downto 1) = 1)
		then
			uBunchCount <= (others => '0');
		elsif TriggerReq = '1' then
			uBunchCount <= uBunchCount + 1;
		else
			uBunchCount <= uBunchCount;
		end if;


		-- Actually trigger the FM to send
		if TriggerReq = '1'
		then
			HrtBtFMTxEn <= '1';
			Trig_Tx_Req <= '1';
		else
			HrtBtFMTxEn <= '0';
			Trig_Tx_Req <= '0';
		end if;


	end if; 


end process;
	
----------------------- 100 Mhz clocked logic -----------------------------

Debug (2 downto 1) <= RDDL;
Debug (4 downto 3) <= WRDL;
Debug (5) <= TestCount(0);
Debug (8 downto 6) <= uCA(2 downto 0);
Debug (10 downto 9) <= uCD(1 downto 0);

main : process(SysClk, CpldRst)
begin 

-- asynchronous reset/preset
	if CpldRst = '0'
	then
		--Setup unused pins
		TrigLED <= '0';
		TDisA <= '0';
		TDisB <= '0';
		GPO(1) <= '0';
		TrigFM <= '0';
		
		-- Synchronous edge detectors for various strobes
		RDDL <= "00"; WRDL <= "00"; 

		-- Trigger and spill generator logic
		FreqReg <= X"00000237"; PhaseAcc <= (others => '0');
		Buff_Rst <= '0'; Seq_Rst <= '0'; 
		EventWdCnt <= (others => '0');  BmOnTrigReq <= '0';

		UpTimeStage <= (others => '0'); UpTimeCount <= (others => '0');

		LEDRst <= '1'; LEDSDat <= "000"; LEDSClk <= "000"; LEDLd <= "000000";
		uBunchLED <= '0'; 

		CMDwr_en <= '0'; CMDrd_en <= '0'; 
		ClkDiv <= "000"; CMDBitCount <= (others => '0'); 
		LEDShiftReg <= (others => '0');	LED_Shift <= Idle;
		DReqBuff_uCRd <= '0'; LinkBusy <= '0'; 
		DCSPktBuff_uCRd <= '0'; 
		-- Pll Chip Shifter signals
		PLLBuffwr_en <= '0'; PLLBuffrd_en <= '0'; PllPDn <= '1';
		PllStage <= X"00"; PllShiftReg <= (others => '0'); 
		PllBitCount <= (others => '0'); Pll_Shift <= Idle;
		PllSClk <= '0'; PllSDat <= '0'; PllLd <= '0';

		TestCount <= (others => '0'); 

		FEBID_wea <= "0"; 
		FEBID_addra <= (others => '0'); FEBID_addrb <= (others => '0');

	elsif rising_edge (SysClk)
	then 

		-- Synchronous edge detectors for read and write strobes
		RDDL(0) <= not uCRD and not CpldCS;
		RDDL(1) <= RDDL(0);

		WRDL(0) <= not uCWR and not CpldCS;
		WRDL(1) <= WRDL(0);

		LinkBusy <= LinkFIFOEmpty(0) and LinkFIFOEmpty(1) and LinkFIFOEmpty(2);


		if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = IDregAddr 
		then
			IDReg <= uCD(3 downto 0);
		else
			IDReg <= IDReg;
		end if;

		-- 1 Second timer
		if	Counter1s /= Count1s
		then
			Counter1s <= Counter1s + 1;
		else
			Counter1s <= (others => '0');
		end if;

		-- Uptime in seconds since the last FPGA configure
		if	Counter1s = Count1s
		then
			UpTimeCount <= UpTimeCount + 1;
		else
			UpTimeCount <= UpTimeCount;
		end if;

		-- Register for staging uptime count.
		if CpldCS = '1'
		then
			UpTimeStage <= UpTimeCount;
		else
			UpTimeStage <= UpTimeStage;
		end if;

		-- Testcounter counter is writeable. For each read of the lower half, the entire
		-- 32 bit counter increments
		if    WRDL = 1 and  uCA(11 downto 10) = GA and uCA(9 downto 0) = TestCounterHiAd 
		then
			TestCount <= (uCD & TestCount(15 downto 0));
		elsif WRDL = 1 and  uCA(11 downto 10) = GA and uCA(9 downto 0) = TestCounterLoAd 
		then
			TestCount <= (TestCount(31 downto 16) & uCD);
		elsif RDDL = 2 and AddrReg(11 downto 10) = GA and AddrReg(9 downto 0) = TestCounterLoAd 
		then
			TestCount <= TestCount + 1;
		else
			TestCount <= TestCount;
		end if;


		-- Serializer for front panel LEDs
		ClkDiv <= ClkDiv + 1;

		if    WRDL = 1 and  uCA(11 downto 10) = GA 
					and uCA(9 downto 0) >= LEDDatAddr(0) and  uCA(9 downto 0) <= LEDDatAddr(5)
		then
			CMDwr_en <= '1';
		else
			CMDwr_en <= '0';
		end if;

		-- Idle,Load,Shift,SendPClk,RdFIFO 
		case LED_Shift is
		when Idle => 
			if CMD_Empty = '0' and ClkDiv = 7
			then 
				LED_Shift <= Load;
			elsif WRDL = 1 and  uCA(11 downto 10) = GA and uCA(9 downto 0) = LEDRstAddr
						and uCD(0) = '1'
			then
				LED_Shift <= WaitRst;
			else LED_Shift <= Idle;
			end if;
		when Load => 
			if ClkDiv = 7
			then 
				LED_Shift <= Shift;
			else
				LED_Shift <= Load;
			end if;
		when Shift =>
			if CMDBitCount = 0 and ClkDiv = 7
			then
				LED_Shift <= RdFIFO;
			else
				LED_Shift <= Shift;
			end if;
		when RdFIFO => 
			LED_Shift <= SendPClk;
		when WaitRst =>
			if ClkDiv = 7
			then
				LED_Shift <= SendRst;
			else
				LED_Shift <= WaitRst;
			end if;
		when SendRst =>
			if ClkDiv = 7
			then
				LED_Shift <= WaitPClk;
			else
				LED_Shift <= SendRst;
			end if;
		when WaitPClk =>
			if ClkDiv = 7
			then
				LED_Shift <= SendPClk;
			else
				LED_Shift <= WaitPClk;
			end if;
		when SendPClk => 
			if ClkDiv = 7
			then
				LED_Shift <= Idle;
			else
				LED_Shift <= SendPClk;
			end if;
		end case;

		if LED_Shift = SendRst
		then
			LEDRst <= '0';
		else
			LEDRst <= '1';
		end if;

		if LED_Shift = Load and ClkDiv = 7
		then
			CMDBitCount <= X"F";
		elsif LED_Shift = Shift and ClkDiv = 7
		then
			CMDBitCount <= CMDBitCount - 1;
		else
			CMDBitCount <= CMDBitCount;
		end if;

		if LED_Shift = Load and ClkDiv = 7
		then
			LEDShiftReg <= CMD_Out(15 downto 0);
		elsif LED_Shift = Shift and ClkDiv = 7
		then
			LEDShiftReg <= LEDShiftReg(14 downto 0) & '0';
		else
			LEDShiftReg <= LEDShiftReg;
		end if;

		Case CMD_Out(18 downto 17) is
		when "00" => 
			LEDSDat <= "00" & LEDShiftReg(15);
			LEDSClk <= "00" & ClkDiv(2); -- Clock overwritten in next piece of logic so it can always be 0 if not Shift
		when "01" =>
			LEDSDat <= '0' & LEDShiftReg(15) & '0';
			LEDSClk <= '0' & ClkDiv(2) & '0';
		when "10" => 
			LEDSDat <= LEDShiftReg(15) & "00";
			LEDSClk <= ClkDiv(2) & "00";
		when others =>
			LEDSDat <= "000";
			LEDSClk <= "000";
		end case;

		if LED_Shift /= Shift then 
			LEDSClk <= "000";
		end if;

		if LED_Shift = SendPClk then
			Case CMD_Out(18 downto 16) is
			when "000" =>
				LEDLd <= "000001";
			when "001" =>
				LEDLd <= "000010";
			when "010" =>
				LEDLd <= "000100";
			when "011" =>
				LEDLd <= "001000";
			when "100" =>
				LEDLd <= "010000";
			when "101" =>
				LEDLd <= "100000";
			when others =>
				LEDLd <= "000000";
			end case;
		else
			LEDLd <= "000000";
		end if;

		if LED_Shift = RdFIFO
		then
			CMDrd_en <= '1';
		else
			CMDrd_en <= '0';
		end if;

		-- Serializer for the PLL chip
		-- Pll data is 24 bits. Stage the upper order eight bits in a register
		if WRDL = 1 and  uCA(11 downto 10) = GA and uCA(9 downto 0) = PLLHiAddr
		then
			PllStage <= uCD(7 downto 0);
		else
			PllStage <= PllStage;
		end if;
		-- Apply the Staging register contents and the uC data bus to the FIFO input
		if WRDL = 1 and  uCA(11 downto 10) = GA and uCA(9 downto 0) = PLLLoAddr
		then
			PLLBuffwr_en <= '1';
		else
			PLLBuffwr_en <= '0';
		end if;

		-- Idle,Load,Shift,WaitLd,SendLd
		Case Pll_Shift is when 
		Idle => 
			if PLLBuff_empty = '0' and ClkDiv = 7 
			then
				Pll_Shift <= Load;
			else
				Pll_Shift <= Idle;
			end if;
		When Load =>
			if ClkDiv = 7
			then
				Pll_Shift <= Shift;
			else
				Pll_Shift <= Load;
			end if;
		When Shift =>
			if PllBitCount = 0 and ClkDiv = 7 
			then
				Pll_Shift <= WaitLd;
			else
				Pll_Shift <= Shift;
			end if;
		When WaitLd =>
			if ClkDiv = 7
			then
				Pll_Shift <= SendLd;
			else
				Pll_Shift <= WaitLd;
			end if;
		When SendLd =>
			if ClkDiv = 7
			then 
				Pll_Shift <= Idle;
			else
				Pll_Shift <= SendLd;
			end if;
		end Case;

		-- Pll Shifter bit counter
		if Pll_Shift = Load and ClkDiv = 7
		then
			PllBitCount <= '1' & X"7";
		elsif Pll_Shift = Shift and ClkDiv = 7
		then
			PllBitCount <= PllBitCount - 1;
		else
			PllBitCount <= PllBitCount;
		end if;

		-- Pll Shiftter shifter register
		if Pll_Shift = Load and ClkDiv = 7
		then
			PllShiftReg <= PLLBuff_Out;
		elsif Pll_Shift = Shift and ClkDiv = 7
		then
			PllShiftReg <= PllShiftReg(22 downto 0) & '0';
		else
			PllShiftReg <= PllShiftReg;
		end if;

		-- Read the PLL fifo at the end of the shift sequence
		if Pll_Shift = Load and ClkDiv = 7
		then
			PLLBuffrd_en <= '1'; 
		else
			PLLBuffrd_en <= '0'; 
		end if;

		-- PLL SPI port serial clock
		if Pll_Shift = Shift
		then
			PllSClk <= ClkDiv(2);
		else
			PllSClk <= '0';
		end if;

		PllSDat <= PllShiftReg(23); 

		-- Assert a load pulse after the shift sequence is done
		if Pll_Shift = SendLd
		then
			PllLd <= '1'; 
		else
			PllLd <= '0'; 
		end if;

		if WRDL = 1 and  uCA(11 downto 10) = GA and uCA(9 downto 0) = PLLPDnAddr 
		then
			PllPDn <= uCD(0);
		else
			PllPDn <= PllPDn;
		end if;

	end if; --rising edge

end process;

------------------- mux for reading back registers -------------------------

with uCA(9 downto 0) select

iCD <= X"0" & '0' & '0' & '0' & '0' & '0' & '0' 
		 & '0' & '0' & '0' & FormHold & '0' & '0' when CSRRegAddr,

		 X"000" & "000" & PllPDn when PLLPDnAddr,
		 DReqBuff_Out(15 downto 0) when TRigReqBuffAd,
		 X"0" & '0' & TrgPktCnt when TRigReqWdUsedAd,
		 X"00" & ActiveReg(23 downto 16) when ActvRegAddrHi,
		 ActiveReg(15 downto 0) when ActvRegAddrLo,
		 X"000" & IDReg when IDregAddr,
		 X"0" & "00" & Debug when DebugPinAd,
		 UpTimeStage(31 downto 16) when UpTimeRegAddrHi,
		 UpTimeStage(15 downto 0) when UpTimeRegAddrLo,
		 TestCount(31 downto 16) when TestCounterHiAd,
		 TestCount(15 downto 0) when TestCounterLoAd,
		 LinkFIFOOut(0) when LinkRdAddr(0),
		 LinkFIFOOut(1) when LinkRdAddr(1),
		 LinkFIFOOut(2) when LinkRdAddr(2),
		 X"00" & '0' & LinkFIFOFull & '0' & LinkFIFOEmpty when LinkCSRAddr,
		 "000" & LinkFIFORdCnt(0) when LinkWdCnt0Ad,
		 "000" & LinkFIFORdCnt(1) when LinkWdCnt1Ad,
		 "000" & LinkFIFORdCnt(2) when LinkWdCnt2Ad,
		 X"000" & "00" & EventBuff_Full & EventBuff_empty when EvBuffStatAd,
		 uBunchCount(47 downto 32) when MicroBunchAdHi,
		 uBunchCount(31 downto 16) when MicroBunchAdMid,
		 uBunchCount(15 downto 0) when MicroBunchAdLo,
		 TriggerHoldoff when TriggerHoldoffAddress,
		 X"0000" when others;

-- Select between the Orange Tree port and the rest of the registers
uCD <= iCD when uCRd = '0' and CpldCS = '0' and uCA(11 downto 10) = GA 
		 else iDQ when uCRd = '0' and EthCS = '0'  
		 else (others => 'Z');

end behavioural;
