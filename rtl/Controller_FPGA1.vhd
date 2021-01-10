-- Firmware for apex FPGA. SBND version

-- Sten Hansen Fermilab 10/15/2015
-- Daniel Mishins, Gray Putnam, Chris Barnes 2018-2020

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

signal ZEthANull0: std_logic;

-- Synchronous edge detectors of uC read and write strobes
Signal RDDL,WRDL : std_logic_vector (1 downto 0);
signal EthWRDL,EthRDDL : std_logic_vector (4 downto 0);

-- Clock and reset signals
signal Buff_Rst,SysClk,FMGenClk,ResetHi,Pll_Locked,nEthClk,
		 EthClk,SerdesRst,LinkBuffRst,GTPRst, Seq_Rst : std_logic;

-- Counter that determines the trig out pulse width
signal GPOCount : std_logic_vector(2 downto 0);

-- Signals for decoding duty cycle modulated microbunch marker
signal DDRBits : std_logic_vector(1 downto 0);
signal MarkerBits : std_logic_vector(7 downto 0);
signal GaurdCount : std_logic_vector(3 downto 0);
signal Even_Odd,Marker : std_logic;

-- Orange tree signals
signal iDQ : std_logic_vector (15 downto 0);
Signal DQWrtDly : Array_3x16;
signal DQEn : std_logic;

-- uC data bus
signal iCD : std_logic_vector(15 downto 0);
signal AddrReg : std_logic_vector(11 downto 0);
-- FM transmit enable
signal EnTx1,TrgSrc,TrigReq,TrigPls : std_logic;

-- Timing interval counters
signal Counter1us : std_logic_vector (7 downto 0);
signal Counter100us : std_logic_vector (13 downto 0);
signal Counter1ms : std_logic_vector (17 downto 0); 
signal Counter1s : std_logic_vector (27 downto 0);
signal GateCounter, TurnOnTime, TurnOffTime : std_logic_vector (8 downto 0);

signal TrigEn,TstTrigEn,TstTrigCE,Spill_Req,Beam_On,Seq_Busy : std_logic; 

signal TstPlsEn,TstPlsEnReq,SS_FR,IntTrig,ExtTrig,IntTmgEn,TmgCntEn: std_logic;
-- DG: signal to determine whether to increment Microbunch number periodically
-- 	 or in response to an external trigger
signal PeriodicMicroBunch: std_logic;

-- DG: more trigger/timing signals
signal COUNTRESET, MANTRIG : std_logic;

-- DG: External trigger inhibit
signal ExtTriggerInhibit : std_logic_vector (47 downto 0);
signal ExtTriggerInhibitCount : std_logic_vector (47 downto 0);


signal SpillWidth,InterSpill,InterSpillCount : std_logic_vector (7 downto 0);

-- Signals for generating fake accelerator timing signals
signal SpillWidthCount : std_logic_vector (8 downto 0);
signal SuperCycleCount : std_logic_vector (13 downto 0);
signal MicrobunchCount : std_logic_vector (47 downto 0);
-- Counter for counting down heartbeat bursts
signal HrtBtBrstCntReg,HrtBtBrstCounter  : std_logic_vector (23 downto 0);
signal uBunchLEDCnt : std_logic_vector (4 downto 0);
signal TrigType : std_logic_vector (3 downto 0);
signal DRFreq : std_logic_vector (31 downto 0);
signal DRCount : std_logic_vector (7 downto 0);
signal Int_uBunch : std_logic_vector (1 downto 0);

-- Count the number of triggers
signal TrigCounter : std_logic_vector (31 downto 0);
signal DReq_Count : std_logic_vector (15 downto 0);
-- Make a test counter that increments with each read
signal TestCount : std_logic_vector (31 downto 0);
-- Uptime counter to check for un-anticipated resets
signal UpTimeCount,UpTimeStage : std_logic_vector (31 downto 0);
-- Number of data words per spill

-- Spill counter, event word cout, spill word count
signal SpillCount,EventWdCnt : std_logic_vector (15 downto 0);

signal GPIDL,TrigDL,iWrtDL  : Array_2x2;
signal GateWidth0,GateWidth1,PedWidth0,PedWidth1 : std_logic_vector (7 downto 0);
-- Test Pulse generator signals
-- DDS frequency registers
signal FreqReg,PhaseAcc : std_logic_vector (31 downto 0);

-- Heartbeat length
-- DDS frequency registers
signal HeartBeatFreqReg, HeartBeatPhaseAcc : std_logic_vector (31 downto  0);

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

-- Signal used by LVDS FM links
signal Rx1Dat : std_logic_vector(15 downto 0);
signal RxOut : RxOutRec;

-- Signals used by GTP transceivers
--Signal tile0_gtp0_refclk_i,tile0_gtp1_refclk_i,GTPRxRst : std_logic;
--Signal PllLkDtct,GTPRstDn,RxUsrClk,BuffOut_DCMIn,
--		 UsrClk2,UsrClk,TxDCMLock,Reframe : std_logic_vector (1 downto 0);
--signal RxLOS,GTPTxClk,Rx_IsComma,InvalidChar,GTPDisp,GTPSysClk,
--		 UsrWRDL,UsrRDDL,Rx_IsCtrl : Array_2x2;
signal TxCharIsK,TxCharErr : Array_2x2;
--signal GTPTxStage,GTPTx,GTPRx,GTPRxReg,GTPRxBuff_Out : Array_2x16;

-- Signals used by GTP Tx and Rx FIFOs
--signal DCM_Locked : std_logic_vector (1 downto 0);

--signal GTPRxBuff_wr_en,GTPRxBuff_rd_en,GTPRxBuff_full,
--		 GTPRxBuff_Emtpy,PRBSCntRst,PRBSErr : std_logic_vector (1 downto 0);
--signal EnPRBSTst,En_PRBS : Array_2x3;
--signal GTPRxBuff_RdCnt : Array_2x13;
-- Signals used by the microbunch,trigger FM transmitters
signal HrtBtTxOuts,DreqTxOuts : TxOutRec;
signal HrtBtTxEn,DReqTxEn,LinkBusy : std_logic;
signal HrtBtData : std_logic_vector (23 downto 0);
signal TrigFMDat : std_logic_vector (15 downto 0);

-- Trigger request packet buffer, FIFO status bits
signal DReqBuff_Out : std_logic_vector (15 downto 0);
-- DG: signal to manage Data Request Buffer input
signal DReqBuff_In : std_logic_vector (15 downto 0);
signal DReqBuff_wr_en,DReqBuff_rd_en,DReqBuff_uCRd,
		 DReqBuff_Full,TrigTx_Sel,DReqBuff_Emtpy,Trig_Tx_Req,
		 Trig_Tx_Ack,BmOnTrigReq,Trig_Tx_ReqD : std_logic;
signal LinkFIFOStatReg,Lower_FM_Bits : std_logic_vector (2 downto 0);  

Type Trig_Tx_State is (Idle,SendTrigHdr,SendPad0,SendPktType,SenduBunch0,SenduBunch1,
								SenduBunch2,SendPad1,SendPad2,SendPad3,WaitCRC,SendCRC);
signal IntTrigSeq : Trig_Tx_State;
signal DReqBrstCntReg,DReqBrstCounter : std_logic_vector (15 downto 0);
signal TrigReqWdCnt,DCSReqWdCnt : std_logic_vector (3 downto 0);
signal TrgPktRdCnt : std_logic_vector (10 downto 0);

-- Time stamp FIFO
signal TStmpBuff_Out : std_logic_vector (47 downto 0); -- DG: change from 16 bits to 48
signal TStmpBuff_wr_en,TStmpBuff_rd_en,TStmpBuff_Full,TStmpBuff_Empty : std_logic;
signal TStmpWds : std_logic_vector (8 downto 0); 

-- DG: External Trigger TimeStamp FIFO:
signal ExtTrigTStampBuff_Out : std_logic_vector(47 downto 0);
signal ExtTrigTStampBuff_wr_en, ExtTrigTStampBuff_rd_en, ExtTrigTStampBuff_Full, ExtTrigTStampBuff_Empty : std_logic;
signal ExtTrigTStampBuffWds : std_logic_vector(8 downto 0);

-- DCS request FIFO
-- DM was unused - removed
--signal DCSTxBuff_wr_en,DCSTxBuff_rd_en,DCSTxBuff_Full,DCSTxBuff_Emtpy : std_logic;
--signal DCSPktRdCnt : std_logic_vector (12 downto 0); 
--signal DCSTxBuff_Out : std_logic_vector (15 downto 0); 
--signal DCSTxBuffWds : std_logic_vector (8 downto 0); 

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
signal StatOr : std_logic_vector (7 downto 0); 
-- CRC generator signals
Signal RdCRCEn,TxCRCEn,RxCRCRst,RxCRCRstD,TxCRCRst : std_logic_vector (1 downto 0);
signal TxCRC,RxCRC,TxCRCDat : Array_2x16;
-- Sequencers to read and send packet data
Type Packet_Parser_Seq is (Idle,Read_Type,Check_Seq_No,Wrt_uC_Queue,
									Wrt_FPGA_Queue,Check_CRC);
signal Packet_Parser : Packet_Parser_Seq;

Type Packet_Former_Seq is (Idle,WrtPktCnt,WrtHdrPkt,WrtCtrlHdrPkt,WrtDatPkt);
signal Packet_Former : Packet_Former_Seq;
signal ChkCntr,FormStatReg,EmptyLatch : std_logic_vector (2 downto 0);
signal Pkt_Timer : std_logic_vector (3 downto 0);
signal TxPkCnt : std_logic_vector (10 downto 0);
signal EvTxWdCnt : std_logic_vector (13 downto 0);
signal FormHold,FormRst,ExtTmg,EvTxWdCntTC : std_logic;

signal TxSeqNo,RxSeqNo,WrtCount,GtpRxBuffStat,GtpRxBuffCnt : Array_2x3;
signal RxSeqNoErr : std_logic_vector (1 downto 0);

signal PunchBits : std_logic_vector (3 downto 0);

-- DG: EXTERNAL TRIGGER SIGNALS --
signal NimTrigCount: std_logic_vector(15 downto 0);
signal NimTrigBUF: std_logic_vector (1 downto 0);
-- signal NimTrigOLD: std_logic;

-- DG: signals to recognize external PPS (on GPI input)
-- signal ExtPPSHold: std_logic;
signal ExtPPSBUF: std_logic_vector(1 downto 0);

-- signals for enabling sending heart-beats
signal HrtBtTxEnExtTrig: std_logic;
signal HrtBtTxEnPeriodic: std_logic;

-- signals for periodic generation of Microbunch counts
signal PeriodicMicrobunchCount: std_logic_vector(31 downto 0);
signal PeriodicMicrobunchPeriod: std_logic_vector(31 downto 0);

-- (more) DG: signals for generation of trigger time stamps
signal TriggerTimeStampCount: std_logic_vector(47 downto 0);
signal DoTriggerTimeStampExtReset: std_logic;

-- DG: signals for determining  trigger type
signal TriggerTypeGatePeriod: std_logic_vector(5 downto 0);
signal TriggerTypeGateCount: std_logic_vector(5 downto 0);
Type TriggerVariant is (TriggerVariantBeamOn, TriggerVariantBeamOff);
signal TriggerType: TriggerVariant;

-- when to issue trigger
signal IssueExtNimTrigger: std_logic;
signal ClkDiv2: std_logic;

--Signals for OT FSM
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
--    signal EthClk : std_logic;
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
--    signal nRefClk, ClkDiv2 : std_logic := '0';
--	 signal ResetHi, Pll_Locked : std_logic;

begin
-- DG: debug stuff
Debug(1) <= ClkDiv2;
Debug(2) <= '1' when NimTrigBUF = 1 else '0';
Debug(3) <= '1' when TriggerTypeGateCount  = 1 else  '0';
Debug(4) <= IssueExtNimTrigger;
Debug(5) <= HrtBtTxEn;
Debug(6) <= ExtPPSBuf(1);
Debug(7) <= '1' when TriggerTimeStampCount = 0 else '0';
Debug(8) <= '1' when ExtTriggerInhibitCount /= 0 else '0';

ZEthANull0 <= '0';
TrigLED <= '0';

Sys_Pll : SysPll
  port map(
 -- Clock in ports
    CLK_IN1_P => ClkB_P,
    CLK_IN1_N => ClkB_N,
 -- Clock out ports
    CLK_OUT1 => SysClk,   -- 100 MHz
    CLK_OUT2 => EthClk,   -- 125 MHz 180 deg. phase
--	 CLK_OUT3 => EthClk,   -- 160 MHz used for Orange Tree I/O
 -- Status and control signals
    RESET  => ResetHi,
    LOCKED => Pll_Locked);

HrtBtData(19 downto 0) <= MicrobunchCount(19 downto 0);
HrtBtData(20) <= '1' when TriggerType = TriggerVariantBeamOn else '0';
HrtBtData(21) <= '0' when MicrobunchCount(31 downto 20) = 0 else '1';
HrtBtData(23 downto 22) <= "00";
-- FM transmitter for boadcasting microbunch numbers to the FEBs
HeartBeatTx : FM_Tx 
	generic map (Pwidth => 24)
		 port map(clock => SysClk, 
					 reset => ResetHi,
					 Enable => HrtBtTxEn,
					 Data => HrtBtData, 
					 Tx_Out => HrtBtTxOuts);
HeartBeatFM <= HrtBtTxOuts.FM when ExtTmg = '0' else GPI;
-- DG: MUX to handle whether HrtBtTxEn is set periodically or by external trigger
HrtBtTxEn <= HrtBtTxEnPeriodic when PeriodicMicrobunch = '1'
					else HrtBtTxEnExtTrig;

-- FM transmitter for data requests 
DReqTx : FM_Tx
	generic map (Pwidth => 16)
		 port map(clock => SysClk, 
					 reset => ResetHi,
					 Enable => DReqTxEn,
					 Data => DReqBuff_Out,
					 Tx_Out => DreqTxOuts);
TrigFM <= DreqTxOuts.FM when TrigTx_Sel = '1'
			 else LinkBusy when TrigTx_Sel = '0';

DReqTxEn <= '1' when TrigTx_Sel = '1' and DReqBuff_Emtpy = '0' and DreqTxOuts.Done = '0' 
					  else '0';
-- DG: TODO: switch to configure DReqBuff source from external trigger/fiber
--DReqBuff_In <= GTPRxReg(0)   -- TO USE FIBER LINK


-- DG: TODO: figure out how to have buffer use different clocks

-- FIFO for buffering broadcast trigger requests, 
-- WAS crossing clock domains from UsrClk to Sysclk
-- DG - Now dreqs come from triger on io port for SBND
DReqBuff : FIFO_DC_1kx16
  PORT MAP (rst => ResetHI,
    wr_clk => SysClk,
	 rd_clk => SysClk,
    din => DReqBuff_In,
    wr_en => DReqBuff_wr_en,
    rd_en => DReqBuff_rd_en,
    dout => DReqBuff_Out,
    full => DReqBuff_Full,
    empty => DReqBuff_Emtpy,
	rd_data_count => TrgPktRdCnt);

DReqBuff_rd_en <= DreqTxOuts.Done when TrigTx_Sel = '1' else DReqBuff_uCRd;


-- DG: change FIFO type b.c. now TimeStamps are generated
-- 	 in response to external triggers (on Sysclk) and are
--     read back by Packet_Former (on UserClk2(0))
-- Queue up time stamps for later checking
TimeStampBuff : ExtTrigToFiber
  PORT MAP (rst => ResetHI,
    wr_clk => SysClk, -- Clock for recognizing external trigger
	 rd_clk => EthClk, -- Clock for sending data over fiber
    din => MicrobunchCount, -- DG: change to take in Microbunchcount
    wr_en => TStmpBuff_wr_en,
    rd_en => TStmpBuff_rd_en,
    dout => TStmpBuff_Out,
    full => TStmpBuff_Full,
    empty => TStmpBuff_Empty,
	 rd_data_count => TStmpWds);
	 
-- Save External Trigger TimeStamp
ExtTrigTimeStampBuff : ExtTrigToFiber
	PORT MAP (rst => ResetHI,
	wr_clk => SysClk,
	rd_clk => EthClk,
	din => TriggerTimeStampCount,
	wr_en => ExtTrigTStampBuff_wr_en,
	rd_en => ExtTrigTStampBuff_rd_en,
	dout => ExtTrigTStampBuff_Out,
	full => ExtTrigTStampBuff_Full,
	empty => ExtTrigTStampBuff_Empty,
	rd_data_count => ExtTrigTSTampBuffWds);

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
  port map (clk => EthClk,
		rst => ResetHi,
		wr_en => EventBuff_WrtEn,
		rd_en => EventBuff_RdEn,
      din => EventBuff_Dat,
      dout => EventBuff_Out,
      full => EventBuff_Full,
	   empty => EventBuff_Empty);


-- CRC generators for transmit data
GenTxCRC : for i in 0 to 1 generate
TxCRCGen : crc 
 port map(data_in => TxCRCDat(i),
    crc_en => TxCRCEn(i), rst => TxCRCRst(i), clk => EthClk,
    crc_out => TxCRC(i));

end generate;

-- WAS GTP logic processes
-- NOW ETH logic Process

TrigReqTx : process (EthClk, CpldRst, LinkBuffRst)

begin

 if CpldRst = '0' then 

	TxSeqNo(0) <= "000"; TxCRCRst(0) <= '0';
    TxCRCEn(0) <= '0'; TrigReqWdCnt <= X"0"; 
	-- DG: TODO: where does the reset for the Data Request write come from?
	-- DReqBuff_wr_en <= '0'; --DANIEL
	Event_Builder <= Idle;
	LinkFIFORdReq <= (others =>'0'); StatOr <= X"00"; 
	EvTxWdCnt <= (others => '0'); EvTxWdCntTC <= '0'; EventBuff_RdEn <= '0';
	FIFOCount <= (others => (others => '0')); EventBuff_WrtEn <= '0';
	-- DG: reset external trigger timing buff rd en
	ExtTrigTStampBuff_rd_en <= '0';
	-- DG: move TStmpBuff_wr_en to SysClk
	TStmpBuff_rd_en <= '0'; EvBuffWrtGate <= '0';
	-- TStmpBuff_Full <= '0'; TStmpBuff_Empty <= '0';
	-- ExtTrigTStampBuff_Full <= '0'; ExtTrigTStampBuff_Empty <= '0';
	TxPkCnt <= (others => '0'); Pkt_Timer <= X"0";
	EmptyLatch <= "000"; 
	ActiveReg <= X"000000"; LinkFIFOStatReg <= "000";
	AddrReg <= (others =>'0');
	
elsif rising_edge (EthClk) then

-- Store the empty flag values when they make a transition, 
-- then try and send the updated value
	if DreqTxOuts.Done = '1'
	then LinkFIFOStatReg <= LinkFIFOEmpty;
	else LinkFIFOStatReg <= LinkFIFOStatReg;
	end if;

-- DG: Turn off setting TStmpBuff_wr_en on reception of a Data Request packet.
-- 	 This is now done by the Ext Trig code
-- -- Store the time stamp subfield from the trigger request packet for later checking
--	if Rx_IsComma(0) = "00" and ReFrame(0) = '0' and Rx_IsCtrl(0) = "00" 
--	and TrigReqWdCnt >= 5 and TrigReqWdCnt <= 7 	
--	then TStmpBuff_wr_en <= '1';
--	else TStmpBuff_wr_en <= '0';
--	end if;

---------------------------------------------------------------------------
-- Idle,RdInWdCnt0,RdInWdCnt1,RdInWdCnt2,SumWdCnt,WrtWdCnt,RdStat0,
-- RdStat1,RdStat2,WrtStat,WaitEvent,ReadFIFO0,ReadFIFO1,ReaddFIFO2
---------------------------------------------------------------------------
Case Event_Builder is
	when Idle => --Debug(10 downto 7) <= X"0";
		if LinkFIFOEmpty /= 7 and FormHold = '0' and TStmpBuff_Empty = '0' --DG: used  to check if TStmpWds >= 3. Now  the whole timestamp is only in one word, so we only need to check if the buffer has data 
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
	elsif Event_Builder = WrtStat	then EventBuff_Dat <= X"00" & StatOR;
	elsif LinkFIFORdReq(0) = '1' then EventBuff_Dat <= LinkFIFOOut(0);
	elsif LinkFIFORdReq(1) = '1' then EventBuff_Dat <= LinkFIFOOut(1);
	elsif LinkFIFORdReq(2) = '1' then EventBuff_Dat <= LinkFIFOOut(2);
	else EventBuff_Dat <= EventBuff_Dat;
	end if;

-- Do an "or" of the FEB error words for the cotroller error word
   if Event_Builder = RdStat0 then 
				StatOr <= StatOr or LinkFIFOOut(0)(7 downto 0);
elsif Event_Builder = RdStat1 then 
				StatOr <= StatOr or LinkFIFOOut(1)(7 downto 0);
elsif Event_Builder = RdStat2 then 
				StatOr <= StatOr or LinkFIFOOut(2)(7 downto 0);
else StatOr <= StatOr;
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
--	Read of the trigger request FIFO
	if RDDL = 2 and AddrReg(11 downto 10) = GA and AddrReg(9 downto 0) = EventBuffAd 
	then EventBuff_RdEn <= '1';
	else EventBuff_RdEn <= '0';
	end if;



--if (Packet_Former = WrtCtrlHdrPkt and (Pkt_Timer = 8 or Pkt_Timer = 4))
-- or (Packet_Former = WrtDatPkt and Pkt_Timer > 2 and EvTxWdCnt > 0)
--then EventBuff_RdEn <= '1';  --Debug(5) <= '1';
--else EventBuff_RdEn <= '0';  --Debug(5) <= '0';
--end if;

--Debug(4) <= EventBuff_Empty;

--Debug(3 downto 1) <= LinkFIFOEmpty;
--
-----------------------------------------------------------------------------
---- Idle,WrtPktCnt,WrtHdrPkt,WrtCtrlHdrPkt,WrtDatPkt
-----------------------------------------------------------------------------
--Case Packet_Former is 
--	when Idle => FormStatReg <= "000"; 
--		if EventBuff_Empty = '0' then Packet_Former <= WrtPktCnt;
--		else Packet_Former <= Idle;
--		end if;
---- Divide by eight to get the number of packets
--	when WrtPktCnt => Packet_Former <= WrtHdrPkt;  FormStatReg <= "001";  
---- Send the packet header, packet type, packet count, time stamp and status
--	when WrtHdrPkt => FormStatReg <= "010"; 
--		if Pkt_Timer = 0 then Packet_Former <= WrtCtrlHdrPkt;
--	    elsif FormRst = '1' then Packet_Former <= Idle; 
--		else Packet_Former <= WrtHdrPkt;
--		end if;
--	when WrtCtrlHdrPkt =>  FormStatReg <= "011";  
--		if Pkt_Timer = 0 then Packet_Former <= WrtDatPkt;
--	 elsif FormRst = '1' then Packet_Former <= Idle; 
--	else Packet_Former <= WrtCtrlHdrPkt;
--		end if;
---- After Controller header is sent, the packets contain data for this FEB
---- The FEB header data is embedded in the sream coming from the front FPGAs
--	when WrtDatPkt => FormStatReg <= "100";   
--		if EvTxWdCnt = 0 and Pkt_Timer = 0
--			then Packet_Former <= Idle;
--		 elsif FormRst = '1' then Packet_Former <= Idle; 
--		else Packet_Former <= WrtDatPkt;
--		end if;
--	when others => Packet_Former <= Idle;  FormStatReg <= "101";
--end Case;
--
---- Sum word counts, divide by eight and add 1 for the controller header packet
--   if Packet_Former = WrtPktCnt and EventBuff_Out(2 downto 0)  = 0 then TxPkCnt <= EventBuff_Out(13 downto 3) + 1;
---- If not and even multiple of eight, account for final partially filled packet
--elsif Packet_Former = WrtPktCnt and EventBuff_Out(2 downto 0) /= 0 then TxPkCnt <= EventBuff_Out(13 downto 3) + 2;
---- Decrement the packet count once for each packet ID write
--elsif ((Packet_Former = WrtDatPkt or Packet_Former = WrtCtrlHdrPkt) and Pkt_Timer = 9) then TxPkCnt <= TxPkCnt - 1;
--else TxPkCnt <= TxPkCnt;
--end if;
--
---- Extract the word count from the event buffer FIFO 
--   if Packet_Former = WrtPktCnt
--		then EvTxWdCnt <= (13 downto 0);
---- Decrement the word count for each word sent within a data packet
--	elsif EvTxWdCnt /= 0 and Packet_Former = WrtDatPkt and Pkt_Timer > 2
--		then EvTxWdCnt <= EvTxWdCnt - 1;
--	else EvTxWdCnt <= EvTxWdCnt;
--	end if;
--
---- Use this word count terminal count to distinguish the last valid word read
---- from the event buffer FIFO
--	if EvTxWdCnt = 1 and Packet_Former = WrtDatPkt and Pkt_Timer > 2
--		then EvTxWdCntTC <= '1';
--	elsif EvTxWdCnt /= 1 and Packet_Former = WrtDatPkt and Pkt_Timer > 2
--		then EvTxWdCntTC <= '0';
--	else EvTxWdCntTC <= EvTxWdCntTC;
--	end if;
--
---- DG: change TStmpBuff setup
---- 
---- Before, TStmpBuff was 16bits wide and so neded to be read 3 times to get
---- the full 48bit Timestamp. Now it is 48 bits, so it only needs to be read once
--
---- Read of timestamps for use in forming the header packet
--
---- if (Packet_Former = WrtHdrPkt and Pkt_Timer <= 7 and Pkt_Timer >= 5) -- DG: OLD
--if (Packet_Former = WrtHdrPkt and Pkt_Timer = 7) -- DG: NEW 
--	then TStmpBuff_rd_en <= '1';
--	else TStmpBuff_rd_en <= '0';
--end if;
--
---- DG: also read the trigger time stamp FIFO
--if (Packet_Former = WrtHdrPkt and Pkt_Timer = 4)
--	then ExtTrigTStampBuff_rd_en <= '1';
--	else ExtTrigTStampBuff_rd_en <= '0';
--end if;
--
---- Also read trigger time stamp for 
--
---- Increment the data request counter when forming the header packet.
--if Packet_Former = WrtHdrPkt and Pkt_Timer = 9 then DReq_Count <= DReq_Count + 1;
--elsif GTPRxRst = '1' then DReq_Count <= (others => '0');
--else DReq_Count <= DReq_Count;
--end if;
--
---- Counter for dividing data into packets
--if Packet_Former = WrtPktCnt then Pkt_Timer <= X"A";
--elsif Pkt_Timer /= 0 and (Packet_Former = WrtHdrPkt 
--or Packet_Former = WrtCtrlHdrPkt or Packet_Former = WrtDatPkt)
--	then Pkt_Timer <= Pkt_Timer - 1;
--elsif Pkt_Timer = 0 and (Packet_Former = WrtHdrPkt 
--or Packet_Former = WrtCtrlHdrPkt or Packet_Former = WrtDatPkt)
--	then Pkt_Timer <= X"A";
--elsif Packet_Former = Idle then Pkt_Timer <= X"0";
--else Pkt_Timer <= Pkt_Timer;
--end if;

end if; -- CpldRst

end process;


-- DG: new process -- handles generation of Data Request from external LEMO-NIM trigger
Daniel_DATAREQ : process (SysClk, CpldRst)
begin

 if CpldRst = '0' then 

	TxCharIsK(1) <= "11"; 
	TxSeqNo(1) <= "000"; TxCRCRst(1) <= '0';
   TxCRCEn(1) <= '0'; TxCRCDat(1) <= X"0000";
	Trig_Tx_Ack <= '0';
	IntTrigSeq <= Idle; 
elsif rising_edge (SysClk) then


-- DG: TODO: is Req/Ack necessary?
-- Request/Acknowledge to cross clock domains
	Trig_Tx_Ack  <= Trig_Tx_Req;

-- State machine for sending trigger requests from internal trigger generator
-- Idle,SendTrigHdr,SendPktType,SendPad0,SenduBunch0,SenduBunch1,
--	SenduBunch2,SendPad1,SendPad2,SendPad3,SendCRC
Case IntTrigSeq is
	when Idle =>
	  if Trig_Tx_Ack = '1' then IntTrigSeq <= SendTrigHdr;
	  else IntTrigSeq <= Idle;
	  end if;
	when SendTrigHdr => IntTrigSeq <= SendPad0;
	when SendPad0 => IntTrigSeq <= SendPktType;
	when SendPktType =>  IntTrigSeq <= SenduBunch0;
	when SenduBunch0 => IntTrigSeq <= SenduBunch1;
	when SenduBunch1 => IntTrigSeq <= SenduBunch2;
	when SenduBunch2 => IntTrigSeq <= SendPad1;
	when SendPad1 => IntTrigSeq <= SendPad2;
	when SendPad2 => IntTrigSeq <= SendPad3;
	when SendPad3 => IntTrigSeq <= WaitCRC; 
	when WaitCRC => IntTrigSeq <= SendCRC;
	when SendCRC => IntTrigSeq <= Idle;
	when others => IntTrigSeq <= Idle;  
	-- DG: TODO: is this necessary?
	DReqBuff_wr_en <= '0';
end Case;
DReqBuff_wr_en <= '0';
-- Use this address to append K28.0 to Dx.y where x is 5 bits of data and
-- y is the packet sequence number
	if IntTrigSeq = SendTrigHdr  
		--DG TODO: make it more explicit that header is not included in Data Request Packet
		-- HEADER IS NOT INCLUDED IN DATA REQUEST PACKET
	then if IntTrigSeq = SendTrigHdr 
				then DReqBuff_In <= X"1C" & TxSeqNo(1) & "00010";
				else DReqBuff_In <= X"1C" & TxSeqNo(1) & uCD(4 downto 0);
			 end if;
		   TxSeqNo(1) <= TxSeqNo(1) + 1;
			TxCharIsK(1) <= "10";
			TxCRCRst(1) <= '1';
			TxCRCEn(1) <= '0';
			TxCRCDat(1) <= (others => '0');
			--DReqBuff_wr_en <= '1';

-- Use this address to send unmodified data
	elsif IntTrigSeq = SendPad0
	 then if IntTrigSeq = SendPad0
				then DReqBuff_In <= (others => '0');
					  TxCRCDat(1) <= (others => '0');
				else DReqBuff_In <= uCD;
					  TxCRCDat(1) <= uCD;
			 end if;
			TxCharIsK(1) <= "00";
			TxCRCRst(1) <= '0';
			TxCRCEn(1) <= '1';
			DReqBuff_wr_en <= '1';
	 elsif IntTrigSeq = SendPktType  
	  then DReqBuff_In <= X"0020";
			TxCRCDat(1) <= X"0020"; 
			TxCharIsK(1) <= "00";
			TxCRCRst(1) <= '0';
			TxCRCEn(1) <= '1';
			DReqBuff_wr_en <= '1';

	 elsif IntTrigSeq = SenduBunch0  
	  then DReqBuff_In <= MicrobunchCount(15 downto 0);
	       TxCRCDat(1) <= MicrobunchCount(15 downto 0); 
			TxCharIsK(1) <= "00";
			TxCRCRst(1) <= '0';
			TxCRCEn(1) <= '1';
			DReqBuff_wr_en <= '1';

	 elsif IntTrigSeq = SenduBunch1  
	  then DReqBuff_In <= MicrobunchCount(31 downto 16);
	      TxCRCDat(1) <= MicrobunchCount(31 downto 16); 
			TxCharIsK(1) <= "00";
			TxCRCRst(1) <= '0';
			TxCRCEn(1) <= '1';
			DReqBuff_wr_en <= '1';

	 elsif IntTrigSeq = SenduBunch2  
	  then DReqBuff_In <= MicrobunchCount(47 downto 32);
	      TxCRCDat(1) <= MicrobunchCount(47 downto 32); 
			TxCharIsK(1) <= "00";
			TxCRCRst(1) <= '0';
			TxCRCEn(1) <= '1';
			DReqBuff_wr_en <= '1';

	 elsif IntTrigSeq = SendPad1  
	  then DReqBuff_In <= (others => '0');
			 TxCRCDat(1) <= (others => '0');
			TxCharIsK(1) <= "00";
			TxCRCRst(1) <= '0';
			TxCRCEn(1) <= '1';
			DReqBuff_wr_en <= '1';

	 elsif IntTrigSeq = SendPad2  
	  then DReqBuff_In <= (others => '0');
	  		TxCRCDat(1) <= (others => '0');
			TxCharIsK(1) <= "00";
			TxCRCRst(1) <= '0';
			TxCRCEn(1) <= '1';
			DReqBuff_wr_en <= '1';

	 elsif IntTrigSeq = SendPad3  
	  then DReqBuff_In <= (others => '0');
			TxCRCDat(1) <= (others => '0');
			TxCharIsK(1) <= "00";
			TxCRCRst(1) <= '0';
			TxCRCEn(1) <= '1';
			DReqBuff_wr_en <= '1';

	 elsif IntTrigSeq = WaitCRC  
	  then DReqBuff_In <= X"BC3C";
			TxCRCDat(1) <= (others => '0');
			TxCharIsK(1) <= "11";
			TxCRCRst(1) <= '0';
			TxCRCEn(1) <= '0';
-- Use this address to send the check sum
	elsif IntTrigSeq = SendCRC
	 then DReqBuff_In <= TxCRC(1);
			TxCharIsK(1) <= "00";
			TxCRCRst(1) <= '0';
			TxCRCEn(1) <= '0';
			DReqBuff_wr_en <= '1';

-- Pad is K28.5 K28.1 pair
	 else DReqBuff_In <= X"BC3C";
			TxCharIsK(1) <= "11";
			TxCRCRst(1) <= '0';
			TxCRCEn(1) <= '0';
	end if;

end if;

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
  port map (rst => LinkBuffRst, wr_clk => RxOutClk(i), rd_clk => Ethclk, 
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
nullbit <= '0';
----------------------- Orange tree interface logic -----------------------------


EthProc : process(EthClk, CpldRst)
begin
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
				GPO <= "00";
				--ClkDiv2 <= '0';
        elsif (EthClk'event and EthClk='1') then
				--ClkDiv2 <= not ClkDiv2;
            -- Counter of completed register reads
            if (UserReadDataValid='1') then
                UserValidCount <= UserValidCount + 1;
            end if;

            case UserFPGAState is
                -- Power on startup delay until GigExpedite reports a valid IP address
                when USER_FPGA_DELAY_INIT =>
						  --Debug(10 downto 7) <= "0000";
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
						  --Debug(10 downto 7) <= "0001";
                    UserFPGASubState <= UserFPGASubState + 1;
                    if (UserFPGASubState=X"000f") then
                        UserFPGASubState <= (others=>'0');
                        UserFPGAState <= USER_FPGA_CLEAN_CHECK;
                        Clean <= '1';
                    end if;
                when USER_FPGA_CLEAN_CHECK =>
						  --Debug(10 downto 7) <= "0010";
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
						  --Debug(10 downto 7) <= "0011";
                    UserFPGASubState <= UserFPGASubState + 1;
                    if (UserFPGASubState=X"0003") then
                        UserFPGAState <= USER_FPGA_IDLE;
                    end if;
                
                -- Wait for interrupt from GigExpedite device
                when USER_FPGA_IDLE =>
						  --Debug(10 downto 7) <= "0100";
                    UserFPGASubState <= (others=>'0');
                    UserValidCount <= (others=>'0');
                    if (UserInterrupt='1') then
                        -- Interrupt has been received
                        UserFPGAState <= USER_FPGA_CHECK_STATE;
                    end if;

                -- Check if the connection state has changed
                when USER_FPGA_CHECK_STATE =>
						  --Debug(10 downto 7) <= "0101";

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
						  --Debug(10 downto 7) <= "0110";

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
						  --Debug(10 downto 7) <= "0111";
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
						  --Debug(10 downto 7) <= "1000";
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
						  --Debug(10 downto 7) <= "1001";

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
						  --Debug(10 downto 7) <= "1010";

								UserFPGAState <= USER_FPGA_WAIT1;
					  when USER_FPGA_WAIT1 =>
						  --Debug(10 downto 7) <= "1011";
								UserFPGAState <= USER_FPGA_WAIT3;
					  when USER_FPGA_WAIT3 =>
						  --Debug(10 downto 7) <= "1111";
								UserFPGAState <= USER_FPGA_WRITE_MAIL1;
					  when USER_FPGA_WRITE_MAIL1 =>
						  --Debug(10 downto 7) <= "1100";
								UserFPGAState <= USER_FPGA_WAIT2;
					  when USER_FPGA_WAIT2 =>
						  --Debug(10 downto 7) <= "1101";
						  if(UserReadDataValid='1') then

								UserFPGAState <= USER_FPGA_WRITE_MAIL2;
								end if;

					  when USER_FPGA_WRITE_MAIL2 =>
						  --Debug(10 downto 7) <= "1110";
								UserFPGAState <= USER_FPGA_IDLE;
                when others => 
						  --Debug(10 downto 7) <= "1111";
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
                UserWriteData <= X"456F";
					 
                
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
				Debug => open,

            GigEx_Clk => ZEthClk,
            GigEx_nCS => ZEthCS,
            GigEx_nWE => ZEthWE,
            GigEx_Addr(9 downto 1) => ZEthA,
				GigEx_Addr(0) => ZEthANull0, --open,

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
    


----------------------- 100 Mhz clocked logic -----------------------------

main : process(SysClk, CpldRst)

 begin 

-- asynchronous reset/preset
 if CpldRst = '0' then

-- Synchronous edge detectors for various strobes
	RDDL <= "00"; WRDL <= "00"; 

-- Trigger and spill generator logic
	FreqReg <= X"00000237"; PhaseAcc <= (others => '0');
	HeartBeatFreqReg <= X"0C154C98"; -- set to Mu2e valuee by default (to generate 4.72Mhz clock)
	Buff_Rst <= '0'; Seq_Rst <= '0'; 
	Beam_On <= '0'; TrigReq <= '0'; TrigPls <= '0';	TrigEn <= '1'; 
	TstPlsEn <= '0';  TstPlsEnReq <= '0'; SS_FR <= '0';  TstTrigEn <= '0';
	ExtTrig <= '0'; IntTrig <= '0'; TrigType <= X"0"; 
	SpillWidth <= X"02"; Spill_Req <= '0'; TstTrigCE <= '0';
	EventWdCnt <= (others => '0'); InterSpill <= X"04"; BmOnTrigReq <= '0';
	PhaseAccD <= '0';
	UpTimeStage <= (others => '0'); UpTimeCount <= (others => '0');
	Counter1us <= X"00"; Counter1ms <= (others => '0');
	SuperCycleCount <= (others => '0'); SpillWidthCount <= (others => '0');
	InterSpillCount <= (others => '0'); 
	-- DG: No longer reset HrtBt enable -- the two input signals instead are reset
	-- HrtBtTxEn <= '0'; 
	MicrobunchCount <= (others => '0'); 
	-- DG: reset External Trigger Inhibit
	ExtTriggerInhibit <= (others => '0');
	ExtTriggerInhibitCount <= (others => '0');
	PeriodicMicrobunchCount <= (others => '0');
	PeriodicMicrobunchPeriod <= (others => '0');
	DRFreq <= (others => '0'); -- Delivery ring DDS
	Int_uBunch <= "00"; -- Rising edge of DDS terminal count
	DRCount <= (others => '0'); -- Delivery ring bunch counter
	Counter1s <= (others => '0');	TestCount <= (others => '0'); 
	Counter100us <= (others => '0');	
	TrigCounter <= (others => '0'); SpillCount <= (others => '0'); 
	LEDRst <= '1'; LEDSDat <= "000"; LEDSClk <= "000"; LEDLd <= "000000";
	uBunchLED <= '0'; uBunchLEDCnt <= (others => '0'); IntTmgEn <= '0';
   HrtBtBrstCntReg <= (X"001000"); HrtBtBrstCounter <= (others => '0');
	CMDwr_en <= '0'; CMDrd_en <= '0';  TmgCntEn <= '0';
	ClkDiv <= "000"; CMDBitCount <= (others => '0'); 
	LEDShiftReg <= (others => '0');	LED_Shift <= Idle;
	DReqBuff_uCRd <= '0'; LinkBusy <= '0';

-- Pll Chip Shifter signals
	PLLBuffwr_en <= '0'; PLLBuffrd_en <= '0'; PllPDn <= '1';
	PllStage <= X"00"; PllShiftReg <= (others => '0'); 
	PllBitCount <= (others => '0'); Pll_Shift <= Idle;
	PllSClk <= '0'; PllSDat <= '0'; PllLd <= '0';

-- TDAQ Receive link signals
	WrtCount(0) <= "000";
	TDisA <= '0'; TDisB <= '0';

	PunchBits <= X"0"; FormHold <= '0'; ExtTmg <= '0';
	IDReg <= X"1";

	DReqBrstCntReg <= X"0001"; DReqBrstCounter <= (others => '0');
	Trig_Tx_Req <= '0'; Trig_Tx_ReqD <= '0';
	
-- DG: reset external timing stuff
	PeriodicMicrobunch <= '0';
	NimTrigCount <= (others => '0');
	COUNTRESET <= '0';
	MANTRIG <= '0';
	
-- DG: reset Write-end of Ext Trg -> Fiber FIFO's
	TStmpBuff_wr_en <= '0';
	ExtTrigTStampBuff_wr_en <= '0';
	
	HrtBtTxEnPeriodic <= '0';
	HrtBtTxEnExtTrig <= '0';
	
	TriggerTimeStampCount <= (others => '0');
	DoTriggerTimeStampExtReset <= '0';
	
-- DG: trigger gate signals
	TriggerTypeGatePeriod <= (others => '0');
	TriggerTypeGateCount <= (others => '0');
	TriggerType <= TriggerVariantBeamOff;
	IssueExtNimTrigger <= '0';
	
	ClkDiv2 <= '0';
	
 elsif rising_edge (SysClk) then 
 
 ClkDiv2 <= not ClkDiv2;

-- Synchronous edge detectors for read and write strobes
RDDL(0) <= not uCRD and not CpldCS;
RDDL(1) <= RDDL(0);

WRDL(0) <= not uCWR and not CpldCS;
WRDL(1) <= WRDL(0);

LinkBusy <= LinkFIFOEmpty(0) and LinkFIFOEmpty(1) and LinkFIFOEmpty(2);

-- Select between LEMO and LVDS inputs for the triggers 
if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = TrigCtrlAddr 
then TstPlsEn <= uCD(0);
	  TrgSrc <= uCD(1);
else TstPlsEn <= TstPlsEn;
	  TrgSrc <= TrgSrc;
end if;

-- Internal beam on trigger generator
   if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = FreqRegAdHi 
	then FreqReg <= uCD & FreqReg(15 downto 0);
elsif WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = FreqRegAdLo 
	then FreqReg <= FreqReg(31 downto 16) & uCD;
 else FreqReg <= FreqReg;
 end if;
 
 -- heartbeat changer timer
	if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = HeartBeatFreqRegAdHi
		then HeartBeatFreqReg <= uCD & HeartBeatFreqReg(15 downto 0);
	elsif WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = HeartBeatFreqRegAdLo
		then HeartBeatFreqReg <= HeartBeatFreqReg(31 downto 16) & uCD;
	else HeartBeatFreqReg <= HeartBeatFreqReg;
	end if;
	
-- Choose between internal and TDAQ supplied timing
if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = CSRRegAddr
then FormHold <= uCD(2);
	  ExtTmg <= uCD(4);
	  TrigTx_Sel <= uCD(6);
	  TstTrigCE <= uCD(9);
else FormHold <= FormHold;
	  ExtTmg <= ExtTmg;
	  TrigTx_Sel <= TrigTx_Sel;
	  TstTrigCE <= TstTrigCE;
end if;

-- Enable the transmitting of heartbeats
if IntTmgEn = '0' 
	and WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = CSRRegAddr and uCD(0) = '1'
  then IntTmgEn <= '1';
-- If the finite length is enabled, stop after the count has expired
 elsif IntTmgEn = '1' 
   and ((WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = CSRRegAddr and uCD(0) = '0')
	-- DG: TODO -- configure whether heart beats are generated internally or w/ Data Request
    --or  (HrtBtBrstCounter = 1 and Int_uBunch = 1 and ((Beam_On = '1' and DRCount = 7) or DRCount = 143))
	 )
	 
  then IntTmgEn <= '0';
 else IntTmgEn <= IntTmgEn;
 end if;

-- Finite heartbeat transmit sequence length enable bit
if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = CSRRegAddr 
	and TmgCntEn = '0' and uCD(0) = '1' and uCD(1) = '1'
  then TmgCntEn <= '1';
-- If the finite length is enabled, stop after the count has expired
 elsif TmgCntEn = '1' and ((WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = CSRRegAddr 
	and uCD(1) = '0')
    or (HrtBtBrstCounter = 1 and Int_uBunch = 1 
	 and ((Beam_On = '1' and DRCount = 7) or DRCount = 1)))
  then TmgCntEn <= '0';
 else TmgCntEn <= TmgCntEn;
 end if;

-- Finite timing transmission burst down counter;
if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = CSRRegAddr 
	and TmgCntEn = '0' and uCD(0) = '1' and uCD(1) = '1'
  then HrtBtBrstCounter <= HrtBtBrstCntReg;
 elsif HrtBtBrstCounter /= 0 and Int_uBunch = 1 and ((Beam_On = '1' and DRCount = 7) or DRCount = 143)
  then HrtBtBrstCounter <= HrtBtBrstCounter - 1;
 else HrtBtBrstCounter <= HrtBtBrstCounter;
 end if;

-- Enable the transmitting of trigger request packet on GTPTx(1)
if TstTrigEn = '0' and WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = CSRRegAddr 
	and uCD(8) = '1'
  then TstTrigEn <= '1';
-- If the finite length is enabled, stop after the count has expired
 elsif TstTrigEn = '1' and ((WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = CSRRegAddr 
	and uCD(8) = '0')
  or (TstTrigCE = '1' and DReqBrstCounter = 1 and Trig_Tx_Req = '1' and Trig_Tx_ReqD = '0')
)
  then TstTrigEn <= '0';
 else TstTrigEn <= TstTrigEn;
 end if;

-- Finite trigger burst length enable bit
if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = CSRRegAddr 
	and TstTrigCE = '0' and uCD(8) = '1' and uCD(9) = '1'
  then TstTrigCE <= '1';
-- If the finite length is enabled, stop after the count has expired
 elsif TstTrigCE = '1' and ((WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = CSRRegAddr 
	and uCD(9) = '0')
    or (TstTrigCE = '1' and DReqBrstCounter = 1 and Trig_Tx_Req = '1' and Trig_Tx_ReqD = '0'))
  then TstTrigCE <= '0';
 else TstTrigCE <= TstTrigCE;
 end if;

-- Finite trigger burst down counter;
 if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = CSRRegAddr 
	and TstTrigEn = '0' and  uCD(8) = '1'
  then DReqBrstCounter <= DReqBrstCntReg;
 elsif DReqBrstCounter /= 0 and TstTrigEn = '1' and TstTrigCE = '1' and Trig_Tx_Req = '1' and Trig_Tx_ReqD = '0'
  then DReqBrstCounter <= DReqBrstCounter - 1;
  else DReqBrstCounter <= DReqBrstCounter;
  end if;

-- DG: turns off internally generated heartbeats
-- DG: TODO -- configure whether heart beats are generated internally or w/ Data Request
--if TstTrigEn = '1' and IntTmgEn = '1' and Int_uBunch = 1 
--	and ((Beam_On = '1' and DRCount = 7 and BmOnTrigReq = '1') or DRCount = 143)
--	then Trig_Tx_Req <= '1';
--elsif Trig_Tx_Ack = '1'
--	then Trig_Tx_Req <= '0';
--end if;

Trig_Tx_ReqD <= Trig_Tx_Req;

if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = IDregAddr 
then IDReg <= uCD(3 downto 0);
else IDReg <= IDReg;
end if;

--	Read of the trigger request FIFO
	if RDDL = 2 and AddrReg(11 downto 10) = GA and AddrReg(9 downto 0) = TRigReqBuffAd 
	then DReqBuff_uCRd <= '1';
	else DReqBuff_uCRd <= '0';
	end if;

-- 1us time base
if Counter1us /= Count1us then Counter1us <= Counter1us + 1;
else Counter1us <= X"00";
end if;

-- 100us time base
if Counter100us /= Count100us then Counter100us <= Counter100us + 1;
else Counter100us <= (others => '0');
end if;

-- 1ms time base
if Counter1ms = Count1ms then Counter1ms <= (others => '0');
else Counter1ms <= Counter1ms + 1;
end if;

-- 1 second time base
if	Counter1s /= Count1s then Counter1s <= Counter1s + 1;
else Counter1s <= (others => '0');
end if;

-- 1.4 second super cycle count in 100 us steps (14000)
if IntTmgEn = '1' and Counter100us = Count100us and SuperCycleCount /= SuperCycleLength 
 then SuperCycleCount <= SuperCycleCount + 1;
elsif (Counter100us = Count100us and SuperCycleCount = SuperCycleLength) or IntTmgEn = '0'
 then SuperCycleCount <= (others => '0');
else SuperCycleCount <= SuperCycleCount;
end if;

-- X"0C154C98": generates a 4.72 MHz signal in Mu2e w/ a clock of 100MHz

-- setable in general
if IntTmgEn = '1' then DRFreq <= DRFreq + HeartBeatFreqReg;
else DRFreq <= (others => '0');
end if;

-- Edge detector for the DDS MSB
if DRFreq(31) = '1' then Int_uBunch(0) <= '1';
else Int_uBunch(0) <= '0';
end if;
Int_uBunch(1) <= Int_uBunch(0);

-- For now define the on spill to be 8 4.7MHz ticks and the off spill 144 ticks
-- "DR" for delivery ring 

-- DG: turns off internally generated heartbeats
-- DG: TODO -- configure whether heart beats are generated internally or w/ Data Request

-- DG: TODO: what is the DRCount?

--if IntTmgEn = '1' and 
--   Int_uBunch = 1 and not((Beam_On = '1' and DRCount = 7) or DRCount = 143)
--then DRCount <= DRCount + 1;
--elsif Int_uBunch = 1 and ((Beam_On = '1' and DRCount = 7) or DRCount = 143)
--then DRCount <= (others => '0');
--else DRCount <= DRCount;
--end if;

-- set the periodic microbunch period / count
if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = PeriodicMicrobunchPeriodAddrHi
then PeriodicMicrobunchPeriod(31 downto 16) <= uCD;
elsif WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = PeriodicMicrobunchPeriodAddrLo
then PeriodicMicrobunchPeriod(15 downto 0) <= uCD;
else PeriodicMicrobunchPeriod <= PeriodicMicrobunchPeriod;
end if;

if PeriodicMicrobunch = '1'
then
	if PeriodicMicrobunchPeriod /= 0 and PeriodicMicrobunchPeriod /= PeriodicMicrobunchCount
	then PeriodicMicrobunchCount <= PeriodicMicrobunchCount + 1;
	else PeriodicMicrobunchCount <= (others => '0');
	end if;
else PeriodicMicrobunchCount <= (others => '0');
end if;

-- DG: turns off transmission of hearbeat messages on internal increment
--     of Microbunch number

-- Send a start transmit pulse to the FM transmitter at the beginning of 
-- each microbunch
if PeriodicMicrobunchPeriod /= 0 and PeriodicMicrobunchPeriod = PeriodicMicrobunchCount
	and IntTmgEn = '1' and PeriodicMicrobunch = '1'
  then HrtBtTxEnPeriodic <= '1'; 
  else HrtBtTxEnPeriodic <= '0';
end if;

if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = DReqBrstCntAd 
 then DReqBrstCntReg <= uCD;
else DReqBrstCntReg <= DReqBrstCntReg;
end if;

-- Counter used to send a burst of mirobunches.
 if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = HrtBtBrstCntAdHi
 then HrtBtBrstCntReg(23 downto 16) <= uCD(7 downto 0);
 else HrtBtBrstCntReg(23 downto 16) <= HrtBtBrstCntReg(23 downto 16);
 end if;

 if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = HrtBtBrstCntAdLo
 then HrtBtBrstCntReg(15 downto 0) <= uCD(15 downto 0);
 else HrtBtBrstCntReg(15 downto 0) <= HrtBtBrstCntReg(15 downto 0);
 end if;

-- Overall gate to denote the 380 ms spill region of the super cycle
if Spill_Req = '0' and SuperCycleCount = SpillBegin and Counter100us = Count100us 
then Spill_Req <= '1'; 
elsif (Spill_Req = '1' and Counter100us = Count100us and SuperCycleCount = SpillEnd)
	   or IntTmgEn = '0'
then Spill_Req <= '0'; 
end if;

-- A flag to indicate the individual 53 ms spills
if Counter100us = Count100us and Beam_On = '0' 
 and ((Spill_Req = '0' and SuperCycleCount = SpillBegin)
	or (Spill_Req = '1' and InterSpillCount = InterSpillLength))
then Beam_On <= '1';
elsif (Beam_On = '1' and SpillWidthCount = SpillLength and Counter100us = Count100us)
	   or IntTmgEn = '0'
then Beam_On <= '0';
else Beam_On <= Beam_On;
end if;

-- Count 53.1 ms spill length
if Spill_Req = '1' and Beam_On = '1' and SpillWidthCount /= SpillLength
 and Counter100us = Count100us
then SpillWidthCount <= SpillWidthCount + 1;
elsif (Counter100us = Count100us and SpillWidthCount = SpillLength)
      or IntTmgEn = '0'
then SpillWidthCount <= (others => '0');
end if;

-- Count the 5 ms interspill length
if Spill_Req = '1' and Beam_On = '0' and InterSpillCount /= InterSpillLength 
	and Counter100us = Count100us
 then InterSpillCount <= InterSpillCount  + 1;
elsif IntTmgEn = '0' or Spill_Req = '0' or (InterSpillCount = InterSpillLength 
	and Counter100us = Count100us)
 then InterSpillCount <= (others => '0');
else InterSpillCount <= InterSpillCount;
end if;

if uBunchLED = '0' and Beam_On = '0' and Spill_Req = '1' 
	and InterSpillCount = 0 and Counter100us = Count100us
then uBunchLED <= '1';
elsif uBunchLED = '1' and uBunchLEDCnt = 1 then uBunchLED <= '0'; 
else uBunchLED <=  uBunchLED;
end if;

if uBunchLED = '0' and Beam_On = '0' and Spill_Req = '1' 
	and InterSpillCount = 0 and Counter100us = Count100us
then uBunchLEDCnt <= '1' & X"4";
elsif uBunchLEDCnt /= 0 and Counter1ms = Count1ms
then uBunchLEDCnt <= uBunchLEDCnt - 1;
else uBunchLEDCnt <= uBunchLEDCnt;
end if; 

-- Uptime in seconds since th last FPGA configure
if	Counter1s = Count1s then UpTimeCount <= UpTimeCount + 1;
else UpTimeCount <= UpTimeCount;
end if;

-- Register for staging uptime count.
if CpldCS = '1' then UpTimeStage <= UpTimeCount;
else UpTimeStage <= UpTimeStage;
end if;

-- Testcounter counter is writeable. For each read of the lower half, the entire
-- 32 bit counter increments
if    WRDL = 1 and  uCA(11 downto 10) = GA and uCA(9 downto 0) = TestCounterHiAd 
then TestCount <= (uCD & TestCount(15 downto 0));
elsif WRDL = 1 and  uCA(11 downto 10) = GA and uCA(9 downto 0) = TestCounterLoAd 
then TestCount <= (TestCount(31 downto 16) & uCD);
elsif RDDL = 2 and AddrReg(11 downto 10) = GA and AddrReg(9 downto 0) = TestCounterLoAd 
then TestCount <= TestCount + 1;
else TestCount <= TestCount;
end if;


if WRDL = 1 and  uCA(11 downto 10) = GA and uCA(9 downto 0) = GTPCSRAddr  
 then   TDisA <= uCD(0); 
		TDisB <= uCD(8);
 else 
		TDisA <= TDisA;
		TDisB <= TDisB;
end if;

-- Serializer for front panel LEDs
ClkDiv <= ClkDiv + 1;

if    WRDL = 1 and  uCA(11 downto 10) = GA 
		and uCA(9 downto 0) >= LEDDatAddr(0) and  uCA(9 downto 0) <= LEDDatAddr(5)
then CMDwr_en <= '1';
else CMDwr_en <= '0';
end if;

-- Idle,Load,Shift,SendPClk,RdFIFO 
case LED_Shift is
	when Idle => 
		if CMD_Empty = '0' and ClkDiv = 7 then LED_Shift <= Load;
		elsif 
			WRDL = 1 and  uCA(11 downto 10) = GA and uCA(9 downto 0) = LEDRstAddr
			and uCD(0) = '1'  then LED_Shift <= WaitRst;
		else LED_Shift <= Idle;
		end if;
	when Load => 
		   if ClkDiv = 7 then LED_Shift <= Shift;
			else LED_Shift <= Load;
			end if;
	when Shift =>
		if CMDBitCount = 0 and ClkDiv = 7 then LED_Shift <= RdFIFO;
		else LED_Shift <= Shift;
		end if;
	when RdFIFO => LED_Shift <= SendPClk;
	when WaitRst =>
	    if ClkDiv = 7 then LED_Shift <= SendRst;
		 else LED_Shift <= WaitRst;
		 end if;
	when SendRst =>
	    if ClkDiv = 7 then LED_Shift <= WaitPClk;
		 else LED_Shift <= SendRst;
		 end if;
   when WaitPClk =>
	    if ClkDiv = 7 then LED_Shift <= SendPClk;
		 else LED_Shift <= WaitPClk;
		 end if;
	when SendPClk => 
		if ClkDiv = 7 then LED_Shift <= Idle;
		else LED_Shift <= SendPClk;
		end if;
end case;

if LED_Shift = SendRst then LEDRst <= '0';
else LEDRst <= '1';
end if;

if LED_Shift = Load and ClkDiv = 7 then CMDBitCount <= X"F";
elsif LED_Shift = Shift and ClkDiv = 7 then CMDBitCount <= CMDBitCount - 1;
else CMDBitCount <= CMDBitCount;
end if;

if LED_Shift = Load and ClkDiv = 7 then LEDShiftReg <= CMD_Out(15 downto 0);
elsif LED_Shift = Shift and ClkDiv = 7 then LEDShiftReg <= LEDShiftReg(14 downto 0) & '0';
else LEDShiftReg <= LEDShiftReg;
end if;

Case CMD_Out(18 downto 17) is
	when "00" => LEDSDat <= "00" & LEDShiftReg(15);
					 LEDSClk <= "00" & ClkDiv(2);
	when "01" => LEDSDat <= '0' & LEDShiftReg(15) & '0';
					 LEDSClk <= '0' & ClkDiv(2) & '0';
	when "10" => LEDSDat <= LEDShiftReg(15) & "00";
					 LEDSClk <= ClkDiv(2) & "00";
	when others => LEDSDat <= "000";
					   LEDSClk <= "000";
end case;

if LED_Shift = Shift then 
	Case CMD_Out(18 downto 17) is
		when "00" => LEDSClk <= "00" & ClkDiv(2);
		when "01" => LEDSClk <= '0' & ClkDiv(2) & '0';
		when "10" => LEDSClk <= ClkDiv(2) & "00";
		when others => LEDSClk <= "000";
	end case;
  else LEDSClk <= "000";
end if;

if LED_Shift = SendPClk then
Case CMD_Out(18 downto 16) is
	when "000" => LEDLd <= "000001";
	when "001" => LEDLd <= "000010";
	when "010" => LEDLd <= "000100";
	when "011" => LEDLd <= "001000";
	when "100" => LEDLd <= "010000";
	when "101" => LEDLd <= "100000";
	when others => LEDLd <= "000000";
end case;
else LEDLd <= "000000";
end if;

if LED_Shift = RdFIFO then CMDrd_en <= '1';
else CMDrd_en <= '0';
end if;

-- Serializer for the PLL chip
-- Pll data is 24 bits. Stage the upper order eight bits in a register
if WRDL = 1 and  uCA(11 downto 10) = GA and uCA(9 downto 0) = PLLHiAddr
 then PllStage <= uCD(7 downto 0);
else PllStage <= PllStage;
end if;
-- Apply the Staging register contents and the uC data bus to the FIFO input
if WRDL = 1 and  uCA(11 downto 10) = GA and uCA(9 downto 0) = PLLLoAddr
 then PLLBuffwr_en <= '1';
else PLLBuffwr_en <= '0';
end if;

-- Idle,Load,Shift,WaitLd,SendLd
Case Pll_Shift is when 
	Idle => 
	  if PLLBuff_empty = '0' and ClkDiv = 7 
	  then Pll_Shift <= Load;
	 else Pll_Shift <= Idle;
	 end if;
	When Load =>
	  if ClkDiv = 7 then Pll_Shift <= Shift;
	  else Pll_Shift <= Load;
	  end if;
	When Shift =>
	 if PllBitCount = 0 and ClkDiv = 7 
		then Pll_Shift <= WaitLd;
	 else Pll_Shift <= Shift;
	end if;
	When WaitLd =>
		if ClkDiv = 7 then Pll_Shift <= SendLd;
		else Pll_Shift <= WaitLd;
		end if;
	When SendLd =>
	 if ClkDiv = 7 then Pll_Shift <= Idle;
	 else Pll_Shift <= SendLd;
	 end if;
end Case;

-- Pll Shifter bit counter
if Pll_Shift = Load and ClkDiv = 7 then PllBitCount <= '1' & X"7";
elsif Pll_Shift = Shift and ClkDiv = 7 then PllBitCount <= PllBitCount - 1;
else PllBitCount <= PllBitCount;
end if;

-- Pll Shiftter shifter register
if Pll_Shift = Load and ClkDiv = 7 then PllShiftReg <= PLLBuff_Out;
elsif Pll_Shift = Shift and ClkDiv = 7 then PllShiftReg <= PllShiftReg(22 downto 0) & '0';
else PllShiftReg <= PllShiftReg;
end if;

-- Read the PLL fifo at the end of the shift sequence
if Pll_Shift = Load and ClkDiv = 7 then PLLBuffrd_en <= '1'; 
else PLLBuffrd_en <= '0'; 
end if;

-- PLL SPI port serial clock
if Pll_Shift = Shift then PllSClk <= ClkDiv(2);
else PllSClk <= '0';
end if;

PllSDat <= PllShiftReg(23); 

-- Assert a load pulse after the shift sequence is done
if Pll_Shift = SendLd then PllLd <= '1'; 
else PllLd <= '0'; 
end if;

if WRDL = 1 and  uCA(11 downto 10) = GA and uCA(9 downto 0) = PLLPDnAddr 
 then PllPDn <= uCD(0);
else PllPDn <= PllPDn;
end if;

------------------------------- Trigger Logic ----------------------------

PhaseAcc <= PhaseAcc + FreqReg;
PhaseAccD <= PhaseAcc(31);


if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = CSRRegAddr 
and uCD(5) = '1' then Buff_Rst <= '1';
else Buff_Rst <= '0';
end if;

-- Reset the trigger counter on begin spill and uCA(11 downto 10) = GA 
if Buff_Rst = '1' 
then TrigCounter <= (others => '0');
elsif TrigPls = '1' then TrigCounter <= TrigCounter + 1;
else TrigCounter <= TrigCounter;
end if;

-- The "OR" of the various trigger sources
if TrigEn = '1' and
-- LEMO trigger, software trigg
 (((Beam_On = '1' and TrgSrc = '0' and GPIDL(0) = 1) or (WrDL = 1 and uCA(9 downto 0) = TrigCtrlAddr and uCD(0) = '1')
-- internal trigger rate generator
	or (TstPlsEn = '1' and PhaseAcc(31) = '1' and PhaseAccD = '0'))
-- external triggers from the controller
or (TrgSrc = '1' and Beam_On = '1' and ((RxOut.Done = '1' and (Rx1Dat(15 downto 12) = EventTrig  
	or Rx1Dat(15 downto 12) = EventTrigD)))))
then TrigPls <= '1'; 
else TrigPls <= '0'; 
end if;

if TstTrigEn = '1' and HrtBtTxEn = '0' and PhaseAcc(31) = '1' and PhaseAccD = '0' then BmOnTrigReq <= '1';
elsif HrtBtTxEn = '1' then BmOnTrigReq <= '0';
else BmOnTrigReq <= BmOnTrigReq;
end if;

-- Make a trigger pulse two clocks wide for crossing clock boundaries to the AFE sections
if (TrigEn = '1' and
 (((Beam_On = '1' and TrgSrc = '0' and GPIDL(0) = 1) or (WrDL = 1 and uCA(9 downto 0) = TrigCtrlAddr and uCD(0) = '1')
	or (TstPlsEn = '1' and PhaseAcc(31) = '1' and PhaseAccD = '0'))
or (TrgSrc = '1' and Beam_On = '1' and 
	((RxOut.Done = '1' and (Rx1Dat(15 downto 12) = EventTrig or Rx1Dat(15 downto 12) = EventTrigD))))))
or TrigPls = '1'
then TrigReq <= '1'; 
else TrigReq <= '0';
end if;

-- Trig out width counter
if GPOCount = 0 and TrigReq = '1' then GPOCount <= "111";
elsif GPOCount /= 0 then GPOCount <= GPOCount - 1;
else GPOCount <= GPOCount;
end if;

-- Flag bit indicating a LEMO trigger 
	if GPIDL(0) = 1 then	ExtTrig <= '1';
elsif (WrDL = 1 and uCA(9 downto 0) = TrigCtrlAddr and uCD(0) = '1') or TstTrigEn = '1'
then ExtTrig <= '0';
else ExtTrig <= ExtTrig;
end if;

-- Flag bit indicating a software trigger 
if WrDL = 1 and uCA(9 downto 0) = TrigCtrlAddr and uCD(0) = '1'
 then IntTrig <= '1';
 elsif GPIDL(0) = 1 or TstTrigEn = '1'
 then  IntTrig <= '0';
 else  IntTrig <= IntTrig;
end if;

-- DG: TTL PPS logic

-- use GPI input (ExtPPSBUF) as reset to trigger time counter
if ExtPPSBUF = 1 and DoTriggerTimeStampExtReset = '1'
	then
		TriggerTimeStampCount <= (others => '0');
	else
		TriggerTimeStampCount <= TriggerTimeStampCount + 1;
end if;

-- DG: Nim-Trig logic
-- Write to ExternalTriggerControl D0 to reset trigger count
if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = ExternalTriggerControlAddress and uCD(0)='1'
	then COUNTRESET <='1';
	else COUNTRESET <= '0';
end if;
-- Write to ExternalTriggerControl D1 to manually trigger
if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = ExternalTriggerControlAddress  and uCD(1)='1'
	then MANTRIG <='1';
	else MANTRIG <= '0';
end if;
-- DG: D2 to set whether to make microbunch number generation periodic
-- ignored if D0 or D1 are set
if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = ExternalTriggerControlAddress
	and uCD(1 downto 0) = "00"
	then PeriodicMicrobunch <= uCD(2);
	else PeriodicMicrobunch <= PeriodicMicrobunch;
end if;
-- DG: D3 to set whether to reset TriggerTimeSTamp on external GPI pulse (PPS)
-- ignored if D0 or D1 are set
if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = ExternalTriggerControlAddress
	and uCD(1 downto 0) = "00"
	then DoTriggerTimeStampExtReset <= uCD(3);
	else DoTriggerTimeStampExtReset <= DoTriggerTimeStampExtReset;
end if;
-- DG: bits 4-9 set the trigger type gate
-- ignored if D) or D1 are set
if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = ExternalTriggerControlAddress
	and uCD(1 downto 0) = "00"
	then TriggerTypeGatePeriod <= uCD(9 downto 4);
	else TriggerTypeGatePeriod <=  TriggerTypeGatePeriod;
end if;
	
-- DG: TODO: add configuration if trigger signal is synchronous w.r.t. SysClk
NimTrigBUF(0) <= NimTrig;
NimTrigBUF(1) <= NimTrigBUF(0);

-- DG: input PPS
-- ExtPPSHold <= GPI;
ExtPPSBUF(0) <= GPI;
ExtPPSBUF(1) <= ExtPPSBUF(0);

-- Recognizing External NIM Trigger
if ExtTriggerInhibitCount = 0 and
	TriggerTypeGateCount = 0 and
	COUNTRESET = '0' and
	(NimTrigBuf = 2 or MANTRIG = '1')
then
	-- begin the gate
	TriggerTypeGateCount <= TriggerTypeGatePeriod;
-- reset
elsif COUNTRESET = '1'
then
	TriggerTypeGateCount <= (others => '0');
-- countdown
elsif TriggerTypeGateCount /= 0
then
	TriggerTypeGateCount <= TriggerTypeGateCount - 1;
else
	TriggerTypeGateCount <= TriggerTypeGateCount;
end if;

-- set to issue trigger
if COUNTRESET = '0' and (
		-- Gate is enabled -- wait for the count to wind down
		TriggerTypeGateCount = 1 or 
		-- Gate is disabled -- edge detect external trigger
		(TriggerTypeGatePeriod = 0 and ExtTriggerInhibitCount = 0 and (NimTrigBuf = 2 or MANTRIG = '1')))
then
	IssueExtNimTrigger <= '1';

else
	IssueExtNimTrigger <= '0';
end if;

-- determine the trigger variant when we issue the trigger
if IssueExtNimTrigger = '1' and COUNTRESET = '0'
then
	-- not configured to check for on/off beam -- default to off beam
	if TriggerTypeGatePeriod = 0
		then TriggerType <= TriggerVariantBeamOff;
	-- configured to check
	-- if trigger is still active (active-lo) after gate time, beam ON
	elsif NimTrigBuf(0) = '0'
		then TriggerType <= TriggerVariantBeamOn;
	-- if trigger is low, set to beam OFF
	else TriggerType <= TriggerVariantBeamOff;
	end if;
else
	TriggerType <= TriggerType;
end if;

-- Ready to issue the trigger
if IssueExtNimTrigger = '1' and COUNTRESET = '0'
then
	NimTrigCount <= NimTrigCount + '1';
	if IntTmgEn = '1' then
		HrtBtTxEnExtTrig <= '1'; 
	else
		HrtBtTxEnExtTrig <= '0'; 
	end if;
	if TstTrigEn = '1' then
		Trig_Tx_Req <= '1';
	else
		Trig_Tx_Req <= '0';
	end if;
else
	if COUNTRESET = '1'
		then NimTrigCount <= (others => '0');
		else NimTrigCount <= NimTrigCount;
	end if;
	NimTrigCount <= NimTrigCount;
	HrtBtTxEnExtTrig <= '0';
	Trig_Tx_Req <= Trig_Tx_Req;
end if;

if Trig_Tx_Ack = '1'
	then Trig_Tx_Req <= '0';
end if;

-- external trigger inhibit setting
if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = ExternalTriggerInhibitAddrLo
then 
	ExtTriggerInhibit(15 downto 0) <= uCD(15 downto 0);
elsif WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = ExternalTriggerInhibitAddrMd
then 
	ExtTriggerInhibit(31 downto 16) <= uCD(15 downto 0);
elsif WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = ExternalTriggerInhibitAddrHi
	then 
	ExtTriggerInhibit(47 downto 32) <= uCD(15 downto 0);
else 
	ExtTriggerInhibit <= ExtTriggerInhibit;
end if;

-- Count of the Inhibit -- if it is not zero, decrement
if ExtTriggerInhibitCount /= 0
	then 
	ExtTriggerInhibitCount <= ExtTriggerInhibitCount - 1;
-- if we see a trigger, reset the inhibit
elsif NimTrigBuf = 2
	then 
	ExtTriggerInhibitCount <= ExtTriggerInhibit;
else
	ExtTriggerInhibitCount <= ExtTriggerInhibitCount;
end if;

-- Increment the microbunch number
-- 
if 
	-- whether controller should set microbunch
	IntTmgEn = '1' and CountReset = '0' and 
	-- if setting microbunch periodically
	((PeriodicMicrobunch = '1' and PeriodicMicrobunchPeriod /= 0 and PeriodicMicrobunchPeriod = PeriodicMicrobunchCount)
	-- if setting microbunch from external trigger
	or (PeriodicMicrobunch = '0' and IssueExtNimTrigger = '1'))
		then MicrobunchCount <= MicrobunchCount + 1;
elsif CountReset = '1' or IntTmgEn = '0'
then 
		MicrobunchCount <= (others => '0');
else
		MicrobunchCount <= MicrobunchCount;
end if;

-- Save the trigger timestamp:
if TstTrigEn = '1' and
	-- This buffer should save the exact clock cycle where
	-- the trigger  arrived, so don't  wait for "IssueExtNimTrigger" to be ready
	ExtTriggerInhibitCount = 0 and
	COUNTRESET = '0' and
	(NimTrigBuf = 2 or MANTRIG = '1')
then
	ExtTrigTStampBuff_wr_en <= '1';
else 
	ExtTrigTStampBuff_wr_en <= '0';
end if;
-- Save the Microbunch Count:
if TstTrigEn = '1' and
	IssueExtNimTrigger = '1'
then
	TStmpBuff_wr_en <= '1';
else
	TStmpBuff_wr_en <= '0';
end if;
end if; --rising edge

end process;

------------------- mux for reading back registers -------------------------

with uCA(9 downto 0) select

iCD <= X"0" & "00" & TstTrigCE & TstTrigEn & '0' & TrigTx_Sel 
		 & '0' & ExtTmg & '0' & FormHold & TmgCntEn & IntTmgEn when CSRRegAddr,
		 X"0000" when GTPCSRAddr,
		 X"0000" when GTPFIFOAddr,
		 X"000" & "000" & PllPDn when PLLPDnAddr,
		 DReqBuff_Out(15 downto 0) when TRigReqBuffAd,
		 X"0" & '0' & TrgPktRdCnt when TRigReqWdUsedAd,
		 EventBuff_Out when EventBuffAd,
		 X"000" & "00" & EventBuff_Full & EventBuff_Empty when EventBuffStatAd,
		 X"000" & "00" & TrgSrc & TstPlsEn when TrigCtrlAddr,
		 X"00" & ActiveReg(23 downto 16) when ActvRegAddrHi,
		 ActiveReg(15 downto 0) when ActvRegAddrLo,
		 X"000" & IDReg when IDregAddr,
		 X"0" & "00" & Debug when DebugPinAd,
		 X"000" & '0' & FormStatReg when GTPSeqStatAd,
		 TrigCounter(31 downto 16) when SpillTrigCntAdHi,
		 TrigCounter(15 downto 0) when SpillTrigCntAdLo,
		 X"000" & '0' & Beam_On & '0' & Seq_Busy when SpillStatAddr,
		 UpTimeStage(31 downto 16) when UpTimeRegAddrHi,
		 UpTimeStage(15 downto 0) when UpTimeRegAddrLo,
		 TestCount(31 downto 16) when TestCounterHiAd,
		 TestCount(15 downto 0) when TestCounterLoAd,
		 LinkFIFOOut(0) when LinkRdAddr(0),
		 LinkFIFOOut(1) when LinkRdAddr(1),
		 LinkFIFOOut(2) when LinkRdAddr(2),
		 X"00" & '0' & LinkFIFOFull & '0' & LinkFIFOEmpty when LinkCSRAddr,
		 --GTPRxBuff_Out(0) when GTPRdAddr0,
		 --GTPRxBuff_Out(1) when GTPRdAddr1,
		 TxCRC(0) when CRCRdAddr(0),
		 TxCRC(1) when CRCRdAddr(1),
		 --RxCRC(0) when CRCRdAddr(2),
		 --RxCRC(1) when CRCRdAddr(3),
		 "00" & EvTxWdCnt when EvTxWdCntAd,
		 "000" & LinkFIFORdCnt(0) when LinkWdCnt0Ad,
		 "000" & LinkFIFORdCnt(1) when LinkWdCnt1Ad,
		 "000" & LinkFIFORdCnt(2) when LinkWdCnt2Ad,
		 X"000" & "00" & EventBuff_Full & EventBuff_empty when EvBuffStatAd,
		 '0' & GtpRxBuffStat(1) & '0' & GtpRxBuffCnt(1) 
	  & '0' & GtpRxBuffStat(0) & '0' & GtpRxBuffCnt(0) when ElasticStatAd,
	    DReqBrstCntReg when DReqBrstCntAd,
	    X"00" & HrtBtBrstCntReg(23 downto 16) when HrtBtBrstCntAdHi,
	    HrtBtBrstCntReg(15 downto 0) when HrtBtBrstCntAdLo,
		 MicrobunchCount(47 downto 32) when MicroBunchAdHi,
		 MicrobunchCount(31 downto 16) when MicroBunchAdMid,
		 MicrobunchCount(15 downto 0) when MicroBunchAdLo,
		 FreqReg(31 downto 16) when FreqRegAdHi,
		 FreqReg(15 downto 0) when FreqRegAdLo,
		 HeartBeatFreqReg(31 downto 16) when HeartBeatFreqRegAdHi,
		 HeartBeatFreqReg(15 downto 0)  when HeartBeatFreqRegAdLo,
		 X"00" & MarkerBits when MarkerBitsAd,
		 -- DG: address to querry number of triggers
		 COUNTRESET & NimTrigCount(14 downto 0) when ExternalTriggerInfoAddress,
		 X"0" & "00" & TriggerTypeGatePeriod & DoTriggerTimeStampExtReset & PeriodicMicrobunch & "00" when ExternalTriggerControlAddress,
		 PeriodicMicrobunchPeriod(31 downto 16) when PeriodicMicrobunchPeriodAddrHi,
		 PeriodicMicrobunchPeriod(15 downto 0) when PeriodicMicrobunchPeriodAddrLo,
		 ExtTriggerInhibit(47 downto 32) when ExternalTriggerInhibitAddrHi,
		 ExtTriggerInhibit(31 downto 16) when ExternalTriggerInhibitAddrMd,
		 ExtTriggerInhibit(15 downto 0) when ExternalTriggerInhibitAddrLo,
		 X"0000" when others;

-- Select between the Orange Tree port and the rest of the registers
uCD <= iCD when uCRd = '0' and CpldCS = '0' and uCA(11 downto 10) = GA 
		 else iDQ when uCRd = '0' and EthCS = '0'  
		 else (others => 'Z');

end behavioural;
