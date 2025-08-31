/******************************************************************************
* Author :  YY
*
******************************************************************************/

 #include <stdio.h>
 #include "platform.h"
 #include "common.h"
 #include "xil_printf.h"

 #define	TSN_BASEADDR		(0x40000000)
 #define	DMA_BASEADDR_0		(0x40400000)
 #define	DMA_BASEADDR_1		(0x40410000)
 #define	BRAM_BASEADDR		(0x42000000)
 #define	IRQ_REG_BASEADDR	(0x44000000)
 //#define	INTC_BASEADDR		(0xa0000000)

 #define RCV_CFG_WORD_0_OFFSET (0x400)
 #define RCV_CFG_WORD_1_OFFSET (0x404)

 #define RX_PTP_BUFFER_BASEADDR	(TSN_BASEADDR + 0x10000)
 #define RX_PTP_BUFFER_OFFSET   	(0x100)

 #define TX_PTP_BUFFER_BASEADDR	(TSN_BASEADDR + 0x11000)
 #define TX_PTP_BUFFER_CTRL		(TSN_BASEADDR + 0x12000)

 #define TX_PTP_CTRL_OFFSET (0x12000)
 #define RX_PTP_CTRL_OFFSET (0x12004)

 #define RTC_NANOSECONDS_FIELD_OFFSET (0x12800)
 #define RTC_SECONDS_LSB_FIELD_OFFSET (0x12808)
 #define RTC_SECONDS_MSB_FIELD_OFFSET (0x1280C)

 #define RTC_IVCR_OFFSET (0x12810)
 #define RTC_IVCR_OFFSET (0x12810)
 #define CURRENT_RTC_NANOSECONDS_OFFSET (0x12814)
 #define CURRENT_RTC_SECONDS_LSB_OFFSET (0x12818)
 #define CURRENT_RTC_SECONDS_MSB_OFFSET (0x1281C)

 #define Syntonized_NANOSECONDS_FIELD_OFFSET (0x1282c)
 #define Syntonized_SECONDS_LSB_FIELD_OFFSET (0x12830)
 #define Syntonized_SECONDS_MSB_FIELD_OFFSET (0x12834)

#define INCRVALUE (0x00800000)
 #define SYNC                   0x0
 #define PDELAY_REQ             0x2
 #define PDELAY_RESP            0x3
 #define FOLLOWUP               0x8
 #define PDELAY_RESPFOLLOWUP    0xa
 #define ANNOUNCE               0xb
 #define SIGNALING              0xc

#define TWO_STEP_FLAG 0x1

 #define SYNC_BUFNUM                   0x0
 #define PDELAY_REQ_BUFNUM             0x1
 #define PDELAY_RESP_BUFNUM            0x2
 #define FOLLOWUP_BUFNUM               0x3
 #define PDELAY_RESPFOLLOWUP_BUFNUM    0x4
 #define ANNOUNCE_BUFNUM               0x5
 #define SIGNALING_BUFNUM              0x6

#define MM2S_DMACR_OFFSET    0x00
#define MM2S_DMASR_OFFSET	 0x04
#define MM2S_DA_OFFSET	 	 0x18
#define MM2S_LENGTH			 0x28

#define S2MM_DMACR_OFFSET    0x30
#define S2MM_DMASR_OFFSET	 0x34
#define S2MM_DA_OFFSET	 	 0x48
#define S2MM_LENGTH			 0x58
// tx ptp buffer
typedef struct
{
  union
  {
    struct
    {
      UINT8     frmLen[2];
      UINT8     cmd[6];
      UINT8     dstAddr[6]; //start of frmLen
      UINT8     srcAddr[6];
      UINT8     ethType[2];
      UINT8     messageType; // start of messageLength
      UINT8     versionPTP;
      UINT8     messageLength[2];
      UINT8     domainNumber;
      UINT8     minorSdoId;
      UINT8     flags[2];
      UINT8     correctionField[8];
      UINT8     messageTypeSpecific[4];
      UINT8     clockId[8];
      UINT8     portNum[2];
      UINT8     sequenceID[2];
      UINT8     controlFiled;
      UINT8     logMessageInterval;
      UINT8     originTimestamp[10];
      union
      {
        UINT8     reqPortId[10];
        UINT8     TLV[32];
        struct {
        	UINT8 currentUtcOffset[2];
        	UINT8 rsv;
        	UINT8 grandmasterPriority1;
        	UINT8 grandmasterClockclass;  		// grandmasterClockQuality[0];
        	UINT8 grandmasterClockAccuracy;		// grandmasterClockQuality[1];
        	UINT8 grandmasterClockVariance[2];	// grandmasterClockQuality[2]-[3];
        	UINT8 grandmasterPriority2;
        	UINT8 grandmasterIdentity[8];
        	UINT8 stepsRemoved[2];
        	UINT8 timeSource;
        	struct{
        		UINT8 tlvType[2];
        		UINT8 lengthFiled[2];
        		UINT8 pathSequence[][8];
        	};
        };
      }
    };
    UINT32 buf[64]; //ptp buffer 256
  };
}TxBuffer;

TxBuffer   PdelayReqBuf;
TxBuffer   PdelayRespBuf;
TxBuffer   PdelayRespFollowUpBuf;
TxBuffer   SyncBuf;
TxBuffer   FollowUpBuf;
TxBuffer   AnnounceBuf;
 typedef struct
{
	UINT32 	MSB;
	UINT32 	LSB;
	UINT32  NS;
}TimeStamp;

TimeStamp g_PTP_TXBuffer_TS;
TimeStamp gRTCOffset;
TimeStamp gRTCOffsetSav;
TimeStamp gSyncInterval[2];

TimeStamp g_NearEnd_PdelayReqEgressT1;
TimeStamp g_NearEnd_PdelayReqIngressT2;
TimeStamp g_NearEnd_PdelayRespEgressT3;
TimeStamp g_NearEnd_PdelayRespIngressT4;
TimeStamp g_FarEnd_PdelayReqEgressT1;
TimeStamp g_FarEnd_PdelayReqIngressT2;
TimeStamp g_FarEnd_PdelayRespEgressT3;
TimeStamp g_FarEnd_PdelayRespIngressT4;

TimeStamp g_SyncTs;
TimeStamp g_NearEnd_Sync_TS[2];
TimeStamp g_FarEnd_Sync_TS[2];

TimeStamp RTC_Time;
TimeStamp UTC_Time;
TimeStamp RX_PTP_TimeStamp;
TimeStamp g_FarEnd_PdelayReqReciptTime;
TimeStamp Syntonized_Time;

TimeStamp FarEndReqRespOffset;

TimeStamp g_InitialTime;

double NearEndRatioOffset;
double NearEndRatioOffsetComp;
double FarEndRatioOffset;

//RTC offset;
double Ratio =1;
double RatioWindow[8] = {1,1,1,1,1,1,1,1};
double RatioSav[2048];
double RatioMeanSav[2048];
double RatioMean =1;
double IncrValueDouble = (double)0x00800000;
UINT32 SecondsInNs = 0x3B9ACA00;
UINT32 IncrValue = 0x00800000;
UINT32 P2PDelay[2];
UINT32 P2PDelayOffset;
UINT32 RatioEn = 0;
UINT32 SyncCnt = 0;
UINT32 Uerr;
UINT32 DelayMeasureStateM = 5;
UINT32 MinsIndex;

UINT32 NanoSecondTimer0 = 0;
UINT32 NanoSecondTimer1 = 0;

UINT32 TxStateBusy = 0;
UINT16 g_PdealyReqSequenceID = 0;
UINT16 g_SyncSequenceID = 0;
UINT16 g_AnnounceSequenceID = 0;
UINT16 g_FollowUpSequenceID = 0;
UINT16 FarEndSequenceID = 0;

UINT64 UTC_TimeInNS;
UINT64 PTP_TimeInNS;

UINT32 StepMode = 2;
UINT32 SequenceID = 0;
UINT32 RequestingPortID[3];
UINT32 MannualDelay = 0xF000000; // 100000000

UINT32 g_TxBusy = 0;

UINT32 TimePlus = 0;
UINT64 TimeOffset;
// init table
UINT32 	logMessageIntervalInit = 0;
UINT8 	DstAddrInit[6] = {0x01,0x80,0xc2,0x00,0x00,0x0e};
UINT8 	SrcAddrInit[6] = {0x00,0x00,0x02,0x01,0x01,0x03};
UINT8	clockIdInit[8] = {0x00,0x00,0x22,0xfa,0xeb,0x01,0x01,0x03};
UINT8	portNumInit[2] = {0x00,0x05};
UINT8 	MacAddrInit[6] = {0x00,0x00,0x02,0x01,0x01,0x03};

typedef struct
{
	UINT64  TimeOffset;
	UINT32 	TimePlus;
}TimeOffsetS;

TimeOffsetS TimeOffsetSav[1024];

double CalculateRatioMean()
{
	UINT32 cnt = 0;
	double Sum = 0;
	double Mean = 0;
	for(cnt = 0; cnt<8; cnt++)
	{
		Sum += RatioWindow[cnt];
	}

	Mean = Sum/cnt;
	return Mean;

}
void GetOriginTimeStamp(UINT32 PTP_Number,TimeStamp *PTP_TimeStamp)
{
	UINT32 TS[3];

	for(int Num = 0; Num <3; Num++)
	{
		TS[Num] = Read32(RX_PTP_BUFFER_BASEADDR + PTP_Number + 48 + Num*4);
	}
	PTP_TimeStamp->MSB =    TS[0] & 0xffff;   //low 16 bit

	PTP_TimeStamp->LSB =   (((TS[0] >> 16) & 0xff) <<24)    \
						+ (((TS[0] >> 24) & 0xff) <<16)     \
						+ (((TS[1] >> 0 ) & 0xff) <<8)      \
						+ (((TS[1] >> 8) & 0xff) );

	PTP_TimeStamp->NS  =   (((TS[1] >> 16) & 0xff) <<24)    \
						+ (((TS[1] >> 24) & 0xff) <<16)     \
						+ (((TS[2] >> 0 ) & 0xff) <<8)      \
						+ (((TS[2] >> 8) & 0xff) );
}

void GetCorrectionField(UINT32 PTP_Number,TimeStamp *PTP_TimeStamp)
{
	UINT32 TS[3];

	for(int Num = 0; Num <3; Num++)
	{
		TS[Num] = Read32(RX_PTP_BUFFER_BASEADDR + PTP_Number + 20 + Num*4);
	}
	PTP_TimeStamp->NS =   (((TS[0] >> 16) & 0xff) <<56)      \
							+ (((TS[0] >> 24) & 0xff) <<48)  \
							+ (((TS[1] >> 0 ) & 0xff) <<40)  \
							+ (((TS[1] >> 8 ) & 0xff) <<32)  \
							+ (((TS[1] >> 16) & 0xff) <<24)  \
							+ (((TS[1] >> 24) & 0xff) <<16);
}

void GetRxPTPTimeStamp(UINT32 PTP_Number,TimeStamp *PTP_TimeStamp)
{
	UINT32 TS[3];

	for(int Num = 0; Num <3; Num++)
	{
		TS[Num] = Read32(RX_PTP_BUFFER_BASEADDR + PTP_Number + 0xf4 + Num*4);
	}
	PTP_TimeStamp->MSB =    0;//TS[0] & 0xffff;   //low 16 bit
	PTP_TimeStamp->LSB =  TS[1];
	PTP_TimeStamp->NS  =  TS[2];
}

void GetTxPTPTimeStamp(UINT32 PTP_Number,TimeStamp *PTP_TimeStamp)
{
	UINT32 TS[3];

	for(int Num = 0; Num <3; Num++)
	{
		TS[Num] = Read32(TX_PTP_BUFFER_BASEADDR + PTP_Number + 0xf4 + Num*4);
	}
	PTP_TimeStamp->MSB =    0;//TS[0] & 0xffff;   //low 16 bit
	PTP_TimeStamp->LSB =  TS[1];
	PTP_TimeStamp->NS  =  TS[2];
}


void CopyTimeStamp(TimeStamp *SrcTS,TimeStamp *DstTS)
{
	DstTS->MSB =  SrcTS->MSB;
	DstTS->LSB =  SrcTS->LSB;
	DstTS->NS  =  SrcTS->NS;
}


void CopyTimeStampToBuf(TimeStamp *SrcTS,TxBuffer *DstBuf)
{
	DstBuf->originTimestamp[0] = ((SrcTS->MSB >> 8  ) & 0xff);
	DstBuf->originTimestamp[1] = ((SrcTS->MSB >> 0  ) & 0xff);
	DstBuf->originTimestamp[2] = ((SrcTS->LSB >> 24 ) & 0xff);
	DstBuf->originTimestamp[3] = ((SrcTS->LSB >> 16 ) & 0xff);
	DstBuf->originTimestamp[4] = ((SrcTS->LSB >> 8  ) & 0xff);
	DstBuf->originTimestamp[5] = ((SrcTS->LSB >> 0  ) & 0xff);
	DstBuf->originTimestamp[6] = ((SrcTS->NS  >> 24 ) & 0xff);
	DstBuf->originTimestamp[7] = ((SrcTS->NS  >> 16 ) & 0xff);
	DstBuf->originTimestamp[8] = ((SrcTS->NS  >> 8  ) & 0xff);
	DstBuf->originTimestamp[9] = ((SrcTS->NS  >> 0  ) & 0xff);
}

void CopyReqPortIdToBuf(UINT32 RequestingPortID[],TxBuffer *DstBuf)
{
	DstBuf->reqPortId[0] = ((RequestingPortID[0] >> 16 ) & 0xff );
	DstBuf->reqPortId[1] = ((RequestingPortID[0] >> 24 ) & 0xff );
	DstBuf->reqPortId[2] = ((RequestingPortID[1] >> 0  ) & 0xff );
	DstBuf->reqPortId[3] = ((RequestingPortID[1] >> 8  ) & 0xff );
	DstBuf->reqPortId[4] = ((RequestingPortID[1] >> 16 ) & 0xff );
	DstBuf->reqPortId[5] = ((RequestingPortID[1] >> 24 ) & 0xff );
	DstBuf->reqPortId[6] = ((RequestingPortID[2] >> 0  ) & 0xff );
	DstBuf->reqPortId[7] = ((RequestingPortID[2] >> 8  ) & 0xff );
	DstBuf->reqPortId[8] = ((RequestingPortID[2] >> 16 ) & 0xff );
	DstBuf->reqPortId[9] = ((RequestingPortID[2] >> 24 ) & 0xff );
}

void  ProcessTXPTP(UINT32 PTP_Number)
{
	UINT32 MssgType = 0xff;

	GetTxPTPTimeStamp(PTP_Number,&g_PTP_TXBuffer_TS);
	MssgType = Read32(TX_PTP_BUFFER_BASEADDR + PTP_Number +12 + 8); //message type
	MssgType = ((MssgType >> 16) & 0xf);
	switch(MssgType)
	{
 		case  SYNC:  // sync
			CopyTimeStamp(&g_PTP_TXBuffer_TS,&g_SyncTs);
			FollowUpMsg(FOLLOWUP_BUFNUM , &FollowUpBuf);
		break;
		case  PDELAY_REQ:   //near end send Pdelay_Req
			CopyTimeStamp(&g_PTP_TXBuffer_TS,&g_NearEnd_PdelayReqEgressT1);
		break;
		case  PDELAY_RESP:  // far end recive pdelay_resp
			CopyTimeStamp(&g_PTP_TXBuffer_TS,&g_FarEnd_PdelayRespEgressT3);
			PdelayRespFollowUpMsg(PDELAY_RESPFOLLOWUP_BUFNUM, &PdelayRespFollowUpBuf);
		break;
		case  PDELAY_RESPFOLLOWUP:  //  pdelay_resp follow up

		break;
		case  ANNOUNCE:  //

		break;
		case  FOLLOWUP:  //
			AnnounceMsg(ANNOUNCE_BUFNUM , &AnnounceBuf);
		break;



		default: break;
	}
}

 void  ProcessRXPTP(UINT32 PTP_Number)
 {
	UINT32 cnt = 0;
	UINT32 MssgType = 0xff;

	MssgType = Read32(RX_PTP_BUFFER_BASEADDR + PTP_Number +12); //message type
	MssgType = ((MssgType >> 16) & 0xf);
	switch(MssgType)
	{
 		case    SYNC:  // 0x0  sync
		 CopyTimeStamp(&(gSyncInterval[1]),&(gSyncInterval[0]));
		 GetRxPTPTimeStamp(PTP_Number,&(gSyncInterval[1]));
		 CopyTimeStamp(&(gSyncInterval[1]),&RX_PTP_TimeStamp);
		 if(MinsIndex == 1){
			 NearEndRatioOffset =(gSyncInterval[1].LSB - gSyncInterval[0].LSB)*0x3B9ACA00 \
					 	 	 	+ gSyncInterval[1].NS - gSyncInterval[0].NS - NearEndRatioOffsetComp;
		 }
		 else{
			 NearEndRatioOffset =(gSyncInterval[1].LSB - gSyncInterval[0].LSB)*0x3B9ACA00 \
					 	 	    + gSyncInterval[1].NS - gSyncInterval[0].NS + NearEndRatioOffsetComp;
		 }
		 break;
        case    PDELAY_REQ:   // 0x2 Pdelay_Req
			GetRxPTPTimeStamp(PTP_Number,&g_FarEnd_PdelayReqIngressT2);
			FarEndSequenceID = Read32(RX_PTP_BUFFER_BASEADDR + PTP_Number + 44);
			FarEndSequenceID = (FarEndSequenceID )&0xffff;
			RequestingPortID[0] = Read32(RX_PTP_BUFFER_BASEADDR + PTP_Number + 32);
			RequestingPortID[1] = Read32(RX_PTP_BUFFER_BASEADDR + PTP_Number + 36);
			RequestingPortID[2] = Read32(RX_PTP_BUFFER_BASEADDR + PTP_Number + 40);
			// send PdelayResp
			PdelayRespMsg(PDELAY_RESP_BUFNUM, &PdelayRespBuf);

		break;
        case  PDELAY_RESP:  //  0x3 Pdelay_Resp
			// T2
			if(StepMode == 1)
			{
				GetCorrectionField(PTP_Number,&FarEndReqRespOffset);
			}
			else
			{
				GetOriginTimeStamp(PTP_Number,&g_NearEnd_PdelayReqIngressT2);
				GetRxPTPTimeStamp(PTP_Number,&g_NearEnd_PdelayRespIngressT4);
				cnt++;
			}

		break;
        case  FOLLOWUP:   //0x8 Follow_up   for 2 step mode
			GetOriginTimeStamp(PTP_Number,&UTC_Time);
			//GetRxPTPTimeStamp(PTP_Number,&RX_PTP_TimeStamp);

			//nano seconds use ptp
			if(UTC_Time.LSB > RX_PTP_TimeStamp.LSB ){
				if(UTC_Time.NS >= RX_PTP_TimeStamp.NS){
					gRTCOffset.LSB = gRTCOffset.LSB+  UTC_Time.LSB - RX_PTP_TimeStamp.LSB;
					gRTCOffset.NS = gRTCOffset.NS+ ( UTC_Time.NS - RX_PTP_TimeStamp.NS )& 0x3fffffff;
				}
				else if(UTC_Time.NS < RX_PTP_TimeStamp.NS) {
					gRTCOffset.LSB = gRTCOffset.LSB+ UTC_Time.LSB - RX_PTP_TimeStamp.LSB -1;
					gRTCOffset.NS =  gRTCOffset.NS + (0x3B9ACA00  - RX_PTP_TimeStamp.NS + UTC_Time.NS ) & 0x3fffffff;
				}
			}
			else  if (UTC_Time.LSB = RX_PTP_TimeStamp.LSB ){
				if(UTC_Time.NS >= RX_PTP_TimeStamp.NS){
					gRTCOffset.LSB = gRTCOffset.LSB+  UTC_Time.LSB - RX_PTP_TimeStamp.LSB;
					gRTCOffset.NS = gRTCOffset.NS+ ( UTC_Time.NS - RX_PTP_TimeStamp.NS )& 0x3fffffff;
				}
				else
				{
					gRTCOffset.LSB = gRTCOffset.LSB+  UTC_Time.LSB - RX_PTP_TimeStamp.LSB;
					gRTCOffset.NS = gRTCOffset.NS - (RX_PTP_TimeStamp.NS - UTC_Time.NS)& 0x3fffffff;
				}
			}
			else { // LSB<
				if(UTC_Time.NS <= RX_PTP_TimeStamp.NS){
					gRTCOffset.LSB = gRTCOffset.LSB - ( RX_PTP_TimeStamp.LSB - UTC_Time.LSB );
					gRTCOffset.NS = gRTCOffset.NS- (RX_PTP_TimeStamp.NS -  UTC_Time.NS)& 0x3fffffff;
				}
				else{
					gRTCOffset.LSB = gRTCOffset.LSB - ( RX_PTP_TimeStamp.LSB - UTC_Time.LSB -1) ;
					gRTCOffset.NS = gRTCOffset.NS - (0x3B9ACA00  + RX_PTP_TimeStamp.NS - UTC_Time.NS)& 0x3fffffff;
				}
			}

			if(P2PDelay[1] >= P2PDelay[0])
			{
				P2PDelayOffset = P2PDelay[1] - P2PDelay[0];
				if(gRTCOffset.NS + P2PDelayOffset >= 0x3B9ACA00){
					gRTCOffset.LSB += 1;
					gRTCOffset.NS =gRTCOffset.NS + P2PDelayOffset - 0x3B9ACA00;
				}
				else
				{
					gRTCOffset.NS += P2PDelayOffset;
				}
			}
			else {
				P2PDelayOffset = P2PDelay[0] - P2PDelay[1];
				if(gRTCOffset.NS - P2PDelayOffset >= 0){
					gRTCOffset.NS =gRTCOffset.NS - P2PDelayOffset;
				}
				else
				{
					gRTCOffset.LSB -= 1;
					gRTCOffset.NS = gRTCOffset.NS + 0x3B9ACA00 - P2PDelayOffset;
				}
			}
			Write32(TSN_BASEADDR + RTC_SECONDS_LSB_FIELD_OFFSET, 0);
			Write32(TSN_BASEADDR + RTC_NANOSECONDS_FIELD_OFFSET,0); // clear off set
			Write32(TSN_BASEADDR + RTC_SECONDS_LSB_FIELD_OFFSET, gRTCOffset.LSB);
			Write32(TSN_BASEADDR + RTC_NANOSECONDS_FIELD_OFFSET,gRTCOffset.NS); // Write Nano Seconds last
			UTC_TimeInNS = (UINT64)(UTC_Time.LSB & 0xffff) * 0x3B9ACA00 + (UINT64)(UTC_Time.NS);
			PTP_TimeInNS = (UINT64)(RX_PTP_TimeStamp.LSB & 0xffff) * 0x3B9ACA00 + (UINT64)(RX_PTP_TimeStamp.NS);
			if(UTC_TimeInNS >= PTP_TimeInNS){
				TimePlus = 1;
				TimeOffset = UTC_TimeInNS - PTP_TimeInNS;
			}
			else{
				TimePlus = 0;
				TimeOffset = PTP_TimeInNS - UTC_TimeInNS;
			}
			TimeOffsetSav[SyncCnt].TimePlus = TimePlus;
			TimeOffsetSav[SyncCnt].TimeOffset = TimeOffset;
			if(gRTCOffset.LSB > gRTCOffsetSav.LSB ){
				MinsIndex = 1;
				NearEndRatioOffsetComp = (gRTCOffset.LSB -  gRTCOffsetSav.LSB) * 0x3B9ACA00  + gRTCOffset.NS - gRTCOffsetSav.NS;
			}
			else  if (gRTCOffset.LSB = gRTCOffsetSav.LSB ){
				if(gRTCOffset.NS >= gRTCOffsetSav.NS){
					MinsIndex = 1;
					NearEndRatioOffsetComp =  gRTCOffset.NS - gRTCOffsetSav.NS;
				}
				else
				{
					MinsIndex = 0;
					NearEndRatioOffsetComp =  gRTCOffsetSav.NS - gRTCOffset.NS;
				}
			}
			else { // LSB<
				MinsIndex = 0;
				NearEndRatioOffsetComp = (gRTCOffsetSav.LSB -  gRTCOffset.LSB) * 0x3B9ACA00  + gRTCOffsetSav.NS - gRTCOffset.NS;
			}

			CopyTimeStamp(&gRTCOffset,&gRTCOffsetSav);
			if(DelayMeasureStateM == 5)
			{
				DelayMeasureStateM=0;
			}
			cnt++;
			CopyTimeStamp(&(g_FarEnd_Sync_TS[1]),&(g_FarEnd_Sync_TS[0]));
			CopyTimeStamp(&UTC_Time,&(g_FarEnd_Sync_TS[1]));
			SyncCnt++;
			FarEndRatioOffset =	(g_FarEnd_Sync_TS[1].LSB - g_FarEnd_Sync_TS[0].LSB)*0x3B9ACA00 \
							   + g_FarEnd_Sync_TS[1].NS  - g_FarEnd_Sync_TS[0].NS;
			Ratio = FarEndRatioOffset/NearEndRatioOffset;
			if((Ratio >=1.1  )|| (Ratio <0.9))
			{
				Ratio =1;
			}
			if(SyncCnt >=2){
				RatioEn =1;
			}
			if(RatioEn == 1){
				RatioWindow[SyncCnt & 0x3] = Ratio;
				RatioMean = CalculateRatioMean();
				RatioSav[SyncCnt] = Ratio;
				RatioMeanSav[SyncCnt] = RatioMean;

				IncrValueDouble = IncrValueDouble * Ratio;
				IncrValue = (UINT32) IncrValueDouble;
				Write32(TSN_BASEADDR+ RTC_IVCR_OFFSET, IncrValue);    //incr value ctrl reg
			}
			 PdelayReqMsg(PDELAY_REQ_BUFNUM, &PdelayReqBuf);

			cnt++;
		break;
        case  PDELAY_RESPFOLLOWUP:    // 0xa Pdelay_Resp_Follow_Up
			GetOriginTimeStamp(PTP_Number,&g_NearEnd_PdelayRespEgressT3);
			//T3
			P2PDelay[0] = P2PDelay[1];
			P2PDelay[1] = (g_NearEnd_PdelayRespIngressT4.LSB - g_NearEnd_PdelayReqEgressT1.LSB) * 0x3B9ACA00 \
					 	 + g_NearEnd_PdelayRespIngressT4.NS  - g_NearEnd_PdelayReqEgressT1.NS	\
					     - ((g_NearEnd_PdelayRespEgressT3.LSB  - g_NearEnd_PdelayReqIngressT2.LSB) * 0x3B9ACA00 \
					     + g_NearEnd_PdelayRespEgressT3.NS   - g_NearEnd_PdelayReqIngressT2.NS);
			P2PDelay[1] = (P2PDelay[1] /2);	//  divide by 2(TS[0] >> 16)
			if(P2PDelay[1] >= 0x800)
			{
				P2PDelay[1] = 0;
			}
			cnt++;
		break;
        case  ANNOUNCE:  //0xb announce

		break;
		default: break;
	 }
 }


 UINT32 DmaSend(UINT32 Len,UINT32 DmaAddr,UINT32 LocalAddr)
 {
	 //ta dma
	 UINT32 Status = 0;
	 UINT32 Rslt = 0;

	 Status = Read32(DmaAddr + MM2S_DMASR_OFFSET);
	 Rslt = Status &0x3;
	 if(Rslt !=0)
	 {
		 Write32(DmaAddr + MM2S_DMACR_OFFSET, 0x4);  //ctrl
		 Write32(DmaAddr + MM2S_DMACR_OFFSET, 0x0);  //ctrl reset
		 Write32(DmaAddr + MM2S_DMACR_OFFSET, 0x1);  //start
		 Write32(DmaAddr + MM2S_DA_OFFSET, LocalAddr);  // Source addr  Low 32bit
		 Write32(DmaAddr + MM2S_LENGTH, Len);  // write length to start
		 return 1;
	 }
	 return 0;
 }

 UINT32 DmaRecv(UINT32 Len,UINT32 DmaAddr,UINT32 LocalAddr)
 {
	 //ta dma
	 UINT32 Status = 0;
	 UINT32 Rslt = 0;

	 Status = Read32(DmaAddr + S2MM_DMASR_OFFSET);
	 Rslt = Status &0x3;
	 if(Rslt !=0)
	 {
		 Write32(DmaAddr + S2MM_DMACR_OFFSET, 0x4);  //ctrl
		 Write32(DmaAddr + S2MM_DMACR_OFFSET, 0x0);  //ctrl reset
		 Write32(DmaAddr + S2MM_DMACR_OFFSET, 0x1);  //start
		 Write32(DmaAddr + S2MM_DA_OFFSET, LocalAddr);  // Source addr  Low 32bit
		 Write32(DmaAddr + S2MM_LENGTH, Len);  // write length to start
		 return 1;
	 }
	 return 0;
 }

 UINT32 GetDmaTransRecvLen(UINT32 BaseAddr)
 {
	 UINT32 RecvLen = 0;

	 RecvLen = Read32(BaseAddr + S2MM_LENGTH);
	 return RecvLen;

 }

 void CfgMacAddr(UINT8  MacAddr[])
 {
     UINT32 RWC0 = 0;
     UINT32 RWC1 = 0;

     //RWC0 = Read32( TSN_BASEADDR + RCV_CFG_WORD_0_OFFSET);
     RWC0 =   (MacAddr[0] << 0)  \
            | (MacAddr[1] << 8)  \
            | (MacAddr[2] << 16) \
            | (MacAddr[3] << 24);

     Write32( TSN_BASEADDR + RCV_CFG_WORD_0_OFFSET,RWC0);

     RWC1 = Read32( TSN_BASEADDR + RCV_CFG_WORD_1_OFFSET);
     RWC1 =  	(MacAddr[4]	<< 0) 	\
    		 | 	(MacAddr[5] << 8)	\
			 | 	(RWC1 & 0xffff0000) ;

     Write32( TSN_BASEADDR + RCV_CFG_WORD_1_OFFSET,RWC1);
 }

 void cfgLocalTime()
 {
	g_InitialTime.LSB = 0x00006866393f;
	g_InitialTime.NS  = 0x00000f;
	Write32(TSN_BASEADDR + RTC_SECONDS_LSB_FIELD_OFFSET, 0);
	Write32(TSN_BASEADDR + RTC_NANOSECONDS_FIELD_OFFSET,0); // clear off set
	Write32(TSN_BASEADDR + RTC_SECONDS_LSB_FIELD_OFFSET, g_InitialTime.LSB);
	Write32(TSN_BASEADDR + RTC_NANOSECONDS_FIELD_OFFSET,g_InitialTime.NS); // Write Nano Seconds last
 }
 void PdelayReqMsg( UINT32 PTP_Number , TxBuffer * PdelayReqBuf)
{
	 UINT32 i = 0;
	 UINT32 TxPtpAddr = 0;

	 g_TxBusy = 1;

     TxPtpAddr = TX_PTP_BUFFER_BASEADDR + PTP_Number*0x100;
     PdelayReqBuf->frmLen[0] = 0x44;  // 68bytes+4crc

	 PdelayReqBuf->cmd[0]  = 0x00;   //0:2step;1 :1step
	 PdelayReqBuf->cmd[1]  = 0x01;   // 1:update check sum

	 for(i = 0; i<6; i++){
	 	PdelayReqBuf->dstAddr[i] = DstAddrInit[i];
     }

	 for(i = 0; i<6; i++){
		PdelayReqBuf->srcAddr[i] = SrcAddrInit[i];
	}
	PdelayReqBuf->ethType[0]     = 0x88;
	PdelayReqBuf->ethType[1]     = 0xf7;
	PdelayReqBuf->messageType 	 = 0x10 | PDELAY_REQ;
	PdelayReqBuf->versionPTP  	 = 0x12;
	PdelayReqBuf->messageLength[0] = 0x00;
	PdelayReqBuf->messageLength[1] = 0x44;
	PdelayReqBuf->domainNumber   = 0x00;
	PdelayReqBuf->minorSdoId     = 0x00;
	PdelayReqBuf->flags[0]       = 0x02;
	PdelayReqBuf->flags[1]       = 0x00;
	for(i = 0; i<8; i++){
	    PdelayReqBuf->clockId[i] = clockIdInit[i];
    }
    for(i = 0; i<2; i++){
	    PdelayReqBuf->portNum[i] = portNumInit[i];
    }
    PdelayReqBuf->logMessageInterval = logMessageIntervalInit;
	PdelayReqBuf->sequenceID[0] = ((g_PdealyReqSequenceID >> 8) & 0xff);
	PdelayReqBuf->sequenceID[1] = ((g_PdealyReqSequenceID >> 0) & 0xff);
	g_PdealyReqSequenceID++;

    for(i = 0;i<64;i++){
        Write32(TxPtpAddr + i*4,PdelayReqBuf->buf[i]);
    }
	Write32(TX_PTP_BUFFER_CTRL, 0x1 << PTP_Number);
}


 void PdelayRespMsg(UINT32 PTP_Number , TxBuffer * PdelayRespBuf)
{
	 UINT32 i = 0;
	 UINT32 TxPtpAddr = 0;

	 g_TxBusy = 1;


     TxPtpAddr = TX_PTP_BUFFER_BASEADDR + PTP_Number*0x100;
     PdelayRespBuf->frmLen[0] = 0x68;  // 4c

	 PdelayRespBuf->cmd[0]  = 0x00;   //0:2step;1 :1step
	 PdelayRespBuf->cmd[1]  = 0x01;   // 1:update check sum

	 for(i = 0; i<6; i++){
	 	PdelayRespBuf->dstAddr[i] = DstAddrInit[i];
     }

	 for(i = 0; i<6; i++){
		PdelayRespBuf->srcAddr[i] = SrcAddrInit[i];
	}
	PdelayRespBuf->ethType[0]     = 0x88;
	PdelayRespBuf->ethType[1]     = 0xf7;
	PdelayRespBuf->messageType 	 = 0x10 | PDELAY_RESP; // response :0x3
	PdelayRespBuf->versionPTP  	 = 0x12;
	PdelayRespBuf->messageLength[0] = 0x00;
	PdelayRespBuf->messageLength[1] = 54;
	PdelayRespBuf->domainNumber   = 0x00;
	PdelayRespBuf->minorSdoId     = 0x00;
	PdelayRespBuf->flags[0]       = 0x02;
	PdelayRespBuf->flags[1]       = 0x00;
	for(i = 0; i<8; i++){
	    PdelayRespBuf->clockId[i] = clockIdInit[i];
    }
    for(i = 0; i<2; i++){
	    PdelayRespBuf->portNum[i] = portNumInit[i];
    }
    PdelayRespBuf->logMessageInterval = logMessageIntervalInit;
	PdelayRespBuf->sequenceID[0] = ((FarEndSequenceID >> 0) & 0xff);
	PdelayRespBuf->sequenceID[1] = ((FarEndSequenceID >> 8) & 0xff);

	CopyTimeStampToBuf(&g_FarEnd_PdelayReqIngressT2,PdelayRespBuf);

	CopyReqPortIdToBuf(RequestingPortID,PdelayRespBuf);

    for(i = 0;i<64;i++){
        Write32(TxPtpAddr + i*4,PdelayRespBuf->buf[i]);
    }
	Write32(TX_PTP_BUFFER_CTRL, 0x1 << PTP_Number);
}

 void PdelayRespFollowUpMsg(UINT32 PTP_Number , TxBuffer * PdelayRespFollowUpBuf)
{
	 UINT32 i = 0;
	 UINT32 TxPtpAddr = 0;

	 g_TxBusy = 1;

     TxPtpAddr = TX_PTP_BUFFER_BASEADDR + PTP_Number*0x100;
     PdelayRespFollowUpBuf->frmLen[0] = 68;  // 68bytes+4crc

	 PdelayRespFollowUpBuf->cmd[0]  = 0x00;   //0:2step;1 :1step
	 PdelayRespFollowUpBuf->cmd[1]  = 0x00;   // 1:update check sum

	 for(i = 0; i<6; i++){
	 	PdelayRespFollowUpBuf->dstAddr[i] = DstAddrInit[i];
     }

	 for(i = 0; i<6; i++){
		PdelayRespFollowUpBuf->srcAddr[i] = SrcAddrInit[i];
	}
	PdelayRespFollowUpBuf->ethType[0]     = 0x88;
	PdelayRespFollowUpBuf->ethType[1]     = 0xf7;
	PdelayRespFollowUpBuf->messageType 	 = 0x10 | PDELAY_RESPFOLLOWUP; // response follow up  : 0xa
	PdelayRespFollowUpBuf->versionPTP  	 = 0x12;
	PdelayRespFollowUpBuf->messageLength[0] = 0x00;
	PdelayRespFollowUpBuf->messageLength[1] = 54;
	PdelayRespFollowUpBuf->domainNumber   = 0x00;
	PdelayRespFollowUpBuf->minorSdoId     = 0x00;
	PdelayRespFollowUpBuf->flags[0]       = 0x00;
	//PdelayRespFollowUpBuf->flags[0]       = 0x00 | TWO_STEP_FLAG ;
	PdelayRespFollowUpBuf->flags[1]       = 0x00;
	for(i = 0; i<8; i++){
	    PdelayRespFollowUpBuf->clockId[i] = clockIdInit[i];
    }
    for(i = 0; i<2; i++){
	    PdelayRespFollowUpBuf->portNum[i] = portNumInit[i];
    }
    PdelayRespFollowUpBuf->logMessageInterval = logMessageIntervalInit;

    for(i=0;i<2;i++){
    	PdelayRespFollowUpBuf->sequenceID[i] = PdelayRespBuf.sequenceID[i];
    }
	//PdelayRespFollowUpBuf->sequenceID[1] = ((FarEndSequenceID >> 8) & 0xff);
	CopyTimeStampToBuf(&g_FarEnd_PdelayRespEgressT3,PdelayRespFollowUpBuf);

	for(i = 0;i<10; i++){
		PdelayRespFollowUpBuf->reqPortId[i] = PdelayRespBuf.reqPortId[i];
	}

    for(i = 0;i<64;i++){
        Write32(TxPtpAddr + i*4, PdelayRespFollowUpBuf->buf[i]);
    }
	Write32(TX_PTP_BUFFER_CTRL, 0x1 << PTP_Number);
}

void SyncMsg(UINT32 PTP_Number , TxBuffer * SyncBuf)
{
	 UINT32 i = 0;
	 UINT32 TxPtpAddr = 0;

     TxPtpAddr = TX_PTP_BUFFER_BASEADDR + PTP_Number*0x100;
     SyncBuf->frmLen[0] = 0x44;  // 68bytes+4crc

	 SyncBuf->cmd[0]  = 0x00;   //0:2step;1 :1step
	 SyncBuf->cmd[1]  = 0x01;   // 1:update check sum

	 for(i = 0; i<6; i++){
	 	SyncBuf->dstAddr[i] = DstAddrInit[i];
     }

	 for(i = 0; i<6; i++){
		SyncBuf->srcAddr[i] = SrcAddrInit[i];
	}
	SyncBuf->ethType[0]     = 0x88;
	SyncBuf->ethType[1]     = 0xf7;
	SyncBuf->messageType 	 = 0x10 | SYNC;
	SyncBuf->versionPTP  	 = 0x12;
	SyncBuf->messageLength[0] = 0x00;
	SyncBuf->messageLength[1] = 0x36;
	SyncBuf->domainNumber   = 0x00;
	SyncBuf->minorSdoId     = 0x00;
	SyncBuf->flags[0]       = 0x02;
	SyncBuf->flags[1]       = 0x00;
	for(i = 0; i<8; i++){
	    SyncBuf->clockId[i] = clockIdInit[i];
    }
    for(i = 0; i<2; i++){
	    SyncBuf->portNum[i] = portNumInit[i];
    }
    SyncBuf->logMessageInterval = logMessageIntervalInit;
	SyncBuf->sequenceID[0] = ((g_SyncSequenceID >> 8) & 0xff);
	SyncBuf->sequenceID[1] = ((g_SyncSequenceID >> 0) & 0xff);
	g_FollowUpSequenceID  = g_SyncSequenceID;
    g_SyncSequenceID++;

    for(i = 0;i<64;i++){
        Write32(TxPtpAddr + i*4, SyncBuf->buf[i]);
    }
	Write32(TX_PTP_BUFFER_CTRL, 0x1 << PTP_Number);
}


void FollowUpMsg(UINT32 PTP_Number , TxBuffer * FollowUpBuf)
{
	 UINT32 i = 0;
	 UINT32 TxPtpAddr = 0;

     TxPtpAddr = TX_PTP_BUFFER_BASEADDR + PTP_Number*0x100;
     FollowUpBuf->frmLen[0] = 0x6c;  // 68bytes+4crc

	 FollowUpBuf->cmd[0]  = 0x00;   //0:2step;1 :1step
	 FollowUpBuf->cmd[1]  = 0x01;   // 1:update check sum

	 for(i = 0; i<6; i++){
	 	FollowUpBuf->dstAddr[i] = DstAddrInit[i];
     }

	 for(i = 0; i<6; i++){
		FollowUpBuf->srcAddr[i] = SrcAddrInit[i];
	}
	FollowUpBuf->ethType[0]     = 0x88;
	FollowUpBuf->ethType[1]     = 0xf7;
	FollowUpBuf->messageType 	 = 0x10 | FOLLOWUP;
	FollowUpBuf->versionPTP  	 = 0x12;
	FollowUpBuf->messageLength[0] = 0x00;
	FollowUpBuf->messageLength[1] = 0x6c;//0x44;
	FollowUpBuf->domainNumber   = 0x00;
	FollowUpBuf->minorSdoId     = 0x00;
	FollowUpBuf->flags[0]       = 0x02;
	FollowUpBuf->flags[1]       = 0x00;
	for(i = 0; i<8; i++){
	    FollowUpBuf->clockId[i] = clockIdInit[i];
    }
    for(i = 0; i<2; i++){
	    FollowUpBuf->portNum[i] = portNumInit[i];
    }
    FollowUpBuf->logMessageInterval = logMessageIntervalInit;
	FollowUpBuf->sequenceID[0] = ((g_FollowUpSequenceID >> 8) & 0xff);
	FollowUpBuf->sequenceID[1] = ((g_FollowUpSequenceID >> 0) & 0xff);

	CopyTimeStampToBuf(&g_SyncTs,FollowUpBuf);

	for(i = 0;i<64;i++){
        Write32(TxPtpAddr + i*4, FollowUpBuf->buf[i]);
    }
	Write32(TX_PTP_BUFFER_CTRL, 0x1 << PTP_Number);
}

void AnnounceMsg(UINT32 PTP_Number , TxBuffer * AnnounceBuf)
{
	 UINT32 i = 0;
	 UINT32 TxPtpAddr = 0;

     TxPtpAddr = TX_PTP_BUFFER_BASEADDR + PTP_Number*0x100;
     AnnounceBuf->frmLen[0] = 0x5a;  // 68bytes+4crc

     AnnounceBuf->cmd[0]  = 0x00;   //0:2step;1 :1step
     AnnounceBuf->cmd[1]  = 0x01;   // 1:update check sum

	 for(i = 0; i<6; i++){
		 AnnounceBuf->dstAddr[i] = DstAddrInit[i];
     }

	 for(i = 0; i<6; i++){
		 AnnounceBuf->srcAddr[i] = SrcAddrInit[i];
	}
	AnnounceBuf->ethType[0]     = 0x88;
	AnnounceBuf->ethType[1]     = 0xf7;
	AnnounceBuf->messageType 	 = 0x10 | ANNOUNCE;
	AnnounceBuf->versionPTP  	 = 0x12;
	AnnounceBuf->messageLength[0] = 0x00;
	AnnounceBuf->messageLength[1] = 0x4c;
	AnnounceBuf->domainNumber   = 0x00;
	AnnounceBuf->minorSdoId     = 0x00;
	AnnounceBuf->flags[0]       = 0x0c;
	AnnounceBuf->flags[1]       = 0x00;
	for(i = 0; i<8; i++){
		AnnounceBuf->clockId[i] = clockIdInit[i];
    }
    for(i = 0; i<2; i++){
    	AnnounceBuf->portNum[i] = portNumInit[i];
    }
    AnnounceBuf->logMessageInterval = logMessageIntervalInit;
    AnnounceBuf->sequenceID[0] = ((g_AnnounceSequenceID >> 8) & 0xff);
    AnnounceBuf->sequenceID[1] = ((g_AnnounceSequenceID >> 0) & 0xff);
	g_AnnounceSequenceID++;

	AnnounceBuf->currentUtcOffset[1] 	= 37;
	AnnounceBuf->grandmasterPriority1 	= 128;
	AnnounceBuf->grandmasterClockclass 	= 248;
	AnnounceBuf->grandmasterClockAccuracy = 0x23;
	AnnounceBuf->grandmasterClockVariance[0] = 0x43;
	AnnounceBuf->grandmasterClockVariance[1] = 0x6a;
	AnnounceBuf->grandmasterPriority2 = 128;

	for(i = 0; i<8; i++){
		AnnounceBuf->grandmasterIdentity[i] = clockIdInit[i];
    }
	AnnounceBuf->stepsRemoved[0] = 0;
	AnnounceBuf->stepsRemoved[1] = 0;
	AnnounceBuf->timeSource = 0xa0;  //internal osciliator
	AnnounceBuf->tlvType[1] = 0x8;
	AnnounceBuf->lengthFiled[1] = 0x8;
	for(i = 0; i<8; i++){
		AnnounceBuf->pathSequence[0][i] = clockIdInit[i];
    }

    for(i = 0;i<64;i++){
        Write32(TxPtpAddr + i*4, AnnounceBuf->buf[i]);
    }
	Write32(TX_PTP_BUFFER_CTRL, 0x1 << PTP_Number);
}


 int main()
 {
	// init_platform();

	 print("Hello World\n\r");
	 UINT32 PacketCnt = 0;
	 UINT32 cnt = 0;
	 UINT32 PtpNum = 0;
	 UINT32 RxPktNum = 0;
	 UINT32 Status = 0;
	 UINT32 PTP_Pointer = 0;
	 UINT32 PTP_Offset = 0;
	 UINT32 TxEn = 1;
	 UINT32 RxLen = 0;
	 UINT32 rtc,rtcSav;

	cleanup_platform();
	PackPayloadArpReqest(0x200,0x1234);
	PackPayloadArpReqest(0x00,0x8357);
	// Write32(TSN_BASEADDR + 0x444, 0x901); //preemption control
	// Write32(TSN_BASEADDR + 0x440, 0x1);    //preemption en
	Write32(TSN_BASEADDR + RTC_IVCR_OFFSET, INCRVALUE);    //incr value ctrl reg
	CfgMacAddr(MacAddrInit);
	cfgLocalTime();
//	while(1){
//		//CfgMacAddr(MacAddrInit);
//		DmaRecv(2048,DMA_BASEADDR_1,BRAM_BASEADDR);
//		Status = Read32(DMA_BASEADDR_1+S2MM_DMASR_OFFSET);
//		Status = Status &0x3;
//		if(Status !=0){
//			RxLen = GetDmaTransRecvLen(DMA_BASEADDR_1);
//		}
//
//
//		DmaRecv(2048,DMA_BASEADDR_0,BRAM_BASEADDR+2048);
//		Status = Read32(DMA_BASEADDR_0+S2MM_DMASR_OFFSET);
//		Status = Status &0x3;
//		if(Status !=0){
//			RxLen = GetDmaTransRecvLen(DMA_BASEADDR_0);
//		}
//	}

	while(1){
		//PDELAY_RESP_BUFNUM, &PdelayRespBuf

		rtc =Read32(TSN_BASEADDR + 0x1282c);
		rtc =Read32(TSN_BASEADDR + 0x12830);

		if((rtc != rtcSav) && (g_TxBusy == 0))
		{
			rtcSav = rtc;
			SyncMsg(SYNC_BUFNUM , &SyncBuf);
		}
		Status = Read32(IRQ_REG_BASEADDR + 0x4);   //ptp tx interrupt status
		if(Status !=0)
		{
			PtpNum = Read32(TSN_BASEADDR + TX_PTP_CTRL_OFFSET); //rx ptp packet buffer control register
			PTP_Pointer = (PtpNum >>16) <<8; //mod 0x100
			ProcessTXPTP(PTP_Pointer);
			g_TxBusy = 0;

		}

		Status = Read32(IRQ_REG_BASEADDR + 0x0);   //ptp rx interrupt status
		if(Status !=0)
		{
			PtpNum = Read32(TSN_BASEADDR + RX_PTP_CTRL_OFFSET); //rx ptp packet buffer control register
			PTP_Pointer = (PtpNum >>8) <<8; //mod 0x100
			ProcessRXPTP(PTP_Pointer);
			g_TxBusy = 1;
			// PdelayReqMsg Pdelay_Req(0x0);
		}
	}

	while(1){
		Status = Read32(IRQ_REG_BASEADDR + 0x0);   //ptp rx interrupt status
		if(Status !=0)
		{
			PtpNum = Read32(TSN_BASEADDR + RX_PTP_CTRL_OFFSET); //rx ptp packet buffer control register
			PTP_Pointer = (PtpNum >>8) <<8; //mod 0x100
			ProcessRXPTP(PTP_Pointer);
			g_TxBusy = 1;
			// PdelayReqMsg Pdelay_Req(0x0);
		}


		Status = Read32(IRQ_REG_BASEADDR + 0x4);   //ptp tx interrupt status
		if(Status !=0)
		{
			PtpNum = Read32(TSN_BASEADDR + TX_PTP_CTRL_OFFSET); //rx ptp packet buffer control register
			PTP_Pointer = (PtpNum >>16) <<8; //mod 0x100
			ProcessTXPTP(PTP_Pointer);
			g_TxBusy = 0;
		//cnt = DmaTrans(64,DMA_BASEADDR_0);
		PacketCnt +=cnt;
		//DmaTrans(64,DMA_BASEADDR_1);
		}
		if(g_TxBusy == 0){
			//DmaSend(64,DMA_BASEADDR_1,BRAM_BASEADDR);
		}
	}
	while(1);
	print("Successfully ran Hello World application");

	return 0;
 }



 void PackPayloadArpReqest(UINT32 Offset,UINT32 Addr)
  {

 	 UINT32 i = 0;
 	 UINT32 cnt = 0;
 	 i = Offset;
  //	Write32(BRAM_BASEADDR +(i++), 0x57ffffff); //
  //	Write32(BRAM_BASEADDR +(i++), 0x06088357); //  type 0806 req   0835 reply
  //	Write32(BRAM_BASEADDR +(i++), 0x05040302); //
  //	Write32(BRAM_BASEADDR +(i++), 0x04800081); //
  //	Write32(BRAM_BASEADDR +(i++), 0x28292a2a); //
  //	Write32(BRAM_BASEADDR +(i++), 0x83578357); //
  //	for(cnt=0;cnt<4096;cnt++)
  //	{
  //		Write32(BRAM_BASEADDR +(i+cnt), cnt); //
  //	}


 	 Write8(BRAM_BASEADDR +(i++), 0xff); //DST MAC
 	 Write8(BRAM_BASEADDR +(i++), 0xff); //
 	 Write8(BRAM_BASEADDR +(i++), 0xff); //
 	 Write8(BRAM_BASEADDR +(i++), 0xff); //
 	 Write8(BRAM_BASEADDR +(i++), 0xff); //
 	 Write8(BRAM_BASEADDR +(i++), 0xff); //

 	 Write8(BRAM_BASEADDR +(i++), 0x00); //SRC MAC
 	 Write8(BRAM_BASEADDR +(i++), 0x0C); //
 	 Write8(BRAM_BASEADDR +(i++), 0x29); //
 	 Write8(BRAM_BASEADDR +(i++), 0x70); //
 	 Write8(BRAM_BASEADDR +(i++), ((Addr>>8)& 0xff)); //
 	 Write8(BRAM_BASEADDR +(i++), (Addr& 0xff)); //


 	 Write8(BRAM_BASEADDR +(i++), 0x08); //
 	 Write8(BRAM_BASEADDR +(i++), 0x06); //
 	 Write8(BRAM_BASEADDR +(i++), 0x00); //
 	 Write8(BRAM_BASEADDR +(i++), 0x01); //
 	 Write8(BRAM_BASEADDR +(i++), 0x08); //
 	 Write8(BRAM_BASEADDR +(i++), 0x00); //
 	 Write8(BRAM_BASEADDR +(i++), 0x06); //
 	 Write8(BRAM_BASEADDR +(i++), 0x04); //
 	 Write8(BRAM_BASEADDR +(i++), 0x00); //
 	 Write8(BRAM_BASEADDR +(i++), 0x01); //

 	 Write8(BRAM_BASEADDR +(i++), 0x00); //SRC MAC
 	 Write8(BRAM_BASEADDR +(i++), 0x0C); //
 	 Write8(BRAM_BASEADDR +(i++), 0x29); //
 	 Write8(BRAM_BASEADDR +(i++), 0x70); //
 	 Write8(BRAM_BASEADDR +(i++), 0x2E); //
 	 Write8(BRAM_BASEADDR +(i++), 0x59); //

 	 Write8(BRAM_BASEADDR +(i++), 0xc0); //sender ip  192
 	 Write8(BRAM_BASEADDR +(i++), 0xa8); // 168
 	 Write8(BRAM_BASEADDR +(i++), 0x00); // 0
 	 Write8(BRAM_BASEADDR +(i++), 0xf9); // 249


 	 Write8(BRAM_BASEADDR +(i++), 0x00); //target mac
 	 Write8(BRAM_BASEADDR +(i++), 0x00); //
 	 Write8(BRAM_BASEADDR +(i++), 0x00); //
 	 Write8(BRAM_BASEADDR +(i++), 0x00); //
 	 Write8(BRAM_BASEADDR +(i++), 0x00); //
 	 Write8(BRAM_BASEADDR +(i++), 0x00); //


 	 Write8(BRAM_BASEADDR +(i++), 0xc0); //target ip 192
 	 Write8(BRAM_BASEADDR +(i++), 0xa8); //168
 	 Write8(BRAM_BASEADDR +(i++), 0x00); //0
 	 Write8(BRAM_BASEADDR +(i++), 0xb4); //180


 	 for(cnt=0;cnt<128;cnt++)
 	 {
 		 Write8(BRAM_BASEADDR +(i+cnt),0); //
 	 }
  }



