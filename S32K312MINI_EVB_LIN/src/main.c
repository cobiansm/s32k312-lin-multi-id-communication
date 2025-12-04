/*==================================================================================================
* Project : RTD AUTOSAR 4.7
* Platform : CORTEXM
* Peripheral : S32K3XX
* Dependencies : none
*
* Autosar Version : 4.7.0
* Autosar Revision : ASR_REL_4_7_REV_0000
* Autosar Conf.Variant :
* SW Version : 5.0.0
* Build Version : S32K3_RTD_5_0_0_D2408_ASR_REL_4_7_REV_0000_20241002
*
* Copyright 2020 - 2024 NXP
*
* NXP Confidential and Proprietary. This software is owned or controlled by NXP and may only be
*   used strictly in accordance with the applicable license terms.  By expressly
*   accepting such terms or by downloading, installing, activating and/or otherwise
*   using the software, you are agreeing that you have read, and that you agree to
*   comply with and are bound by, such license terms.  If you do not agree to be
*   bound by the applicable license terms, then you may not retain, install,
*   activate or otherwise use the software.
==================================================================================================*/

/**
*   @file main.c
*
*   @addtogroup main_module main module documentation
*   @{
*/
#include "Mcal.h"

/* User includes */
#include "OsIf.h"
#include "Flexio_Lin_Ip.h"
#include "Lpuart_Lin_Ip.h"
#include "Flexio_Mcl_Ip.h"
#include "Clock_Ip.h"
#include "IntCtrl_Ip.h"
#include "Siul2_Port_Ip.h"

/* Global Macros */
#define BUFFER_SIZE             (4U)
#define T_LIN_TIME_OUT          (400000U)
#define ID                      0xF
#define DATA_NUMBER             0x01
#define PARITY                  0x3
#define FLEXIO_PID              (0x73)
#define LPUART_PID              (0x1A)
#define MASTER_ROLE             (0U)
#define SLAVE_ROLE              (1U)

#define LIN_ID_1   12
#define LIN_ID_2   19
#define LIN_ID_3   32

/* Global Variables */
uint8 FlexioTxBuff1[BUFFER_SIZE] = { 0x01, 0x02, 0x03, 0x04};
uint8 FlexioTxBuff2[BUFFER_SIZE] = { 0x05, 0x06, 0x07, 0x08};
uint8 FlexioTxBuff3[BUFFER_SIZE] = {0x09, 0x10, 0x11, 0x12};
uint8 LpuartTxBuff[BUFFER_SIZE] = {0xDD, 0xEE, 0xFF, 0x55U};

Lpuart_Lin_Ip_PduType LinLpuartPdu[] =
{
    {
		.Pid = (uint8)LPUART_PID,
		.Cs = LPUART_LIN_IP_ENHANCED_CS,
		.SduPtr = LpuartTxBuff,
		.Drc = LPUART_LIN_IP_FRAMERESPONSE_TX,
		.Dl = (uint8)BUFFER_SIZE
    },
    {
		.Pid = (uint8)LPUART_PID,
		.Cs = LPUART_LIN_IP_ENHANCED_CS,
		.SduPtr = NULL_PTR,
		.Drc = LPUART_LIN_IP_FRAMERESPONSE_RX,
		.Dl = (uint8)(BUFFER_SIZE)
    }
};

Flexio_Lin_Ip_PduType LinFlexioPdu[] =
{
    {
		.Pid = (uint8)FLEXIO_PID,
		.Cs = FLEXIO_LIN_IP_ENHANCED_CS,
		.SduPtr = FlexioTxBuff1,
		.Drc = FLEXIO_LIN_IP_FRAMERESPONSE_TX,
		.Dl = (uint8)BUFFER_SIZE
    },
    {
		.Pid = (uint8)FLEXIO_PID,
		.Cs = FLEXIO_LIN_IP_ENHANCED_CS,
		.SduPtr = FlexioTxBuff2,
		.Drc = FLEXIO_LIN_IP_FRAMERESPONSE_TX,
		.Dl = (uint8)BUFFER_SIZE
    },
    {
		.Pid = (uint8)FLEXIO_PID,
		.Cs = FLEXIO_LIN_IP_ENHANCED_CS,
		.SduPtr = FlexioTxBuff3,
		.Drc = FLEXIO_LIN_IP_FRAMERESPONSE_TX,
		.Dl = (uint8)BUFFER_SIZE
    }
};


static uint8_t Lin_CalcPid(uint8_t id);
boolean CheckReceiveBuffer(uint8 *OriginalBuffer, uint8 * ReceiveBuffer);
void LpuartSlaveCallback (uint8 Instance, const  Lpuart_Lin_Ip_StateStructType *LpuartStateStruct);

/*!
  \brief The main function for the project.
  \details The startup initialization sequence is the following:
* - startup asm routine
* - main()
*/
int main(void)
{
	uint8 CheckData = 0;
	const uint8 *DummyBuffer;
	uint8 *RecvBuffer[BUFFER_SIZE/2];
	uint32 TimeoutValue = 0u;
	volatile Flexio_Lin_Ip_TransferStatusType FlexioMasterStatus = FLEXIO_LIN_IP_STATUS_OPERATIONAL;
	volatile Lpuart_Lin_Ip_TransferStatusType LpuartSlaveStatus = LPUART_LIN_IP_STATUS_OPERATIONAL;

	/* Init clock  */
	Clock_Ip_Init(&Clock_Ip_aClockConfig[0]);

	IP_SIUL2->MSCR[31] |= SIUL2_MSCR_OBE_MASK;
	IP_SIUL2->GPDO31 = SIUL2_GPDO0_PDO_n_MASK;

	uint8 slave1 = Lin_CalcPid(LIN_ID_1);
	uint8 slave2 = Lin_CalcPid(LIN_ID_2);
	uint8 slave3 = Lin_CalcPid(LIN_ID_3);

	LinFlexioPdu[0].Pid = slave1;
	LinFlexioPdu[1].Pid = slave2;
	LinFlexioPdu[2].Pid = slave3;

	LinLpuartPdu[SLAVE_ROLE].Pid = slave3;

	/* Initialize all pins */
	Siul2_Port_Ip_Init(NUM_OF_CONFIGURED_PINS_PortContainer_0_BOARD_InitPeripherals,
			g_pin_mux_InitConfigArr_PortContainer_0_BOARD_InitPeripherals);

	/* Initialize IRQs */
	IntCtrl_Ip_Init(&IntCtrlConfig_0);

	/*Intialize Lpuart_Lin */
	Lpuart_Lin_Ip_Init(Lpuart_Lin_Ip_Sa_pxHwConfigPB_0.Instance, &Lpuart_Lin_Ip_Sa_pxHwConfigPB_0);

	Flexio_Mcl_Ip_InitDevice(0u);
	Flexio_Lin_Ip_Init(Flexio_Lin_Ip_Sa_pxHwConfigPB_0.Instance, &Flexio_Lin_Ip_Sa_pxHwConfigPB_0);

	/* Start of the sending frame from Flexio Master*/
	for(uint8 i = 0; i < 3; i++)
	{
		Flexio_Lin_Ip_SendFrame(Flexio_Lin_Ip_Sa_pxHwConfigPB_0.Instance, &LinFlexioPdu[i]);

	    do {
	        FlexioMasterStatus = Flexio_Lin_Ip_GetStatus(Flexio_Lin_Ip_Sa_pxHwConfigPB_0.Instance, (const uint8 **)&DummyBuffer);
	    }
	    while ((FLEXIO_LIN_IP_STATUS_TX_OK != FlexioMasterStatus) && (TimeoutValue-- > 1));
	}

	/* Wait for the transmission done */
	 do
	 {
		 LpuartSlaveStatus = Lpuart_Lin_Ip_GetStatus(Lpuart_Lin_Ip_Sa_pxHwConfigPB_0.Instance, (uint8 **)&RecvBuffer[0]);
	 }
	 while ((LPUART_LIN_IP_STATUS_RX_OK != LpuartSlaveStatus) && (TimeoutValue-- > 1));

	 CheckData = CheckReceiveBuffer(LinFlexioPdu[2].SduPtr, RecvBuffer[0]);

	 if(CheckData)
	 {
		 IP_SIUL2->GPDO31 = ~SIUL2_GPDO0_PDO_n_MASK;
	 }

	 while(1);
	 return(0);
}

/* Master headers */
static uint8_t Lin_CalcPid(uint8_t id)
{
    id &= 0x3FU;

    /* ID */
    uint8_t id0 = (id >> 0) & 0x1U;
    uint8_t id1 = (id >> 1) & 0x1U;
    uint8_t id2 = (id >> 2) & 0x1U;
    uint8_t id3 = (id >> 3) & 0x1U;
    uint8_t id4 = (id >> 4) & 0x1U;
    uint8_t id5 = (id >> 5) & 0x1U;

    /* Parity) */
    uint8_t p0 = (uint8_t)(id0 ^ id1 ^ id2 ^ id4);
    uint8_t p1 = (uint8_t)(~(id1 ^ id3 ^ id4 ^ id5) & 0x1U);

    /* PID */
    uint8_t pid = (uint8_t)(id | (p0 << 6) | (p1 << 7));

    return pid;
}

void LpuartSlaveCallback (uint8 Instance, const  Lpuart_Lin_Ip_StateStructType *LpuartStateStruct)
{
    switch (LpuartStateStruct->CurrentEventId)
    {
		/* Header transmission ok */
		case LPUART_LIN_IP_RECV_HEADER_OK:
			if (LpuartStateStruct->CurrentPid == LinLpuartPdu[SLAVE_ROLE].Pid)
			{
				/* Master start send header */
				(void)Lpuart_Lin_Ip_SendFrame(Instance, (const Lpuart_Lin_Ip_PduType *)&LinLpuartPdu[SLAVE_ROLE]);
			}
			break;

		default:
			/* do nothing */
			break;
	}
}

boolean CheckReceiveBuffer(uint8 *OriginalBuffer, uint8 * ReceiveBuffer)
{
    uint8 Index;
    volatile boolean RetVal = TRUE;
    for (Index = 0; Index < BUFFER_SIZE; Index++)
    {
        if (OriginalBuffer[Index] != *(ReceiveBuffer + Index))
        {
            RetVal = FALSE;
        }
    }
    return RetVal;
}

/** @} */
