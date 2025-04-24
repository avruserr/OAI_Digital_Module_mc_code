#include "avruser_CAN.h"
#include "stdlib.h"

canInitResult avruser_CAN_Init(CAN_TypeDef* CAN_to_init, type_can_settings_struct* initStruct)
{
	uint32_t temporary = CAN_to_init->MCR;
	temporary |= CAN_MCR_INRQ_Msk;
	temporary &= ~CAN_MCR_SLEEP_Msk;
	CAN_to_init->MCR = temporary; // requesting CAN to switch to initialization mode
	
	for(int32_t att = NUMBER_OF_ATTEMPTS; ; att--)
	{
		if ((CAN_to_init->MSR & CAN_MSR_INAK_Msk) && ~(CAN_to_init->MSR | ~CAN_MSR_SLAK_Msk)) //waiting for CAN to switch to initialization mode
		{
			break;
		}
		if (att == 0)
		{
			return canInitError;
		}
	}
	
	if (initStruct->DBF) CAN_to_init->MCR |= CAN_MCR_DBF_Msk;
	else CAN_to_init->MCR &= ~CAN_MCR_DBF_Msk;
	
	CAN_to_init->MCR &= ~CAN_MCR_TTCM_Msk;
	
	if (initStruct->ABOM) CAN_to_init->MCR |= CAN_MCR_ABOM_Msk;
	else CAN_to_init->MCR &= ~CAN_MCR_ABOM_Msk;
	
	if (initStruct->AWUM) CAN_to_init->MCR |= CAN_MCR_AWUM_Msk;
	else CAN_to_init->MCR &= ~CAN_MCR_AWUM_Msk;
	
	if (initStruct->NART) CAN_to_init->MCR |= CAN_MCR_NART_Msk;
	else CAN_to_init->MCR &= ~CAN_MCR_NART_Msk;
	
	if (initStruct->RFLM) CAN_to_init->MCR |= CAN_MCR_RFLM_Msk;
	else CAN_to_init->MCR &= ~CAN_MCR_RFLM_Msk;
	
	if (initStruct->TXFP) CAN_to_init->MCR |= CAN_MCR_TXFP_Msk;
	else CAN_to_init->MCR &= ~CAN_MCR_TXFP_Msk;
	
	if (initStruct->SILM) CAN_to_init->BTR |= CAN_BTR_SILM_Msk;
	else CAN_to_init->BTR &= ~CAN_BTR_SILM_Msk;
	
	if (initStruct->LBKM) CAN_to_init->BTR |= CAN_BTR_LBKM_Msk;
	else CAN_to_init->BTR &= ~CAN_BTR_LBKM_Msk;
	
	if (initStruct->BRP > 512) initStruct->BRP = 512;
	else if (initStruct->BRP < 1) initStruct->BRP = 1;
	
	if (initStruct->TS1 > 16) initStruct->TS1 = 16;
	else if (initStruct->TS1 < 1) initStruct->TS1 = 1;
	
	if (initStruct->TS2 > 8) initStruct->TS2 = 8;
	else if (initStruct->TS2 < 1) initStruct->TS2 = 1;
	
	if (initStruct->SJW > 4) initStruct->SJW = 4;
	else if (initStruct->SJW < 1) initStruct->SJW = 1;
	
	CAN_to_init->BTR &= ~(CAN_BTR_BRP_Msk | CAN_BTR_TS1_Msk | CAN_BTR_TS2_Msk | CAN_BTR_SJW_Msk);
	CAN_to_init->BTR |= (((uint32_t)(initStruct->BRP) - 1) << CAN_BTR_BRP_Pos);
	CAN_to_init->BTR |= (((uint32_t)(initStruct->TS1) - 1) << CAN_BTR_TS1_Pos);
	CAN_to_init->BTR |= (((uint32_t)(initStruct->TS2) - 1) << CAN_BTR_TS2_Pos);
	CAN_to_init->BTR |= (((uint32_t)(initStruct->SJW) - 1) << CAN_BTR_SJW_Pos);
	
	CAN_to_init->IER |= (CAN_IER_TMEIE_Msk | CAN_IER_FMPIE0_Msk | CAN_IER_FMPIE1_Msk | CAN_IER_ERRIE_Msk | CAN_IER_LECIE_Msk | CAN_IER_WKUIE_Msk);
	CAN_to_init->IER &= ~(CAN_IER_FFIE0_Msk | CAN_IER_FOVIE0_Msk | CAN_IER_FFIE1_Msk | CAN_IER_FOVIE1_Msk | CAN_IER_EWGIE_Msk | CAN_IER_EPVIE_Msk | CAN_IER_BOFIE_Msk | CAN_IER_SLKIE_Msk);
	
	temporary = CAN_to_init->MCR;
	temporary &= ~CAN_MCR_INRQ_Msk;
	if (initStruct->SLEEP) temporary |= CAN_MCR_SLEEP_Msk;
	CAN_to_init->MCR = temporary; // requesting CAN to switch to normal/sleep mode
	
	for(int32_t att = NUMBER_OF_ATTEMPTS; ; att--)
	{
		if (~(CAN_to_init->MSR | ~CAN_MSR_INAK_Msk) && ((initStruct->SLEEP != 0) == ((CAN_to_init->MSR & CAN_MSR_SLAK_Msk) != 0))) //waiting for CAN to switch to normal/sleep mode
		{
			break;
		}
		if (att == 0)
		{
			return canInitError;
		}
	}
	
	return canInitSuccess;
}

void avruser_CAN_Pull_Settings(CAN_TypeDef* CAN_to_read, type_can_settings_struct* targetStruct)
{
	targetStruct->DBF = (uint16_t)((CAN_to_read->MCR & CAN_MCR_DBF_Msk) >> CAN_MCR_DBF_Pos);
	targetStruct->ABOM = (uint16_t)((CAN_to_read->MCR & CAN_MCR_ABOM_Msk) >> CAN_MCR_ABOM_Pos);
	targetStruct->AWUM = (uint16_t)((CAN_to_read->MCR & CAN_MCR_AWUM_Msk) >> CAN_MCR_AWUM_Pos);
	targetStruct->NART = (uint16_t)((CAN_to_read->MCR & CAN_MCR_NART_Msk) >> CAN_MCR_NART_Pos);
	targetStruct->RFLM = (uint16_t)((CAN_to_read->MCR & CAN_MCR_RFLM_Msk) >> CAN_MCR_RFLM_Pos);
	targetStruct->TXFP = (uint16_t)((CAN_to_read->MCR & CAN_MCR_TXFP_Msk) >> CAN_MCR_TXFP_Pos);
	targetStruct->SLEEP = (uint16_t)((CAN_to_read->MSR & CAN_MSR_SLAK_Msk) >> CAN_MSR_SLAK_Msk);
	targetStruct->SILM = (uint16_t)((CAN_to_read->BTR & CAN_BTR_SILM_Msk) >> CAN_BTR_SILM_Pos);
	targetStruct->LBKM = (uint16_t)((CAN_to_read->BTR & CAN_BTR_LBKM_Msk) >> CAN_BTR_LBKM_Pos);
	targetStruct->BRP = (uint16_t)(((CAN_to_read->BTR) & CAN_BTR_BRP_Msk) >> CAN_BTR_BRP_Pos) + 1;
	targetStruct->TS1 = (uint16_t)(((CAN_to_read->BTR) & CAN_BTR_TS1_Msk) >> CAN_BTR_TS1_Pos) + 1;
	targetStruct->TS2 = (uint16_t)(((CAN_to_read->BTR) & CAN_BTR_TS2_Msk) >> CAN_BTR_TS2_Pos) + 1;
	targetStruct->SJW = (uint16_t)(((CAN_to_read->BTR) & CAN_BTR_SJW_Msk) >> CAN_BTR_SJW_Pos) + 1;
}

void avruser_CAN_Filter_Init(type_can_filter_settings_struct* filterInitStruct)
{
	if ((filterInitStruct->filterNumber) <= 27)
	{
		CAN1->FA1R &= ~(1U << (filterInitStruct->filterNumber));
		(CAN1->sFilterRegister[(filterInitStruct->filterNumber)]).FR1 = (uint32_t)(filterInitStruct->CAN_FR[4*(filterInitStruct->filterNumber) + 0]) + ((uint32_t)(filterInitStruct->CAN_FR[4*(filterInitStruct->filterNumber) + 1]) << 16);
		(CAN1->sFilterRegister[(filterInitStruct->filterNumber)]).FR2 = (uint32_t)(filterInitStruct->CAN_FR[4*(filterInitStruct->filterNumber) + 2]) + ((uint32_t)(filterInitStruct->CAN_FR[4*(filterInitStruct->filterNumber) + 3]) << 16);
		CAN1->FA1R |= (1U << (filterInitStruct->filterNumber));
	}
	else
	{
		CAN1->FMR |= CAN_FMR_FINIT_Msk;
		
		if ((filterInitStruct->CAN2SB) > 28) filterInitStruct->CAN2SB = 28;
		
		CAN1->FMR &= ~CAN_FMR_CAN2SB_Msk;
		CAN1->FMR |= ((uint32_t)(filterInitStruct->CAN2SB) << CAN_FMR_CAN2SB_Pos);
		
		CAN1->FM1R = 0x0FFFFFFF & ((uint32_t)(filterInitStruct->CAN_FM1R_L) + ((uint32_t)(filterInitStruct->CAN_FM1R_H) << 16));
		CAN1->FS1R = 0x0FFFFFFF & ((uint32_t)(filterInitStruct->CAN_FS1R_L) + ((uint32_t)(filterInitStruct->CAN_FS1R_H) << 16));
		CAN1->FFA1R = 0x0FFFFFFF & ((uint32_t)(filterInitStruct->CAN_FFA1R_L) + ((uint32_t)(filterInitStruct->CAN_FFA1R_H) << 16));
		CAN1->FA1R = 0x0FFFFFFF & ((uint32_t)(filterInitStruct->CAN_FA1R_L) + ((uint32_t)(filterInitStruct->CAN_FA1R_H) << 16));
		
		for (uint8_t filterBankIndex = 0; filterBankIndex < 28; filterBankIndex++)
		{
			(CAN1->sFilterRegister[filterBankIndex]).FR1 = (uint32_t)(filterInitStruct->CAN_FR[4*filterBankIndex + 0]) + (uint32_t)(filterInitStruct->CAN_FR[4*filterBankIndex + 1] << 16);
			(CAN1->sFilterRegister[filterBankIndex]).FR2 = (uint32_t)(filterInitStruct->CAN_FR[4*filterBankIndex + 2]) + (uint32_t)(filterInitStruct->CAN_FR[4*filterBankIndex + 3] << 16);
		}
	}
	
	CAN1->FMR &= ~CAN_FMR_FINIT_Msk;
}

void avruser_CAN_Pull_Filter_Settings(type_can_filter_settings_struct* targetStruct)
{
	targetStruct->CAN2SB = (((CAN1->FMR) & CAN_FMR_CAN2SB_Msk) >> CAN_FMR_CAN2SB_Pos);
	targetStruct->CAN_FM1R_L = (uint16_t)(CAN1->FM1R & 0xFFFF);
	targetStruct->CAN_FM1R_H = (uint16_t)((CAN1->FM1R >> 16) & 0xFFFF);
	targetStruct->CAN_FS1R_L = (uint16_t)(CAN1->FS1R & 0xFFFF);
	targetStruct->CAN_FS1R_H = (uint16_t)((CAN1->FS1R >> 16) & 0xFFFF);
	targetStruct->CAN_FFA1R_L = (uint16_t)(CAN1->FFA1R & 0xFFFF);
	targetStruct->CAN_FFA1R_H = (uint16_t)((CAN1->FFA1R >> 16) & 0xFFFF);
	targetStruct->CAN_FA1R_L = (uint16_t)(CAN1->FA1R & 0xFFFF);
	targetStruct->CAN_FA1R_H = (uint16_t)((CAN1->FA1R >> 16) & 0xFFFF);
	
	for (uint8_t filterBankIndex = 0; filterBankIndex < 28; filterBankIndex++)
	{
		targetStruct->CAN_FR[4*filterBankIndex + 0] = (uint16_t)(((CAN1->sFilterRegister[filterBankIndex]).FR1) & 0xFFFF);
		targetStruct->CAN_FR[4*filterBankIndex + 1] = (uint16_t)((((CAN1->sFilterRegister[filterBankIndex]).FR1) >> 16) & 0xFFFF);
		targetStruct->CAN_FR[4*filterBankIndex + 2] = (uint16_t)(((CAN1->sFilterRegister[filterBankIndex]).FR2) & 0xFFFF);
		targetStruct->CAN_FR[4*filterBankIndex + 3] = (uint16_t)((((CAN1->sFilterRegister[filterBankIndex]).FR2) >> 16) & 0xFFFF);
	}
}

uint16_t increaseCycledIndex(uint16_t input)
{
	if (input == (SOFTWARE_FIFO_SIZE - 1)) return 0;
	else return (input + 1);
}

void FIFO_Reset(FIFO_Typedef* resetFIFO)
{
	resetFIFO->readIndex = 0;
	resetFIFO->writeIndex = 0;
	resetFIFO->emptyFlag = 1;
	resetFIFO->fullFlag = 0;
	resetFIFO->underrunFlag = 0;
	resetFIFO->overrunFlag = 0;
}

void FIFO_Push(FIFO_Data_Typedef* sourceStruct, FIFO_Typedef* pushFIFO)
{
	if ((pushFIFO->writeIndex == pushFIFO->readIndex) && (pushFIFO->fullFlag)) 
	{
		pushFIFO->overrunFlag = 1;
	}
	else
	{
		pushFIFO->data[(pushFIFO->writeIndex)] = *sourceStruct;
		pushFIFO->writeIndex = increaseCycledIndex(pushFIFO->writeIndex);
		pushFIFO->underrunFlag = 0;
		pushFIFO->emptyFlag = 0;
		if (pushFIFO->writeIndex == pushFIFO->readIndex) pushFIFO->fullFlag = 1;
	}
}

void FIFO_Pop(FIFO_Data_Typedef* targetStruct, FIFO_Typedef* popFIFO)
{
	if ((popFIFO->readIndex == popFIFO->writeIndex) && (popFIFO->emptyFlag)) 
	{
		popFIFO->underrunFlag = 1;
	}
	else
	{
		*targetStruct = popFIFO->data[(popFIFO->readIndex)];
		popFIFO->readIndex = increaseCycledIndex(popFIFO->readIndex);
		popFIFO->overrunFlag = 0;
		popFIFO->fullFlag = 0;
		if (popFIFO->writeIndex == popFIFO->readIndex) popFIFO->emptyFlag = 1;
	}
}

uint16_t FIFO_Get_Length(FIFO_Typedef* lengthFIFO)
{
	if (lengthFIFO->emptyFlag) return 0;
	else if (lengthFIFO->fullFlag) return SOFTWARE_FIFO_SIZE;
	else return lengthFIFO->writeIndex - lengthFIFO->readIndex;
}

void avruser_CAN_Receive_IT_Handler(CAN_TypeDef* CAN_to_handle, uint8_t FIFO_Number, FIFO_Typedef* FIFO_to_push)
{
	uint32_t identifierReg = ((CAN_to_handle->sFIFOMailBox)[FIFO_Number]).RIR;
	uint32_t DLCReg = ((CAN_to_handle->sFIFOMailBox)[FIFO_Number]).RDTR;
	uint32_t dataLowReg = ((CAN_to_handle->sFIFOMailBox)[FIFO_Number]).RDLR;
	uint32_t dataHighReg = ((CAN_to_handle->sFIFOMailBox)[FIFO_Number]).RDHR;
	
	FIFO_Data_Typedef dataToPush;
	
	if (FIFO_Number == 0)
	{
		CAN_to_handle->RF0R |= CAN_RF0R_RFOM0_Msk; //releasing FIFO 0 output mailbox
		dataToPush.IDE = (uint8_t)((identifierReg & CAN_RI0R_IDE_Msk) >> CAN_RI0R_IDE_Pos);
		dataToPush.FMI = (uint8_t)((DLCReg & CAN_RDT0R_FMI_Msk) >> CAN_RDT0R_FMI_Pos);
		dataToPush.RTR = (uint8_t)((identifierReg & CAN_RI0R_RTR_Msk) >> CAN_RI0R_RTR_Pos);
		dataToPush.DLC = (uint8_t)((DLCReg & CAN_RDT0R_DLC_Msk) >> CAN_RDT0R_DLC_Pos);
		
		if (dataToPush.IDE) dataToPush.ID = (identifierReg & (CAN_RI0R_EXID_Msk | CAN_RI0R_STID_Msk)) >> CAN_RI0R_EXID_Pos;
		else dataToPush.ID = (identifierReg & CAN_RI0R_STID_Msk) >> CAN_RI0R_STID_Pos;
	}
	else if (FIFO_Number == 1) 
	{
		CAN_to_handle->RF1R |= CAN_RF1R_RFOM1_Msk; //releasing FIFO 1 output mailbox
		dataToPush.IDE = (uint8_t)((identifierReg & CAN_RI1R_IDE_Msk) >> CAN_RI1R_IDE_Pos);
		dataToPush.FMI = (uint8_t)((DLCReg & CAN_RDT1R_FMI_Msk) >> CAN_RDT1R_FMI_Pos);
		dataToPush.RTR = (uint8_t)((identifierReg & CAN_RI1R_RTR_Msk) >> CAN_RI1R_RTR_Pos);
		dataToPush.DLC = (uint8_t)((DLCReg & CAN_RDT1R_DLC_Msk) >> CAN_RDT1R_DLC_Pos);
		
		if (dataToPush.IDE) dataToPush.ID = (identifierReg & (CAN_RI1R_EXID_Msk | CAN_RI1R_STID_Msk)) >> CAN_RI1R_EXID_Pos;
		else dataToPush.ID = (identifierReg & CAN_RI1R_STID_Msk) >> CAN_RI1R_STID_Pos;
	}
	
	for (uint8_t byteIndex = 0; byteIndex < (dataToPush.DLC); byteIndex++)
	{
		if (byteIndex < 4) (dataToPush.DATA)[byteIndex] = (uint8_t)(0x000000FF & (dataLowReg >> (byteIndex*8)));
		else (dataToPush.DATA)[byteIndex] = (uint8_t)(0x000000FF & (dataHighReg >> ((byteIndex - 4)*8)));
	}
	FIFO_Push(&dataToPush, FIFO_to_push);
}

void avruser_CAN_Get_Frame(type_can_receive_struct* destinationStruct, FIFO_Typedef* FIFO_to_get_from)
{
	if (!(FIFO_to_get_from->emptyFlag))
	{
		FIFO_Data_Typedef frame;
		FIFO_Pop(&frame, FIFO_to_get_from);
		destinationStruct->ID_L = (uint16_t)(frame.ID & 0xFFFF);
		destinationStruct->ID_H = (uint16_t)((frame.ID & 0xFFFF0000) >> 16);
		destinationStruct->IDE = frame.IDE;
		destinationStruct->FMI = frame.FMI;
		destinationStruct->RTR = frame.RTR;
		destinationStruct->DLC = frame.DLC;
		destinationStruct->DATA[0] = (uint16_t)(frame.DATA[0]) | ((uint16_t)(frame.DATA[1]) << 8);
		destinationStruct->DATA[1] = (uint16_t)(frame.DATA[2]) | ((uint16_t)(frame.DATA[3]) << 8);
		destinationStruct->DATA[2] = (uint16_t)(frame.DATA[4]) | ((uint16_t)(frame.DATA[5]) << 8);
		destinationStruct->DATA[3] = (uint16_t)(frame.DATA[6]) | ((uint16_t)(frame.DATA[7]) << 8);
	}
}

void avruser_CAN_Update_Status_Struct(CAN_TypeDef* CAN_to_read, type_can_status_struct* destinationStruct, FIFO_Typedef** FIFO_to_read_overrun)
{
	destinationStruct->mode = (uint16_t)((CAN_to_read->MSR & (CAN_MSR_SLAK_Msk | CAN_MSR_INAK_Msk)) >> CAN_MSR_INAK_Pos);
	uint32_t TSReg = CAN_to_read->TSR;
	destinationStruct->TME = (uint16_t)(((TSReg & CAN_TSR_TME0_Msk) >> (CAN_TSR_TME0_Pos - 0)) | ((TSReg & CAN_TSR_TME1_Msk) >> (CAN_TSR_TME1_Pos - 1)) | ((TSReg & CAN_TSR_TME2_Msk) >> (CAN_TSR_TME2_Pos - 2)));
	destinationStruct->TERR = (uint16_t)(((TSReg & CAN_TSR_TERR0_Msk) >> (CAN_TSR_TERR0_Pos - 0)) | ((TSReg & CAN_TSR_TERR1_Msk) >> (CAN_TSR_TERR1_Pos - 1)) | ((TSReg & CAN_TSR_TERR2_Msk) >> (CAN_TSR_TERR2_Pos - 2)));
	destinationStruct->ALST = (uint16_t)(((TSReg & CAN_TSR_ALST0_Msk) >> (CAN_TSR_ALST0_Pos - 0)) | ((TSReg & CAN_TSR_ALST1_Msk) >> (CAN_TSR_ALST1_Pos - 1)) | ((TSReg & CAN_TSR_ALST2_Msk) >> (CAN_TSR_ALST2_Pos - 2)));
	destinationStruct->TXOK = (uint16_t)(((TSReg & CAN_TSR_TXOK0_Msk) >> (CAN_TSR_TXOK0_Pos - 0)) | ((TSReg & CAN_TSR_TXOK1_Msk) >> (CAN_TSR_TXOK1_Pos - 1)) | ((TSReg & CAN_TSR_TXOK2_Msk) >> (CAN_TSR_TXOK2_Pos - 2)));
	destinationStruct->RQCP = (uint16_t)(((TSReg & CAN_TSR_RQCP0_Msk) >> (CAN_TSR_RQCP0_Pos - 0)) | ((TSReg & CAN_TSR_RQCP1_Msk) >> (CAN_TSR_RQCP1_Pos - 1)) | ((TSReg & CAN_TSR_RQCP2_Msk) >> (CAN_TSR_RQCP2_Pos - 2)));
	destinationStruct->softwareFIFO_Overrun = (FIFO_to_read_overrun[0]->overrunFlag) | ((FIFO_to_read_overrun[1]->overrunFlag) << 1);
	destinationStruct->REC = (uint16_t)((CAN_to_read->ESR & CAN_ESR_REC_Msk) >> CAN_ESR_REC_Pos);
	destinationStruct->TEC = (uint16_t)((CAN_to_read->ESR & CAN_ESR_TEC_Msk) >> CAN_ESR_TEC_Pos);
	destinationStruct->BOFF = (uint16_t)((CAN_to_read->ESR & CAN_ESR_BOFF_Msk) >> CAN_ESR_BOFF_Pos);
	destinationStruct->EPVF = (uint16_t)((CAN_to_read->ESR & CAN_ESR_EPVF_Msk) >> CAN_ESR_EPVF_Pos);
}

void avruser_CAN_Request_Transmittion(CAN_TypeDef* CAN_to_request_from, uint8_t mailboxNumber, type_can_transmit_struct* transmissionData)
{
	(CAN_to_request_from->sTxMailBox[mailboxNumber]).TIR = 0;
	(CAN_to_request_from->sTxMailBox[mailboxNumber]).TDTR = 0;
	if (transmissionData->IDE)
	{
		transmissionData->ID_L &= 0xFFFF;
		transmissionData->ID_H &= 0x1FFF;
	}
	else
	{
		transmissionData->ID_L &= 0x07FF;
		transmissionData->ID_H &= 0x0000;
	}
	transmissionData->DLC &= 0x000F;
	switch (mailboxNumber)
	{
		case 0:
			(CAN_to_request_from->sTxMailBox[mailboxNumber]).TIR |= ((uint32_t)(transmissionData->ID_L) | ((uint32_t)(transmissionData->ID_H) << 16)) << ((transmissionData->IDE) ? CAN_TI0R_EXID_Pos : CAN_TI0R_STID_Pos);
			if (transmissionData->IDE) (CAN_to_request_from->sTxMailBox[mailboxNumber]).TIR |= CAN_TI0R_IDE_Msk;
			if (transmissionData->RTR) (CAN_to_request_from->sTxMailBox[mailboxNumber]).TIR |= CAN_TI0R_RTR_Msk;
			(CAN_to_request_from->sTxMailBox[mailboxNumber]).TDTR |= ((uint32_t)(transmissionData->DLC) << CAN_TDT0R_DLC_Pos);
			break;
		
		case 1:
			(CAN_to_request_from->sTxMailBox[mailboxNumber]).TIR |= ((uint32_t)(transmissionData->ID_L) | ((uint32_t)(transmissionData->ID_H) << 16)) << ((transmissionData->IDE) ? CAN_TI1R_EXID_Pos : CAN_TI1R_STID_Pos);
			if (transmissionData->IDE) (CAN_to_request_from->sTxMailBox[mailboxNumber]).TIR |= CAN_TI1R_IDE_Msk;
			if (transmissionData->RTR) (CAN_to_request_from->sTxMailBox[mailboxNumber]).TIR |= CAN_TI1R_RTR_Msk;
			(CAN_to_request_from->sTxMailBox[mailboxNumber]).TDTR |= ((uint32_t)(transmissionData->DLC) << CAN_TDT1R_DLC_Pos);
			break;
		
		case 2:
			(CAN_to_request_from->sTxMailBox[mailboxNumber]).TIR |= ((uint32_t)(transmissionData->ID_L) | ((uint32_t)(transmissionData->ID_H) << 16)) << ((transmissionData->IDE) ? CAN_TI2R_EXID_Pos : CAN_TI2R_STID_Pos);
			if (transmissionData->IDE) (CAN_to_request_from->sTxMailBox[mailboxNumber]).TIR |= CAN_TI2R_IDE_Msk;
			if (transmissionData->RTR) (CAN_to_request_from->sTxMailBox[mailboxNumber]).TIR |= CAN_TI2R_RTR_Msk;
			(CAN_to_request_from->sTxMailBox[mailboxNumber]).TDTR |= ((uint32_t)(transmissionData->DLC) << CAN_TDT2R_DLC_Pos);
			break;
	}
	(CAN_to_request_from->sTxMailBox[mailboxNumber]).TDLR = ((uint32_t)(transmissionData->DATA[0]) | ((uint32_t)(transmissionData->DATA[1]) << 16));
	(CAN_to_request_from->sTxMailBox[mailboxNumber]).TDHR = ((uint32_t)(transmissionData->DATA[2]) | ((uint32_t)(transmissionData->DATA[3]) << 16));
	switch (mailboxNumber)
	{
		case 0:
			(CAN_to_request_from->sTxMailBox[mailboxNumber]).TIR |= CAN_TI0R_TXRQ_Msk;
			break;		
		case 1:
			(CAN_to_request_from->sTxMailBox[mailboxNumber]).TIR |= CAN_TI1R_TXRQ_Msk;
			break;		
		case 2:
			(CAN_to_request_from->sTxMailBox[mailboxNumber]).TIR |= CAN_TI2R_TXRQ_Msk;
			break;
	}
}

void avruser_CAN_request_abort(CAN_TypeDef* CAN_to_request_from, uint8_t mailboxNumber)
{
	switch (mailboxNumber)
	{
		case 0:
			CAN_to_request_from->TSR |= CAN_TSR_ABRQ0;
			break;
		case 1:
			CAN_to_request_from->TSR |= CAN_TSR_ABRQ1;
			break;
		case 2:
			CAN_to_request_from->TSR |= CAN_TSR_ABRQ2;
			break;
	}
}

void avruser_CAN_Transmit_IT_Handler(CAN_TypeDef* CAN_to_handle)
{
	if (CAN_to_handle->TSR & CAN_TSR_RQCP0_Msk) CAN_to_handle->TSR |= CAN_TSR_RQCP0_Msk;
	if (CAN_to_handle->TSR & CAN_TSR_RQCP1_Msk) CAN_to_handle->TSR |= CAN_TSR_RQCP1_Msk;
	if (CAN_to_handle->TSR & CAN_TSR_RQCP2_Msk) CAN_to_handle->TSR |= CAN_TSR_RQCP2_Msk;
}

void avruser_CAN_Status_Change_IT_Handler(CAN_TypeDef* CAN_to_handle, FIFO_Typedef* FIFO_to_push_errors)
{
	if (CAN_to_handle->MSR & CAN_MSR_ERRI_Msk)
	{
		CAN_to_handle->MSR |= CAN_MSR_ERRI_Msk;
		FIFO_Data_Typedef element_to_push;
		element_to_push.LEC = (uint16_t)((CAN_to_handle->ESR & CAN_ESR_LEC_Msk) >> CAN_ESR_LEC_Pos);
		FIFO_Push(&element_to_push, FIFO_to_push_errors);
	}
	if (CAN_to_handle->MSR & CAN_MSR_WKUI_Msk)
	{
		CAN_to_handle->MSR |= CAN_MSR_WKUI_Msk;
	}
}

void avruser_CAN_Get_Error_Message(type_can_error_struct* destinationStruct, FIFO_Typedef* FIFO_to_get_from)
{
	destinationStruct->errorCode = 0;
	FIFO_Data_Typedef error_element; 
	FIFO_Pop(&error_element, FIFO_to_get_from);
	destinationStruct->errorCode = error_element.LEC;
}

void avruser_CAN_Update_FIFO_Length(type_can_receive_struct* targetStruct, FIFO_Typedef* lengthFIFO)
{
	targetStruct->numberOfFrames = FIFO_Get_Length(lengthFIFO);
}

canInitResult avruser_CAN_Recover_Bus_Off(CAN_TypeDef* CAN_to_recover)
{
	uint8_t sleepMode = (CAN_to_recover->MSR & CAN_MSR_SLAK_Msk) ? 1 : 0;
	uint32_t temporary = CAN_to_recover->MCR;
	temporary |= CAN_MCR_INRQ_Msk;
	temporary &= ~CAN_MCR_SLEEP_Msk;
	CAN_to_recover->MCR = temporary; // requesting CAN to switch to initialization mode
	
	for(int32_t att = NUMBER_OF_ATTEMPTS; ; att--)
	{
		if ((CAN_to_recover->MSR & CAN_MSR_INAK_Msk) && ~(CAN_to_recover->MSR | ~CAN_MSR_SLAK_Msk)) //waiting for CAN to switch to initialization mode
		{
			break;
		}
		if (att == 0)
		{
			return canInitError;
		}
	}
	
	temporary = CAN_to_recover->MCR;
	temporary &= ~CAN_MCR_INRQ_Msk;
	if (sleepMode) temporary |= CAN_MCR_SLEEP_Msk;
	CAN_to_recover->MCR = temporary; // requesting CAN to switch to normal/sleep mode
	
	for(int32_t att = NUMBER_OF_ATTEMPTS; ; att--)
	{
		if (~(CAN_to_recover->MSR | ~CAN_MSR_INAK_Msk) && ((sleepMode) == ((CAN_to_recover->MSR & CAN_MSR_SLAK_Msk) != 0))) //waiting for CAN to switch to normal/sleep mode
		{
			break;
		}
		if (att == 0)
		{
			return canInitError;
		}
	}
	
	return canInitSuccess;
}
