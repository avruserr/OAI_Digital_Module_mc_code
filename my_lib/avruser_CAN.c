#include "avruser_CAN.h"

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
	CAN_to_init->BTR |= (((initStruct->BRP) - 1) << CAN_BTR_BRP_Pos);
	CAN_to_init->BTR |= (((initStruct->TS1) - 1) << CAN_BTR_TS1_Pos);
	CAN_to_init->BTR |= (((initStruct->TS2) - 1) << CAN_BTR_TS2_Pos);
	CAN_to_init->BTR |= (((initStruct->SJW) - 1) << CAN_BTR_SJW_Pos);
	
	temporary = CAN_to_init->MCR;
	temporary &= ~CAN_MCR_INRQ_Msk;
	if (initStruct->SLEEP) temporary |= CAN_MCR_SLEEP_Msk;
	CAN_to_init->MCR = temporary; // requesting CAN to switch to normal/sleep mode
	
	for(int32_t att = NUMBER_OF_ATTEMPTS; ; att--)
	{
		if (~(CAN_to_init->MSR | ~CAN_MSR_INAK_Msk) && ((initStruct->SLEEP != 0) == ((CAN_to_init->MSR & CAN_MSR_SLAK_Msk) != 0))) //waiting for CAN to switch to initialization mode
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
	if (CAN_to_read->MCR & CAN_MCR_DBF_Msk) targetStruct->DBF = 1;
	else targetStruct->DBF = 0;
	
	if (CAN_to_read->MCR & CAN_MCR_ABOM_Msk) targetStruct->ABOM = 1;
	else targetStruct->ABOM = 0;
	
	if (CAN_to_read->MCR & CAN_MCR_AWUM_Msk) targetStruct->AWUM = 1;
	else targetStruct->AWUM = 0;
	
	if (CAN_to_read->MCR & CAN_MCR_NART_Msk) targetStruct->NART = 1;
	else targetStruct->NART = 0;
	
	if (CAN_to_read->MCR & CAN_MCR_RFLM_Msk) targetStruct->RFLM = 1;
	else targetStruct->RFLM = 0;
	
	if (CAN_to_read->MCR & CAN_MCR_TXFP_Msk) targetStruct->TXFP = 1;
	else targetStruct->TXFP = 0;
	
	if (CAN_to_read->BTR & CAN_BTR_SILM_Msk) targetStruct->SILM = 1;
	else targetStruct->SILM = 0;
	
	if (CAN_to_read->BTR & CAN_BTR_LBKM_Msk) targetStruct->LBKM = 1;
	else targetStruct->LBKM = 0;
	
	targetStruct->BRP = ((CAN_to_read->BTR) & CAN_BTR_BRP_Msk) >> CAN_BTR_BRP_Pos;
	targetStruct->TS1 = ((CAN_to_read->BTR) & CAN_BTR_TS1_Msk) >> CAN_BTR_TS1_Pos;
	targetStruct->TS2 = ((CAN_to_read->BTR) & CAN_BTR_TS2_Msk) >> CAN_BTR_TS2_Pos;
	targetStruct->SJW = ((CAN_to_read->BTR) & CAN_BTR_SJW_Msk) >> CAN_BTR_SJW_Pos;
}

void avruser_CAN_FilterInit(type_can_filter_settings_struct* filterInitStruct)
{
	if ((filterInitStruct->filterNumber) != 28)
	{
		if (filterInitStruct->filterNumber > 27) filterInitStruct->filterNumber = 27;
		
		CAN1->FA1R &= ~(1U << (filterInitStruct->filterNumber));
		(CAN1->sFilterRegister[(filterInitStruct->filterNumber)]).FR1 = filterInitStruct->CAN_FR[4*(filterInitStruct->filterNumber) + 0] + (filterInitStruct->CAN_FR[4*(filterInitStruct->filterNumber) + 1] << 16);
		(CAN1->sFilterRegister[(filterInitStruct->filterNumber)]).FR2 = filterInitStruct->CAN_FR[4*(filterInitStruct->filterNumber) + 2] + (filterInitStruct->CAN_FR[4*(filterInitStruct->filterNumber) + 3] << 16);
		CAN1->FA1R |= (1U << (filterInitStruct->filterNumber));
	}
	else
	{
		CAN1->FMR |= CAN_FMR_FINIT_Msk;
		
		if ((filterInitStruct->CAN2SB) > 28) filterInitStruct->CAN2SB = 28;
		
		CAN1->FMR &= ~CAN_FMR_CAN2SB_Msk;
		CAN1->FMR |= ((filterInitStruct->CAN2SB) << CAN_FMR_CAN2SB_Pos);
		
		CAN1->FM1R = (filterInitStruct->CAN_FM1R_L + ((filterInitStruct->CAN_FM1R_H) << 16));
		CAN1->FS1R = (filterInitStruct->CAN_FS1R_L + ((filterInitStruct->CAN_FS1R_H) << 16));
		CAN1->FFA1R = (filterInitStruct->CAN_FFA1R_L + ((filterInitStruct->CAN_FFA1R_H) << 16));
		CAN1->FA1R = (filterInitStruct->CAN_FA1R_L + ((filterInitStruct->CAN_FA1R_H) << 16));
		
		for (uint8_t filterBankIndex = 0; filterBankIndex < 28; filterBankIndex++)
		{
			(CAN1->sFilterRegister[filterBankIndex]).FR1 = filterInitStruct->CAN_FR[4*filterBankIndex + 0] + (filterInitStruct->CAN_FR[4*filterBankIndex + 1] << 16);
			(CAN1->sFilterRegister[filterBankIndex]).FR2 = filterInitStruct->CAN_FR[4*filterBankIndex + 2] + (filterInitStruct->CAN_FR[4*filterBankIndex + 3] << 16);
		}
	}
	
	CAN1->FMR &= ~CAN_FMR_FINIT_Msk;
	
	filterInitStruct->CAN2SB = (((CAN1->FMR) & CAN_FMR_CAN2SB_Msk) >> CAN_FMR_CAN2SB_Pos);
	filterInitStruct->CAN_FM1R_L = (uint16_t)(CAN1->FM1R & 0xFFFF);
	filterInitStruct->CAN_FM1R_H = (uint16_t)((CAN1->FM1R >> 16) & 0xFFFF);
	filterInitStruct->CAN_FS1R_L = (uint16_t)(CAN1->FS1R & 0xFFFF);
	filterInitStruct->CAN_FS1R_H = (uint16_t)((CAN1->FS1R >> 16) & 0xFFFF);
	filterInitStruct->CAN_FFA1R_L = (uint16_t)(CAN1->FFA1R & 0xFFFF);
	filterInitStruct->CAN_FFA1R_H = (uint16_t)((CAN1->FFA1R >> 16) & 0xFFFF);
	filterInitStruct->CAN_FA1R_L = (uint16_t)(CAN1->FA1R & 0xFFFF);
	filterInitStruct->CAN_FA1R_H = (uint16_t)((CAN1->FA1R >> 16) & 0xFFFF);
	
	for (uint8_t filterBankIndex = 0; filterBankIndex < 28; filterBankIndex++)
	{
		filterInitStruct->CAN_FR[4*filterBankIndex + 0] = (uint16_t)(((CAN1->sFilterRegister[filterBankIndex]).FR1) & 0xFFFF);
		filterInitStruct->CAN_FR[4*filterBankIndex + 1] = (uint16_t)((((CAN1->sFilterRegister[filterBankIndex]).FR1) >> 16) & 0xFFFF);
		filterInitStruct->CAN_FR[4*filterBankIndex + 2] = (uint16_t)(((CAN1->sFilterRegister[filterBankIndex]).FR2) & 0xFFFF);
		filterInitStruct->CAN_FR[4*filterBankIndex + 3] = (uint16_t)((((CAN1->sFilterRegister[filterBankIndex]).FR2) >> 16) & 0xFFFF);
	}
}
