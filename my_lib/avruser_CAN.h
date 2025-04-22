#ifndef _AVRUSER_CAN_H
#define _AVRUSER_CAN_H

#include "main.h"

#define NUMBER_OF_ATTEMPTS 100000
#define SOFTWARE_FIFO_MAX_SIZE 1000

typedef enum
{
	canInitSuccess = 0,
	canInitError = 1,
} 
canInitResult;

#pragma pack(push, 2)

typedef struct 
{
	uint16_t scaler; //making this register value not 0 triggers bxCAN settings to update, it automaticaly resets when update has been completed
	uint16_t DBF; //if this field is not 0, bxCAN transmissions and receptions are frozen during debug
	uint16_t ABOM; //if this field is not 0, no software request is needed for bxCAN to leave Bus-Off state
	uint16_t AWUM; //if this field is not 0, bxCAN will automatically leave Bus-Off state when bus is idle
	uint16_t NART; //if this field is not 0, bxCAN will automatically try to retransmit message in case of transmission error
	uint16_t RFLM; //if this field is not 0, receive FIFO will be locked in case of overrun
	uint16_t TXFP; //if this field is not 0, priority of transmit mailboxes will be driven by the request order, otherwise by the identifier
	uint16_t SLEEP; //if this field is not 0, bxCAN will enter the sleep mode
	uint16_t SILM; //if this field is not 0, bxCAN will be in silent mode
	uint16_t LBKM; //if this field is not 0, bxCAN will be in loopback mode
	uint16_t BRP; //baudrate prescaler, equals to BRP + 1 in CAN_BTR register
	uint16_t TS1; //time segment 1 length in time quanta, equals to TS1 + 1 in CAN_BTR register
	uint16_t TS2; //time segment 2 length in time quanta, equals to TS2 + 1 in CAN_BTR register
	uint16_t SJW; //resynchronization jump width in time quanta, equals to SJW + 1 in CAN_BTR register
}
type_can_settings_struct;

typedef struct 
{
	uint16_t scaler; //making this register value not 0 triggers bxCAN filters settings to update, it automaticaly resets when update has been completed
	uint16_t filterNumber; // if this field = 28 all filters will be deactivated during its setup. If number between 0 or 27 is given only filter with accociated number will be deactivated and set up
	uint16_t CAN2SB; //there are 28 filter banks shared between CAN1 and CAN2. Filter banks from 0 to CAN2SB - 1 are assigned to CAN1, banks from CAN2SB to 27 are assigned to CAN2
	uint16_t CAN_FM1R_L; //two least significant bytes of CAN_FM1R (filter mode) register. Refer to Reference Manual 32.9.4 for details on how to use it  
	uint16_t CAN_FM1R_H; //two most significant bytes of CAN_FM1R (filter mode) register. Refer to Reference Manual 32.9.4 for details on how to use it
	uint16_t CAN_FS1R_L; //two least significant bytes of CAN_FS1R (filter scale) register. Refer to Reference Manual 32.9.4 for details on how to use it  
	uint16_t CAN_FS1R_H; //two most significant bytes of CAN_FS1R (filter scale) register. Refer to Reference Manual 32.9.4 for details on how to use it
	uint16_t CAN_FFA1R_L; //two least significant bytes of CAN_FFA1R (fifo assignment) register. Refer to Reference Manual 32.9.4 for details on how to use it  
	uint16_t CAN_FFA1R_H; //two most significant bytes of CAN_FFA1R (fifo assignment) register. Refer to Reference Manual 32.9.4 for details on how to use it
	uint16_t CAN_FA1R_L; //two least significant bytes of CAN_FA1R (filter activation) register. Refer to Reference Manual 32.9.4 for details on how to use it  
	uint16_t CAN_FA1R_H; //two most significant bytes of CAN_FA1R (filter activation) register. Refer to Reference Manual 32.9.4 for details on how to use it
	uint16_t CAN_FR[112]; /*filter bank registers. Matching to bxCAN registers:
	CAN_FR[0] = CAN_F0R0 two least significant bytes
	CAN_FR[1] = CAN_F0R0 two most significant bytes
	CAN_FR[2] = CAN_F0R1 two least significant bytes
	CAN_FR[3] = CAN_F0R1 two most significant bytes
	
	CAN_FR[4] = CAN_F1R0 two least significant bytes
	... and so on*/
	//read reference manual 32.7.4 for details on how to use CAN_FiRx registers
}
type_can_filter_settings_struct;

typedef struct
{
	uint16_t mode; //mode
	uint16_t TME; //transmit mailbox empty register. If mailbox x is empty bit x is set
	uint16_t TERR; //transmittion error register. If transmittion at mailbox x failed bit x is set
	uint16_t ALST; //arbitration lost register. If arbitration at mailbox x lost bit x is set
	uint16_t TXOK; //transmit OK register. If mailbox x transmit complete bit x is set
	uint16_t RQCP; //request completed register. If mailbox x request completed bit x is set
	uint16_t softwareFIFO_Overrun; //software FIFO overrun. If software fifo x met overrun bit x is set
	uint16_t REC; //receive error counter
	uint16_t TEC; //transmit error counter
	uint16_t BOFF; //bus-off flag
	uint16_t EPVF; //error passive flag
}
type_can_status_struct;

typedef struct
{
	uint16_t receive_ack_0; //if this field is not 0, the next CAN message will be presented in CAN input Modbus registers for FIFO0 software FIFO, then this field will be reset
	uint16_t receive_ack_1; //the same for software FIFO1
	uint16_t error_ack; //the same for error FIFO 
	uint16_t busOffRecover; //if this field is not 0 bxCAN will recover from the Bus-Off state then the field will be reset
}
type_can_control_struct;

typedef struct 
{
	uint16_t numberOfFrames; //number of frames available in software FIFO
	uint16_t ID_L; //standart ID or 16 least significant bits of extended ID
	uint16_t ID_H; //13 most significant bits of extended ID
	uint16_t IDE; //1 if the identifier is extended
	uint16_t FMI; //filter match index
	uint16_t RTR; //1 if frame is remote
	uint16_t DLC; //data length code
	uint16_t DATA[4]; /*frame data. mapping:
	DATA[0] = byte 0 + (byte 1 << 8)
	DATA[1] = byte 2 + (byte 3 << 8)
	DATA[2] = byte 4 + (byte 5 << 8)
	DATA[3] = byte 6 + (byte 7 << 8) where byte 0 is received first*/
}
type_can_receive_struct;

typedef struct 
{
	uint16_t TXRQ; //set this field to start the transmission
	uint16_t ABRQ; //set this field to abort transmission
	uint16_t ID_L; //standart ID or 16 least significant bits of extended ID
	uint16_t ID_H; //13 most significant bits of extended ID
	uint16_t IDE; //set this field to make the identifier extended
	uint16_t RTR; //set this field to make the frame remote
	uint16_t DLC; //data length code
	uint16_t DATA[4]; /*frame data. mapping:
	DATA[0] = byte 0 + (byte 1 << 8)
	DATA[1] = byte 2 + (byte 3 << 8)
	DATA[2] = byte 4 + (byte 5 << 8)
	DATA[3] = byte 6 + (byte 7 << 8) where byte 0 is sent first*/
}
type_can_transmit_struct;

typedef struct 
{
	uint16_t errorCode; //error code according to LEC field of CAN_ESR register. Error codes are stored in FIFO
}
type_can_error_struct;

#pragma pack(pop)

typedef struct
{
	uint32_t ID;
	uint8_t IDE;
	uint8_t FMI;
	uint8_t RTR;
	uint8_t DLC;
	uint8_t DATA[8];
	uint8_t LEC;
}
FIFO_Element_Data_Typedef;

struct FIFO_Element
{
	FIFO_Element_Data_Typedef Data;
	struct FIFO_Element* next;
};

typedef struct FIFO_Element FIFO_Element_Typedef;

typedef struct
{
	FIFO_Element_Typedef* FI;
}
FIFO_Typedef;

typedef FIFO_Typedef Error_FIFO_Typedef;

FIFO_Typedef* FIFO_Create(void);
void FIFO_Delete(FIFO_Typedef* deleteFIFO);
void FIFO_Push(FIFO_Element_Data_Typedef pushData, FIFO_Typedef* pushFIFO);
FIFO_Element_Data_Typedef FIFO_Pop(FIFO_Typedef* popFIFO);
uint16_t FIFO_Get_Length(FIFO_Typedef* lengthFIFO);

typedef enum
{
	noOverrun = 0,
	overrunOccured = 1,
} 
FIFO_Overrun_Flag_Typedef;

#endif
