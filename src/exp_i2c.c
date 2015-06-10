/*****************************************************************************
 *   i2c.c:  I2C C file for NXP LPC17xx Family Microprocessors
 *
 *   Copyright(C) 2009, NXP Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2009.05.26  ver 1.00    Prelimnary version, first Release
 *
 *****************************************************************************/
#include <bsp.h>
#include "lpc17xx.h"
#include "exp_i2c.h"

volatile uint32_t I2C1_MasterState,I2C0_MasterState = I2C_IDLE;
volatile uint32_t I2C1_SlaveState,I2C0_SlaveState = I2C_IDLE;

volatile uint32_t I2C1_Cmd,I2C0_Cmd;
volatile uint32_t I2C1_Mode,I2C0_Mode;

volatile uint8_t I2C1_MasterBuffer[BUFSIZE],I2C0_MasterBuffer[BUFSIZE];
volatile uint8_t I2C1_SlaveBuffer[BUFSIZE],I2C0_SlaveBuffer[BUFSIZE];
volatile uint32_t I2C1_Count,I2C0_Count = 0;
volatile uint32_t I2C1_ReadLength,I2C0_ReadLength;
volatile uint32_t I2C1_WriteLength,I2C0_WriteLength;

volatile uint32_t RdIndex_1,RdIndex_0 = 0;
volatile uint32_t WrIndex_1,WrIndex_0 = 0;


void I2C0_IRQHandler(void) {
	uint8_t StatValue;
	/* this handler deals with master read and master write only */
	StatValue = LPC_I2C0->I2STAT;
	switch (StatValue) {
		case 0x08: /* A Start condition is issued. */
			LPC_I2C0->I2DAT = I2C0_MasterBuffer[WrIndex_0++];
			LPC_I2C0->I2CONCLR = (I2CONCLR_SIC | I2CONCLR_STAC);
			I2C0_MasterState = I2C_STARTED;
			break;
		case 0x10: /* A repeated started is issued */
			if (!I2C0_Cmd) {
				LPC_I2C0->I2DAT = I2C0_MasterBuffer[WrIndex_0++];
			}
			LPC_I2C0->I2CONCLR = (I2CONCLR_SIC | I2CONCLR_STAC);
			I2C0_MasterState = I2C_RESTARTED;
			break;
		case 0x18: /* Regardless, it's a ACK */
			if (I2C0_MasterState == I2C_STARTED) {
				LPC_I2C0->I2DAT = I2C0_MasterBuffer[WrIndex_0++];
				I2C0_MasterState = DATA_ACK;
			}
			LPC_I2C0->I2CONCLR = I2CONCLR_SIC;
			break;
		case 0x28: /* Data byte has been transmitted, regardless ACK or NACK */
			 if ( WrIndex_0 < I2C0_WriteLength )
				{
				  LPC_I2C0->I2DAT = I2C0_MasterBuffer[WrIndex_0++]; /* this should be the last one */
				}
				else
				{
				  if ( I2C0_ReadLength != 0 )
				  {
						LPC_I2C0->I2CONSET = I2CONSET_STA;   /* Set Repeated-start flag */
				  }
				  else
				  {
						LPC_I2C0->I2CONSET = I2CONSET_STO;      /* Set Stop flag */
						I2C0_MasterState = I2C_IDLE;
				  }
				}
				LPC_I2C0->I2CONCLR = I2CONCLR_SIC;
				break;
		case 0x30:
			if (WrIndex_0 != I2C0_WriteLength) {
				LPC_I2C0->I2DAT = I2C0_MasterBuffer[1 + WrIndex_0]; /* this should be the last one */
				WrIndex_0++;
				if (WrIndex_0 != I2C0_WriteLength) {
					I2C0_MasterState = DATA_ACK;
				} else {
					I2C0_MasterState = DATA_NACK;
					if (I2C0_ReadLength != 0) {
						LPC_I2C0->I2CONSET = I2CONSET_STA; /* Set Repeated-start flag */
						I2C0_MasterState = I2C_REPEATED_START;
					}
				}
			} else {
				if (I2C0_ReadLength != 0) {
					LPC_I2C0->I2CONSET = I2CONSET_STA; /* Set Repeated-start flag */
					I2C0_MasterState = I2C_REPEATED_START;
				} else {
					I2C0_MasterState = DATA_NACK;
					LPC_I2C0->I2CONSET = I2CONSET_STO; /* Set Stop flag */
				}
			}
			LPC_I2C0->I2CONCLR = I2CONCLR_SIC;
			break;
		case 0x40: /* Master Receive, SLA_R has been sent */
			LPC_I2C0->I2CONSET = I2CONSET_AA; /* assert ACK after data is received */
			LPC_I2C0->I2CONCLR = I2CONCLR_SIC;
			break;
		case 0x50: /* Data byte has been received, regardless following ACK or NACK */
		case 0x58:
			I2C0_MasterBuffer[3 + RdIndex_0] = LPC_I2C0->I2DAT;
			RdIndex_0++;
			if (RdIndex_0 != I2C0_ReadLength) {
				I2C0_MasterState = DATA_ACK;
			} else {
				RdIndex_0 = 0;
				I2C0_MasterState = DATA_NACK;
			}
			LPC_I2C0->I2CONSET = I2CONSET_AA; /* assert ACK after data is received */
			LPC_I2C0->I2CONCLR = I2CONCLR_SIC;
			break;
		case 0x20: /* regardless, it's a NACK */
		case 0x48:
			LPC_I2C0->I2CONCLR = I2CONCLR_SIC;
			I2C0_MasterState = DATA_NACK;
			break;
		case 0x38: /* Arbitration lost, in this example, we don't
		 deal with multiple master situation */
		default:
			LPC_I2C0->I2CONCLR = I2CONCLR_SIC;
			break;
	}
}
/*
 From device to device, the I2C communication protocol may vary,
 in the example below, the protocol uses repeated start to read data from or
 write to the device:
 For master read: the sequence is: STA,Addr(W),offset,RE-STA,Addr(r),data...STO
 for master write: the sequence is: STA,Addr(W),length,RE-STA,Addr(w),data...STO
 Thus, in state 8, the address is always WRITE. in state 10, the address could
 be READ or WRITE depending on the I2CCmd.
 */
/*****************************************************************************
 ** Function name:		I2C0_IRQHandler
 **
 ** Descriptions:		I2C0 interrupt handler, deal with master mode
 **						only.
 **
 ** parameters:			None
 ** Returned value:		None
 **
 *****************************************************************************/
void I2C1_IRQHandler(void) {
	uint8_t StatValue;
	/* this handler deals with master read and master write only */
	StatValue = LPC_I2C1->I2STAT;
	switch (StatValue) {
	case 0x08: /* A Start condition is issued. */
		LPC_I2C1->I2DAT = I2C1_MasterBuffer[0];
		LPC_I2C1->I2CONCLR = (I2CONCLR_SIC | I2CONCLR_STAC);
		I2C1_MasterState = I2C_STARTED;
		break;
	case 0x10: /* A repeated started is issued */
		if (!I2C1_Cmd) {
			LPC_I2C1->I2DAT = I2C1_MasterBuffer[2];
		}
		LPC_I2C1->I2CONCLR = (I2CONCLR_SIC | I2CONCLR_STAC);
		I2C1_MasterState = I2C_RESTARTED;
		break;
	case 0x18: /* Regardless, it's a ACK */
		if (I2C1_MasterState == I2C_STARTED) {
			LPC_I2C1->I2DAT = I2C1_MasterBuffer[1 + WrIndex_1];
			WrIndex_1++;
			I2C1_MasterState = DATA_ACK;
		}
		LPC_I2C1->I2CONCLR = I2CONCLR_SIC;
		break;
	case 0x28: /* Data byte has been transmitted, regardless ACK or NACK */
	case 0x30:
		if (WrIndex_1 != I2C1_WriteLength) {
			LPC_I2C1->I2DAT = I2C1_MasterBuffer[1 + WrIndex_1]; /* this should be the last one */
			WrIndex_1++;
			if (WrIndex_1 != I2C1_WriteLength) {
				I2C1_MasterState = DATA_ACK;
			} else {
				I2C1_MasterState = DATA_NACK;
				if (I2C1_ReadLength != 0) {
					LPC_I2C1->I2CONSET = I2CONSET_STA; /* Set Repeated-start flag */
					I2C1_MasterState = I2C_REPEATED_START;
				}
			}
		} else {
			if (I2C1_ReadLength != 0) {
				LPC_I2C1->I2CONSET = I2CONSET_STA; /* Set Repeated-start flag */
				I2C1_MasterState = I2C_REPEATED_START;
			} else {
				I2C1_MasterState = DATA_NACK;
				LPC_I2C1->I2CONSET = I2CONSET_STO; /* Set Stop flag */
			}
		}
		LPC_I2C1->I2CONCLR = I2CONCLR_SIC;
		break;
	case 0x40: /* Master Receive, SLA_R has been sent */
		LPC_I2C1->I2CONSET = I2CONSET_AA; /* assert ACK after data is received */
		LPC_I2C1->I2CONCLR = I2CONCLR_SIC;
		break;
	case 0x50: /* Data byte has been received, regardless following ACK or NACK */
	case 0x58:
		I2C1_MasterBuffer[3 + RdIndex_1] = LPC_I2C1->I2DAT;
		RdIndex_1++;
		if (RdIndex_1 != I2C1_ReadLength) {
			I2C1_MasterState = DATA_ACK;
		} else {
			RdIndex_1 = 0;
			I2C1_MasterState = DATA_NACK;
		}
		LPC_I2C1->I2CONSET = I2CONSET_AA; /* assert ACK after data is received */
		LPC_I2C1->I2CONCLR = I2CONCLR_SIC;
		break;
	case 0x20: /* regardless, it's a NACK */
	case 0x48:
		LPC_I2C1->I2CONCLR = I2CONCLR_SIC;
		I2C1_MasterState = DATA_NACK;
		break;
	case 0x38: /* Arbitration lost, in this example, we don't
	 deal with multiple master situation */
	default:
		LPC_I2C1->I2CONCLR = I2CONCLR_SIC;
		break;
	}
}
/*****************************************************************************
 ** Function name:		I2CStart
 **
 ** Descriptions:		Create I2C start condition, a timeout
 **				value is set if the I2C never gets started,
 **				and timed out. It's a fatal error.
 **
 ** parameters:			None
 ** Returned value:		true or false, return false if timed out
 **
 *****************************************************************************/
uint32_t I2CStart(uint8_t port) {
	uint32_t timeout = 0;
	uint32_t retVal = 0;
	if (port == 0){
			/*--- Issue a start condition ---*/
			LPC_I2C0->I2CONSET = I2CONSET_STA; /* Set Start flag */
			/*--- Wait until START transmitted ---*/
			while (1) {
				if (I2C0_MasterState == I2C_STARTED) {
					retVal = 1;
					break;
				}
				if (timeout >= MAX_TIMEOUT) {
					retVal = 0;
					break;
				}
				timeout++;
			}
			return (retVal);
		}
	else if (port == 1){
		/*--- Issue a start condition ---*/
		LPC_I2C1->I2CONSET = I2CONSET_STA; /* Set Start flag */
		/*--- Wait until START transmitted ---*/
		while (1) {
			if (I2C1_MasterState == I2C_STARTED) {
				retVal = 1;
				break;
			}
			if (timeout >= MAX_TIMEOUT) {
				retVal = 0;
				break;
			}
			timeout++;
		}
		return (retVal);
	}
	return 0;
}

/*****************************************************************************
 ** Function name:		I2CStop
 **
 ** Descriptions:		Set the I2C stop condition, if the routine
 **				never exit, it's a fatal bus error.
 **
 ** parameters:			None
 ** Returned value:		true or never return
 **
 *****************************************************************************/
uint32_t I2CStop(uint8_t port) {
	if (port == 0){
		LPC_I2C0->I2CONSET = I2CONSET_STO; /* Set Stop flag */
		LPC_I2C0->I2CONCLR = I2CONCLR_SIC; /* Clear SI flag */
		/*--- Wait for STOP detected ---*/
		while (LPC_I2C0->I2CONSET & I2CONSET_STO);
		return 1;
	}else if(port == 1){
		LPC_I2C1->I2CONSET = I2CONSET_STO; /* Set Stop flag */
		LPC_I2C1->I2CONCLR = I2CONCLR_SIC; /* Clear SI flag */
		/*--- Wait for STOP detected ---*/
		while (LPC_I2C1->I2CONSET & I2CONSET_STO);
		return 1;
	}
	return 0;
}

/*****************************************************************************
 ** Function name:		I2CInit
 **
 ** Descriptions:		Initialize I2C controller
 **
 ** parameters:			I2c mode is either MASTER or SLAVE
 ** Returned value:		true or false, return false if the I2C
 **				interrupt handler was not installed correctly
 **
 *****************************************************************************/
uint32_t I2CInit(uint8_t port,uint32_t I2cMode) {
	if (port == 0){
		LPC_SC->PCONP |= (1 << 7);
	    //LPC_PINCON->PINSEL1 &= ~0x03C00000;
	    //LPC_PINCON->PINSEL1 |=  0x01400000;
		/* set PIO0.27 and PIO0.28 to I2C0 SDA and SCL */
		/* function to 01 on both SDA and SCL. */
		LPC_PINCON->PINSEL1 &= ~((0x03<<22)|(0x03<<24));
		LPC_PINCON->PINSEL1 |= ((0x01<<22)|(0x01<<24));
	    //LPC_SC->PCLKSEL0 &= ~(3 << 14); // clear bits
	    //LPC_SC->PCLKSEL0 |=  (3 << 14); // set to "01" (full speed)
		//PINMODE is not required for P0.27, P0.28 these are dedicated open drain I2C pins.
		/*--- Clear flags ---*/
		LPC_I2C0->I2CONCLR = I2CONCLR_AAC | I2CONCLR_SIC | I2CONCLR_STAC
				| I2CONCLR_I2ENC;
		/*--- Reset registers ---*/
		LPC_PINCON->I2CPADCFG &= ~((0x1<<0)|(0x1<<2));
		LPC_I2C0->I2SCLL   = I2SCLL_SCLL;
		LPC_I2C0->I2SCLH   = I2SCLH_SCLH;
		/* Install interrupt handler */
		NVIC_EnableIRQ(I2C0_IRQn);
		LPC_I2C0->I2CONSET = I2CONSET_I2EN;
		return (1);
	}
	else if (port == 1){
		LPC_SC->PCONP |= (1 << 19);
		/* set PIO0.19 and PIO0.20 to I2C1 SDA and SCL */
		/* function to 11 on both SDA and SCL. */
		LPC_PINCON->PINSEL1 &= ~((0x3 << 6) | (0x3 << 8));
		LPC_PINCON->PINSEL1 |= ((0x3 << 6) | (0x3 << 8));
		LPC_PINCON->PINMODE1 &= ~((0x3 << 6) | (0x3 << 8));
		LPC_PINCON->PINMODE1 |= ((0x2 << 6) | (0x2 << 8)); /* No pull-up no pull-down */
		LPC_PINCON->PINMODE_OD0 |= ((0x1 << 19) | (0x1 << 20));

		/*--- Clear flags ---*/
		LPC_I2C1->I2CONCLR = I2CONCLR_AAC | I2CONCLR_SIC | I2CONCLR_STAC
				| I2CONCLR_I2ENC;

		/*--- Reset registers ---*/
		LPC_I2C1->I2SCLL = I2SCLL_SCLL;
		LPC_I2C1->I2SCLH = I2SCLH_SCLH;

		/* Install interrupt handler */
		NVIC_EnableIRQ(I2C1_IRQn);

		LPC_I2C1->I2CONSET = I2CONSET_I2EN;
		return (1);
	}
	return (0);
}

/*****************************************************************************
 ** Function name:		I2CEngine
 **
 ** Descriptions:		The routine to complete a I2C transaction
 **				from start to stop. All the intermitten
 **				steps are handled in the interrupt handler.
 **				Before this routine is called, the read
 **				length, write length, I2C master buffer,
 **				and I2C command fields need to be filled.
 **				see i2cmst.c for more details.
 **
 ** parameters:			None
 ** Returned value:		true or false, return false only if the
 **				start condition can never be generated and
 **				timed out.
 **
 *****************************************************************************/
uint32_t I2CEngine(uint8_t port) {
	int timeout = 0;
	if (port == 0){
			I2C0_MasterState = I2C_IDLE;
			RdIndex_0 = 0;
			WrIndex_0 = 0;
			if (I2CStart(port) != 1) {
				I2CStop(port);
				return (0);
			}

			while (1) {
				if (I2C0_MasterState == DATA_NACK) {
					I2CStop(port);
					break;
				}
				if (timeout >= 0xFFFF) {
					break;
				}
				timeout++;
			}
			return (1);
		}
	else if (port == 1){
		I2C1_MasterState = I2C_IDLE;
		RdIndex_1 = 0;
		WrIndex_1 = 0;
		if (I2CStart(port) != 1) {
			I2CStop(port);
			return (0);
		}

		while (1) {
			if (I2C1_MasterState == DATA_NACK) {
				I2CStop(port);
				break;
			}
		}
		return (1);
	}
	return 0;
}

uint8_t i2c_write(uint8_t port, uint8_t register_address, uint8_t data) {
	int i;
	if (port == 0){
		for (i = 0; i < BUFSIZE; i++) /* clear buffer */
			{
				I2C0_MasterBuffer[i] = 0;
			}
			I2C0_WriteLength = 3;
			I2C0_ReadLength = 0;
			I2C0_MasterBuffer[0] = ADR_BYTE_MMA7455;
			I2C0_MasterBuffer[1] = register_address;
			I2C0_MasterBuffer[2] = data;
			if (I2CEngine(port))
				return 1;
			else
				return 0;
	}
	else if (port == 1){
		for (i = 0; i < BUFSIZE; i++) /* clear buffer */
		{
			I2C1_MasterBuffer[i] = 0;
		}
		I2C1_WriteLength = 3;
		I2C1_ReadLength = 0;
		I2C1_MasterBuffer[0] = ADR_BYTE_24AA02;
		I2C1_MasterBuffer[1] = register_address;
		I2C1_MasterBuffer[2] = data;
		if (I2CEngine(port))
			return 1;
		else
			return 0;
	}
	return 0;
}

uint8_t i2c_read(uint8_t port, uint8_t register_addr) {
	int i;
	/* clear buffer */
	if (port == 0){
		for (i = 0; i < BUFSIZE; i++) {
			I2C0_MasterBuffer[i] = 0;
		}
		I2C0_WriteLength = 2;
		I2C0_ReadLength = 1;
		I2C0_MasterBuffer[0] = 0x3A;//ADR_BYTE_MMA7455;
		I2C0_MasterBuffer[1] = register_addr;
		I2C0_MasterBuffer[2] = 0x3B;//ADR_BYTE_MMA7455 | RD_BIT;
		I2CEngine(port);
		I2CStop(port);
		return (I2C0_MasterBuffer[3]);
	}
	else if (port == 1){
		for (i = 0; i < BUFSIZE; i++) {
			I2C1_MasterBuffer[i] = 0;
		}
		I2C1_WriteLength = 2;
		I2C1_ReadLength = 1;
		I2C1_MasterBuffer[0] = ADR_BYTE_24AA02;
		I2C1_MasterBuffer[1] = register_addr;
		I2C1_MasterBuffer[2] = ADR_BYTE_24AA02 | RD_BIT;
		I2CEngine(port);
		I2CStop(port);
		return (I2C1_MasterBuffer[3]);
	}
	return (0);
}

/******************************************************************************
 **                            End Of File
 ******************************************************************************/

