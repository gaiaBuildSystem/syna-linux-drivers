// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2025 Synaptics Incorporated */

#ifndef _SYNA_CEC_PRV_C_
#define _SYNA_CEC_PRV_C_

#include <linux/io.h>
#include "syna_cec_prv.h"
#include "syna_cec.h"

extern int berlin_pe_dev;
extern int berlin_cec_dev;
/******************************************************************************
 * FUNCTION : Calculates timer count based on block clock (in MHz)
 * PARAMS   : timeVal	- Time value in microseconds
 *		  : timerCount - Pointer to return timer count (Output)
 * RETURN   : 0 on success and -1 on error
 *****************************************************************************/
static int CEC_CalcTimerCount (u32 timeVal, u32 *pTimerCount);

/******************************************************************************
 * FUNCTION : Enables/Disables the given interrupt(s)
 * PARAMS   : *ded_dev - Pointer to CEC device object
 *		  : intr	 - Interrupt(s) to be enabled/disabled
 *		  : enable   - 1, to enable and 0 to disable
 *		  : bufWrite - Flag to indicate if data has to be written to
 *		  :		  - BCM buffer or to register immediately
 * RETURN   : 0, on success
 *		  : Error code, otherwise
 *****************************************************************************/
static int berlin_cec_enable_interrupt (struct cec_device_t *cec_dev, u16 intr,
								bool enable);
/******************************************************************************
 * FUNCTION : Loads default values of CEC block to registers
 * PARAMS   : *cec_dev - Pointer to CEC device object
 * RETURN   : 0, on success
 *		  : Error code, otherwise
 *****************************************************************************/
int berlin_cec_load_default_val(struct cec_device_t *cec_dev)
{
	u32 reg = 0;
	int i;

	// Write signal free time count
	CEC_CalcTimerCount (CEC_SIGNAL_FREE_TIME, &reg);
	berlin_cec_reg_write (cec_dev, CEC_SIGNAL_FREE_TIME_0_ADDR, (u8*)(&reg), 4);

	// Write start bit low duration
	CEC_CalcTimerCount (CEC_START_BIT_0_LOW_TIME, &reg);
	berlin_cec_reg_write (cec_dev, CEC_START_BIT_LO_THRESH_0_ADDR, (u8*)(&reg), 4);

	// Write start bit total duration
	CEC_CalcTimerCount ((CEC_START_BIT_0_LOW_TIME + CEC_START_BIT_0_HI_TIME), &reg);
	berlin_cec_reg_write (cec_dev, CEC_START_BIT_HI_THRESH_0_ADDR, (u8*)(&reg), 4);

	// Write data bit 0 low duration
	CEC_CalcTimerCount (CEC_DATA_BIT_0_LOW_TIME, &reg);
	berlin_cec_reg_write (cec_dev, CEC_DATA_BIT_0_LO_THRESH_0_ADDR, (u8*)(&reg), 4);

	// Write data bit 1 low duration
	CEC_CalcTimerCount (CEC_DATA_BIT_1_LOW_TIME, &reg);
	berlin_cec_reg_write (cec_dev, CEC_DATA_BIT_1_LO_THRESH_0_ADDR, (u8*)(&reg), 4);

	// Write data bit 0 total duration
	CEC_CalcTimerCount ((CEC_DATA_BIT_0_LOW_TIME + CEC_DATA_BIT_0_HI_TIME), &reg);
	berlin_cec_reg_write (cec_dev, CEC_DATA_BIT_0_HI_THRESH_0_ADDR, (u8*)(&reg), 4);

	// Write data bit 1 total duration
	CEC_CalcTimerCount ((CEC_DATA_BIT_1_LOW_TIME + CEC_DATA_BIT_1_HI_TIME), &reg);
	berlin_cec_reg_write (cec_dev, CEC_DATA_BIT_1_HI_THRESH_0_ADDR, (u8*)(&reg), 4);

	// Write safe sampling time for ACK
	CEC_CalcTimerCount (CEC_SSP_ACK_TIME, &reg);
	berlin_cec_reg_write (cec_dev, CEC_SSP_ACK_TIME_0_ADDR, (u8*)(&reg), 4);

	// Write nominal sampling time for data bit
	CEC_CalcTimerCount (CEC_NOMINAL_SAMPLE_TIME, &reg);
	berlin_cec_reg_write (cec_dev, CEC_NOMINAL_SAMPLE_TIME_0_ADDR, (u8*)(&reg), 4);

	// Write hysterisis time count
	CEC_CalcTimerCount (CEC_FOLL_HYST_TIME, &reg);
	berlin_cec_reg_write (cec_dev, CEC_HYST_TIME_0_ADDR, (u8*)(&reg), 4);

	// Write follower ack time count
	CEC_CalcTimerCount (CEC_FOLL_ACK_ASSRT_TIME, &reg);
	berlin_cec_reg_write (cec_dev, CEC_FOLLOWER_ACK_TIME_0_ADDR, (u8*)(&reg), 4);

	// Write collision window time count
	CEC_CalcTimerCount (CEC_COLL_WINDOW_TIME, &reg);
	berlin_cec_reg_write (cec_dev, CEC_COLL_WINDOW_TIME_REG_0_ADDR, (u8*)(&reg), 4);

	// Enable collision detection
	reg = 0x01;
	berlin_cec_reg_write (cec_dev, CEC_COLL_CTRL_REG_ADDR, (u8*)(&reg), 1);

	// Write start bit jitter count
	CEC_CalcTimerCount (CEC_STARTBIT_JITTER_TIME, &reg);
	berlin_cec_reg_write (cec_dev, CEC_JITTER_CNT_SB_0, (u8*)(&reg), 4);

	// Write data bit jitter count
	CEC_CalcTimerCount (CEC_DATABIT_JITTER_TIME, &reg);
	berlin_cec_reg_write (cec_dev, CEC_JITTER_CNT_0_ADDR, (u8*)(&reg), 4);

	// Write error notification time
	CEC_CalcTimerCount (CEC_ERR_GEN_TIME, &reg);
	berlin_cec_reg_write (cec_dev, CEC_ERR_NOTIF_TIME_0, (u8*)(&reg), 4);

	/* Changed to avoid bit period long error. JIRA:CSMLGUKAO-569/CSMUBQ-124 */
	reg = 0x20; // CSMLGU4TCH-350 Extended glitch filter size to 32(1.28usec) for LGE OLED55C4SNA and 43UR80006LA due to induced noise from DDC
	berlin_cec_reg_write (cec_dev, CEC_GLITCH_FILT_W_L, (u8*)(&reg), 1);

	// Disable all logical addresses
	for (i = 0; i < 5; i++)
		berlin_cec_enable_log_addr(cec_dev, false, i, 0);

	// Disable PMode
	reg = 0x00;
	berlin_cec_reg_write (cec_dev, CEC_PMODE_ADDR, (u8*)(&reg),1);

	// Clear all interrupts and enable required ones
	berlin_cec_enable_interrupt (cec_dev, BERLIN_CEC_INTR_ALL, false);
	berlin_cec_enable_interrupt (cec_dev, (BERLIN_CEC_INTR_TX_FAIL | BERLIN_CEC_INTR_TX_COMPLETE | BERLIN_CEC_INTR_RX_COMPLETE | BERLIN_CEC_INTR_RX_FAIL), true);

	// Disable transmit mode and Enable receive mode
	berlin_cec_set_mode (cec_dev, BERLIN_CEC_MODE_TX, false);
	berlin_cec_set_mode (cec_dev, BERLIN_CEC_MODE_RX, true);
	return 0;
}
EXPORT_SYMBOL_GPL(berlin_cec_load_default_val);

/******************************************************************************
 * FUNCTION : Sets/Resets transmit/receive mode
 * PARAMS   : *cec_dev - Pointer to CEC device object
 *		  : cecMode  - CEC mode (Tx/Rx)
 *		  : enable   - 1, to enable and 0 to disable
 *		  : bufWrite - Flag to indicate if data has to be written to
 *		  :		  - BCM buffer or to register immediately
 * RETURN   : 0, on success
 *		  : Error code, otherwise
 *****************************************************************************/
int berlin_cec_set_mode (struct cec_device_t *cec_dev, berlin_cec_mode cecMode,
					bool enable)
{
	u8 reg;

	if (cecMode > BERLIN_CEC_MODE_MAX)
	{
		return -EINVAL;
	}

	if (cecMode == BERLIN_CEC_MODE_TX)
	{
		if (enable)
		{
			// Reset Tx FIFO
			reg = 0x00;
			berlin_cec_reg_write (cec_dev, CEC_TX_FIFO_RESET_ADDR, &reg, 1);

			reg = 0x01;
			berlin_cec_reg_write (cec_dev, CEC_TX_FIFO_RESET_ADDR, &reg, 1);

			// Enable required interrupts
			berlin_cec_enable_interrupt (cec_dev, (BERLIN_CEC_INTR_TX_FAIL | BERLIN_CEC_INTR_TX_COMPLETE), false);
			berlin_cec_enable_interrupt (cec_dev, (BERLIN_CEC_INTR_TX_FAIL | BERLIN_CEC_INTR_TX_COMPLETE), true);
		}
		else
		{
			reg = 0x00;

			// Disable CEC_RDY
			berlin_cec_reg_write (cec_dev, CEC_RDY_ADDR, &reg, 1);

		}
	}
	else if (cecMode == BERLIN_CEC_MODE_RX)
	{
		if (enable)
		{
			// Reset Rx FIFO
			reg = 0x00;
			berlin_cec_reg_write (cec_dev, CEC_RX_FIFO_RESET_ADDR, &reg, 1);

			reg = 0x01;
			berlin_cec_reg_write (cec_dev, CEC_RX_FIFO_RESET_ADDR, &reg, 1);

			// Enable Rx Ready
			reg = 0x01;
			berlin_cec_reg_write (cec_dev, CEC_RX_RDY_ADDR, &reg, 1);

			// Enable required interrupts
			berlin_cec_enable_interrupt (cec_dev, (BERLIN_CEC_INTR_RX_FAIL | BERLIN_CEC_INTR_RX_COMPLETE), false);
			berlin_cec_enable_interrupt (cec_dev, (BERLIN_CEC_INTR_RX_FAIL | BERLIN_CEC_INTR_RX_COMPLETE), true);
		}
		else
		{
			// Disable Rx Ready
			reg = 0x00;
			berlin_cec_reg_write (cec_dev, CEC_RX_RDY_ADDR, &reg, 1);
		}
	}
	return 0;
}
EXPORT_SYMBOL_GPL(berlin_cec_set_mode);

u8 berlin_cec_rx_line_status(struct cec_device_t *cec_dev)
{
	u8 value = 0;
	berlin_cec_reg_read (cec_dev, CEC_RX_PRESENT_STATE_REG_ADDR , &value, 1);
	return value;
}
/******************************************************************************
 * FUNCTION : Sends data to the transmit FIFO and initiates transfer on the
 *		  : CEC line, It also monitors for tx errors and handles
 *		  : retransmission if specified
 * PARAMS   : *cec_dev - Pointer to CEC device object
 *		  : msg  - Pointer to transmit data structure
          :signal_free_time : signal free time multiplier
 * RETURN   : 0, on success
 *		  : Error code, otherwise
 *****************************************************************************/
int berlin_cec_transmit_data(struct cec_device_t *cec_dev, struct cec_msg *msg, int signal_free_time)
{
	u32  reg = 0;
	u32  signalFreeTime;
	int	 i;

	if (msg == NULL)
		return -EINVAL;

	signal_free_time += 2; //Additional 2 time frame
	signalFreeTime = (signal_free_time * CEC_DATA_BIT_TIME);
	// Set signal free time
	CEC_CalcTimerCount (signalFreeTime, &reg);
	berlin_cec_reg_write (cec_dev, CEC_SIGNAL_FREE_TIME_0_ADDR, (u8*)(&reg), 4);

	// Set mode to transmit
	berlin_cec_set_mode (cec_dev, BERLIN_CEC_MODE_TX, true);

	// Set transmit type
	reg = cec_msg_is_broadcast(msg);
	berlin_cec_reg_write (cec_dev, CEC_TX_TYPE_ADDR, (u8*)&reg, 1);
	// Write data to FIFO
	for (i = 0; i < msg->len; i++)
	{
		// Write into data register
		berlin_cec_reg_write (cec_dev, CEC_DATA_REG_ADDR, &msg->msg[i], 1);

		// If last block, set EOM
		reg = (i == msg->len-1)?0x01:0x00;
		berlin_cec_reg_write (cec_dev, CEC_EOM_REG_ADDR, (u8*)&reg, 1);

		// Push data to FIFO
		reg = 0x00;
		berlin_cec_reg_write (cec_dev, CEC_TOGGLE_FOR_WRITE_REG_ADDR, (u8*)&reg, 1);
   }

	// Set initiator ready register
	reg = 0x01;
	berlin_cec_reg_write (cec_dev, CEC_RDY_ADDR, (u8*)&reg, 1);
	return 0;
}
EXPORT_SYMBOL_GPL(berlin_cec_transmit_data);

/******************************************************************************
 * FUNCTION : Controls logical address at given index
 * PARAMS   : *cec_dev - Pointer to CEC device object
 *		  : enable	   - true/false to enable/disable logical address
 *		  : logAddr	  - Logical address (if disabled, it is forced to 0)
 * RETURN   : 0, on success
 *		  : Error code, otherwise
 *****************************************************************************/
int berlin_cec_enable_log_addr (struct cec_device_t *cec_dev,
						  bool enable, u8 addrIndex, u8 logAddr)
{
	u32  regAddr;

	switch (addrIndex)
	{
		case 0: regAddr = CEC_LOGICAL_ADDR0_REG_ADDR; break;
		case 1: regAddr = CEC_LOGICAL_ADDR1_REG_ADDR; break;
		case 2: regAddr = CEC_LOGICAL_ADDR2_REG_ADDR; break;
		case 3: regAddr = CEC_LOGICAL_ADDR3_REG_ADDR; break;
		case 4: regAddr = CEC_LOGICAL_ADDR4_REG_ADDR; break;
		default: return -EINVAL;
	}

	if (enable)
	{
		logAddr |= (1 << 4);
		berlin_cec_reg_write (cec_dev, regAddr, (u8*)(&logAddr),1);
	}
	else
	{
		logAddr = 0;
		berlin_cec_reg_write (cec_dev, regAddr, (u8*)(&logAddr),1);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(berlin_cec_enable_log_addr);

///////////////////////////////////////////////////////////////////////////////
/////////////////////////////LOCAL STATIC FUNCTIONS////////////////////////////
///////////////////////////////////////////////////////////////////////////////

/******************************************************************************
 * FUNCTION : Calculates timer count based on block clock (in MHz)
 * PARAMS   : timeVal	- Time value in microseconds
 *		  : timerCount - Pointer to return timer count (Output)
 * RETURN   : 0 on success, -1 on failure
 *****************************************************************************/
static int CEC_CalcTimerCount (u32 timeVal, u32 *pTimerCount)
{
	// Clock frequency has to be in MHz
	int clkFreq = 25;//GaloisGetFrequency(SOC_FREQ_CFG);

	if (clkFreq > 100) {
		return -EINVAL;
	}
	*pTimerCount = (timeVal * clkFreq);
	return 0;
}

/******************************************************************************
 * FUNCTION : Enables/Disables the given interrupt(s)
 * PARAMS   : *cec_dev - Pointer to CEC device object
 *		  : intr	 - Interrupt(s) to be enabled/disabled
 *		  : enable   - 1, to enable and 0 to disable
 * RETURN   : 0, on success
 *		  : Error code, otherwise
 *****************************************************************************/
static int berlin_cec_enable_interrupt (struct cec_device_t *cec_dev, u16 intr,
								bool enable)
{
	u8 reg;

	berlin_cec_reg_read (cec_dev, CEC_INTR_ENABLE0_REG_ADDR, &reg, 1);
	if (enable)
	{
		berlin_cec_reg_read (cec_dev, CEC_INTR_ENABLE0_REG_ADDR, &reg, 1);
		reg |= (intr & 0x00ff);
		berlin_cec_reg_write(cec_dev,CEC_INTR_ENABLE0_REG_ADDR,&reg,1);
		berlin_cec_reg_read (cec_dev, CEC_INTR_ENABLE1_REG_ADDR, &reg, 1);
		reg |= ((intr>>8) & 0x00ff);
		berlin_cec_reg_write(cec_dev,CEC_INTR_ENABLE1_REG_ADDR,&reg,1);
	}
	else
	{
		berlin_cec_reg_read (cec_dev, CEC_INTR_ENABLE0_REG_ADDR, &reg, 1);
		reg &= ~(intr & 0x00ff);
		berlin_cec_reg_write(cec_dev,CEC_INTR_ENABLE0_REG_ADDR,&reg,1);
		berlin_cec_reg_read (cec_dev, CEC_INTR_ENABLE1_REG_ADDR, &reg, 1);
		reg &= ~((intr>>8) & 0x00ff);
		berlin_cec_reg_write(cec_dev,CEC_INTR_ENABLE1_REG_ADDR,&reg,1);
	}
	return 0;
}

/******************************************************************************
 * FUNCTION : Enables/Disables the given interrupt(s)
 * PARAMS   : *cec_dev - Pointer to CEC device object
 *		  : failMode - Failure mode (Tx or Rx mode)
 *		  : *pStsInfo- Pointer to return status information
 * RETURN   : 0, on success
 *		  : Error code, otherwise
 *****************************************************************************/
int berlin_cec_get_fail_status (struct cec_device_t *cec_dev,
							  berlin_cec_mode failMode, int *pStsInfo, u16 intr_status)
{
	if (failMode == BERLIN_CEC_MODE_TX)
	{
		if (intr_status & (BERLIN_CEC_INTR_TX_UNICAST_NOACK | BERLIN_CEC_INTR_TX_BRDCAST_NOACK))
		{
			*pStsInfo = CEC_TX_STATUS_NACK;
		}
		else if (intr_status & BERLIN_CEC_INTR_TX_COLLISION)
		{
			*pStsInfo = CEC_TX_STATUS_LOW_DRIVE;
		}
		else if (intr_status & BERLIN_CEC_INTR_TX_SFT_FAIL)
		{
			*pStsInfo = CEC_TX_STATUS_ARB_LOST;
		}
		else
			*pStsInfo = CEC_TX_STATUS_OK;
	}
	else if(failMode ==  BERLIN_CEC_MODE_RX)
	{
		if (intr_status & BERLIN_CEC_INTR_RX_LOW_DT_ERROR)
		{
			 pr_err("RX error : %d\n", BERLIN_CEC_INTR_RX_LOW_DT_ERROR);
			//*pStsInfo = BERLIN_CEC_STS_RX_FAIL_LOW_DB_TIME;
		}
		else if (intr_status & BERLIN_CEC_INTR_RX_HIGH_DT_ERROR)
		{
			 pr_err("RX error : %d\n", BERLIN_CEC_INTR_RX_HIGH_DT_ERROR);
			//*pStsInfo = BERLIN_CEC_STS_RX_FAIL_HIGH_DB_TIME;
		}
		else
		{
			//*pStsInfo = BERLIN_CEC_STS_RX_FAIL_INFO_NONE;
		}
	}
	return 0;
}
EXPORT_SYMBOL_GPL(berlin_cec_get_fail_status);

static inline unsigned short cec_read(struct cec_device_t *cec_dev, u32 reg)
{
	return readb_relaxed((reg << 2) + cec_dev->cec_virt_addr);
}

static inline void cec_write(struct cec_device_t *cec_dev, u32 reg, unsigned short val)
{
	writeb_relaxed(val, ((reg << 2) + cec_dev->cec_virt_addr));
}

/******************************************************************************
 * FUNCTION : Writes block of data starting at specified register address
 * PARAMS   : *cec_dev - Pointer to CEC device object
 *		  : pDataBuf - Pointer to data buffer
 *		  : length   - Length of data to be written
 * RETURN   : 0, on success
 *		  : Error code, otherwise
 *****************************************************************************/

int berlin_cec_reg_write (struct cec_device_t *cec_dev,
						 u32 startAddr, u8 *pDataBuf,
						 u32 length)
{
	u32  i;
	for (i = 0; i < length; i++)
	{
		cec_write(cec_dev, startAddr + i, *pDataBuf);
		pDataBuf++;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(berlin_cec_reg_write);

/******************************************************************************
 * FUNCTION : Reads block of data starting at specified register address
 * PARAMS   : *cec_dev - Pointer to CEC device object
 *		  : pDataBuf - Pointer to data buffer (Output)
 *		  : length   - Length of data to be read
 * RETURN   : 0, on success
 *		  : Error code, otherwise
 *****************************************************************************/
int berlin_cec_reg_read (struct cec_device_t *cec_dev,
						u32 startAddr, u8 *pDataBuf, u32 length)
{
	u32  i;
	for (i = 0; i < length; i++)
	{
		 pDataBuf[i] = cec_read(cec_dev, startAddr + i);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(berlin_cec_reg_read);

#endif /* _SYNA_CEC_PRV_C_ */
MODULE_LICENSE("GPL");
