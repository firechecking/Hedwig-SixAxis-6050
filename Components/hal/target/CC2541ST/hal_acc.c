/**************************************************************************************************
  Filename:       hal_acc.c
  Revised:        $Date: 2013-03-26 07:47:25 -0700 (Tue, 26 Mar 2013) $
  Revision:       $Revision: 33597 $

  Description:    Driver for the Kionix KXTI9 Accelerometer.

  Copyright 2012-2013 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED ìAS ISî WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.

**************************************************************************************************/

/* ------------------------------------------------------------------------------------------------
*                                          Includes
* ------------------------------------------------------------------------------------------------
*/

#include "hal_acc.h"
#include "hal_sensor.h"
#include "hal_i2c.h"
#include "hal_board_cfg.h"

/* ------------------------------------------------------------------------------------------------
*                                           Constants
* ------------------------------------------------------------------------------------------------
*/
// Sensor I2C address
#define HAL_ACCEL_I2C_ADDRESS          0x68


#define HAL_GYRO_REG_WHOAMI                     0x75 // R/W

// Offset configuration registers
#define HAL_GYRO_REG_XOFFS_USRH                 0x0C // R/W
#define HAL_GYRO_REG_XOFFS_USRL                 0x0D // R/W
#define HAL_GYRO_REG_YOFFS_USRH                 0x0E // R/W
#define HAL_GYRO_REG_YOFFS_USRL                 0x0F // R/W
#define HAL_GYRO_REG_ZOFFS_USRH                 0x10 // R/W
#define HAL_GYRO_REG_ZOFFS_USRL                 0x11 // R/W

// Configuration registers
#define HAL_GYRO_REG_FIFO_EN                    0x23 // R/W
#define HAL_GYRO_REG_AUX_VDDIO                  0x13 // R/W
#define HAL_GYRO_REG_AUX_SLV_ADDR               0x14 // R/W
#define HAL_GYRO_REG_SMPLRT_DIV                 0x19 // R/W
#define HAL_GYRO_REG_DLPF_FS                    0x37 // R/W
#define HAL_GYRO_REG_INT_CFG                    0x17 // R/W
#define HAL_GYRO_REG_AUX_BURST_ADDR             0x18 // R/W
#define HAL_GYRO_REG_INT_STATUS                 0x58 // R

// Sensor data registers
#define HAL_GYRO_REG_TEMP_OUT_H                 0x41 // R
#define HAL_GYRO_REG_TEMP_OUT_L                 0x42 // R
#define HAL_GYRO_REG_GYRO_XOUT_H                0x43 // R
#define HAL_GYRO_REG_GYRO_XOUT_L                0x44 // R
#define HAL_GYRO_REG_GYRO_YOUT_H                0x45 // R
#define HAL_GYRO_REG_GYRO_YOUT_L                0x46 // R
#define HAL_GYRO_REG_GYRO_ZOUT_H                0x47 // R
#define HAL_GYRO_REG_GYRO_ZOUT_L                0x48 // R
#define HAL_GYRO_REG_Accel_XOUT_H               0x3B // R
#define HAL_GYRO_REG_Accel_XOUT_L               0x3C // R
#define HAL_GYRO_REG_Accel_YOUT_H               0x3D // R
#define HAL_GYRO_REG_Accel_YOUT_L               0x3E // R
#define HAL_GYRO_REG_Accel_ZOUT_H               0x3F // R
#define HAL_GYRO_REG_Accel_ZOUT_L               0x40 // R

// FIFO registers
#define HAL_GYRO_REG_FIFO_COUNTH                0x72 // R
#define HAL_GYRO_REG_FIFO_COUNTL                0x73 // R
#define HAL_GYRO_REG_FIFO_R                     0x74 // R

// User control and Power Management registers
#define HAL_GYRO_REG_USER_CTRL                  0x6 // R/W
#define HAL_GYRO_REG_PWR_MGM                    0x6B // R/W

/* GYRO Register Bit masks */
#define HAL_GYRO_REG_FIFO_EN_TEMP_OUT           0x80
#define HAL_GYRO_REG_FIFO_EN_GYRO_XOUT          0x40
#define HAL_GYRO_REG_FIFO_EN_GYRO_YOUT          0x20
#define HAL_GYRO_REG_FIFO_EN_GYRO_ZOUT          0x10
#define HAL_GYRO_REG_FIFO_EN_ACCEL_XOUT           0x08
#define HAL_GYRO_REG_FIFO_EN_ACCEL_YOUT           0x04
#define HAL_GYRO_REG_FIFO_EN_ACCEL_ZOUT           0x02
#define HAL_GYRO_REG_FIFO_EN_FIFO_FOOTER        0x01

#define HAL_GYRO_USER_CTRL_DMP_EN               0x80
#define HAL_GYRO_USER_CTRL_FIFO_EN              0x40
#define HAL_GYRO_USER_CTRL_ACCEL_IF_EN            0x20
#define HAL_GYRO_USER_CTRL_ACCEL_IF_RST_EN        0x08
#define HAL_GYRO_USER_CTRL_DMP_RST              0x04
#define HAL_GYRO_USER_CTRL_FIFO_RST             0x02
#define HAL_GYRO_USER_CTRL_GYRO_RST             0x01

#define HAL_GYRO_PWR_MGM_H_RESET                0x80
#define HAL_GYRO_PWR_MGM_SLEEP                  0x4B
#define HAL_GYRO_PWR_MGM_STBY_XG                0x04
#define HAL_GYRO_PWR_MGM_STBY_YG                0x02
#define HAL_GYRO_PWR_MGM_STBY_ZG                0x01
#define HAL_GYRO_PWR_MGM_STBY_ALL               0x07  // All axes

// Clock select
#define HAL_GYRO_PWR_MGM_CLOCK_INT_OSC          0x00
#define HAL_GYRO_PWR_MGM_CLOCK_PLL_X            0x01
#define HAL_GYRO_PWR_MGM_CLOCK_PLL_Y            0x02
#define HAL_GYRO_PWR_MGM_CLOCK_PLL_Z            0x03
#define HAL_GYRO_PWR_MGM_CLOCK_PLL_32768KHZ     0x04
#define HAL_GYRO_PWR_MGM_CLOCK_PLL_19_2MHZ      0x05
#define HAL_GYRO_PWR_MGM_CLOCK_STOP             0x07

#define ACC_REG_ADDR_CTRL_REG1 0x1C  //º”ÀŸ∂»¥´∏–∆˜¡ø≥Ã…Ë÷√


#define ACC_REG_CTRL_ON_2G                0x01
#define ACC_REG_CTRL_OFF_2G               0
#define ACC_REG_CTRL_ON_4G                0x09
#define ACC_REG_CTRL_OFF_4G               0
#define ACC_REG_CTRL_ON_8G                0x11
#define ACC_REG_CTRL_OFF_8G               0
#define ACC_REG_CTRL_ON_16G               0x19
#define ACC_REG_CTRL_OFF_16G              0

#define HAL_ACCEL_DATA_SIZE 6
/* ------------------------------------------------------------------------------------------------
*                                           Typedefs
* ------------------------------------------------------------------------------------------------
*/

/* ------------------------------------------------------------------------------------------------
*                                           Macros
* ------------------------------------------------------------------------------------------------
*/

/* ------------------------------------------------------------------------------------------------
*                                           Local Functions
* ------------------------------------------------------------------------------------------------
*/
static void HalAccSelect(void);

/* ------------------------------------------------------------------------------------------------
*                                           Local Variables
* ------------------------------------------------------------------------------------------------
*/
static uint8 accSensorConfig;
static uint8 accSensorOff;
static uint8 accRange;
/**************************************************************************************************
* @fn          HalAccInit
*
* @brief       This function initializes the HAL Accelerometer abstraction layer.
*
* @return      None.
*/
void HalAccInit(void)
{
  HalAccSetRange(HAL_ACC_RANGE_8G);
//  HalAccSelect();
//  HalSensorWriteReg(ACC_REG_ADDR_CTRL_REG1, &accSensorConfig, sizeof(accSensorConfig));
//  ST_HAL_DELAY(180);
}
/**************************************************************************************************
* @fn          HalAccSetRange
*
* @brief       Set the range of the accelerometer
*
* @param       range: HAL_ACC_RANGE_2G, HAL_ACC_RANGE_4G, HAL_ACC_RANGE_8G
*
* @return      None
*/
void HalAccSetRange(uint8 range)
{
  accRange = range;
  
  switch (accRange)
  {
  case HAL_ACC_RANGE_2G:
    accSensorConfig = ACC_REG_CTRL_ON_2G;
    accSensorOff = ACC_REG_CTRL_OFF_2G;
    break;
  case HAL_ACC_RANGE_4G:
    accSensorConfig = ACC_REG_CTRL_ON_4G;
    accSensorOff = ACC_REG_CTRL_OFF_4G;
    break;
  case HAL_ACC_RANGE_8G:
    accSensorConfig = ACC_REG_CTRL_ON_8G;
    accSensorOff = ACC_REG_CTRL_OFF_8G;
    break;
  case HAL_ACC_RANGE_16G:
  //‘⁄tiµƒGATT server÷–£¨≤ªƒ‹…Ë÷√16G¡ø≥Ã
    accSensorConfig = ACC_REG_CTRL_ON_16G;
    accSensorOff = ACC_REG_CTRL_OFF_16G;
    break;
  default:
    // Should not get here
    break;
  }

}


/**************************************************************************************************
* @fn          HalAccRead
*
* @brief       Read data from the accelerometer - X, Y, Z - 3 bytes
*
* @return      TRUE if valid data, FALSE if not
*/
bool HalAccRead(uint8 *pBuf )
{
  uint8 buf[HAL_ACCEL_DATA_SIZE];
  bool success;
  
  // Select this sensor
  HalAccSelect();
  //…Ë÷√¡ø≥Ã
  HalSensorWriteReg(ACC_REG_ADDR_CTRL_REG1, &accSensorConfig, sizeof(accSensorConfig));
  // Wait for measurement ready (appx. 1.45 ms)
  ST_HAL_DELAY(180);
  
  // Read the three registers
  //success = HalSensorReadReg(HAL_GYRO_REG_GYRO_XOUT_H,buf,HAL_ACCEL_DATA_SIZE);
  success = HalSensorReadReg(HAL_GYRO_REG_Accel_XOUT_H,buf,HAL_ACCEL_DATA_SIZE);
  if (success)
  {
    // Result in LE
      pBuf[0] = buf[0];
      pBuf[1] = buf[2];
      pBuf[2] = buf[4];
  }
  // Put Sensor to sleep
   // HalSensorWriteReg(HAL_GYRO_REG_PWR_MGM,HAL_GYRO_PWR_MGM_SLEEP,1);

  return success;
}


///**************************************************************************************************
// * @fn          HalAccTest
// *
// * @brief       Run a sensor self-test
// *
// * @return      TRUE if passed, FALSE if failed
// */
//bool HalAccTest(void)
//{
//  uint8 val;
//
//  // Select this sensor on the I2C bus
//  HalAccSelect();
//
//  // Check the DCST_RESP (pattern 0x55)
//  ST_ASSERT(HalSensorReadReg(ACC_REG_ADDR_DCST_RESP, &val, 1));
//  ST_ASSERT(val==0x55);
//
//  // Check the DCST_RESP (pattern 0xAA)
//  val = 0x10;     // Sets the DCST bit
//  ST_ASSERT(HalSensorWriteReg(ACC_REG_ADDR_CTRL_REG3, &val, 1));
//  ST_ASSERT(HalSensorReadReg(ACC_REG_ADDR_DCST_RESP, &val, 1));
//  ST_ASSERT(val==0xAA);
//
//  // Check the WHO AM I register
//  ST_ASSERT(HalSensorReadReg(ACC_REG_ADDR_WHO_AM_I, &val, 1));
//  ST_ASSERT(val==REG_VAL_WHO_AM_I);
//
//  return TRUE;
//}

/* ------------------------------------------------------------------------------------------------
*                                           Private functions
* -------------------------------------------------------------------------------------------------
*/

/**************************************************************************************************
* @fn          HalAccSelect
*
* @brief       Select the accelerometer on the I2C-bus
*
* @return
*/
static void HalAccSelect(void)
{
  //Set up I2C that is used to communicate with MPU-6050 Accelerometer Sensor
  HalI2CInit(HAL_ACCEL_I2C_ADDRESS,   i2cClock_533KHZ);
}

/*  Conversion algorithm for X, Y, Z
 *  ================================
 *
float calcAccel(int8 rawX)
{
    float v;

    //-- calculate acceleration, unit G, range -2, +2
    v = (rawX * 1.0) / (256/4);

    return v;
}
*/

/*********************************************************************
*********************************************************************/
