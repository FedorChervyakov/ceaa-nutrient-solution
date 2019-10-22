/*
 * =====================================================================================
 *
 *       Filename:  sensors.h
 *
 *    Description:  Header 
 *
 *        Version:  1.0
 *        Created:  15/10/19 00:55:41
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Fedor Chervyakov (), fchervyakov@gmail.com
 *   Organization:  
 *
 * =====================================================================================
 */

#ifndef __SENSORS_H_
#define __SENSORS_H_

/*-----------------------------------------------------------------------------
 *  Header includes
 *-----------------------------------------------------------------------------*/
#include "main.h"

/*-----------------------------------------------------------------------------
 *  Function prototypes
 *-----------------------------------------------------------------------------*/
void sensors_Init(void);
float getpH(void);
float getEC(void);
float getTemperature(void);

/*-----------------------------------------------------------------------------
 *  Private defines
 *-----------------------------------------------------------------------------*/
#define SENSORS_FLAG_TEMPERATURE_READY  ((uint32_t) 0x0001)
#define SENSORS_FLAG_PH_READY           ((uint32_t) 0x0010)
#define SENSORS_FLAG_EC_READY           ((uint32_t) 0x0100)
#define SENSORS_FLAG_ADC_READY		    ((uint32_t) 0x1000)

#define SENSORS_FLAG_PH_CALIB_BEGIN     ((uint32_t) 0x0002) 
#define SENSORS_FLAG_EC_CALIB_BEGIN     ((uint32_t) 0x0020) 

#define PH_SLOPE_EE_ADDR                ((uint16_t) 0x1000)
#define PH_INTCPT_EE_ADDR               ((uint16_t) 0x1004)
#define EC_SLOPE_EE_ADDR                ((uint16_t) 0x1008)
#define EC_INTCPT_EE_ADDR               ((uint16_t) 0x100C)

#endif /* __SENSORS_H_ */
