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

float getpH(void);
float getEC(void);
float getTemperature(void);
#endif /* __SENSORS_H_ */
