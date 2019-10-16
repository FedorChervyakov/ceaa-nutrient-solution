/*
 * =====================================================================================
 *
 *       Filename:  24xx256.c
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  16/10/19 18:07:06
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Fedor Chervyakov (), fchervyakov@gmail.com
 *   Organization:  
 *
 * =====================================================================================
 */

#include "24xx256.h"
#include "stm32wbxx_hal.h"
#include "cmsis.os"

extern I2C_HandleTypeDef hi2c1;
extern osMutexId_t hi2c1_mx;

/*-----------------------------------------------------------------------------
 *  Private function prototypes
 *-----------------------------------------------------------------------------*/
void 24xx256_Init(void);
uint8_t EE_Write(uint8_t i2c_addr, uint16_t data_addr, uint8_t *data, uint8_t len); 
uint8_t EE_Read(uint8_t i2c_addr, uint16_t data_addr, uint8_t *data, uint8_t len);

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  24xx256_Init
 *  Description:  
 * =====================================================================================
 */
void 24xx256_Init(void)
{

}		/* -----  end of function 24xx256_Init  ----- */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  EE_Write
 *  Description:  
 * =====================================================================================
 */
uint8_t EE_Write(uint8_t i2c_addr, uint16_t data_addr, uint8_t *data, uint8_t len)
{
    uint8_t status = EE_OK;

    return status;
}		/* -----  end of function EE_Write  ----- */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  EE_Read
 *  Description:  
 * =====================================================================================
 */
uint8_t EE_Read(uint8_t i2c_addr, uint16_t data_addr, uint8_t *data, uint8_t len)
{
    uint8_t status = EE_OK;

    return status;
}		/* -----  end of function EE_Read  ------ */
