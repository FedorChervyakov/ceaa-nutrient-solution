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
#include "cmsis_os.h"

extern I2C_HandleTypeDef hi2c1;
extern osMutexId_t hi2c1_mx;
extern osEventFlagsId_t evt_id;

/*-----------------------------------------------------------------------------
 *  Private function prototypes
 *-----------------------------------------------------------------------------*/
void 24xx256_Init(void);
EEErr_t EE_Write8(uint8_t i2c_addr, uint16_t data_addr, uint8_t *data, uint8_t len); 
EEErr_t EE_Read8(uint8_t i2c_addr, uint16_t data_addr, uint8_t *data, uint8_t len);
EEErr_t EE_Write32(uint8_t i2c_addr, uint16_t data_addr, uint32_t *data, uint8_t len); 
EEErr_t EE_Read32(uint8_t i2c_addr, uint16_t data_addr, uint32_t *data, uint8_t len);

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef * hi2c);
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef * hi2c);
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
 *         Name:  EE_Write8
 *  Description:  
 * =====================================================================================
 */
EEErr_t EE_Write8(uint8_t i2c_addr, uint16_t data_addr, uint8_t *data, uint8_t len)
{
    EEErr_t status = EE_OK;
    HAL_StatusTypeDef i2c_status = HAL_OK;

    osMutexAcquire(hi2c1_mx, osWaitForever);
    
    i2c_status = HAL_I2C_Mem_Write_DMA( &hi2c1, (uint16_t) i2c_addr,
                                        data_addr, EE_MEM_ADDR_SIZE,
                                        data, (uint16_t) len);
    if (i2c_status != HAL_OK)
    {
        status = EE_ERR_WRITE;
    }

    osEventFlagsWait(evt_id, EE_FLAG_I2C_WRITE_CPLT, oFlagsWaitAll, osWaitForever);

    osMutexRelease(hi2c1_mx);

    return status;
}		/* -----  end of function EE_Write8  ----- */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  EE_Read8
 *  Description:  
 * =====================================================================================
 */
EEErr_t EE_Read8(uint8_t i2c_addr, uint16_t data_addr, uint8_t *data, uint8_t len)
{
    EEErr_t status = EE_OK;
    HAL_StatusTypeDef i2c_status = HAL_OK;

    osMutexAcquire(hi2c1_mx, osWaitForever);

    i2c_status = HAL_I2C_Mem_Read_DMA( &hi2c1, (uint16_t) i2c_addr,
                                        data_addr, EE_MEM_ADDR_SIZE,
                                        data, (uint16_t) len);
    if (i2c_status != HAL_OK)
    {
        status = EE_ERR_READ;
    }

    osEventFlagsWait(evt_id, EE_FLAG_I2C_READ_CPLT, oFlagsWaitAll, osWaitForever);

    osMutexRelease(hi2c1_mx);

    return status;
}		/* -----  end of function EE_Read8  ------ */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  EE_Write32
 *  Description:  
 * =====================================================================================
 */
EEErr_t EE_Write32(uint8_t i2c_addr, uint16_t data_addr, uint32_t *data, uint8_t len)
{
    EEErr_t status = EE_OK;
    uint8_t data_8[64] = {0};

    for (uint8_t i=0; i < len; i++)
    {
       data_8[4*i] = (uint8_t) (*(data+i) >> 24); 
       data_8[4*i+1] = (uint8_t) (*(data+i) >> 16); 
       data_8[4*i+2] = (uint8_t) (*(data+i) >> 8); 
       data_8[4*i+3] = (uint8_t) (*(data+i)); 
    }

    status = EE_Write8(i2c_addr, data_addr, &data_8[0], 4*len);

    return status;
}		/* -----  end of function EE_Write32  ----- */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  EE_Read32
 *  Description:  
 * =====================================================================================
 */
EEErr_t EE_Read32(uint8_t i2c_addr, uint16_t data_addr, uint32_t *data, uint8_t len)
{
    EEErr_t status = EE_OK;
    uint8_t data_8[64] = {0};
    uint32_t temp = 0;

    status = EE_Read8(i2c_addr, data_addr, &data_8[0], 4*len);

    for (uint8_t i=0; i < len; i++)
    {
       temp = (uint32_t) ((*(data_8+i) << 24) + (*(data_8+i+1) << 16)
                       + (*(data_8+i+2) << 8) + (*(data_8+i+3)); 
       *(data+i) = temp;
    }

    return status;
}		/* -----  end of function EE_Read32  ------ */

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef * hi2c)
{
    osEventFlagsSet(evt_id, EE_FLAG_I2C_READ_CPLT);
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef * hi2c)
{
    osEventFlagsSet(evt_id, EE_FLAG_I2C_WRITE_CPLT);
}
