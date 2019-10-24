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

#include "stm32wbxx_hal.h"
#include "cmsis_os.h"
#include "main.h"
#include "24xx256.h"

extern I2C_HandleTypeDef hi2c1;
extern osMutexId_t hi2c1_mx;
extern osEventFlagsId_t evt_id;

/*-----------------------------------------------------------------------------
 *  Function prototypes
 *-----------------------------------------------------------------------------*/
void Init_24xx256(void);
EEErr_t EE_Write8(uint8_t i2c_addr, uint16_t data_addr, uint8_t *data, uint8_t len); 
EEErr_t EE_Read8(uint8_t i2c_addr, uint16_t data_addr, uint8_t *data, uint8_t len);
EEErr_t EE_Write32(uint8_t i2c_addr, uint16_t data_addr, uint32_t *data, uint8_t len); 
EEErr_t EE_Read32(uint8_t i2c_addr, uint16_t data_addr, uint32_t *data, uint8_t len);
EEErr_t EE_WriteFloat(uint8_t i2c_addr, uint16_t data_addr, float *data, uint8_t len); 
EEErr_t EE_ReadFloat(uint8_t i2c_addr, uint16_t data_addr, float *data, uint8_t len);

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef * hi2c);
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef * hi2c);

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  Init_24xx256
 *  Description:  
 * =====================================================================================
 */
void Init_24xx256(void)
{

}		/* -----  end of function Init_24xx256  ----- */

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
    osEventFlagsClear(evt_id, EE_FLAG_I2C_WRITE_CPLT);
    
    i2c_status = HAL_I2C_Mem_Write_DMA( &hi2c1, (uint16_t) i2c_addr,
                                        data_addr, I2C_MEMADD_SIZE_16BIT,
                                        data, (uint16_t) len);
    if (i2c_status != HAL_OK)
    {
        status = EE_ERR_WRITE;
    }

    osDelay(50); 

    osEventFlagsWait(evt_id, EE_FLAG_I2C_WRITE_CPLT, osFlagsWaitAll, osWaitForever);

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

    osMutexAcquire(hi2c1_mx, osWaitForever);
    osEventFlagsClear(evt_id, EE_FLAG_I2C_READ_CPLT);

    if (HAL_I2C_Mem_Read_DMA( &hi2c1, (uint16_t) i2c_addr,
                              data_addr, I2C_MEMADD_SIZE_16BIT,
                              data, (uint16_t) len) != HAL_OK)
    {
        status = EE_ERR_READ;
        osMutexRelease(hi2c1_mx);
        return status;
    }

    osEventFlagsWait(evt_id, EE_FLAG_I2C_READ_CPLT, osFlagsWaitAll, osWaitForever);

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
       temp = (uint32_t) ((*(data_8+i) << 24) + (*(data_8+i+1) << 16) \
                       + (*(data_8+i+2) << 8) + (*(data_8+i+3))); 
       *(data+i) = temp;
    }

    return status;
}		/* -----  end of function EE_Read32  ------ */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  EE_WriteFloat
 *  Description:  
 * =====================================================================================
 */
EEErr_t EE_WriteFloat(uint8_t i2c_addr, uint16_t data_addr, float *data, uint8_t len)
{
    EEErr_t status = EE_OK;
    uint8_t data_8[sizeof(float)*16];

    for (uint8_t i=0; i < len; i++)
    {
       *(float*)(data_8+i*sizeof(float)) = *(data+i); 
    }

    status = EE_Write8(i2c_addr, data_addr, &data_8[0], sizeof(float)*len);

    return status;
}		/* -----  end of function EE_WriteFloat  ----- */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  EE_ReadFloat
 *  Description:  
 * =====================================================================================
 */
EEErr_t EE_ReadFloat(uint8_t i2c_addr, uint16_t data_addr, float *data, uint8_t len)
{
    EEErr_t status = EE_OK;
    uint8_t data_8[sizeof(float)*16];

    status = EE_Read8(i2c_addr, data_addr, &data_8[0], sizeof(float)*len);

    for (uint8_t i=0; i < len; i++)
    {
       *(data+i) = *(float*)(&data_8[i*sizeof(float)]); 
    }

    return status;
}		/* -----  end of function EE_ReadFloat  ------ */

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef * hi2c)
{
    osEventFlagsSet(evt_id, EE_FLAG_I2C_READ_CPLT);
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef * hi2c)
{
    osEventFlagsSet(evt_id, EE_FLAG_I2C_WRITE_CPLT);
}
