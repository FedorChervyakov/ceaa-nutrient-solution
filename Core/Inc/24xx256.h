/*
 * =====================================================================================
 *
 *       Filename:  24xx256.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  16/10/19 18:18:25
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Fedor Chervyakov (), fchervyakov@gmail.com
 *   Organization:  
 *
 * =====================================================================================
 */

#ifndef __24xx256_H_
#define __24xx256_H_


/*-----------------------------------------------------------------------------
 *  Defines
 *-----------------------------------------------------------------------------*/
#define EE_I2C_ADDR            ((uint16_t) 0xA0)/* I2C address of the eeprom */
#define EE_I2C_TIMEOUT         ((uint16_t) 2500) /* I2C_CPLT timeout in ms   */

/*-----------------------------------------------------------------------------
 *  Status typedef
 *-----------------------------------------------------------------------------*/
typedef enum
{
    EE_OK,
    EE_ERR_READ,
    EE_ERR_WRITE,
    EE_ERR_I2C_TIMEOUT
} EEErr_t;

/*-----------------------------------------------------------------------------
 *  Exported function prototypes
 *-----------------------------------------------------------------------------*/
void Init_24xx256(void);
EEErr_t EE_Write8(uint8_t i2c_addr, uint16_t data_addr, uint8_t *data, uint8_t len); 
EEErr_t EE_Read8(uint8_t i2c_addr, uint16_t data_addr, uint8_t *data, uint8_t len);
EEErr_t EE_Write32(uint8_t i2c_addr, uint16_t data_addr, uint32_t *data, uint8_t len); 
EEErr_t EE_Read32(uint8_t i2c_addr, uint16_t data_addr, uint32_t *data, uint8_t len);
EEErr_t EE_WriteFloat(uint8_t i2c_addr, uint16_t data_addr, float *data, uint8_t len); 
EEErr_t EE_ReadFloat(uint8_t i2c_addr, uint16_t data_addr, float *data, uint8_t len);


#endif /* __24xx256_H_ */
