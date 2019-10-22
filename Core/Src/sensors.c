/*
 * =====================================================================================
 *
 *       Filename:  sensors.c
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  15/10/19 01:08:21
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Fedor Chervyakov (), fchervyakov@gmail.com
 *   Organization:  
 *
 * =====================================================================================
 */

#include "sensors.h"
#include "cmsis_os.h"
#include "stm32wbxx_hal.h"
#include "24xx256.h"
#include "sw_led.h"

extern ADC_HandleTypeDef hadc1;
extern osEventFlagsId_t sw_evt_id;

/*-----------------------------------------------------------------------------
 *  Private defines
 *-----------------------------------------------------------------------------*/
/* Size of ADC buffer */
#define ADC_BUFFERSIZE          ((uint32_t) 4)
#define ADC_COMPLETE_FLAG       ((uint32_t) 1)
#define ADC_BEGIN_FLAG          ((uint32_t) 2)
#define ADC_DELAY               ((uint32_t) 1000)   /* 1 Hz */
#define DIGITAL_SCALE_12BITS    ((uint16_t) 0x0FFF)
#define USER_TIMEOUT            ((uint32_t) 30000) /* 30 seconds */

#define LM35_AMP_GAIN             ((float) 6.1)

#define PH_CAL_SOLUTION_1       ((float) 7.0)   /* pH of a calibration solution 1 */
#define PH_CAL_SOLUTION_2       ((float) 4.8)   /* pH of a calibration solution 2 */

#define EC_CAL_SOLUTION_1       ((float) 1.413)   /* EC mS/cm of a calibration solution 1 */
#define EC_CAL_SOLUTION_2       ((float) 12.88)   /* EC mS/cm of a calibration solution 2 */

/*-----------------------------------------------------------------------------
 *  Private macro
 *-----------------------------------------------------------------------------*/
/**
  * @brief  Macro to calculate the voltage (unit: mVolt)
  *         corresponding to a ADC conversion data (unit: digital value).
  * @note   ADC measurement data must correspond to a resolution of 12bits
  *         (full scale digital value 4095). If not the case, the data must be
  *         preliminarily rescaled to an equivalent resolution of 12 bits.
  * @note   Analog reference voltage (Vref+) must be known from
  *         user board environment.
  * @param  __VREFANALOG_VOLTAGE__ Analog reference voltage (unit: mV)
  * @param  __ADC_DATA__ ADC conversion data (resolution 12 bits)
  *                       (unit: digital value).
  * @retval ADC conversion data equivalent voltage value (unit: mVolt)
  */
#define __ADC_CALC_DATA_VOLTAGE(__VREFANALOG_VOLTAGE__, __ADC_DATA__)       \
   (__ADC_DATA__) * (__VREFANALOG_VOLTAGE__) / DIGITAL_SCALE_12BITS

/*-----------------------------------------------------------------------------
 *  Private variables
 *-----------------------------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
osThreadId_t calcTTaskHandle;
uint32_t calcTTaskBuffer[ 128 ];
osStaticThreadDef_t calcTTaskControlBlock;
osThreadId_t calcPHTaskHandle;
uint32_t calcPHTaskBuffer[ 128 ];
osStaticThreadDef_t calcPHTaskControlBlock;
osThreadId_t calcECTaskHandle;
uint32_t calcECTaskBuffer[ 128 ];
osStaticThreadDef_t calcECTaskControlBlock;
osThreadId_t readADCTaskHandle;
uint32_t readADCTaskBuffer[ 128 ];
osStaticThreadDef_t readADCTaskControlBlock;
osThreadId_t calibratePHTaskHandle;
uint32_t calibratePHTaskBuffer[ 128 ];
osStaticThreadDef_t calibratePHTaskControlBlock;
osThreadId_t calibrateECTaskHandle;
uint32_t calibrateECTaskBuffer[ 128 ];
osStaticThreadDef_t calibrateECTaskControlBlock;

osEventFlagsId_t sens_evt_id;

static volatile float ch1_mv;
static volatile float ch2_mv;
static volatile float ch3_mv;

static volatile float pH;
static volatile float EC;
static volatile float temperature;

static float pH_slope;
static float pH_intercept;

static float EC_slope;
static float EC_intercept;

/*-----------------------------------------------------------------------------
 *  Private function prototypes
 *-----------------------------------------------------------------------------*/
int8_t readStoredCalibration(void);
int8_t storePHCalibration(void);
int8_t storeECCalibration(void);

static void calibratePH(void *argument);
static void calibrateEC(void *argument);

static void calcT(void *argument);
static void calcPH(void *argument);
static void calcEC(void *argument);
static void readADC(void *argument);

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  sensors_Init
 *  Description:  
 * =====================================================================================
 */
void sensors_Init(void)
{
  /* definition and creation of sensors' eventFlags */
  sens_evt_id = osEventFlagsNew(NULL);

  readStoredCalibration();

  /* definition and creation of readADCTask */
  const osThreadAttr_t readADCTask_attributes = {
    .name = "readADCTask",
    .stack_mem = &readADCTaskBuffer[0],
    .stack_size = sizeof(readADCTaskBuffer),
    .cb_mem = &readADCTaskControlBlock,
    .cb_size = sizeof(readADCTaskControlBlock),
    .priority = (osPriority_t) osPriorityAboveNormal,
  };

  readADCTaskHandle = osThreadNew(readADC, NULL, &readADCTask_attributes);
  /* definition and creation of calcTTask */
  const osThreadAttr_t calcTTask_attributes = {
    .name = "calcTTask",
    .stack_mem = &calcTTaskBuffer[0],
    .stack_size = sizeof(calcTTaskBuffer),
    .cb_mem = &calcTTaskControlBlock,
    .cb_size = sizeof(calcTTaskControlBlock),
    .priority = (osPriority_t) osPriorityNormal,
  };
  calcTTaskHandle = osThreadNew(calcT, NULL, &calcTTask_attributes);

  /* definition and creation of calcPHTask */
  const osThreadAttr_t calcPHTask_attributes = {
    .name = "calcPHTask",
    .stack_mem = &calcPHTaskBuffer[0],
    .stack_size = sizeof(calcPHTaskBuffer),
    .cb_mem = &calcPHTaskControlBlock,
    .cb_size = sizeof(calcPHTaskControlBlock),
    .priority = (osPriority_t) osPriorityNormal,
  };
  calcPHTaskHandle = osThreadNew(calcPH, NULL, &calcPHTask_attributes);

  /* definition and creation of calcECTask */
  const osThreadAttr_t calcECTask_attributes = {
    .name = "calcECTask",
    .stack_mem = &calcECTaskBuffer[0],
    .stack_size = sizeof(calcECTaskBuffer),
    .cb_mem = &calcECTaskControlBlock,
    .cb_size = sizeof(calcECTaskControlBlock),
    .priority = (osPriority_t) osPriorityNormal,
  };
  calcECTaskHandle = osThreadNew(calcEC, NULL, &calcECTask_attributes);
  
  /* definition and creation of calibratePHTask */
  const osThreadAttr_t calibratePHTask_attributes = {
    .name = "calibratePHTask",
    .stack_mem = &calibratePHTaskBuffer[0],
    .stack_size = sizeof(calibratePHTaskBuffer),
    .cb_mem = &calibratePHTaskControlBlock,
    .cb_size = sizeof(calibratePHTaskControlBlock),
    .priority = (osPriority_t) osPriorityNormal,
  };
  calibratePHTaskHandle = osThreadNew(calibratePH, NULL, &calibratePHTask_attributes);

  /* definition and creation of calibrateECTask */
  const osThreadAttr_t calibrateECTask_attributes = {
    .name = "calibrateECTask",
    .stack_mem = &calibrateECTaskBuffer[0],
    .stack_size = sizeof(calibrateECTaskBuffer),
    .cb_mem = &calibrateECTaskControlBlock,
    .cb_size = sizeof(calibrateECTaskControlBlock),
    .priority = (osPriority_t) osPriorityNormal,
  };
  calibrateECTaskHandle = osThreadNew(calibrateEC, NULL, &calibrateECTask_attributes);
}

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  readStoredCalibration
 *  Description:  
 * =====================================================================================
 */
int8_t readStoredCalibration ( void )
{
    EEErr_t eeerr = EE_OK;

    eeerr = EE_ReadFloat(EE_I2C_ADDR, PH_SLOPE_EE_ADDR, &pH_slope, 1); 
    if (eeerr != EE_OK)
    {
        return SENSORS_CAL_READ_FAIL;
    }

    eeerr = EE_ReadFloat(EE_I2C_ADDR, PH_INTCPT_EE_ADDR, &pH_intercept, 1); 
    if (eeerr != EE_OK)
    {
        return SENSORS_CAL_READ_FAIL;
    }

    eeerr = EE_ReadFloat(EE_I2C_ADDR, EC_SLOPE_EE_ADDR, &EC_slope, 1); 
    if (eeerr != EE_OK)
    {
        return SENSORS_CAL_READ_FAIL;
    }

    eeerr = EE_ReadFloat(EE_I2C_ADDR, EC_INTCPT_EE_ADDR, &EC_intercept, 1); 
    if (eeerr != EE_OK)
    {
        return SENSORS_CAL_READ_FAIL;
    }

    return SENSORS_OK;
}		/* -----  end of function readStoredCalibration  ----- */


/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  storePHCalibration
 *  Description:  
 * =====================================================================================
 */
int8_t storePHCalibration(void)
{
    EEErr_t eeerr = EE_OK;

    eeerr = EE_WriteFloat(EE_I2C_ADDR, PH_SLOPE_EE_ADDR, &pH_slope, 1); 
    if (eeerr != EE_OK)
    {
        return SENSORS_CAL_WRITE_FAIL;
    }

    eeerr = EE_WriteFloat(EE_I2C_ADDR, PH_INTCPT_EE_ADDR, &pH_intercept, 1); 
    if (eeerr != EE_OK)
    {
        return SENSORS_CAL_WRITE_FAIL;
    }

    return SENSORS_OK;
}

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  storeECCalibration
 *  Description:  
 * =====================================================================================
 */
int8_t storeECCalibration(void)
{
    EEErr_t eeerr = EE_OK;

    eeerr = EE_WriteFloat(EE_I2C_ADDR, EC_SLOPE_EE_ADDR, &EC_slope, 1); 
    if (eeerr != EE_OK)
    {
        return SENSORS_CAL_WRITE_FAIL;
    }

    eeerr = EE_WriteFloat(EE_I2C_ADDR, EC_INTCPT_EE_ADDR, &EC_intercept, 1); 
    if (eeerr != EE_OK)
    {
        return SENSORS_CAL_WRITE_FAIL;
    }

    return SENSORS_OK;

}

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  calibratePH
 *  Description:  
 * =====================================================================================
 */
static void calibratePH (void *argument)
{

    float v_s1 = 0;
    float v_s2 = 0;

    for (;;)
    {
        osEventFlagsWait( sw_evt_id, BT1_VERY_LONG_PRESS, 
                          osFlagsWaitAll, osWaitForever);
    
        v_s1 = 0;
        v_s2 = 0;

        LED_Blue(FAST_TOGGLING);

        if (osEventFlagsWait( sw_evt_id, BT1_LONG_PRESS,
                              osFlagsWaitAll, USER_TIMEOUT) == osErrorTimeout)
        {
            LED_Blue(OFF);
            continue;
        }
        
        LED_Blue(SLOW_TOGGLING);

        osDelay(5000);

        osThreadFlagsSet(readADCTaskHandle, ADC_BEGIN_FLAG);

        osEventFlagsWait( sens_evt_id, SENSORS_FLAG_ADC_READY,
                          osFlagsWaitAll, osWaitForever);

        v_s1 = ch2_mv;

        LED_Blue(FAST_TOGGLING);

        if (osEventFlagsWait( sw_evt_id, BT1_LONG_PRESS,
                              osFlagsWaitAll, USER_TIMEOUT) == osErrorTimeout)
        {
            LED_Blue(OFF);
            continue;
        }

        LED_Blue(SLOW_TOGGLING);

        osDelay(5000);

        osThreadFlagsSet(readADCTaskHandle, ADC_BEGIN_FLAG);

        osEventFlagsWait( sens_evt_id, SENSORS_FLAG_ADC_READY,
                          osFlagsWaitAll, osWaitForever);

        v_s2 = ch2_mv;

        LED_Blue(OFF);

        LED_Green(FAST_TOGGLING);

        pH_slope = (PH_CAL_SOLUTION_1 - PH_CAL_SOLUTION_1) / (v_s1 - v_s2);
        pH_intercept = (v_s1 * pH_slope) - PH_CAL_SOLUTION_1;

        if (storePHCalibration() != SENSORS_OK)
        {
            LED_Green(OFF);

            LED_Red(ON);    
            osDelay(250);
            LED_Red(OFF);    
            osDelay(250);
            LED_Red(ON);    
            osDelay(250);
            LED_Red(OFF);    
            osDelay(250);
            LED_Red(ON);    
            osDelay(250);
            LED_Red(OFF);    
            osDelay(250);
        }
        else
        {

            LED_Green(OFF);

            LED_Green(ON);    
            osDelay(250);
            LED_Green(OFF);    
            osDelay(250);
            LED_Green(ON);    
            osDelay(250);
            LED_Green(OFF);    
            osDelay(250);
            LED_Green(ON);    
            osDelay(250);
            LED_Green(OFF);    
            osDelay(250);
        }
    }
}		/* -----  end of function calibratePH  ----- */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  calibrateEC
 *  Description:  
 * =====================================================================================
 */
static void calibrateEC (void *argument)
{
    for (;;)
    {
        osEventFlagsWait( sw_evt_id, BT2_VERY_LONG_PRESS, 
                          osFlagsWaitAll, osWaitForever);

    }
}		/* -----  end of function calibrateEC  ----- */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  calcT
 *  Description:  
 * =====================================================================================
 */
static void calcT(void *argument)
{
    for (;;)
    {
        osEventFlagsWait(sens_evt_id, SENSORS_FLAG_ADC_READY, osFlagsWaitAll, osWaitForever);
        temperature = 0.1 * ch1_mv / LM35_AMP_GAIN;
	    osEventFlagsSet(sens_evt_id, SENSORS_FLAG_TEMPERATURE_READY);
    }   
}

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  calcPH
 *  Description:  
 * =====================================================================================
 */
static void calcPH(void *argument)
{
    for (;;)
    {
        osEventFlagsWait(sens_evt_id, SENSORS_FLAG_TEMPERATURE_READY, 
			 osFlagsWaitAll, osWaitForever);
        pH = pH_slope * ch2_mv + pH_intercept;
	    osEventFlagsSet(sens_evt_id, SENSORS_FLAG_PH_READY);
    }   
}

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  calcEC
 *  Description:  
 * =====================================================================================
 */
static void calcEC(void *argument)
{
    for (;;)
    {
        osEventFlagsWait(sens_evt_id, SENSORS_FLAG_TEMPERATURE_READY, 
			 osFlagsWaitAll, osWaitForever);
    
        EC = EC_slope * ch3_mv + EC_intercept;
	    osEventFlagsSet(sens_evt_id, SENSORS_FLAG_EC_READY);
    }   
}

/**
* @brief Function implementing the readADCTask thread.
* @param argument: Not used
* @retval None
*/
static void readADC(void *argument)
{
  /* Infinite loop */
  uint16_t uhADCxConvertedData[ADC_BUFFERSIZE];
  uint16_t int_ref;
  /* Infinite loop */
  for(;;)
  {
    /*## Wait for ADC begin flag #############################################*/
    osThreadFlagsWait(ADC_BEGIN_FLAG, osFlagsWaitAll, ADC_DELAY);
      
    /*## Start ADC conversions ###############################################*/
    /* Clear ADC conversion flag */
    osThreadFlagsClear(ADC_COMPLETE_FLAG);

    /* Start ADC conversion with DMA */
    if ( HAL_ADC_Start_DMA(&hadc1, (uint32_t *) uhADCxConvertedData, ADC_BUFFERSIZE) != HAL_OK)
    {
        /* ADC conversion start error */
        Error_Handler();
    }

    /* Wait till conversion is done */
    osThreadFlagsWait(ADC_COMPLETE_FLAG, osFlagsWaitAny, osWaitForever);

    /*## Stop ADC conversions ################################################*/
    /* Stop ADC conversion with DMA */
    if ( HAL_ADC_Stop_DMA(&hadc1) != HAL_OK)
    {
        /* ADC conversion stop error */
        Error_Handler();
    }

    /*## Compute voltages from adc readings ##################################*/
    int_ref = __LL_ADC_CALC_VREFANALOG_VOLTAGE(uhADCxConvertedData[0],
                            ADC_RESOLUTION_12B);

    ch1_mv = __ADC_CALC_DATA_VOLTAGE((float) int_ref,
                            (float) uhADCxConvertedData[1]);
    ch2_mv = __ADC_CALC_DATA_VOLTAGE((float) int_ref,
                            (float) uhADCxConvertedData[2]);
    ch3_mv = __ADC_CALC_DATA_VOLTAGE((float) int_ref,
                            (float) uhADCxConvertedData[3]);

    /*## Set ADC ready event flag #############################################*/
    osEventFlagsSet(sens_evt_id, SENSORS_FLAG_ADC_READY);
  }
}

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  getpH
 *  Description:  
 * =====================================================================================
 */
float getpH ( void )
{
    return pH;
}		/* -----  end of function getpH  ----- */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  getEC
 *  Description:  
 * =====================================================================================
 */
float getEC ( void )
{
    return EC;
}		/* -----  end of function getEC  ----- */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  getTemperature
 *  Description:  
 * =====================================================================================
 */
float getTemperature ( void )
{
    return temperature;
}		/* -----  end of function getTemperature  ----- */

/**
  * @brief  ADC conversion completed callback
  * @param  hadc: ADC handle
  * @retval None
  */
void HAL_ADC_ConvCpltCallback ( ADC_HandleTypeDef *hadc)
{
    /* Set event flag to indicate ADC completion */
    osThreadFlagsSet(readADCTaskHandle, ADC_COMPLETE_FLAG);
}		/* -----  end of function HAL_ADC_ConvCpltCallback  ----- */
