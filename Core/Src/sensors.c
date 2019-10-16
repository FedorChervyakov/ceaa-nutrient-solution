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

/*-----------------------------------------------------------------------------
 *  Private defines
 *-----------------------------------------------------------------------------*/
/* Size of ADC buffer */
#define ADC_BUFFERSIZE          ((uint32_t) 4)
#define ADC_COMPLETE_FLAG       ((uint32_t) 1)
#define ADC_DELAY               ((uint32_t) 1000)   /* 1 Hz */
#define DIGITAL_SCALE_12BITS    ((uint16_t) 0x0FFF)

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
extern ADC_HandleTypeDef hadc1;

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

osEventFlagsId_t sens_evt_id;

static volatile float ch1_mv;
static volatile float ch2_mv;
static volatile float ch3_mv;

static volatile float pH;
static volatile float EC;
static volatile float temperature;

/*-----------------------------------------------------------------------------
 *  Private function prototypes
 *-----------------------------------------------------------------------------*/
void calcT(void *argument);
void calcPH(void *argument);
void calcEC(void *argument);
void readADC(void *argument);

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
}

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  calcT
 *  Description:  
 * =====================================================================================
 */
void calcT(void *argument)
{
    for (;;)
    {
        osEventFlagsWait(sens_evt_id, SENSORS_FLAG_ADC_READY, osFlagsWaitAll, osWaitForever);
        temperature = 0.1 * ch1_mv;
	    osEventFlagsSet(sens_evt_id, SENSORS_FLAG_TEMPERATURE_READY);
    }   
}

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  calcPH
 *  Description:  
 * =====================================================================================
 */
void calcPH(void *argument)
{
    for (;;)
    {
        osEventFlagsWait(sens_evt_id, SENSORS_FLAG_TEMPERATURE_READY, 
			 osFlagsWaitAll, osWaitForever);
        pH = ch2_mv;
	    osEventFlagsSet(sens_evt_id, SENSORS_FLAG_PH_READY);
    }   
}

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  calcEC
 *  Description:  
 * =====================================================================================
 */
void calcEC(void *argument)
{
    for (;;)
    {
        osEventFlagsWait(sens_evt_id, SENSORS_FLAG_TEMPERATURE_READY, 
			 osFlagsWaitAll, osWaitForever);
    
        EC = ch3_mv;
	    osEventFlagsSet(sens_evt_id, SENSORS_FLAG_EC_READY);
    }   
}

/**
* @brief Function implementing the readADCTask thread.
* @param argument: Not used
* @retval None
*/
void readADC(void *argument)
{
  /* Infinite loop */
  uint16_t uhADCxConvertedData[ADC_BUFFERSIZE];
  uint16_t int_ref;
  /* Infinite loop */
  for(;;)
  {
    /*## Start ADC conversions ###############################################*/
    /* Clear ADC conversion flag */
    osThreadFlagsClear(ADC_COMPLETE_FLAG);

//    osEventFlagsClear(sens_evt_id, SENSORS_FLAG_ADC_READY | SENSORS_FLAG_PH_READY
//                      | SENSORS_FLAG_TEMPERATURE_READY | SENSORS_FLAG_EC_READY);

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

    /*## Generate voltage strings ############################################*/
//    gcvt(ch1_mv, 5, ch1_str);
//    gcvt(ch2_mv, 5, ch2_str);
//    gcvt(ch3_mv, 5, ch3_str);

    /*## Set ADC ready event flag #############################################*/
    osEventFlagsSet(sens_evt_id, SENSORS_FLAG_ADC_READY);

    /*## Delay ###############################################################*/
    osDelay(ADC_DELAY);
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
