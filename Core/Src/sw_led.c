/*
 * =====================================================================================
 *
 *       Filename:  sw_led.c
 *
 *    Description:  Processes and functions to handle buttons and leds
 *
 *        Version:  1.0
 *        Created:  21/10/19 03:06:27
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Fedor Chervyakov (), fchervyakov@gmail.com
 *   Organization:  
 *
 * =====================================================================================
 */

/*-----------------------------------------------------------------------------
 *  Includes
 *-----------------------------------------------------------------------------*/
#include "sw_led.h"

#include "stm32wbxx_hal.h"
#include "cmsis_os.h"

#include "app_thread.h"
#include "sensors.h"

extern osThreadId_t JoinerTaskId;
extern osEventFlagsId_t sens_evt_id;

/*-----------------------------------------------------------------------------
 *  Private defines
 *-----------------------------------------------------------------------------*/

#define BT1_TASK_FLAG_BEGIN         ((uint32_t) 0x1U)
#define BT2_TASK_FLAG_BEGIN         ((uint32_t) 0x2U)

#define BT1_DELAY                   ((uint16_t) 3000) /* Hold button 1 for             */
#define BT1_UPDATE                  ((uint16_t) 30)   /* Read button 1 state every     */

#define BT2_DELAY                   ((uint16_t) 3000) /* Hold button 2 for             */
#define BT2_UPDATE                  ((uint16_t) 30)   /* Read button 2 2 state every   */

#define LED_UPDATE                  ((uint16_t) 100)  /* Update LED every              */

/*-----------------------------------------------------------------------------
 *  Private function prototypes
 *-----------------------------------------------------------------------------*/
static void LED_Process(void *argument);
static void BT1_Process(void *argument);
static void BT2_Process(void *argument);

/*-----------------------------------------------------------------------------
 *  Private variables
 *-----------------------------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
static osThreadId_t BT1_TaskId;               /* Task managing button 1 (SW1)   */
uint32_t BT1_TaskBuffer[ 128 ];
osStaticThreadDef_t BT1_TaskControlBlock;
static osThreadId_t BT2_TaskId;               /* Task managing button 2 (SW2)   */
uint32_t BT2_TaskBuffer[ 128 ];
osStaticThreadDef_t BT2_TaskControlBlock;
static osThreadId_t LED_TaskId;               /* Task managing various leds     */
uint32_t LED_TaskBuffer[ 128 ];
osStaticThreadDef_t LED_TaskControlBlock;

static volatile LED_ToggleMode_t LED_Red_Mode;
static volatile LED_ToggleMode_t LED_Blue_Mode;
static volatile LED_ToggleMode_t LED_Green_Mode;

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  UI_Init
 *  Description:  
 * =====================================================================================
 */
void UI_Init (void)
{
    const osThreadAttr_t BT1_Process_attr = {
        .name = "BT1_Task",
        .stack_mem = &BT1_TaskBuffer[0],
        .stack_size = sizeof(BT1_TaskBuffer),
        .cb_mem = &BT1_TaskControlBlock,
        .cb_size = sizeof(BT1_TaskControlBlock),
        .priority = (osPriority_t) osPriorityNormal,
    };
    BT1_TaskId = osThreadNew(BT1_Process, NULL, &BT1_Process_attr);
    if (BT1_TaskId == NULL) 
    {
        Error_Handler();
    }

    const osThreadAttr_t BT2_Process_attr = {
        .name = "BT2_Task",
        .stack_mem = &BT2_TaskBuffer[0],
        .stack_size = sizeof(BT2_TaskBuffer),
        .cb_mem = &BT2_TaskControlBlock,
        .cb_size = sizeof(BT2_TaskControlBlock),
        .priority = (osPriority_t) osPriorityNormal,
    };
    BT2_TaskId = osThreadNew(BT2_Process, NULL, &BT2_Process_attr);
    if (BT2_TaskId == NULL) 
    {
        Error_Handler();
    }

    const osThreadAttr_t LED_Process_attr = {
        .name = "LED_Task",
        .stack_mem = &LED_TaskBuffer[0],
        .stack_size = sizeof(LED_TaskBuffer),
        .cb_mem = &LED_TaskControlBlock,
        .cb_size = sizeof(LED_TaskControlBlock),
        .priority = (osPriority_t) osPriorityNormal,
    };
    LED_TaskId = osThreadNew(LED_Process, NULL, &LED_Process_attr);
    if (LED_TaskId == NULL) 
    {
        Error_Handler();
    }

    return;
}		/* -----  end of function UI_Init  ----- */


/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  LED_Green
 *  Description:  
 * =====================================================================================
 */
void LED_Green (LED_ToggleMode_t mode)
{
    LED_Green_Mode = mode;
    return; 
}		/* -----  end of function LED_Green  ----- */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  LED_Blue
 *  Description:  
 * =====================================================================================
 */
void LED_Blue (LED_ToggleMode_t mode)
{
    LED_Blue_Mode = mode;
    return; 
}		/* -----  end of function LED_Blue  ----- */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  LED_Red
 *  Description:  
 * =====================================================================================
 */
void LED_Red (LED_ToggleMode_t mode)
{
    LED_Red_Mode = mode;
    return; 
}		/* -----  end of function LED_Red  ----- */


/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  LED_Process
 *  Description:  
 * =====================================================================================
 */
static void LED_Process(void *argument )
{
    UNUSED(argument);
    
    uint32_t count = 0;

    LED_Red(OFF);
    LED_Blue(OFF);
    LED_Green(OFF);

    for (;;)
    {
        switch (LED_Green_Mode)
        {
        case ON:
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
            break;
        case OFF:
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
            break;
        case FAST_TOGGLING:
            if (count > (LED_TOGGLE_DELAY_FAST / LED_UPDATE))
            {
                HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            }
            break;
        case SLOW_TOGGLING:
            if (count > (LED_TOGGLE_DELAY_SLOW / LED_UPDATE))
            {
                HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
                count = 0;
            }
            break;
        default:
            break;
        }

        switch (LED_Red_Mode)
        {
        case ON:
            HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
            break;
        case OFF:
            HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
            break;
        case FAST_TOGGLING:
            if (count > (LED_TOGGLE_DELAY_FAST / LED_UPDATE))
            {
                HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
            }
            break;
        case SLOW_TOGGLING:
            if (count > (LED_TOGGLE_DELAY_SLOW / LED_UPDATE))
            {
                HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
                count = 0;
            }
            break;
        default:
            break;
        }

        switch (LED_Blue_Mode)
        {
        case ON:
            HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
            break;
        case OFF:
            HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
            break;
        case FAST_TOGGLING:
            if (count > (LED_TOGGLE_DELAY_FAST / LED_UPDATE))
            {
                HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
            }
            break;
        case SLOW_TOGGLING:
            if (count > (LED_TOGGLE_DELAY_SLOW / LED_UPDATE))
            {
                HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
                count = 0;
            }
            break;
        default:
            break;
        }

        count++;
        osDelay(LED_UPDATE);
    }
}		/* -----  end of function LED_Process  ----- */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  BT2_Process
 *  Description:  
 * =====================================================================================
 */
static void BT2_Process(void *argument)
{
    UNUSED(argument);

    for (;;)
    {
        
        osThreadFlagsWait(BT2_TASK_FLAG_BEGIN, osFlagsWaitAll, osWaitForever);
        
        uint16_t i = 0;
        do
        {
            i++;
            osDelay(BT2_UPDATE);
        } while (HAL_GPIO_ReadPin(B2_GPIO_Port, B2_Pin) == GPIO_PIN_RESET);

        if (i > (BT2_DELAY / BT2_UPDATE))
        {
            osEventFlagsSet(sens_evt_id, SENSORS_FLAG_PH_CALIB_BEGIN);
        }


        osThreadFlagsClear(BT2_TASK_FLAG_BEGIN);
    }
}

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  BT1_Process
 *  Description:  
 * =====================================================================================
 */
static void BT1_Process(void *argument)
{
    UNUSED(argument);

    for (;;)
    {
        
        osThreadFlagsWait(BT1_TASK_FLAG_BEGIN, osFlagsWaitAll, osWaitForever);
        
        uint16_t i = 0;
        do
        {
            i++;
            osDelay(BT1_UPDATE);
        } while (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET);

        if (i > (BT1_DELAY / BT1_UPDATE))
        {
            osThreadFlagsSet(JoinerTaskId, JOIN_TASK_FLAG_BEGIN);
        }

        osThreadFlagsClear(BT1_TASK_FLAG_BEGIN);
    }
}

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  HAL_GPIO_EXTI_Callback
 *  Description:  
 * =====================================================================================
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin)
    {
    case B1_Pin:
        osThreadFlagsSet(BT1_TaskId, BT1_TASK_FLAG_BEGIN);
        break;
    case B2_Pin:
        osThreadFlagsSet(BT2_TaskId, BT2_TASK_FLAG_BEGIN);
        break;
    default:
        break;
    }
}

