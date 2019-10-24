/*
 * =====================================================================================
 *
 *       Filename:  sw_led.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  20/10/19 03:24:56
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Fedor Chervyakov (), fchervyakov@gmail.com
 *   Organization:  
 *
 * =====================================================================================
 */

#ifndef __SW_LED_H_
#define __SW_LED_H_


/*-----------------------------------------------------------------------------
 *  Header includes
 *-----------------------------------------------------------------------------*/
#include "main.h"

/*-----------------------------------------------------------------------------
 *  Type definitions
 *-----------------------------------------------------------------------------*/
typedef enum
{
    ON,
    OFF,
    FAST_TOGGLING,
    SLOW_TOGGLING
} LED_ToggleMode_t;


/*-----------------------------------------------------------------------------
 *  Button event flags
 *-----------------------------------------------------------------------------*/
#define BT1_SHORT_PRESS             ((uint32_t) 0x1U)
#define BT1_LONG_PRESS              ((uint32_t) 0x2U)
#define BT1_VERY_LONG_PRESS         ((uint32_t) 0x3U)
#define BT2_SHORT_PRESS             ((uint32_t) 0x4U)
#define BT2_LONG_PRESS              ((uint32_t) 0x5U)
#define BT2_VERY_LONG_PRESS         ((uint32_t) 0x6U)
#define BT3_SHORT_PRESS             ((uint32_t) 0x7U)
#define BT3_LONG_PRESS              ((uint32_t) 0x8U)
#define BT3_VERY_LONG_PRESS         ((uint32_t) 0x9U)

/*-----------------------------------------------------------------------------
 *  Private defines
 *-----------------------------------------------------------------------------*/
#define LED_UPDATE                  ((uint16_t) 100)  /* Update LED every              */

#define BT1_TASK_FLAG_BEGIN         ((uint32_t) 0x1U)
#define BT2_TASK_FLAG_BEGIN         ((uint32_t) 0x2U)
#define BT3_TASK_FLAG_BEGIN         ((uint32_t) 0x3U)

#define SHORT_PRESS_DELAY           ((uint16_t) 250)  /* Short press delay ms          */
#define LONG_PRESS_DELAY            ((uint16_t) 2000) /* Long press delay ms           */
#define VERY_LONG_PRESS_DELAY       ((uint16_t) 6600) /* Very long press delay ms      */
#define UPDATE_BUTTON_DELAY         ((uint16_t) 30)   /* Read button state every       */

#define LED_TOGGLE_DELAY_FAST       ((uint16_t) 450) /* LED Toggle fast delay */
#define LED_TOGGLE_DELAY_SLOW       ((uint16_t) 1420)/* LED Toggle slow delay */

/*-----------------------------------------------------------------------------
 *  Exported function prototypes
 *-----------------------------------------------------------------------------*/
void UI_Init(void);
void LED_Green(LED_ToggleMode_t mode);
void LED_Blue(LED_ToggleMode_t mode);
void LED_Red(LED_ToggleMode_t mode);

#endif
