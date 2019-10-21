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

#define LED_TOGGLE_DELAY_FAST       ((uint16_t) 300) /* LED Toggle fast delay */
#define LED_TOGGLE_DELAY_SLOW       ((uint16_t) 900) /* LED Toggle slow delay */
/*-----------------------------------------------------------------------------
 *  Exported function prototypes
 *-----------------------------------------------------------------------------*/
void UI_Init(void);
void LED_Green(LED_ToggleMode_t mode);
void LED_Blue(LED_ToggleMode_t mode);
void LED_Red(LED_ToggleMode_t mode);

#endif
