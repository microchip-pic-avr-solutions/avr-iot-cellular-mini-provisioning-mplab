/**
 * System Driver Source File
 *
 * @file system.c
 *
 * @ingroup systemdriver
 *
 * @brief This is the generated driver implementation file for the System Driver.
 *
 * @version Driver Version 1.0.1
*/
/*
© [2022] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip
    software and any derivatives exclusively with Microchip products.
    You are responsible for complying with 3rd party license terms
    applicable to your use of 3rd party software (including open source
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.?
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR
    THIS SOFTWARE.
*/

 /**
   Section: Included Files
 */
#include "../system.h"
#include <util/delay.h>

/**
 * Initializes MCU, drivers and middleware in the project
**/


void SYSTEM_Initialize(void)
{
    PIN_MANAGER_Initialize();
    CAL_I2C_HOST_Initialize();
    CLOCK_Initialize();
    USART1_Initialize();
    USART3_Initialize();
    CPUINT_Initialize();
    CryptoAuth_Initialize();

    // Active modem reset to ensure it starts in a known state
    CELL_RESETN_SetHigh();
    // Reset should be active for at least 100ns according to datasheet
    _delay_us(1);
    CELL_RESETN_SetLow();
}

