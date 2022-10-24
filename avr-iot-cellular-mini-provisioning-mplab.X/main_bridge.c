 /*
 * MAIN File for permanent bridge firmware (no CLI)
 *
 * @file main_bridge.c
 *
 * @defgroup main MAIN
 *
 * @brief This is the main file when building a permanent bridge firmware without any CLI
 *
 */

/*
    (c) [2022] Microchip Technology Inc. and its subsidiaries.

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
#include "mcc_generated_files/system/system.h"
#include "mcc_generated_files/uart/usart3.h"
#include "command_handler/mc_board.h"
#include "command_handler/parser/mc_parser.h"


/*
    Main application
*/

int main(void)
{
    uint8_t c = 0;
    
    SYSTEM_Initialize();

    mc_board_init();

    // Turn on DATA and CELL LEDs to show user that bridge is active
    LED_DATA_SetLow();
    LED_CELL_SetLow();

    // Just forward data to/from UART
    while (true) {
        // Forward data from host to modem
        if (cdc__IsRxReady()) {
            c = cdc_Read();
            while (!USART1_IsTxReady()){
                // Wait for UART to be ready for more data
            }
            modem_Write(c);
        }
        // Send back data from modem to host
        if (modem__IsRxReady()) {
            while (!USART3_IsTxReady()) {
                // Wait for UART to be ready for more data
            }
            // Send back data from modem to host
            cdc_Write(modem_Read());
        }
    }
}