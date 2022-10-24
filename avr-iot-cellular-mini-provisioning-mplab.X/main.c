 /*
 * MAIN Generated Driver File
 *
 * @file main.c
 *
 * @defgroup main MAIN
 *
 * @brief This is the generated driver implementation file for the MAIN driver.
 *
 * @version MAIN Driver Version 1.0.0
*/

/*
� [2022] Microchip Technology Inc. and its subsidiaries.

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

// Helper to convert from char to uint8_t since the driver layer requires uint8_t argument
void cdc_putc(char data)
{
    while(!cdc_IsTxReady())
    {
        // Waiting for CDC UART to be ready for more data
    }
    cdc_Write((const uint8_t) data);
}

/*
    Main application
*/

int main(void)
{
    SYSTEM_Initialize();

    mc_board_init();

    // Initializing the parser includes sending a welcome message to the host
    // This message can be used by the host to know when the boot-up is done, but
    // then it must be done as the last step before entering the parser loop
    mc_parser_init(cdc_putc);
    while(1)
    {
        while(!cdc__IsRxReady())
        {
            // Waiting for incoming byte on CDC UART
        }
        mc_parser(cdc_Read());
    }
}