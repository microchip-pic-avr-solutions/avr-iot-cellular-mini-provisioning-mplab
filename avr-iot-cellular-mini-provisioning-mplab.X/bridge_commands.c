/*
 *  (c) 2022 Microchip Technology Inc. and its subsidiaries.
 *
 *  Subject to your compliance with these terms, you may use Microchip software
 *  and any derivatives exclusively with Microchip products. You're responsible
 *  for complying with 3rd party license terms applicable to your use of 3rd
 *  party software (including open source software) that may accompany
 *  Microchip software.
 *
 *  SOFTWARE IS "AS IS". NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY,
 *  APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF
 *  NON-INFRINGEMENT, MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE.
 *
 *  IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
 *  INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
 *  WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
 *  BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
 *  FULLEST EXTENT ALLOWED BY LAW, MICROCHIP’S TOTAL LIABILITY ON ALL CLAIMS
 *  RELATED TO THE SOFTWARE WILL NOT EXCEED AMOUNT OF FEES, IF ANY, YOU PAID
 *  DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 */
#include "command_handler/parser/mc_parser.h"
#include "command_handler/parser/mc_error.h"
#include "mcc_generated_files/uart/usart1.h"
#include "mcc_generated_files/uart/usart3.h"

#include <stdbool.h>
#include <stdint.h>

#define ASCII_EOT '\x04'

/* Enter bridge mode, just forward data to/from UART.
   By default, bridge mode is terminated by a ^D character (ASCII EOT, 0x04) from the host.
   This is unsuitable when binary data is transmitted over the bridge, in this case the optional
   PERMANENT argument must be used, and the only exit from bridge mode is target reset. */
uint16_t cmd_bridgemode(uint8_t argc, char *argv[], uint8_t *data, uint16_t *data_length)
{
    uint8_t c = 0;
    bool permanent = false;

    if (argc == 1) {
        // Argument is optional and only "PERMANENT" is recognized.
        if (mc_match_string("PERMANENT", argv[0])) {
            permanent = true;
        }
        else {
            return MC_STATUS_BAD_ARGUMENT_VALUE;
        }
    } else if (argc > 1) {
        return MC_STATUS_BAD_ARGUMENT_COUNT;
    }

    // Return OK status to let host know the bridge is ready
    mc_print_status(MC_STATUS_OK);

    // Just forward data to/from UART until terminated by a ^D character
    while (true) {
        // Forward data from host to modem
        if (cdc__IsRxReady()) {
            c = cdc_Read();
            if (!permanent && ASCII_EOT == c) {
                // Bridge mode has been aborted, just return, caller will report status to host
                return MC_STATUS_OK;
            } else {
                while (!USART1_IsTxReady()){
                    // Wait for UART to be ready for more data
                }
                modem_Write(c);
            }
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
