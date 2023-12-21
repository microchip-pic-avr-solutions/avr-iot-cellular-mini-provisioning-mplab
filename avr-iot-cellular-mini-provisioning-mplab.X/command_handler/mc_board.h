
#ifndef __MC_BOARD_H__
#define __MC_BOARD_H__

void mc_board_init( void );

uint16_t mc_set_led(uint8_t argc, char *argv[], uint8_t *data, uint16_t *data_length);
uint16_t mc_get_led(uint8_t argc, char *argv[], uint8_t *data, uint16_t *data_length);
uint16_t mc_reset(uint8_t argc, char *argv[], uint8_t *data, uint16_t *data_length);

#define MC_HELLOSTRING   ("Welcome to the AVR-IoT Cellular Mini provisioning command handler!\r\n")
#define MC_BOARD_NAME    ("AVR-IoT Cellular Mini")
#define MC_FW_NAME       ("AVR-IoT Cellular Mini provisioning FW")
// Version of the complete firmware (as opposed to the Command handler version MC_VERSIONSTRING found in mc_parser.h)
#define MC_FW_VERSION    ("1.1.10")

#endif
