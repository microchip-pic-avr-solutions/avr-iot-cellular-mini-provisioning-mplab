#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "mc_housekeeping.h"
#include "mc_parser.h"
#include "mc_error.h"
#include "../mc_board.h"
#include "../mc_commands.h"

#define VERSION_COMMANDHANDLER ("COMMANDHANDLER")
#define VERSION_FIRMWARE ("FIRMWARE")

uint16_t mc_get_version (uint8_t argc, char *argv[], uint8_t *data, uint16_t *data_length)
{
    if (argc > 1) {
        return MC_STATUS_BAD_ARGUMENT_COUNT;
    }

    if (argc == 0 || mc_match_string(VERSION_COMMANDHANDLER, argv[0])) {
        *data_length = snprintf((char*)data,MC_DATA_BUFFER_LENGTH,"%s\r\n",MC_VERSIONSTRING);
        return MC_STATUS_OK;
    }

    if (mc_match_string(VERSION_FIRMWARE, argv[0])) {
        *data_length = snprintf((char*)data,MC_DATA_BUFFER_LENGTH,"%s\r\n",MC_FW_VERSION);
        return MC_STATUS_OK;
    }

    return MC_STATUS_BAD_ARGUMENT_VALUE;
}



uint16_t mc_about(uint8_t argc, char *argv[], uint8_t *data, uint16_t *data_length)
{
    *data_length = snprintf((char*)data,MC_DATA_BUFFER_LENGTH,"%s\r\nFW %s\r\nCommand handler %s\r\nfor the %s\r\n", MC_FW_NAME, MC_FW_VERSION, MC_VERSIONSTRING, MC_BOARD_NAME);
    return MC_STATUS_OK;
}



uint16_t mc_blobtest(uint8_t argc, char *argv[], uint8_t *data, uint16_t *data_length)
{
    // do nothing to nothing and leave the *data and *data length alone so they are passed back up
    return MC_STATUS_OK;
}


/*
 * MC+PING implementation, can be used for synchronisation between host and firmware
 */
uint16_t mc_ping(uint8_t argc, char *argv[], uint8_t *data, uint16_t *data_length)
{
    // Command MC+PING=<token> responds with MC+PONG=<TOKEN> - same token as it was given
    // converted to upper case (this is a feature of the parser framework)
    if (argc != 1) {
        return MC_STATUS_BAD_ARGUMENT_COUNT;
    }
    *data_length = snprintf(data, MC_DATA_BUFFER_LENGTH, "MC+PONG=%s", argv[0]);
    return MC_STATUS_OK;
}


uint16_t mc_list_commands(uint8_t argc, char *argv[], uint8_t *data, uint16_t *data_length)
{
    *data = '\0';
    for (uint8_t i = 0; i < mc_number_of_commands(); i++) {
        strcat((char*)data, mc_command_set[i].command_string);
        strcat((char*)data,"\r\n");
    }

    *data_length = strlen((char*)data);

    return MC_STATUS_OK;
}
