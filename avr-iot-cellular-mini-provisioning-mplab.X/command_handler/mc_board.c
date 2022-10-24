#include <stdint.h>
#include <stdio.h>
#include <avr/io.h>

#include "parser/mc_parser.h"
#include "parser/mc_error.h"
#include "mc_commands.h"
#include "../mcc_generated_files/system/pins.h"
#include "../mcc_generated_files/reset/rstctrl.h"
#include "mc_board.h"

#define LED_ON ("ON")
#define LED_OFF ("OFF")

static void led_err_set_output(void) { LED_ERROR_SetDigitalOutput(); }
static void led_err_set(bool on) { if(on){LED_ERROR_SetLow();}else{LED_ERROR_SetHigh();} }
static bool led_err_is_on(void) { return LED_ERROR_GetValue() == 0; }

static void led_data_set_output(void) { LED_DATA_SetDigitalOutput(); }
static void led_data_set(bool on) { if(on){LED_DATA_SetLow();}else{LED_DATA_SetHigh();} }
static bool led_data_is_on(void) { return LED_DATA_GetValue() == 0; }

static void led_conn_set_output(void) { LED_CONN_SetDigitalOutput(); }
static void led_conn_set(bool on) { if(on){LED_CONN_SetLow();}else{LED_CONN_SetHigh();} }
static bool led_conn_is_on(void) { return LED_CONN_GetValue() == 0; }

static void led_cell_set_output(void) { LED_CELL_SetDigitalOutput(); }
static void led_cell_set(bool on) { if(on){LED_CELL_SetLow();}else{LED_CELL_SetHigh();} }
static bool led_cell_is_on(void) { return LED_CELL_GetValue() == 0; }

static void led_user_set_output(void) { LED_USER_SetDigitalOutput(); }
static void led_user_set(bool on) { if(on){LED_USER_SetLow();}else{LED_USER_SetHigh();} }
static bool led_user_is_on(void) { return LED_USER_GetValue() == 0; }

static uint8_t parse_leds(const char *ledstr);

struct led_name
{
    const char* name;
    void (*set_output) (void);
    void (*set) (bool on);
    bool (*is_on) (void);
};

struct led_name led_table[] =
{
    {"ERROR", led_err_set_output, led_err_set, led_err_is_on},
    {"DATA", led_data_set_output, led_data_set, led_data_is_on},
    {"CONN", led_conn_set_output, led_conn_set, led_conn_is_on},
    {"CELL", led_cell_set_output, led_cell_set, led_cell_is_on},
    {"USER", led_user_set_output, led_user_set, led_user_is_on}
};

struct led_name *active_led;



void mc_board_init( void )
{
    // All LEDs enabled and off
    for (uint8_t i = 0; i < sizeof(led_table)/sizeof(struct led_name); i++) {
        led_table[i].set_output();
        led_table[i].set(false);
    }
}



uint16_t mc_set_led(uint8_t argc, char *argv[], uint8_t *data, uint16_t *data_length)
{
    if (argc != 2){
        return MC_STATUS_BAD_ARGUMENT_COUNT;

    }
    uint8_t status = parse_leds(argv[0]);
    if (status) {
        return status;
    }

    if (mc_match_string(LED_ON,argv[1])) {
        active_led->set(true);
        return MC_STATUS_OK;
    }

    if (mc_match_string(LED_OFF,argv[1])) {
        active_led->set(false);
        return MC_STATUS_OK;
    }

    return MC_STATUS_BAD_ARGUMENT_VALUE;
}



uint16_t mc_get_led(uint8_t argc, char *argv[], uint8_t *data, uint16_t *data_length)
{
    if (argc != 1) {
        return MC_STATUS_BAD_ARGUMENT_COUNT;

    }
    uint8_t status = parse_leds(argv[0]);
    if (status) {
        return status;
    }
    if (active_led->is_on()) {
        *data_length = snprintf((char*)data,MC_DATA_BUFFER_LENGTH,"LED:%s is ON\n\r",active_led->name);
    } else {
        *data_length = snprintf((char*)data,MC_DATA_BUFFER_LENGTH,"LED:%s is OFF\n\r",active_led->name);
    }

    return MC_STATUS_OK;
}



static uint8_t parse_leds(const char *ledstr)
{
    for (uint8_t i = 0; i < sizeof(led_table)/sizeof(struct led_name); i++) {
        if (mc_match_string(led_table[i].name,ledstr)) {
            active_led = &led_table[i];
            return MC_STATUS_OK;
        }
    }
    return MC_STATUS_BAD_ARGUMENT_VALUE;
}

uint16_t mc_reset(uint8_t argc, char *argv[], uint8_t *data, uint16_t *data_length)
{
    // Software reset
    RSTCTRL_reset();

    // Execution will never reach this statement due to the reset, but it keeps
    // the compiler happy
    return MC_STATUS_OK;
}
