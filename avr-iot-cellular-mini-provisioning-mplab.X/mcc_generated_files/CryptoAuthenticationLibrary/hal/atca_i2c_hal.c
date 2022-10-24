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

#include <string.h>
#include "atca_hal.h"
#include "atca_i2c_hal.h"
#include "../atca_device.h"
#include "../atca_status.h"

#include "../../i2c_host/twi0.h"



/** \brief initialize an I2C interface using given config
 * \param[in] hal - opaque ptr to HAL data
 * \param[in] cfg - interface configuration
 */
uint8_t i2c_address = 0;
const i2c_host_interface_t *i2c_interface = &CAL_I2C_HOST;

ATCA_STATUS hal_i2c_init(void *hal, ATCAIfaceCfg *cfg)
{
        i2c_address = (cfg->atcai2c.slave_address) >> 1;
	
	return ATCA_SUCCESS;
}


/** \brief HAL implementation of I2C post init
 * \param[in] iface  instance
 * \return ATCA_SUCCESS
 */
ATCA_STATUS hal_i2c_post_init(ATCAIface iface)
{
    return ATCA_SUCCESS;
}


/** \brief HAL implementation of I2C send over ASF
 * \param[in] iface     instance
 * \param[in] txdata    pointer to space to bytes to send
 * \param[in] txlength  number of bytes to send
 * \return ATCA_STATUS
 */

ATCA_STATUS hal_i2c_send(ATCAIface iface, uint8_t *txdata, int txlength)
{
	bool retStatus; 
	txdata[0] = 0x03; // insert the Word Address Value, Command token
	txlength++;       // account for word address value byte.

	retStatus = i2c_interface->Write(i2c_address, txdata, txlength);
	if(retStatus)
	{
		while (i2c_interface->IsBusy());
	}

	return ATCA_SUCCESS;
}

/** \brief HAL implementation of I2C receive function for ASF I2C
 * \param[in] iface     instance
 * \param[in] rxdata    pointer to space to receive the data
 * \param[in] rxlength  ptr to expected number of receive bytes to request
 * \return ATCA_STATUS
 */

ATCA_STATUS hal_i2c_receive(ATCAIface iface, uint8_t *rxdata, uint16_t *rxlength)
{
	bool retStatus; 
	uint16_t      rxdata_max_size = *rxlength;

	*rxlength = 0;
	if (rxdata_max_size < 1) {
		return ATCA_SMALL_BUFFER;
	}

	*rxdata = 0;
	retStatus = i2c_interface->Read(i2c_address, rxdata, 1);
	if(retStatus)
	{
		while (i2c_interface->IsBusy());
	}

	if ((rxdata[0] < ATCA_RSP_SIZE_MIN) || (rxdata[0] > ATCA_RSP_SIZE_MAX)) {
		return ATCA_INVALID_SIZE;
	}
	if (rxdata[0] > rxdata_max_size) {
		return ATCA_SMALL_BUFFER;
	}

	retStatus = i2c_interface->Read(i2c_address, &rxdata[1], rxdata[0] - 1);
	if(retStatus)
	{
		while (i2c_interface->IsBusy());
	}
	*rxlength = rxdata[0];

	return ATCA_SUCCESS;
}

/** \brief wake up CryptoAuth device using I2C bus
 * \param[in] iface  interface to logical device to wakeup
 */

ATCA_STATUS hal_i2c_wake(ATCAIface iface)
{
    bool retStatus; 
	ATCAIfaceCfg *cfg  = atgetifacecfg(iface);
	uint32_t      bdrt = cfg->atcai2c.baud;
	uint8_t       data[4];
	uint8_t       zero_byte = 0;

    // Commented out outer do-while loop due to an issue with the generated wake code, see https://jira.microchip.com/browse/PAI-1250
	//do{
		retStatus = i2c_interface->Write(i2c_address, &zero_byte, 1);
		if(retStatus)
		{
				while (i2c_interface->IsBusy());
		}
	//}while((TWI0_ErrorGet()== I2C_ERROR_ADDR_NACK)||(TWI0_ErrorGet()== I2C_ERROR_DATA_NACK));

	atca_delay_us(cfg->wake_delay);

	// receive the wake up response
	retStatus = i2c_interface->Read(i2c_address, data, 4);
	if(retStatus)
	{
		while (i2c_interface->IsBusy());
	}

	return hal_check_wake(data, 4);

}

/** \brief idle CryptoAuth device using I2C bus
 * \param[in] iface  interface to logical device to idle
 */

ATCA_STATUS hal_i2c_idle(ATCAIface iface)
{
	bool retStatus; 
	uint8_t data = 0x02;

	retStatus = i2c_interface->Write(i2c_address, &data, 1);
	if(retStatus)
	{
		while (i2c_interface->IsBusy());
	}

	return ATCA_SUCCESS;
}

/** \brief sleep CryptoAuth device using I2C bus
 * \param[in] iface  interface to logical device to sleep
 */

ATCA_STATUS hal_i2c_sleep(ATCAIface iface)
{
	bool retStatus; 
	uint8_t data = 0x01;

	retStatus = i2c_interface->Write(i2c_address, &data, 1);
	if(retStatus)
	{
		while (i2c_interface->IsBusy());
	}
	
	return ATCA_SUCCESS;
}

/** \brief manages reference count on given bus and releases resource if no more refences exist
 * \param[in] hal_data - opaque pointer to hal data structure - known only to the HAL implementation
 * return ATCA_SUCCESS
 */

ATCA_STATUS hal_i2c_release(void *hal_data)
{
    //TODO: For the moment, don't do anything

    return ATCA_SUCCESS;
}

/** \brief discover i2c buses available for this hardware
 * this maintains a list of logical to physical bus mappings freeing the application
 * of the a-priori knowledge
 * \param[in] i2c_buses - an array of logical bus numbers
 * \param[in] max_buses - maximum number of buses the app wants to attempt to discover
 * \return ATCA_SUCCESS
 */

ATCA_STATUS hal_i2c_discover_buses(int i2c_buses[], int max_buses)
{

    //TODO: For the moment, don't do anything

    return ATCA_SUCCESS;
}

/** \brief discover any CryptoAuth devices on a given logical bus number
 * \param[in]  busNum  logical bus number on which to look for CryptoAuth devices
 * \param[out] cfg     pointer to head of an array of interface config structures which get filled in by this method
 * \param[out] found   number of devices found on this bus
 * \return ATCA_SUCCESS
 */

ATCA_STATUS hal_i2c_discover_devices(int busNum, ATCAIfaceCfg cfg[], int *found)
{
	//TODO: For the moment, don't do anything

    return ATCA_SUCCESS;
}

// Below code is not part of code generated by MCC Melody, it was manually added later

// This code was added to be able to get in sync with an ATECC608 device after
// system resets, ESD events, noise on I2C lines etc.
// The algorithm for this synchronization sequence is described in the ATECC608B
// data sheet section 7.8 I2C synchronization

#include "../../system/pins.h"

static void ecc_sda_set_output(void) { IO_PC2_SetDigitalOutput(); }
static void ecc_sda_set_input(void) { IO_PC2_SetDigitalInput(); }
static void ecc_sda_set(bool on)
{
    if (on) {
        IO_PC2_SetHigh();
    } else {
        IO_PC2_SetLow();
    }
}
static void ecc_scl_set_output(void) { IO_PC3_SetDigitalOutput(); }
static void ecc_scl_set_input(void) { IO_PC3_SetDigitalInput(); }
static void ecc_scl_set(bool on)
{
    if (on) {
        IO_PC3_SetHigh();
    } else {
        IO_PC3_SetLow();
    }
}

// Attempt read, but terminate upon first NACK
static i2c_host_error_t attempt_read(void){
    uint8_t *rxdata = 0;
    bool retStatus = false;

    while (i2c_interface->IsBusy())
    {
        // Wait until I2C interface is ready
    }
    retStatus = i2c_interface->Read(i2c_address, rxdata, 1);
	if(retStatus)
	{
		while (i2c_interface->IsBusy())
        {
            // Wait until I2C interface is ready    
        }
	}

    return TWI0_ErrorGet();
}


/** \brief  Reset I2C bus and re-sync ECC communication
 * \param[in] iface  interface to logical device to wakeup
 */
ATCA_STATUS hal_ecc_sync(ATCAIface iface)
{
    bool retStatus;
    
    // The I2C module does not support I2C bus software reset, so it must be
    // done manually
    CAL_I2C_HOST_Deinitialize();
    
    // Start by SDA driven high by external pull-up and SCL actively driven high
    // since we are the host driving the communication
    ecc_sda_set_input();
    ecc_scl_set(true);
    ecc_scl_set_output();

    // Start condition
    ecc_sda_set(false);
    ecc_sda_set_output();
    // Should be at least 250 ns delay
    atca_delay_us(1);

    // Nine clocks while SDA is held high by pull-up resistor
    ecc_scl_set(false);
    ecc_sda_set_input();
    // Should be at least 400 ns delay
    atca_delay_us(1);
    ecc_scl_set(true);
    // Should be at least 400 ns delay
    atca_delay_us(1);
    // Remaining 8 cycles
    for (int i=0;i<8;i++){
        ecc_scl_set(false);
        // Should be at least 400 ns delay
        atca_delay_us(1);
        ecc_scl_set(true);
        // Should be at least 400 ns delay
        atca_delay_us(1);
    }

    // Start condition
    ecc_sda_set(false);
    ecc_sda_set_output();
    // Should be at least 400 ns delay
    atca_delay_us(1);

    // Stop condition
    ecc_sda_set_input();
    // Should be at least 500 ns delay
    atca_delay_us(1);

    // Release clock (will stay high due to pull-up)
    ecc_scl_set_input();

    // Back to using the I2C module again
    CAL_I2C_HOST_Initialize();

    // Attempt a read
    i2c_host_error_t i2c_err = attempt_read();
    if (I2C_ERROR_NONE != i2c_err){
        // Read failed, ECC device might be at sleep
        hal_i2c_wake(iface);
        i2c_err = attempt_read();
        if (I2C_ERROR_NONE != i2c_err){
            // Read failed again, device might be busy. Wait max execution time
            atca_delay_ms(1900);
            // Try one last time
            i2c_err = attempt_read();
            if (I2C_ERROR_NONE != i2c_err){
                return ATCA_TIMEOUT;
            }
        }
    }

    // Reset the ECC device by writing to the reset register
	uint8_t data = 0x00;
    retStatus = i2c_interface->Write(i2c_address, &data, 1);
	if(retStatus)
	{
		while (i2c_interface->IsBusy());
	}
	
    return ATCA_SUCCESS;
}


/** @} */
