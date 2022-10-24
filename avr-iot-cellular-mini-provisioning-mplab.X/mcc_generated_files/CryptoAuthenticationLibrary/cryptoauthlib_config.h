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

#ifndef CRYPTOAUTHLIB_CONFIG_H
#define CRYPTOAUTHLIB_CONFIG_H


#ifndef ATCA_HAL_I2C
#define ATCA_HAL_I2C 1
#endif

#ifndef ATCA_NO_HEAP
#define ATCA_NO_HEAP 1
#endif

#ifndef ATCA_NO_POLL
#define ATCA_NO_POLL 1
#endif

#ifndef ATCAPRINTF
#define ATCAPRINTF 1
#endif

#ifndef ATCA_USE_CONSTANT_HOST_NONCE
#define ATCA_USE_CONSTANT_HOST_NONCE 1
#endif

#ifndef CONF_CRYPTOAUTHLIB_DEBUG_HELPER
#define CONF_CRYPTOAUTHLIB_DEBUG_HELPER 0
#endif

/* Enable debug helper function */
#if CONF_CRYPTOAUTHLIB_DEBUG_HELPER == 1
#ifndef ATCAdebug_print
#define ATCAdebug_print 1
#endif
#endif


#endif // CRYPTOAUTHLIB_CONFIG_H
