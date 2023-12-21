/**
 * USART1 Generated Driver API Header File
 *
 * @file usart1.c
 *
 * @ingroup usart1
 *
 * @brief This is the generated driver implementation file for the USART1 driver using
 *
 * @version USART1 Driver Version 2.0.0
*/

/*
© [2023] Microchip Technology Inc. and its subsidiaries.

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

#include "../usart1.h"

/**
  Section: Macro Declarations
*/

#define USART1_TX_BUFFER_SIZE (64) //buffer size should be 2^n
#define USART1_TX_BUFFER_MASK (USART1_TX_BUFFER_SIZE - 1)

#define USART1_RX_BUFFER_SIZE (128) //buffer size should be 2^n
#define USART1_RX_BUFFER_MASK (USART1_RX_BUFFER_SIZE - 1)


#define USART1_RX_BUFFER_ALMOST_FULL (USART1_RX_BUFFER_SIZE - 2)

/**
  Section: Driver Interface
 */

const uart_drv_interface_t modem = {
    .Initialize = &USART1_Initialize,
    .Deinitialize = &USART1_Deinitialize,
    .Read = &USART1_Read,
    .Write = &USART1_Write,
    .IsRxReady = &USART1_IsRxReady,
    .IsTxReady = &USART1_IsTxReady,
    .IsTxDone = &USART1_IsTxDone,
    .TransmitEnable = &USART1_TransmitEnable,
    .TransmitDisable = &USART1_TransmitDisable,
    .AutoBaudSet = NULL,
    .AutoBaudQuery = NULL,
    .BRGSet = NULL,
    .BRGGet = NULL,
    .BaudSet = NULL,
    .BaudGet = NULL,
    .ErrorGet = &USART1_ErrorGet,
    .TxCompleteCallbackRegister = &USART1_TxCompleteCallbackRegister,
    .RxCompleteCallbackRegister = &USART1_RxCompleteCallbackRegister,
    .TxCollisionCallbackRegister = NULL,
    .FramingErrorCallbackRegister = &USART1_FramingErrorCallbackRegister,
    .OverrunErrorCallbackRegister = &USART1_OverrunErrorCallbackRegister,
    .ParityErrorCallbackRegister = &USART1_ParityErrorCallbackRegister,
    .EventCallbackRegister = NULL,
};

/**
  Section: USART1 variables
*/
static volatile uint8_t usart1TxHead = 0;
static volatile uint8_t usart1TxTail = 0;
static volatile uint8_t usart1TxBuffer[USART1_TX_BUFFER_SIZE];
volatile uint8_t usart1TxBufferRemaining;
static volatile uint8_t usart1RxHead = 0;
static volatile uint8_t usart1RxTail = 0;
static volatile uint8_t usart1RxBuffer[USART1_RX_BUFFER_SIZE];
static volatile usart1_status_t usart1RxStatusBuffer[USART1_RX_BUFFER_SIZE];
volatile uint8_t usart1RxCount;
static volatile usart1_status_t usart1RxLastError;

/**
  Section: USART1 APIs
*/
void (*USART1_FramingErrorHandler)(void);
void (*USART1_OverrunErrorHandler)(void);
void (*USART1_ParityErrorHandler)(void);
void (*USART1_TxInterruptHandler)(void);
void (*USART1_RxInterruptHandler)(void);

static void USART1_DefaultFramingErrorCallback(void);
static void USART1_DefaultOverrunErrorCallback(void);
static void USART1_DefaultParityErrorCallback(void);
void USART1_TransmitISR (void);
void USART1_ReceiveISR(void);

void USART1_CTSInterruptCallback(void);
static void USART1_RTSFlowControl(void);


/**
  Section: USART1  APIs
*/

void USART1_Initialize(void)
{
    USART1.CTRLA &= ~(USART_RXCIE_bm);
    USART1_RxCompleteCallbackRegister(USART1_ReceiveISR);
    USART1.CTRLA &= ~(USART_DREIE_bm);
    USART1_TxCompleteCallbackRegister(USART1_TransmitISR);

    // Set the USART1 module to the options selected in the user interface.

    //BAUD 833;
    USART1.BAUD = (uint16_t)USART1_BAUD_RATE(115200);

    // ABEIE disabled; DREIE disabled; LBME disabled; RS485 DISABLE; RXCIE enabled; RXSIE enabled; TXCIE enabled;
    USART1.CTRLA = 0xD0;

    // MPCM disabled; ODME disabled; RXEN enabled; RXMODE NORMAL; SFDEN disabled; TXEN enabled;
    USART1.CTRLB = 0xC0;

    // CMODE Asynchronous Mode; UCPHA enabled; UDORD disabled; CHSIZE Character size: 8 bit; PMODE No Parity; SBMODE 1 stop bit;
    USART1.CTRLC = 0x3;

    //DBGRUN disabled;
    USART1.DBGCTRL = 0x0;

    //IREI disabled;
    USART1.EVCTRL = 0x0;

    //RXPL 0x0;
    USART1.RXPLCTRL = 0x0;

    //TXPL 0x0;
    USART1.TXPLCTRL = 0x0;

    USART1_FramingErrorCallbackRegister(USART1_DefaultFramingErrorCallback);
    USART1_OverrunErrorCallbackRegister(USART1_DefaultOverrunErrorCallback);
    USART1_ParityErrorCallbackRegister(USART1_DefaultParityErrorCallback);
    PC4_SetInterruptHandler(USART1_CTSInterruptCallback);

    usart1RxLastError.status = 0;
    usart1TxHead = 0;
    usart1TxTail = 0;
    usart1TxBufferRemaining = sizeof(usart1TxBuffer);
    usart1RxHead = 0;
    usart1RxTail = 0;
    usart1RxCount = 0;
    USART1.CTRLA |= USART_RXCIE_bm;

    // Manually added RTS update in case RTS starts out in deactivated state
    USART1_RTSFlowControl();

}

void USART1_Deinitialize(void)
{
    USART1.CTRLA &= ~(USART_RXCIE_bm);
    USART1.CTRLA &= ~(USART_DREIE_bm);
    USART1.BAUD = 0x00;
    USART1.CTRLA = 0x00;
    USART1.CTRLB = 0x00;
    USART1.CTRLC = 0x00;
    USART1.DBGCTRL = 0x00;
    USART1.EVCTRL = 0x00;
    USART1.RXPLCTRL = 0x00;
    USART1.TXPLCTRL = 0x00;
}

void USART1_Enable(void)
{
    USART1.CTRLB |= USART_RXEN_bm | USART_TXEN_bm;
}

void USART1_Disable(void)
{
    USART1.CTRLB &= ~(USART_RXEN_bm | USART_TXEN_bm);
}

void USART1_TransmitEnable(void)
{
    USART1.CTRLB |= USART_TXEN_bm;
}

void USART1_TransmitDisable(void)
{
    USART1.CTRLB &= ~(USART_TXEN_bm);
}

void USART1_ReceiveEnable(void)
{
    USART1.CTRLB |= USART_RXEN_bm ;
}

void USART1_ReceiveDisable(void)
{
    USART1.CTRLB &= ~(USART_RXEN_bm);
}

void USART1_TransmitInterruptEnable(void)
{
    USART1.CTRLA |= USART_DREIE_bm ;
}

void USART1_TransmitInterruptDisable(void)
{
    USART1.CTRLA &= ~(USART_DREIE_bm);
}

void USART1_ReceiveInterruptEnable(void)
{
    USART1.CTRLA |= USART_RXCIE_bm ;
}
void USART1_ReceiveInterruptDisable(void)
{
    USART1.CTRLA &= ~(USART_RXCIE_bm);
}

bool USART1_IsRxReady(void)
{
    return (usart1RxCount ? true : false);
}

bool USART1_IsTxReady(void)
{
    return (usart1TxBufferRemaining ? true : false);
}

bool USART1_IsTxDone(void)
{
    return (bool)(USART1.STATUS & USART_TXCIF_bm);
}

size_t USART1_ErrorGet(void)
{
    usart1RxLastError.status = usart1RxStatusBuffer[(usart1RxTail + 1) & USART1_RX_BUFFER_MASK].status;
    return usart1RxLastError.status;
}

uint8_t USART1_Read(void)
{
    uint8_t readValue  = 0;
    uint8_t tempRxTail;

    tempRxTail = (usart1RxTail + 1) & USART1_RX_BUFFER_MASK; // Buffer size of RX should be in the 2^n
    usart1RxTail = tempRxTail;
    readValue = usart1RxBuffer[usart1RxTail];
    // TODO, see https://jira.microchip.com/browse/CCSCRIP-3372
    // Changed from Melody driver, using Critical section not just selective interrupt disable
    ENTER_CRITICAL(R);
    //USART1.CTRLA &= ~(USART_RXCIE_bm);
    if(usart1RxCount != 0)
    {
        usart1RxCount--;
    }
    EXIT_CRITICAL(R);
    //USART1.CTRLA |= USART_RXCIE_bm;

    USART1_RTSFlowControl();

    return readValue;
}

/* Interrupt service routine for RX complete */
ISR(USART1_RXC_vect)
{
    if (USART1_RxInterruptHandler != NULL)
    {
        (*USART1_RxInterruptHandler)();
    }
}

void USART1_ReceiveISR(void)
{
    uint8_t regValue;
    uint8_t tempRxHead;

    usart1RxStatusBuffer[usart1RxHead].status = 0;

    if(USART1.RXDATAH & USART_FERR_bm)
    {
        usart1RxStatusBuffer[usart1RxHead].ferr = 1;
        if(NULL != USART1_FramingErrorHandler)
        {
            USART1_FramingErrorHandler();
        }
    }
    if(USART1.RXDATAH & USART_PERR_bm)
    {
        usart1RxLastError.perr = 1;
        if(NULL != USART1_ParityErrorHandler)
        {
            USART1_ParityErrorHandler();
        }
    }
    if(USART1.RXDATAH & USART_BUFOVF_bm)
    {
        usart1RxStatusBuffer[usart1RxHead].oerr = 1;
        if(NULL != USART1_OverrunErrorHandler)
        {
            USART1_OverrunErrorHandler();
        }
    }

    regValue = USART1.RXDATAL;

    tempRxHead = (usart1RxHead + 1) & USART1_RX_BUFFER_MASK;// Buffer size of RX should be in the 2^n
    if (tempRxHead == usart1RxTail) {
		// ERROR! Receive buffer overflow
	}
    else
    {
		usart1RxHead = tempRxHead;

		// Store received data in buffer
		usart1RxBuffer[tempRxHead] = regValue;
		usart1RxCount++;
	}

    USART1_RTSFlowControl();
}

void USART1_Write(uint8_t txData)
{
    uint8_t tempTxHead;

    if(usart1TxBufferRemaining) // check if at least one byte place is available in TX buffer
    {
       tempTxHead = (usart1TxHead + 1) & USART1_TX_BUFFER_MASK;// Buffer size of TX should be in the 2^n

       usart1TxBuffer[tempTxHead] = txData;
       usart1TxHead = tempTxHead;
       ENTER_CRITICAL(W);
       // TODO, https://jira.microchip.com/browse/CCSCRIP-3372
       // Changed from Melody driver, disable all interrupts (e.g. also CTS interrupts)
       // USART1.CTRLA &= ~(USART_DREIE_bm);  //Critical value decrement
       usart1TxBufferRemaining--; // one less byte remaining in TX buffer
       EXIT_CRITICAL(W);
    }
    else
    {
        //overflow condition; TX buffer is full
    }

   if(CTS1_GetValue() == false)
   {
       USART1.CTRLA |= USART_DREIE_bm;
   }
}

/* Interrupt service routine for Data Register Empty */
ISR(USART1_DRE_vect)
{
    if (USART1_TxInterruptHandler != NULL)
    {
        (*USART1_TxInterruptHandler)();
    }
}

ISR(USART1_TXC_vect)
{
    USART1.STATUS |= USART_TXCIF_bm;
}

void USART1_TransmitISR(void)
{
    uint8_t tempTxTail;
    // use this default transmit interrupt handler code
    if(sizeof(usart1TxBuffer) > usart1TxBufferRemaining) // check if all data is transmitted
    {
       tempTxTail = (usart1TxTail + 1) & USART1_TX_BUFFER_MASK;// Buffer size of TX should be in the 2^n

       usart1TxTail = tempTxTail;
       USART1.TXDATAL = usart1TxBuffer[tempTxTail];
       usart1TxBufferRemaining++; // one byte sent, so 1 more byte place is available in TX buffer
    }
    else
    {
        USART1.CTRLA &= ~(USART_DREIE_bm);
    }

    // or set custom function using USART1_SetTxInterruptHandler()
}

static void USART1_DefaultFramingErrorCallback(void)
{

}

static void USART1_DefaultOverrunErrorCallback(void)
{

}

static void USART1_DefaultParityErrorCallback(void)
{

}

void USART1_FramingErrorCallbackRegister(void (* callbackHandler)(void))
{
    if(NULL != callbackHandler)
    {
        USART1_FramingErrorHandler = callbackHandler;
    }
}

void USART1_OverrunErrorCallbackRegister(void (* callbackHandler)(void))
{
    if(NULL != callbackHandler)
    {
        USART1_OverrunErrorHandler = callbackHandler;
    }
}

void USART1_ParityErrorCallbackRegister(void (* callbackHandler)(void))
{
    if(NULL != callbackHandler)
    {
        USART1_ParityErrorHandler = callbackHandler;
    }
}

void USART1_RxCompleteCallbackRegister(void (* callbackHandler)(void))
{
    if(NULL != callbackHandler)
    {
       USART1_RxInterruptHandler = callbackHandler;
    }
}

void USART1_TxCompleteCallbackRegister(void (* callbackHandler)(void))
{
    if(NULL != callbackHandler)
    {
       USART1_TxInterruptHandler = callbackHandler;
    }
}

void USART1_CTSInterruptCallback (void)
{
    // Check pin status of CTS
   if(CTS1_GetValue() == true)
    {
        // CTS is not asserted so disable USART Data Register Empty Interrupt
        // Note manually edited due to https://jira.microchip.com/browse/CCSCRIP-3371
        USART1.CTRLA &= ~(USART_DREIE_bm);
    }
    else
    {
        // CTS is asserted check if there is data to transmit before we enable interrupt
        if(sizeof(usart1TxBuffer) > usart1TxBufferRemaining)
        {
            // Note manually edited due to https://jira.microchip.com/browse/CCSCRIP-3371
            USART1.CTRLA |= USART_DREIE_bm;
        }
    }
}

static void USART1_RTSFlowControl(void)
{
    if (usart1RxCount < (USART1_RX_BUFFER_ALMOST_FULL) ) //almost full
    {
        // Data register empty,
        // Asserting RTS(active low)
	RTS1_SetLow();
    }
    else
    {
        // Data register received one byte only one buffer is left, tell the target to stop sending data for now by
        // already sent byte can be handled by second buffer [RX has 2 buffers]
        // de-asserting RTS

	RTS1_SetHigh();
    }

}

