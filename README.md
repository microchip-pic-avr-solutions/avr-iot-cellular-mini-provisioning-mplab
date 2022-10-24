<!-- Please do not change this logo with link -->
[![MCHP](images/microchip.png)](https://www.microchip.com)

# Provisioning firmware for AVR-IoT Cellular Mini kit
This firmware is used as part of the provisioning process of AVR-IoT Cellular Mini kits for use with various cloud providers.  The firmware communicates over a UART interface and provides commands for accessing the cellular modem and the ECC device on the board.  This functionality is used by the iotprovision script running on the host side.  For more info see https://www.microchip.com/design-centers/internet-of-things/iot-dev-kits/iot-provision-tool and https://pypi.org/project/iotprovision/.

## Related Documentation
https://avr-iot.com

## Software Used
- MPLAB® X IDE 6.00 or newer [(microchip.com/mplab/mplab-x-ide)](http://www.microchip.com/mplab/mplab-x-ide)
- MPLAB® XC8 2.36 or a newer compiler [(microchip.com/mplab/compilers)](http://www.microchip.com/mplab/compilers)
- MPLAB® Code Configurator (MCC) Melody 2.0.46 or newer [(microchip.com/mplab/mplab-code-configurator)](https://www.microchip.com/mplab/mplab-code-configurator)
- MPLAB® Code Configurator (MCC) Device Libraries AVR® MCUs 4.3.0 or newer [(microchip.com/mplab/mplab-code-configurator)](https://www.microchip.com/mplab/mplab-code-configurator)
- Microchip AVR Dx Series Device Support (AVR-Dx_DFP 1.10.124) or newer [(packs.download.microchip.com/)](https://packs.download.microchip.com/)

## Hardware Used
AVR-IoT Cellular Mini Development Board (https://www.microchip.com/en-us/development-tool/EV70N78A)

## Dedicated bridge firmware
This firmware repository also includes a dedicated bridge firmware. If the bridge_only configuration is selected in the MPLAB X project then the main_bridge.c file will be included instead of the normal main.c used by the default configuration. The resulting firmware hex will be a pure UART bridge to the Sequans Monarch 2 modem. There will be no command line interface. Anything sent through the CDC virtual serial port of the debugger will be forwarded directly to the modem. This makes it possible to run AT modem commands directly from the host PC.