# DS2480B-python
This is a basic Python driver for DS2480B serial to 1-wire (OneWire) master controller.

The implementation is naively (is a direct conversion with a few improvements) based on https://www.maximintegrated.com/en/design/technical-documents/app-notes/1/192.html

Tested on a DIY board based on FT232RL USB-UART converter and DS2480B chip. Such connection seems to be fully compatible with DS9097U.

Please do note that some markings are not intuitive:
* Power supply for DS2480B is 5V
* RXD on DS2480B should be connected to RXD of FT232
* TXD on DS2480B should be connected to TXD of FT232
* POL on DS2480B should be tied to 5V for direct connection with FT232
* VPP on DS2480B should be connected to 5V if there is no intention to supply 12V for EPROM programming, otherwise supply 12V
* 1-W on DS18B20 does not require any external pull-up

Please note that very limited testing was done and therefore the code might not be reliable. Also some parts of the code provided in AN192 seemed not to be complete.
