# Arduino Hardware Serial with Flow Control

An Arduino AVR library to add hardware flow control via RTS/CTS to the serial ports. It’s based on the original HardwareSerial files modified to implement the flow control. The library is backward compatible, so it’s possible to compile old projects without any changes.

## How to install

- Click on the "Clone or download" button in the upper right corner.
- Extract the ZIP file.
- Locate the original HardwareSerial .cpp and .h files on your disk drive:
`HardwareSerial.cpp`
`HardwareSerial0.cpp`
`HardwareSerial1.cpp`
`HardwareSerial2.cpp`
`HardwareSerial3.cpp`
`HardwareSerial.h`
`HardwareSerial_private.h`
- Replace the original files with the new version (make a backup copy of the original files for safety)

## Getting started
### Wiring
Connect the 4 wires as described in the figure:

![Hardware connections](https://www.bigmike.it/github/HardwareSerial-Wiring_s.png)

#### NOTE:
**CTS must be connected to an interrupt pin** (marked as INTx). If connected to a PCINTx pin it will not work. See [Arduino external interrupt](https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/) for reference. The library doesn’t check if the CTS pin is an INT pin, so be sure to use the right one.

**RTS** can be connected to any free pin.

### Software
The `begin()` function has been expanded to support the RTS and CTS pin definition.

The old, and still supported, way to call `begin()` without flow control is:

```
begin(115200);
```
or
```
begin(115200, SERIAL_8N1);
```

Now, to use the flow control, it’s possible to call it with the new additional parameters:

```
begin(115200, 3, 4);   // Speed, CTS pin, RTS pin);
```
or
```
begin(115200, SERIAL_8N1, 3, 4);   // Speed, mode, CTS pin, RTS pin);
```

If the communication has to work in one direction only (a device only sends, and the other only receives), it’s possible to use -1 as CTS or RTS pin. In this way the pin will not be assigned and it’s not necessary to wire it.
Example:
```
begin(115200, SERIAL_8N1, 3, -1);   // Speed, mode, CTS pin, no RTS);
```

## Examples
To test the library, two ATmega2560 were connected together as described in the [Wiring](#wiring) section, and two test programs were created (both present in the examples folder): `test-rx` and `test-tx`.
The board with the `test-rx` program sends some info to the IDE monitor console. It shows the receiving progress and it checks if the received data are correct or if  characters were lost. `test-tx` just sends a long train of characters to the specified serial using the flow control.
It’s important to launch the RX program before the TX program. To restart the test, just reset the RX program and then the TX program using the reset button.

## Acknowledgements
The library is based on the original [Arduino AVR Core HardwareSerial](https://github.com/arduino/ArduinoCore-avr) library.
