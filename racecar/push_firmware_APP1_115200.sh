#!/usr/bin/env bash

if [ "$1" != "" ]; then
    echo "Using interface /dev/tty$1"
else
    echo "Usage:"
    echo "   Real arduino:"
    echo "      $ ./push_firmware_APP1_115200.sh ACM0"
    echo "   Fake Arduino ( USB0 or USB1):"
    echo "      $ ./push_firmware_APP1_115200.sh USB0"
    exit
fi

/usr/share/arduino/hardware/tools/avrdude -C/usr/share/arduino/hardware/tools/avrdude.conf -v -patmega2560 -cwiring -P/dev/tty$1 -b115200 -D -V -Uflash:w:firmware_APP1_115200.hex:i

