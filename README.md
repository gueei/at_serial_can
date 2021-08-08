# at_serial_can

Interface for Serial AT Style CAN. Designed to be compatible with python-can module (required)

Tested Hardware:
http://wiki.wit-motion.com/doku.php?id=USB-CAN%E8%B5%84%E6%96%99

Probably Similar: (not tested)
https://www.aliexpress.com/i/32972743395.html

## Installation

    pip install at_serial_can

## Usage

    import at_serial_can
    
    bus = at_serial_can.ATSerialBus(channel='COM14', bitrate=500000)

Once the bus is created, it is the same as python-can:
https://python-can.readthedocs.io/en/master/api.html