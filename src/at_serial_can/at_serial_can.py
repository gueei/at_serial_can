"""
Interface for Serial AT Style CAN. Designed to be compatible with python-can module (required)

Tested Hardware:
http://wiki.wit-motion.com/doku.php?id=USB-CAN%E8%B5%84%E6%96%99

Probably Similar: (not tested)
https://www.aliexpress.com/i/32972743395.html

.. note::

Modified from slcan implementation

"""

from __future__ import absolute_import
from ctypes import ArgumentError

import time
import logging

from can import BusABC, Message

logger = logging.getLogger(__name__)

try:
    import serial
except ImportError:
    logger.warning("You won't be able to use the Serial AT-Style can backend without "
                   "the serial module installed!")
    serial = None


class ATSerialBus(BusABC):
    """
    AT Command Style Serial CAN BUS

    """
    _SLEEP_AFTER_SERIAL_OPEN = 1  # in seconds

    LINE_TERMINATOR = b'\r\n'

    def __init__(self, channel, ttyBaudrate=9600, bitrate=None,
                 btr=None, sleep_after_open=_SLEEP_AFTER_SERIAL_OPEN,
                 rtscts=False, **kwargs):
        """
        :param str channel:
            port of underlying serial or usb device (e.g. /dev/ttyUSB0, COM8, ...)
            Must not be empty.
        :param int ttyBaudrate:
            baudrate of underlying serial or usb device
        :param int bitrate:
            Bitrate in bit/s
        :param float sleep_after_open:
            Time to wait in seconds after opening serial connection
        """

        if not channel:  # if None or empty
            raise TypeError("Must specify a serial port.")

        if '@' in channel:
            (channel, ttyBaudrate) = channel.split('@')

        self.serialPortOrig = serial.serial_for_url(
            channel, baudrate=ttyBaudrate, rtscts=rtscts)

        self._buffer = bytearray()

        time.sleep(sleep_after_open)

        self.enter_config_mode()
        self.set_loopback(False)
        self.set_can_baudrate(bitrate)
        self.enter_running_mode()

        super(ATSerialBus, self).__init__(channel, ttyBaudrate=115200,
                                       bitrate=None, rtscts=False, **kwargs)

    def enter_config_mode(self):
        self._config_mode = True
        self.send_command('CG')

    def enter_running_mode(self):
        self._config_mode = False
        self.send_command('AT')

    def set_loopback(self, loopback: bool):
        self._send_config_command('CAN_MODE', 1 if loopback else 0)

    def set_can_baudrate(self, bitrate):
        self._send_config_command("CAND_BAUD", bitrate)

    def _send_config_command(self, param, value):
        if not self._config_mode:
            raise RuntimeError("ATSerialBus: Config device only allowed in Config Mode, try enter_config_mode() before calling")
        self.send_command(f'{param}={value}')

    def send_command(self, command):
        self.write(f'AT+{command}'.encode())
        result = self.read()
        if (result!=b'OK\r\n'):
            raise RuntimeError(f"ATSerialBus: Error in sending Command {command}")
        
    def write(self, data):
        self.serialPortOrig.write(data + self.LINE_TERMINATOR)
        self.serialPortOrig.flush()
    
    def read(self):
        return self.serialPortOrig.read_until(self.LINE_TERMINATOR)

    def _recv_internal(self, timeout):
        if timeout != self.serialPortOrig.timeout:
            self.serialPortOrig.timeout = timeout

        canId = None
        remote = False
        extended = False
        frame = []

        # First read what is already in the receive buffer
        while (self.serialPortOrig.in_waiting and
               self.LINE_TERMINATOR not in self._buffer):
            self._buffer += self.serialPortOrig.read(1)

        # If we still don't have a complete message, do a blocking read
        if self.LINE_TERMINATOR not in self._buffer:
            self._buffer += self.serialPortOrig.read_until(self.LINE_TERMINATOR)

        if self.LINE_TERMINATOR not in self._buffer:
            # Timed out
            return None, False

        print(self._buffer)

        # byte 0:1 AT
        # byte 2:5 ID
        # byte 6 DLC
        # byte 7:7+DLC Data
        # byte -2:-1 \r\n
        b = self._buffer[:]
        extended = (b[5] & 0x04) > 0
        remote = (b[5] & 0x02) > 0
        canId = b[2] << 24 | b[3] << 16 | b[4] << 8 | b[5] & 0xFF
        if extended:
            canId = canId >> 3
        else:
            canId = canId >> 21
        dlc = b[6]
        frame = b[7:-2]

        del self._buffer[:]
        
        if canId is not None:
            msg = Message(arbitration_id=canId,
                            is_extended_id=extended,
                            timestamp=time.time(),   # Better than nothing...
                            is_remote_frame=remote,
                            dlc=dlc,
                            data=frame)
            return msg, False
        return None, False

    def send(self, msg, timeout=None):
        if timeout != self.serialPortOrig.write_timeout:
            self.serialPortOrig.write_timeout = timeout

        cid = 0
        if msg.is_extended_id:
            cid = msg.arbitration_id << 3 | 0x04
        else:
            cid = msg.arbitration_id << 21
        if msg.is_remote_frame:
            cid = cid | 0x02

        id_dlc = bytes([cid>>24, cid>>16&0xff, cid>>8&0xff, cid&0xff, len(msg.data)])
        w = b"AT" + id_dlc + msg.data

        self.write(w)

    def shutdown(self):
        self.serialPortOrig.close()

    def fileno(self):
        if hasattr(self.serialPortOrig, 'fileno'):
            return self.serialPortOrig.fileno()
        # Return an invalid file descriptor on Windows
        return -1