"""
acb,py
A python implementation of the antenna control board's control system
Author: Weiyang Wang <wew168@ucsd.edu>

XXX: The entire module uses byte strings. For python3 this means convert all
Unicode strings to byte strings is necessary.
"""
from __future__ import print_function
import serial
import struct
import sys
import time
import math

# python version (2 or 3, rest is not important)
PY_VERSION = sys.version_info[0]

# some special bits
CR = b'\r'
LF = b'\n'
ACK = 'A' 
NAK = 'N'

def eprint(*args, **kwargs):
    """Enable print to stderr"""
    print(*args, file=sys.stderr, **kwargs)

def xor_chksum(msg):
    """Compute the CRC for a byte list, as specified by the instruction. 
    Returns the result of checksum as an integer."""

    chk = 0x0

    for byte in msg:
        if PY_VERSION == 2:
            chk ^= ord(byte)
        else:
            chk ^= byte

    return chk

def hexprint(msg):
    """Print a message's hex value to stderr"""
    if PY_VERSION == 2:
        eprint(''.join(["%02x" % ord(b) for b in msg]))   # Python 2
    else:
        eprint(msg.hex())   # py 3

class ACB:
    """Client program for the antenna control board. """

    def __init__(self, port=None, baudrate=9600, timeout=1, **kwargs):
        """Constructor of ACB class. Uses serial communication. 
        
        Arguments:
            port {str} -- address of the serial port
            baudrate {int} -- baudrate of the serial port
            timeout {int} -- timeout of the serial port
            **kwargs {dictionary} -- extra arguments to be passed.
        """
        self.port = port
        self.ser = serial.Serial(port=port, 
                                 baudrate=baudrate, 
                                 timeout=timeout,
                                 parity=serial.PARITY_NONE,
                                 bytesize=serial.EIGHTBITS,
                                 stopbits=serial.STOPBITS_ONE,
                                 **kwargs)
        if not self.ser.isOpen():
            self.ser.open()

        self.trackbeam_count = 0    # number of track beam command issued,
                                    # needed for send such commands
        
    def receive(self, end=LF):
        """Read response from ACB until the @end character is hit.
        Timeout is recommended.
        
        Keyword Arguments:
            end {char} -- Read will return when this char is hit (default: {LF})

        Returns:
            {str} response message from ACB, or None for error occurred
        """
        if len(end) != 1:
            eprint("[ACB] Invalid end character for receive")
            return None

        eol = end
        leneol = len(eol)
        line = bytearray()
        while True:
            c = self.ser.read(1)
            if c:
                line += c
                if line[-leneol:] == eol:
                    break
            else:
                break
        return bytes(line)


    def send(self, msg):
        """send @msg to the ACB.
        
        Unimplemented for now.
        
        Arguments:
            msg {bytes} -- message to be written

        ReturnS:
            {int} number of bytes written.
        """
        sent = self.ser.write(msg)
        if sent != len(msg):
            print("[ACB] Warning: {:d} bytes of data loss during transmission."\
                .format(len(msg) - sent))
        return sent

    def query(self, msg, waittime=0):
        """Send a command and receive the response.
        
        Arguments:
            msg {bytes} -- command to be send
        
        Keyword Arguments:
            waittime {number} -- wait time between command send and receive 
                                 (default: {0})

        Returns:
            None if failed, response message if success.
        """
        self.send(msg)
        time.sleep(waittime)
        return self.receive()

    def assert_response(self, resp):
        """Assert the response from ACB to check if execution was successful
        
        Arguments:
            resp {bytes} -- response string from the ACB

        Returns:
            {bool} True if the response encodes "success", False otherwise
        """
        try:
            if PY_VERSION == 3:
                resp = resp.decode('ascii')

            if resp[0] == ACK:
                print("[ACB] Command executed.")
                return True

            elif resp[0] == NAK:
                print("[ACB] Command execution failed, error {:s}.".\
                    format(resp[1:].strip("\n\r ")))
                return False

            else:
                print("[ACB] Command resulted in unknown return message.")
                return False

        except (IndexError, TypeError, AttributeError):
            print("[ACB] Command resulted in empty return message.")
            return False

    def trackbeam_on(self, retries=1):
        """Turn on the track beam mode. Try at most @retries time.

        Arguments:
            retries {int} -- how many times to try before report fail.
        
        Returns:
            {bool} true for success, False otherwise
        """
        msg = b"TRACK_BEAM" + CR
        while retries > 0:
            if self.assert_response(self.query(msg)):
                print("[ACB] Switched to track beam mode.")
                return True
            else:
                print("[ACB] FAILED to switch to track beam mode on trail {:d}.".\
                    format(retries))
                retries -=1
                continue

        print("[ACB] All trails FAILED for switching to track beam mode, ACB "
            "configuration unchanged...")
        return False

    def encode_trackbeam(self, apos, epos, arate, erate, mode=0):
        """Encode a set of Azimuth position, Elevation position, Azimuth rate
        and Elevation rate values to a message that ACB can understand.
        See the manual for more details about how this message is encoded.

        Arguments:
            apos {float} -- Azimuth position value in degrees        [0, 360)
            epos {float} -- Elevation position value in degrees      [-5, 95)
            arate {float} -- Azimuth rate value in degree/second     [-16, 16)
            erate {float} -- Elevation rate value in degree/second   [-16, 16)
            mode {int} -- 0 for track beam mode continue, 1 for stop {0, 1}

        Returns: 
            Encoded string on success.

        Raises:
            ValueError -- when any of the input is invalid. See argument for 
                          accepted range.
        """
        if not 0 <= apos < 360: 
            raise ValueError("[ACB] Invalid azimuth position.")
        if not -5 <= epos < 95:
            raise ValueError("[ACB] Invalid elevation position.")
        if not -16 <= arate < 16:
            raise ValueError("[ACB] Invalid azimuth rate.")
        if not -16 <= erate < 16:
            raise ValueError("[ACB] Invalid elevation rate.")
        if mode != 0 and mode != 1:
            raise ValueError("[ACB] Invalid mode setting.")

        # scale values
        sapv = int(round(apos / 180.0 * (2 ** 23)))
        sepv = int(round(epos / 180.0 * (2 ** 23)) if epos >= 0 else \
               int(round((epos + 360) / 180.0 * (2 ** 23))))
        sarv = int(round(arate / 16.0 * (2 ** 15)))
        serv = int(round(erate / 16.0 * (2 ** 15)))

        self.trackbeam_count += 1

        # make the message
        msg = b'PX' + struct.pack(">H", self.trackbeam_count % 0xFFFF) +\
                     struct.pack(">h", sarv) + struct.pack(">h", serv) +\
                     struct.pack("?", mode) +\
                     struct.pack(">L", sapv)[-3:] + struct.pack(">L", sepv)[-3:]

        msg += struct.pack("B", xor_chksum(msg))

        hexprint(msg)
        return msg






































