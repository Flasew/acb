"""
acb,py
A python implementation of the antenna control board's control system
Author: Weiyang Wang <wew168@ucsd.edu>
"""
from __future__ import print_function
import serial
import struct
import sys
import time

# some special bits
CR = '\r'
LF = '\n'
ACK = 'A'
NAK = 'N'

# Enable print to stderr
def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)

class ACB:
    """Client program for the antenna control board. """

    def __init__(self, port=None, echo=False, 
        baudrate=9600, timeout=1, **kwargs):
    