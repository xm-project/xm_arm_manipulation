#!/usr/bin/env python
'''
/*********************************************************************
 *  Software License Agreement (BSD License)
 *
 *  Created for the XM Robot Project: http://www.github.com/xmproject
 *  Copyright (c) 2015 The XM Robot Team. All rights reserved
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of XM Robot Project nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
'''

# Description: This script pint dynamixel servos.

# Create Date: 2015.11.1

# Authors: myyerrol

from dynamixel_driver.dynamixel_io import *
from arbotix_python.arbotix import ArbotiX
from arbotix_python.ax12 import *
import sys


# Using arbotix_ros package
class TestDynamixelArbotix(ArbotiX):
    def __init__(self, port="/dev/ttyUSB0", baud=1000000):
        ArbotiX.__init__(self, port, baud)
        print "--- Test Dynamixel Communication ---"
        print "This test use arbitix_ros package"

        while True:
            print ">>> ",
            keyboard_cmd = raw_input().split(" ")
            try:
                if keyboard_cmd[0] == "ping":
                    print "id:"
                elif keyboard_cmd[0] == "get":
                    if keyboard_cmd[1] == "position":
                        pos = self.getPosition(int(keyboard_cmd[2]))
                        if pos > 0:
                            print "position:%d" % pos
                        else:
                            print "error:" + str(pos)
                elif keyboard_cmd[0] == "set":
                    if keyboard_cmd[1] == "position":
                        index = int(keyboard_cmd[2])
                        pos = int(keyboard_cmd[3])
                        result = self.setPosition(index, pos)
                        print "callback:" + str(result)
            except Exception as exce:
                print "Error!", exce

    def execute_test(self, index, ins, params, ret=True):
        values = None
        self._mutex.acquire()
        try:
            self._ser.flushInput()
        except Exception as e:
            print e
        length = 2 + len(params)
        checksum = 255 - ((index + length + ins + sum(params)) % 256)
        try:
            self._ser.write(chr(0xFF) + chr(0xFF) + chr(index) + chr(length) + chr(ins))
        except Exception as e:
            print e
            self._mutex.release()
            return None
        for val in params:
            try:
                self._ser.write(chr(val))
            except Exception as e:
                print e
                self._mutex.release()
                return None
        try:
            self._ser.write(chr(checksum))
        except Exception as e:
            print e
            self._mutex.release()
            return None
        if ret:
            values = self.getPacket(0)
        self._mutex.release()
        return values

    def getPosition(self, index):
        values = self.read(index, P_PRESENT_POSITION_L, 2)
        print values
        try:
            return int(values[0]) + (int(values[1]) << 8)
        except:
            return -1


# Using dynamixel_motor package
class TestDynamixelMotor:
    def __init__(self, port="dev/ttyUSB0", baud=1000000):
        DynamixelIO.__init__(self, port, baud)
        print "--- Test Dynamixel Communication ---"
        print "This test use dynamixel_motor package"


if __name__ == "__main__":
    try:
        if len(sys.argv) > 1:
            if sys.argv[1] == "dynamixel_arbotix":
                if (len(sys.argv) - 1) > 2:
                    dynamixel_arbotix = TestDynamixelArbotix(sys.argv[2], int(sys.argv[3]))
                elif (len(sys.argv) - 1) > 1:
                    dynamixel_arbotix = TestDynamixelArbotix(sys.argv[2])
                else:
                    dynamixel_arbotix = TestDynamixelArbotix()
            elif sys.argv[1] == "dynamixel_motor":
                if (len(sys.argv) - 1) > 2:
                    dynamixel_motor = TestDynamixelMotor(sys.argv[2], int(sys.argv[3]))
                elif (len(sys.argv) - 1) > 1:
                    dynamixel_motor = TestDynamixelMotor(sys.argv[2])
                else:
                    dynamixel_motor = TestDynamixelMotor()
        else:
            print "Need at least one parameter to choose which way you will test dynamixel!"
    except KeyboardInterrupt:
        print "Error!\n"
