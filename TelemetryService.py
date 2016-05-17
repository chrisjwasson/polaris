import serial
import struct
import re
from collections import *
import math



class TM_Service(serial.Serial):


    # define TM frame
    frameFields = [
        #   'elapsedTimeSec',
        #   'stringMessage',
            'ax','ay','az',
            'gx','gy','gz',
            'mx','my','mz',
            'q1','q2','q3','q4',
            'lat','lon','alt',
            'deltaTimeSec',
        #   'loopCounter'
            ]
    frameVarSizes = [
        #       'f',            # elapsed time seconds (float)
        #       '32p',          # string message (32 character string)
                'f','f','f',        # acceleration elements (floats)
                'f','f','f',        # gyro rate elements (floats)
                'f','f','f',        # magnet elements (floats)
                'f','f','f','f',    # attitude quaternion elements (floats)
                'f','f','f',        # gps lat lon alt (floats)
                'f',            # TM frame delta time in seconds (float)
        #       'I'         # TM frame counter since init (uint32)
            ]
    frameVarSizes = ''.join(frameVarSizes) # make single string for struct.unpack function
    totalFrameSizeBytes = struct.calcsize(frameVarSizes)
    totalFrameSizeBytesWithEnds = totalFrameSizeBytes+3 # calculate frame size in bytes

    # declare named tuple with that constitutes frame structure
    TM_DataType = namedtuple('TM_Data',frameFields)

    # form telemetry data named tuple
    TM_Data = TM_DataType._make(struct.unpack(frameVarSizes,bytearray(totalFrameSizeBytes)))

    # declare TM data buffer
    dataBuffer = bytearray()

    # compute checksum
    def computeChecksum(self,inputByteArray):

        checksum = 0
        for byte in inputByteArray:
            checksum ^= byte

        return checksum

    def validateContents(self):
        # check if quaternion is normalized, if not, then reject frame
        tol = 1e-5
        if (abs(math.sqrt(
                  self.TM_Data.q1*self.TM_Data.q1
                + self.TM_Data.q2*self.TM_Data.q2
                + self.TM_Data.q3*self.TM_Data.q3
                + self.TM_Data.q4*self.TM_Data.q4) - 1) < tol):
            # return true indicating frame is good and TM was updated
            return True

        else:
            return False

    # pull any new TM and update buffer
    def updateTM(self):

        # read serial buffer into TM buffer
        while self.inWaiting():
            self.dataBuffer.append(self.read())

        # find indices of frame start pattern
        frameStartIdx = self.dataBuffer.find(str(0xABCD))

        # check if there are enough bytes left in the buffer to allow a frame with checksum
        if (len(self.dataBuffer) - frameStartIdx >= self.totalFrameSizeBytesWithEnds):

            # grab vehicle computed checksum from datastream
            telemeteredChecksum = ord(self.dataBuffer[frameStartIdx+self.totalFrameSizeBytesWithEnds-1:frameStartIdx+self.totalFrameSizeBytesWithEnds])
            computedChecksum = self.computeChecksum(self.dataBuffer[frameStartIdx+2:frameStartIdx+self.totalFrameSizeBytesWithEnds-1])
            
            # if checksums match, unpack data into namedtuple, prune that data from the data buffer, and return true
            if (telemeteredChecksum == computedChecksum):

                # unpack buffer data into TM named tuple
                TM_Data = self.TM_DataType._make(struct.unpack(self.frameVarSizes,self.dataBuffer[frameStartIdx+2:frameStartIdx+self.totalFrameSizeBytesWithEnds-1]))

                # prune this data from the buffer
                del self.dataBuffer[0:frameStartIdx+self.totalFrameSizeBytesWithEnds]

                # copy data to TM service data tuple/struct
                self.TM_Data = TM_Data

                # return true indicating frame is good and TM was updated
                return True

            # checksums didn't match - this isn't a frame, discard data preAmble to not consider again
            else:
                del self.dataBuffer[0:frameStartIdx+2]


        # return false indicating that the TM frame named tuple was not updated
        return False



        
        
