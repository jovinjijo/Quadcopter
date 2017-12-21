#!/usr/bin/env python

####################################################################################################
####################################################################################################
##                                                                                                ##
## Hove's Raspberry Pi Python Quadcopter Flight Controller.  Open Source @ GitHub                 ##
## PiStuffing/Quadcopter under GPL for non-commercial application.  Any code derived from         ##
## this should retain this copyright comment.                                                     ##
##                                                                                                ##
## Copyright 2012 - 2017 Andy Baker (Hove) - andy@pistuffing.co.uk                                ##
##                                                                                                ##
####################################################################################################
####################################################################################################


from __future__ import division
from __future__ import with_statement
import signal
import socket
import time
import sys
import getopt
import math
from array import *
import smbus
import select
import os
import io
import logging
import csv
from RPIO import PWM
import RPi.GPIO as GPIO
import subprocess
import ctypes
from ctypes.util import find_library
import picamera
import struct
import serial
import threading
#import gps


MIN_SATS = 10
EARTH_RADIUS = 6371000 # meters
GRAV_ACCEL = 9.80665   # meters per second per second


####################################################################################################
#
#  Adafruit i2c interface enhanced with performance / error handling enhancements
#
####################################################################################################
class I2C:

    def __init__(self, address, bus=smbus.SMBus(1)):
        self.address = address
        self.bus = bus
        self.misses = 0

    def reverseByteOrder(self, data):
        "Reverses the byte order of an int (16-bit) or long (32-bit) value"
        # Courtesy Vishal Sapre
        dstr = hex(data)[2:].replace('L','')
        byteCount = len(dstr[::2])
        val = 0
        for i, n in enumerate(range(byteCount)):
            d = data & 0xFF
            val |= (d << (8 * (byteCount - i - 1)))
            data >>= 8
        return val

    def writeByte(self, value):
        while True:
            try:
                self.bus.write_byte(self.address, value)
                break
            except IOError, err:
                self.misses += 1

    def write8(self, reg, value):
        "Writes an 8-bit value to the specified register/address"
        while True:
            try:
                self.bus.write_byte_data(self.address, reg, value)
                break
            except IOError, err:
                self.misses += 1

    def writeList(self, reg, list):
        "Writes an array of bytes using I2C format"
        while True:
            try:
                self.bus.write_i2c_block_data(self.address, reg, list)
                break
            except IOError, err:
                self.misses += 1

    def readU8(self, reg):
        "Read an unsigned byte from the I2C device"
        while True:
            try:
                result = self.bus.read_byte_data(self.address, reg)
                return result
            except IOError, err:
                self.misses += 1

    def readS8(self, reg):
        "Reads a signed byte from the I2C device"
        while True:
            try:
                result = self.bus.read_byte_data(self.address, reg)
                if (result > 127):
                    return result - 256
                else:
                    return result
            except IOError, err:
                self.misses += 1

    def readU16(self, reg):
        "Reads an unsigned 16-bit value from the I2C device"
        while True:
            try:
                hibyte = self.bus.read_byte_data(self.address, reg)
                result = (hibyte << 8) + self.bus.read_byte_data(self.address, reg+1)
                return result
            except IOError, err:
                self.misses += 1

    def readS16(self, reg):
        "Reads a signed 16-bit value from the I2C device"
        while True:
            try:
                hibyte = self.bus.read_byte_data(self.address, reg)
                if (hibyte > 127):
                    hibyte -= 256
                result = (hibyte << 8) + self.bus.read_byte_data(self.address, reg+1)
                return result
            except IOError, err:
                self.misses += 1

    def readList(self, reg, length):
        "Reads a byte array value from the I2C device"
        result = self.bus.read_i2c_block_data(self.address, reg, length)
        return result

    def getMisses(self):
        return self.misses


####################################################################################################
#
#  Gyroscope / Accelerometer class for reading position / movement.  Works with the Invensense IMUs:
#
#  - MPU-6050
#  - MPU-9150
#  - MPU-9250
#
####################################################################################################
class MPU6050:
    i2c = None

    # Registers/etc.
    __MPU6050_RA_SELF_TEST_XG = 0x00
    __MPU6050_RA_SELF_TEST_YG = 0x01
    __MPU6050_RA_SELF_TEST_ZG = 0x02
    __MPU6050_RA_SELF_TEST_XA = 0x0D
    __MPU6050_RA_SELF_TEST_YA = 0x0E
    __MPU6050_RA_SELF_TEST_ZA = 0x0F
    __MPU6050_RA_XG_OFFS_USRH = 0x13
    __MPU6050_RA_XG_OFFS_USRL = 0x14
    __MPU6050_RA_YG_OFFS_USRH = 0x15
    __MPU6050_RA_YG_OFFS_USRL = 0x16
    __MPU6050_RA_ZG_OFFS_USRH = 0x17
    __MPU6050_RA_ZG_OFFS_USRL = 0x18
    __MPU6050_RA_SMPLRT_DIV = 0x19
    __MPU6050_RA_CONFIG = 0x1A
    __MPU6050_RA_GYRO_CONFIG = 0x1B
    __MPU6050_RA_ACCEL_CONFIG = 0x1C
    __MPU9250_RA_ACCEL_CFG_2 = 0x1D
    __MPU6050_RA_FF_THR = 0x1D
    __MPU6050_RA_FF_DUR = 0x1E
    __MPU6050_RA_MOT_THR = 0x1F
    __MPU6050_RA_MOT_DUR = 0x20
    __MPU6050_RA_ZRMOT_THR = 0x21
    __MPU6050_RA_ZRMOT_DUR = 0x22
    __MPU6050_RA_FIFO_EN = 0x23
    __MPU6050_RA_I2C_MST_CTRL = 0x24
    __MPU6050_RA_I2C_SLV0_ADDR = 0x25
    __MPU6050_RA_I2C_SLV0_REG = 0x26
    __MPU6050_RA_I2C_SLV0_CTRL = 0x27
    __MPU6050_RA_I2C_SLV1_ADDR = 0x28
    __MPU6050_RA_I2C_SLV1_REG = 0x29
    __MPU6050_RA_I2C_SLV1_CTRL = 0x2A
    __MPU6050_RA_I2C_SLV2_ADDR = 0x2B
    __MPU6050_RA_I2C_SLV2_REG = 0x2C
    __MPU6050_RA_I2C_SLV2_CTRL = 0x2D
    __MPU6050_RA_I2C_SLV3_ADDR = 0x2E
    __MPU6050_RA_I2C_SLV3_REG = 0x2F
    __MPU6050_RA_I2C_SLV3_CTRL = 0x30
    __MPU6050_RA_I2C_SLV4_ADDR = 0x31
    __MPU6050_RA_I2C_SLV4_REG = 0x32
    __MPU6050_RA_I2C_SLV4_DO = 0x33
    __MPU6050_RA_I2C_SLV4_CTRL = 0x34
    __MPU6050_RA_I2C_SLV4_DI = 0x35
    __MPU6050_RA_I2C_MST_STATUS = 0x36
    __MPU6050_RA_INT_PIN_CFG = 0x37
    __MPU6050_RA_INT_ENABLE = 0x38
    __MPU6050_RA_DMP_INT_STATUS = 0x39
    __MPU6050_RA_INT_STATUS = 0x3A
    __MPU6050_RA_ACCEL_XOUT_H = 0x3B
    __MPU6050_RA_ACCEL_XOUT_L = 0x3C
    __MPU6050_RA_ACCEL_YOUT_H = 0x3D
    __MPU6050_RA_ACCEL_YOUT_L = 0x3E
    __MPU6050_RA_ACCEL_ZOUT_H = 0x3F
    __MPU6050_RA_ACCEL_ZOUT_L = 0x40
    __MPU6050_RA_TEMP_OUT_H = 0x41
    __MPU6050_RA_TEMP_OUT_L = 0x42
    __MPU6050_RA_GYRO_XOUT_H = 0x43
    __MPU6050_RA_GYRO_XOUT_L = 0x44
    __MPU6050_RA_GYRO_YOUT_H = 0x45
    __MPU6050_RA_GYRO_YOUT_L = 0x46
    __MPU6050_RA_GYRO_ZOUT_H = 0x47
    __MPU6050_RA_GYRO_ZOUT_L = 0x48
    __MPU6050_RA_EXT_SENS_DATA_00 = 0x49
    __MPU6050_RA_EXT_SENS_DATA_01 = 0x4A
    __MPU6050_RA_EXT_SENS_DATA_02 = 0x4B
    __MPU6050_RA_EXT_SENS_DATA_03 = 0x4C
    __MPU6050_RA_EXT_SENS_DATA_04 = 0x4D
    __MPU6050_RA_EXT_SENS_DATA_05 = 0x4E
    __MPU6050_RA_EXT_SENS_DATA_06 = 0x4F
    __MPU6050_RA_EXT_SENS_DATA_07 = 0x50
    __MPU6050_RA_EXT_SENS_DATA_08 = 0x51
    __MPU6050_RA_EXT_SENS_DATA_09 = 0x52
    __MPU6050_RA_EXT_SENS_DATA_10 = 0x53
    __MPU6050_RA_EXT_SENS_DATA_11 = 0x54
    __MPU6050_RA_EXT_SENS_DATA_12 = 0x55
    __MPU6050_RA_EXT_SENS_DATA_13 = 0x56
    __MPU6050_RA_EXT_SENS_DATA_14 = 0x57
    __MPU6050_RA_EXT_SENS_DATA_15 = 0x58
    __MPU6050_RA_EXT_SENS_DATA_16 = 0x59
    __MPU6050_RA_EXT_SENS_DATA_17 = 0x5A
    __MPU6050_RA_EXT_SENS_DATA_18 = 0x5B
    __MPU6050_RA_EXT_SENS_DATA_19 = 0x5C
    __MPU6050_RA_EXT_SENS_DATA_20 = 0x5D
    __MPU6050_RA_EXT_SENS_DATA_21 = 0x5E
    __MPU6050_RA_EXT_SENS_DATA_22 = 0x5F
    __MPU6050_RA_EXT_SENS_DATA_23 = 0x60
    __MPU6050_RA_MOT_DETECT_STATUS = 0x61
    __MPU6050_RA_I2C_SLV0_DO = 0x63
    __MPU6050_RA_I2C_SLV1_DO = 0x64
    __MPU6050_RA_I2C_SLV2_DO = 0x65
    __MPU6050_RA_I2C_SLV3_DO = 0x66
    __MPU6050_RA_I2C_MST_DELAY_CTRL = 0x67
    __MPU6050_RA_SIGNAL_PATH_RESET = 0x68
    __MPU6050_RA_MOT_DETECT_CTRL = 0x69
    __MPU6050_RA_USER_CTRL = 0x6A
    __MPU6050_RA_PWR_MGMT_1 = 0x6B
    __MPU6050_RA_PWR_MGMT_2 = 0x6C
    __MPU6050_RA_BANK_SEL = 0x6D
    __MPU6050_RA_MEM_START_ADDR = 0x6E
    __MPU6050_RA_MEM_R_W = 0x6F
    __MPU6050_RA_DMP_CFG_1 = 0x70
    __MPU6050_RA_DMP_CFG_2 = 0x71
    __MPU6050_RA_FIFO_COUNTH = 0x72
    __MPU6050_RA_FIFO_COUNTL = 0x73
    __MPU6050_RA_FIFO_R_W = 0x74
    __MPU6050_RA_WHO_AM_I = 0x75

    #-----------------------------------------------------------------------------------------------
    # Compass output registers when using the I2C master / slave
    #-----------------------------------------------------------------------------------------------
    __MPU9250_RA_MAG_XOUT_L = 0x4A
    __MPU9250_RA_MAG_XOUT_H = 0x4B
    __MPU9250_RA_MAG_YOUT_L = 0x4C
    __MPU9250_RA_MAG_YOUT_H = 0x4D
    __MPU9250_RA_MAG_ZOUT_L = 0x4E
    __MPU9250_RA_MAG_ZOUT_H = 0x4F

    #-----------------------------------------------------------------------------------------------
    # Compass output registers when directly accessing via IMU bypass
    #-----------------------------------------------------------------------------------------------
    __AK893_RA_WIA = 0x00
    __AK893_RA_INFO = 0x01
    __AK893_RA_ST1 = 0x00

    __AK893_RA_X_LO = 0x04
    __AK893_RA_X_HI = 0x03
    __AK893_RA_Y_LO = 0x08
    __AK893_RA_Y_HI = 0x07
    __AK893_RA_Z_LO = 0x06
    __AK893_RA_Z_HI = 0x05

    __AK893_RA_ST2 = 0x09
    __AK893_RA_CNTL1 = 0x0A
    __AK893_RA_RSV = 0x0B
    __AK893_RA_ASTC = 0x0C
    __AK893_RA_TS1 = 0x0D
    __AK893_RA_TS2 = 0x0E
    __AK893_RA_I2CDIS = 0x0F
    __AK893_RA_ASAX = 0x10
    __AK893_RA_ASAY = 0x11
    __AK893_RA_ASAZ = 0x12

    __RANGE_ACCEL = 8                                                            #AB: +/- 8g
    __RANGE_GYRO = 250                                                           #AB: +/- 250o/s

    __SCALE_GYRO = math.radians(2 * __RANGE_GYRO / 65536)
    __SCALE_ACCEL = 2 * __RANGE_ACCEL / 65536

    def __init__(self, address=0x68, alpf=2, glpf=1):
        self.i2c = I2C(address)
        self.address = address

        self.min_az = 0
        self.max_az = 0
        self.min_gx = 0
        self.max_gx = 0
        self.min_gy = 0
        self.max_gy = 0
        self.min_gz = 0
        self.max_gz = 0

        self.ax_offset = 0.0
        self.ay_offset = 0.0
        self.az_offset = 0.0

        self.gx_offset = 0.0
        self.gy_offset = 0.0
        self.gz_offset = 0.0

        logger.info('Reseting MPU-6050')

        #-------------------------------------------------------------------------------------------
        # Reset all registers
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_PWR_MGMT_1, 0x80)
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Sets sample rate to 1kHz/(1+0) = 1kHz or 1ms (note 1kHz assumes dlpf is on - setting
        # dlpf to 0 or 7 changes 1kHz to 8kHz and therefore will require sample rate divider
        # to be changed to 7 to obtain the same 1kHz sample rate.
        #-------------------------------------------------------------------------------------------
        sample_rate_divisor = int(round(adc_frequency / sampling_rate))
        logger.warning("SRD:, %d", sample_rate_divisor)
        self.i2c.write8(self.__MPU6050_RA_SMPLRT_DIV, sample_rate_divisor - 1)
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Sets clock source to gyro reference w/ PLL
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_PWR_MGMT_1, 0x01)
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Gyro DLPF => 1kHz sample frequency used above divided by the sample divide factor.
        #
        # 0x00 =  250Hz @ 8kHz sampling - DO NOT USE, THE ACCELEROMETER STILL SAMPLES AT 1kHz WHICH PRODUCES EXPECTED BUT NOT CODED FOR TIMING AND FIFO CONTENT PROBLEMS
        # 0x01 =  184Hz
        # 0x02 =   92Hz
        # 0x03 =   41Hz
        # 0x04 =   20Hz
        # 0x05 =   10Hz
        # 0x06 =    5Hz
        # 0x07 = 3600Hz @ 8kHz
        #
        # 0x0* FIFO overflow overwrites oldest FIFO contents
        # 0x4* FIFO overflow does not overwrite full FIFO contents
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_CONFIG, 0x40 | glpf)
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Disable gyro self tests, scale of +/- 250 degrees/s
        #
        # 0x00 =  +/- 250 degrees/s
        # 0x08 =  +/- 500 degrees/s
        # 0x10 = +/- 1000 degrees/s
        # 0x18 = +/- 2000 degrees/s
        # See SCALE_GYRO for conversion from raw data to units of radians per second
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_GYRO_CONFIG, int(round(math.log(self.__RANGE_GYRO / 250, 2))) << 3)
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Accel DLPF => 1kHz sample frequency used above divided by the sample divide factor.
        #
        # 0x00 = 460Hz
        # 0x01 = 184Hz
        # 0x02 =  92Hz
        # 0x03 =  41Hz
        # 0x04 =  20Hz
        # 0x05 =  10Hz
        # 0x06 =   5Hz
        # 0x07 = 460Hz
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU9250_RA_ACCEL_CFG_2, alpf)
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Disable accel self tests, scale of +/-8g
        #
        # 0x00 =  +/- 2g
        # 0x08 =  +/- 4g
        # 0x10 =  +/- 8g
        # 0x18 = +/- 16g
        # See SCALE_ACCEL for convertion from raw data to units of meters per second squared
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_ACCEL_CONFIG, int(round(math.log(self.__RANGE_ACCEL / 2, 2))) << 3)
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Set INT pin to push/pull, latch 'til read, any read to clear
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_INT_PIN_CFG, 0x30)
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Initialize the FIFO overflow interrupt 0x10 (turned off at startup).
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_INT_ENABLE, 0x00)
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Enabled the FIFO.
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_USER_CTRL, 0x40)

        #-------------------------------------------------------------------------------------------
        # Accelerometer / gyro goes into FIFO later on - see flushFIFO()
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_FIFO_EN, 0x00)

        #-------------------------------------------------------------------------------------------
        # Read ambient temperature
        #-------------------------------------------------------------------------------------------
        temp = self.readTemperature()
        logger.critical("IMU core temp (boot): ,%f", temp / 333.86 + 21.0)

    def readTemperature(self):
        temp = self.i2c.readS16(self.__MPU6050_RA_TEMP_OUT_H)
        return temp

    def enableFIFOOverflowISR(self):
        #-------------------------------------------------------------------------------------------
        # Clear the interrupt status register and enable the FIFO overflow interrupt 0x10
        #-------------------------------------------------------------------------------------------
        '''
        #AB! Something odd here: can't clear the GPIO pin if the ISR is enabled, and then status read
        #AB! in that order
        '''
        self.i2c.write8(self.__MPU6050_RA_INT_ENABLE, 0x10)
        self.i2c.readU8(self.__MPU6050_RA_INT_STATUS)

    def disableFIFOOverflowISR(self):
        #-------------------------------------------------------------------------------------------
        # Disable the FIFO overflow interrupt.
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_INT_ENABLE, 0x00)

    def numFIFOBatches(self):
        #-------------------------------------------------------------------------------------------
        # The FIFO is 512 bytes long, and we're storing 6 signed shorts (ax, ay, az, gx, gy, gz) i.e.
        # 12 bytes per batch of sensor readings
        #-------------------------------------------------------------------------------------------
        fifo_bytes = self.i2c.readU16(self.__MPU6050_RA_FIFO_COUNTH)
        fifo_batches = int(fifo_bytes / 12)  # This rounds down

        return fifo_batches

    def readFIFO(self, fifo_batches):
        #-------------------------------------------------------------------------------------------
        # Read n x 12 bytes of FIFO data averaging, and return the averaged values and inferred time
        # based upon the sampling rate and the number of samples.
        #-------------------------------------------------------------------------------------------
        ax = 0.0
        ay = 0.0
        az = 0.0
        gx = 0.0
        gy = 0.0
        gz = 0.0

        for ii in range(fifo_batches):
            sensor_data = []
            fifo_batch = self.i2c.readList(self.__MPU6050_RA_FIFO_R_W, 12)
            for jj in range(0, 12, 2):
                hibyte = fifo_batch[jj]
                lobyte = fifo_batch[jj + 1]
                if (hibyte > 127):
                    hibyte -= 256

                sensor_data.append((hibyte << 8) + lobyte)

            ax += sensor_data[0]
            ay += sensor_data[1]
            az += sensor_data[2]
            gx += sensor_data[3]
            gy += sensor_data[4]
            gz += sensor_data[5]

            self.max_az = self.max_az if sensor_data[2] < self.max_az else sensor_data[2]
            self.min_az = self.min_az if sensor_data[2] > self.min_az else sensor_data[2]

            self.max_gx = self.max_gx if sensor_data[3] < self.max_gx else sensor_data[3]
            self.min_gx = self.min_gx if sensor_data[3] > self.min_gx else sensor_data[3]

            self.max_gy = self.max_gy if sensor_data[4] < self.max_gy else sensor_data[4]
            self.min_gy = self.min_gy if sensor_data[4] > self.min_gy else sensor_data[4]

            self.max_gz = self.max_gz if sensor_data[5] < self.max_gz else sensor_data[5]
            self.min_gz = self.min_gz if sensor_data[5] > self.min_gz else sensor_data[5]

        ax /= fifo_batches
        ay /= fifo_batches
        az /= fifo_batches
        gx /= fifo_batches
        gy /= fifo_batches
        gz /= fifo_batches
        #return ax, ay, az, gx, gy, gz, fifo_batches / sampling_rate
        return ay, -ax, az, gy, -gx, gz, fifo_batches / sampling_rate #changed
        # ay, -ax, gy, -gx -> YEP
        #az - always +ve
        #gz -> -ve
        # -ay, -ax, -gy, -gx -> NOPE
        # -ay, -ax, gy, -gx -> NOPE
        # -ay, -ax, -gy, gx -> NOPE
        # -ay, -ax, gy, gx -> NOPE
        
        # ay, ax, -gy, -gx -> NOPE
        # ay, ax, gy, gx -> NOPE
        # ay, ax, gy, -gx -> NOPE
        # ay, ax, -gy, gx -> NOPE
        
        # -ay, ax, gy, gx -> NOPE
        # -ay, ax, -gy, gx -> NOPE
        # -ay, ax, gy, -gx -> NOPE
        # -ay, ax, -gy, -gx -> NOPE

        # ay, -ax, -gy, -gx -> NOPE
        # ay, -ax, -gy, gx -> NOPE
        # ay, -ax, gy, gx -> NOPE
        # ay, -ax, gy, -gx -> NOPE

    def flushFIFO(self):
        #-------------------------------------------------------------------------------------------
        # First shut off the feed in the FIFO.
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_FIFO_EN, 0x00)

        #-------------------------------------------------------------------------------------------
        # Empty the FIFO by reading whatever is there
        #-------------------------------------------------------------------------------------------
        SMBUS_MAX_BUF_SIZE = 32

        fifo_bytes = self.i2c.readU16(self.__MPU6050_RA_FIFO_COUNTH)

        for ii in range(int(fifo_bytes / SMBUS_MAX_BUF_SIZE)):
            self.i2c.readList(self.__MPU6050_RA_FIFO_R_W, SMBUS_MAX_BUF_SIZE)

        fifo_bytes = self.i2c.readU16(self.__MPU6050_RA_FIFO_COUNTH)

        for ii in range(fifo_bytes):
            self.i2c.readU8(self.__MPU6050_RA_FIFO_R_W)

        #-------------------------------------------------------------------------------------------
        # Finally start feeding the FIFO with sensor data again
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_FIFO_EN, 0x78)

    def setGyroOffsets(self, gx, gy, gz):
        self.gx_offset = gx
        self.gy_offset = gy
        self.gz_offset = gz

    def scaleSensors(self, ax, ay, az, gx, gy, gz):
        qax = (ax - self.ax_offset) * self.__SCALE_ACCEL
        qay = (ay - self.ay_offset) * self.__SCALE_ACCEL
        qaz = (az - self.az_offset) * self.__SCALE_ACCEL

        qrx = (gx - self.gx_offset) * self.__SCALE_GYRO
        qry = (gy - self.gy_offset) * self.__SCALE_GYRO
        qrz = (gz - self.gz_offset) * self.__SCALE_GYRO

        return qax, qay, qaz, qrx, qry, qrz

    def initCompass(self):
        #-------------------------------------------------------------------------------------------
        # Set up the I2C master pass through.
        #-------------------------------------------------------------------------------------------
        int_bypass = self.i2c.readU8(self.__MPU6050_RA_INT_PIN_CFG)
        self.i2c.write8(self.__MPU6050_RA_INT_PIN_CFG, int_bypass | 0x02)

        #-------------------------------------------------------------------------------------------
        # Connect directly to the bypassed magnetometer, and configured it for 16 bit continuous data
        #-------------------------------------------------------------------------------------------

        #self.i2c_compass = I2C(0x0C)
        self.i2c_compass = I2C(0x1E)
        #self.i2c_compass.write8(self.__AK893_RA_CNTL1, 0x16);  #TODO  Continuous measurement mode 2 - 100Hz refresh rate 16 bit measurements
        #HMC5883L - 75Hz, 8 samples per measurement, normal measurement configuration - write 0x78 to 0x00
        self.i2c_compass.write8(0x00, 0x78)
        #Default gain - write 0x20 to 0x01
        self.i2c_compass.write8(0x01, 0x20)
        #continuous measurement mode - write 0x00 to 0x02
        self.i2c_compass.write8(0x02, 0x00)

    def readCompass(self):
        compass_bytes = self.i2c_compass.readList(self.__AK893_RA_X_LO, 6)

        #-------------------------------------------------------------------------------------------
        # Convert the array of 6 bytes to 3 shorts - 7th byte kicks off another read.
        # Note compass X, Y, Z are aligned with GPS not IMU i.e. X = 0, Y = 1 => 0 degrees North
        #-------------------------------------------------------------------------------------------
        compass_data = []
        for ii in range(0, 6, 2):
            lobyte = compass_bytes[ii + 1]
            hibyte = compass_bytes[ii]
            if (hibyte > 127):
                hibyte -= 256

            compass_data.append((hibyte << 8) + lobyte)

        [mgx, mgz, mgy] = compass_data

        mgx = (mgx - self.mgx_offset) * self.mgx_gain
        mgy = (mgy - self.mgy_offset) * self.mgy_gain
        mgz = (mgz - self.mgz_offset) * self.mgz_gain

        return mgx, mgy, mgz

    def compassCheckCalibrate(self):
        rc = True
        while True:
            coc = raw_input("'check' or 'calibrate'? ")
            if coc == "check":
                self.checkCompass()
                break
            elif coc == "calibrate":
                rc = self.calibrateCompass()
                break
        return rc

    def checkCompass(self):
        print "Pop me on the ground pointing in a known direction based on another compass."
        raw_input("Press enter when that's done, and I'll tell you which way I think I'm pointing")

        self.loadCompassCalibration()
        mgx, mgy, mgz = self.readCompass()

        #-------------------------------------------------------------------------------
        # Convert compass vector into N, S, E, W variants.  Get the compass angle in the
        # range of 0 - 359.99.
        #-------------------------------------------------------------------------------
        compass_angle = math.degrees(math.atan2(mgx, mgy))
        if compass_angle < 0:
            compass_angle += 360

        #-------------------------------------------------------------------------------
        # There are 16 possible compass directions when you include things like NNE at
        # 22.5 degrees.
        #-------------------------------------------------------------------------------
        compass_points = ("N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW")
        num_compass_points = len(compass_points)
        for ii in range(len(compass_points)):

            angle_range_min = 360 * (ii - 0.5) / num_compass_points
            angle_range_max = 360 * (ii + 0.5) / num_compass_points
            if angle_range_max < angle_range_min:
                angle_angle_min -= 360

            if compass_angle > angle_range_min and compass_angle <= angle_range_max:
                break

        print "I think I'm pointing %s?" % compass_points[ii]


    def calibrateCompass(self):
        self.mgx_offset = 0.0
        self.mgy_offset = 0.0
        self.mgz_offset = 0.0
        self.mgx_gain = 1.0
        self.mgy_gain = 1.0
        self.mgz_gain = 1.0
        offs_rc = False

        #-------------------------------------------------------------------------------------------
        # First we need gyro offset calibration.  Flush the FIFO, collect roughly half a FIFO full
        # of samples and feed back to the gyro offset calibrations.
        #-------------------------------------------------------------------------------------------
        raw_input("First, put me on a stable surface, and press enter.")

        mpu6050.flushFIFO()
        time.sleep(20 / sampling_rate)
        nfb = mpu6050.numFIFOBatches()
        qax, qay, qaz, qrx, qry, qrz, dt = mpu6050.readFIFO(nfb)
        mpu6050.setGyroOffsets(qrx, qry, qrz)

        print "OK, thanks.  That's the gyro calibrated."

        #-------------------------------------------------------------------------------------------
        # Open the offset file for this run
        #-------------------------------------------------------------------------------------------
        try:
            with open('CompassOffsets', 'ab') as offs_file:

                mgx, mgy, mgz = self.readCompass()
                max_mgx = mgx
                min_mgx = mgx
                max_mgy = mgy
                min_mgy = mgy
                max_mgz = mgz
                min_mgz = mgz

                #-----------------------------------------------------------------------------------
                # Collect compass X. Y compass values
                #-------------------------------------------------------------------------------
                GPIO.output(GPIO_LED, GPIO.HIGH)
                print "Now, pick me up and rotate me horizontally twice until the light stops flashing."
                raw_input("Press enter when you're ready to go.")

                self.flushFIFO()

                yaw = 0.0
                total_dt = 0.0

                print "ROTATION:    ",
                number_len = 0

                #-------------------------------------------------------------------------------
                # While integrated Z axis gyro < 2 pi i.e. 360 degrees, keep flashing the light
                #-------------------------------------------------------------------------------
                while abs(yaw) < 4 * math.pi:
                    time.sleep(10 / sampling_rate)

                    nfb = mpu6050.numFIFOBatches()
                    ax, ay, az, gx, gy, gz, dt = self.readFIFO(nfb)
                    ax, ay, az, gx, gy, gz = self.scaleSensors(ax, ay, az, gx, gy, gz)

                    '''
                    #AB! Do we get Eulers here to rotate back to earth?
                    '''

                    yaw += gz * dt
                    total_dt += dt

                    mgx, mgy, mgz = self.readCompass()

                    max_mgx = mgx if mgx > max_mgx else max_mgx
                    max_mgy = mgy if mgy > max_mgy else max_mgy
                    min_mgx = mgx if mgx < min_mgx else min_mgx
                    min_mgy = mgy if mgy < min_mgy else min_mgy

                    if total_dt > 0.2:
                        total_dt %= 0.2

                        number_text = str(abs(int(math.degrees(yaw))))
                        if len(number_text) == 2:
                            number_text = " " + number_text
                        elif len(number_text) == 1:
                            number_text = "  " + number_text

                        print "\b\b\b\b%s" % number_text,
                        sys.stdout.flush()

                        GPIO.output(GPIO_LED, not GPIO.input(GPIO_LED))
                print

                #-------------------------------------------------------------------------------
                # Collect compass Z values
                #-------------------------------------------------------------------------------
                GPIO.output(GPIO_LED, GPIO.LOW)
                print "\nGreat!  Now do the same but with my nose down."
                raw_input("Press enter when you're ready to go.")

                self.flushFIFO()

                rotation = 0.0
                total_dt = 0.0

                print "ROTATION:    ",
                number_len = 0

                #-------------------------------------------------------------------------------
                # While integrated X+Y axis gyro < 2 pi i.e. 360 degrees, keep flashing the light
                #-------------------------------------------------------------------------------
                while abs(rotation) < 4 * math.pi:
                    time.sleep(10 / sampling_rate)

                    nfb = self.numFIFOBatches()
                    ax, ay, az, gx, gy, gz, dt = self.readFIFO(nfb)
                    ax, ay, az, gx, gy, gz = self.scaleSensors(ax, ay, az, gx, gy, gz)

                    rotation += math.pow(math.pow(gx, 2) + math.pow(gy, 2), 0.5) * dt
                    total_dt += dt

                    mgx, mgy, mgz = self.readCompass()

                    max_mgz = mgz if mgz > max_mgz else max_mgz
                    min_mgz = mgz if mgz < min_mgz else min_mgz

                    if total_dt > 0.2:
                        total_dt %= 0.2

                        number_text = str(abs(int(math.degrees(rotation))))
                        if len(number_text) == 2:
                            number_text = " " + number_text
                        elif len(number_text) == 1:
                            number_text = "  " + number_text

                        print "\b\b\b\b%s" % number_text,
                        sys.stdout.flush()

                        GPIO.output(GPIO_LED, not GPIO.input(GPIO_LED))
                print

                #-------------------------------------------------------------------------------
                # Turn the light off regardless of the result
                #-------------------------------------------------------------------------------
                GPIO.output(GPIO_LED, GPIO.LOW)

                #-------------------------------------------------------------------------------
                # Write the good output to file.
                #-------------------------------------------------------------------------------
                mgx_offset = (max_mgx + min_mgx) / 2
                mgy_offset = (max_mgy + min_mgy) / 2
                mgz_offset = (max_mgz + min_mgz) / 2
                mgx_gain = 1 / (max_mgx - min_mgx)
                mgy_gain = 1 / (max_mgy - min_mgy)
                mgz_gain = 1 / (max_mgz - min_mgz)

                offs_file.write("%f %f %f %f %f %f\n" % (mgx_offset, mgy_offset, mgz_offset, mgx_gain, mgy_gain, mgz_gain))

                #-------------------------------------------------------------------------------
                # Sanity check.
                #-------------------------------------------------------------------------------
                print "\nLooking good, just one last check to confirm all's well."
                self.checkCompass()

                print "All done - ready to go!"
                offs_rc = True

        except EnvironmentError as e:
            print "Environment Error: '%s'" % e

        return offs_rc

    def loadCompassCalibration(self):

        self.mgx_offset = 0.0
        self.mgy_offset = 0.0
        self.mgz_offset = 0.0
        self.mgx_gain = 1.0
        self.mgy_gain = 1.0
        self.mgz_gain = 1.0

        offs_rc = False
        try:
            with open('CompassOffsets', 'rb') as offs_file:
                mgx_offset = 0.0
                mgy_offset = 0.0
                mgz_offset = 0.0
                mgx_gain = 1.0
                mgy_gain = 1.0
                mgz_gain = 1.0

                for line in offs_file:
                    mgx_offset, mgy_offset, mgz_offset, mgx_gain, mgy_gain, mgz_gain = line.split()

                self.mgx_offset = float(mgx_offset)
                self.mgy_offset = float(mgy_offset)
                self.mgz_offset = float(mgz_offset)
                self.mgx_gain = float(mgx_gain)
                self.mgy_gain = float(mgy_gain)
                self.mgz_gain = float(mgz_gain)

        except EnvironmentError:
            #---------------------------------------------------------------------------------------
            # Compass calibration is essential to exclude soft magnetic fields such as from local
            # metal; enforce a recalibration if not found.
            #---------------------------------------------------------------------------------------
            print "Oops, something went wrong reading the compass offsets file 'CompassOffsets'"
            print "Have you calibrated it (--cc)?"

            offs_rc = False
        else:
            #---------------------------------------------------------------------------------------
            # Calibration results were successful.
            #---------------------------------------------------------------------------------------
            offs_rc = True
        finally:
            pass

        logger.warning("Compass Offsets:, %f, %f, %f, Compass Gains:, %f, %f, %f", self.mgx_offset,
                                                                                   self.mgy_offset,
                                                                                   self.mgz_offset,
                                                                                   self.mgx_gain,
                                                                                   self.mgy_gain,
                                                                                   self.mgz_gain)
        return offs_rc

    def getStats(self):
        return (self.max_az * self.__SCALE_ACCEL,
                self.min_az * self.__SCALE_ACCEL,
                self.max_gx * self.__SCALE_GYRO,
                self.min_gx * self.__SCALE_GYRO,
                self.max_gy * self.__SCALE_GYRO,
                self.min_gy * self.__SCALE_GYRO,
                self.max_gz * self.__SCALE_GYRO,
                self.min_gz * self.__SCALE_GYRO)

####################################################################################################
#
# PID algorithm to take input sensor readings, and target requirements, and output an arbirtrary
# corrective value.
#
####################################################################################################
class PID:

    def __init__(self, p_gain, i_gain, d_gain):
        self.last_error = 0.0
        self.p_gain = p_gain
        self.i_gain = i_gain
        self.d_gain = d_gain
        self.i_error = 0.0

    def Error(self, input, target):
        return (target - input)


    def Compute(self, input, target, dt):
        #-------------------------------------------------------------------------------------------
        # Error is what the PID alogithm acts upon to derive the output
        #-------------------------------------------------------------------------------------------
        error = self.Error(input, target)

        #-------------------------------------------------------------------------------------------
        # The proportional term takes the distance between current input and target
        # and uses this proportially (based on Kp) to control the ESC pulse width
        #-------------------------------------------------------------------------------------------
        p_error = error

        #-------------------------------------------------------------------------------------------
        # The integral term sums the errors across many compute calls to allow for
        # external factors like wind speed and friction
        #-------------------------------------------------------------------------------------------
        self.i_error += (error + self.last_error) * dt
        i_error = self.i_error

        #-------------------------------------------------------------------------------------------
        # The differential term accounts for the fact that as error approaches 0,
        # the output needs to be reduced proportionally to ensure factors such as
        # momentum do not cause overshoot.
        #-------------------------------------------------------------------------------------------
        d_error = (error - self.last_error) / dt

        #-------------------------------------------------------------------------------------------
        # The overall output is the sum of the (P)roportional, (I)ntegral and (D)iffertial terms
        #-------------------------------------------------------------------------------------------
        p_output = self.p_gain * p_error
        i_output = self.i_gain * i_error
        d_output = self.d_gain * d_error

        #-------------------------------------------------------------------------------------------
        # Store off last error for integral and differential processing next time.
        #-------------------------------------------------------------------------------------------
        self.last_error = error

        #-------------------------------------------------------------------------------------------
        # Return the output, which has been tuned to be the increment / decrement in ESC PWM
        #-------------------------------------------------------------------------------------------
        return p_output, i_output, d_output


####################################################################################################
#
# PID algorithm subclass to come with the yaw angles error calculations.
#
####################################################################################################
class YAW_PID(PID):

    def Error(self, input, target):
        #-------------------------------------------------------------------------------------------
        # An example in degrees is the best way to explain this.  If the target is -179 degrees
        # and the input is +179 degrees, the standard PID output would be -358 degrees leading to
        # a very high yaw rotation rate to correct the -358 degrees error.  However, +2 degrees
        # achieves the same result, with a much lower rotation rate to fix the error.
        #-------------------------------------------------------------------------------------------
        error = ((target - input) + math.pi) % (2 * math.pi) - math.pi
        return error


####################################################################################################
#
#  Class for managing each blade + motor configuration via its ESC
#
####################################################################################################
class ESC:

    def __init__(self, pin, location, rotation, name, spin):
        #-------------------------------------------------------------------------------------------
        # The GPIO BCM numbered pin providing PWM signal for this ESC
        #-------------------------------------------------------------------------------------------
        self.bcm_pin = pin

        #-------------------------------------------------------------------------------------------
        # Physical parameters of the ESC / motors / propellers
        #-------------------------------------------------------------------------------------------
        self.motor_location = location
        self.motor_rotation = rotation

        #-------------------------------------------------------------------------------------------
        # Name - for logging purposes only
        #-------------------------------------------------------------------------------------------
        self.name = name

        #-------------------------------------------------------------------------------------------
        # Pulse width - for logging purposes only
        #-------------------------------------------------------------------------------------------
        self.pulse_width = 0

        self.spin = spin #changed

        #-------------------------------------------------------------------------------------------
        # Initialize the RPIO DMA PWM for this ESC.
        #-------------------------------------------------------------------------------------------
        self.set(1000)

    def set(self, pulse_width):
        pulse_width = pulse_width if pulse_width >= 1000 else 1000
        pulse_width = pulse_width if pulse_width <= 1999 else 1999 #changed it was 1999

        self.pulse_width = pulse_width

        PWM.add_channel_pulse(RPIO_DMA_CHANNEL, self.bcm_pin, 0, pulse_width)


####################################################################################################
#
# Get the rotation angles of pitch, roll and yaw from the fixed point of earth reference frame
# gravity + lateral orientation (ultimately compass derived, but currently just the take-off
# orientation) of 0, 0, 1 compared to where gravity is distrubuted across the X, Y and Z axes of the
# accelerometer all based upon the right hand rule.
#
####################################################################################################
def GetRotationAngles(ax, ay, az):

    #-----------------------------------------------------------------------------------------------
    # What's the angle in the x and y plane from horizontal in radians?
    #-----------------------------------------------------------------------------------------------
    pitch = math.atan2(-ax, math.pow(math.pow(ay, 2) + math.pow(az, 2), 0.5))
    roll = math. atan2(ay, az)

    return pitch, roll


####################################################################################################
#
# Absolute angles of tilt compared to the earth gravity reference frame.
#
####################################################################################################
def GetAbsoluteAngles(ax, ay, az):

    pitch = math.atan2(-ax, az)
    roll = math.atan2(ay, az)

    return pitch, roll


####################################################################################################
#
# Convert a body frame rotation rate to the rotation frames
#
####################################################################################################
def Body2EulerRates(qry, qrx, qrz, pa, ra):

    #===============================================================================================
    # Axes: Convert a set of gyro body frame rotation rates into Euler frames
    #
    # Matrix
    # ---------
    # |err|   | 1 ,  sin(ra) * tan(pa) , cos(ra) * tan(pa) | |qrx|
    # |epr| = | 0 ,  cos(ra)           ,     -sin(ra)      | |qry|
    # |eyr|   | 0 ,  sin(ra) / cos(pa) , cos(ra) / cos(pa) | |qrz|
    #
    #===============================================================================================
    c_pa = math.cos(pa)
    t_pa = math.tan(pa)
    c_ra = math.cos(ra)
    s_ra = math.sin(ra)

    err = qrx + qry * s_ra * t_pa + qrz * c_ra * t_pa
    epr =       qry * c_ra        - qrz * s_ra
    eyr =       qry * s_ra / c_pa + qrz * c_ra / c_pa

    return epr, err, eyr


####################################################################################################
#
# Rotate a vector using Euler angles wrt Earth frame co-ordinate system, for example to take the
# earth frame target flight plan vectors, and move it to the quad frame orientations vectors.
#
####################################################################################################
def RotateVector(evx, evy, evz, pa, ra, ya):

    #===============================================================================================
    # Axes: Convert a vector from earth- to quadcopter frame
    #
    # Matrix
    # ---------
    # |qvx|   | cos(pa) * cos(ya),                                 cos(pa) * sin(ya),                               -sin(pa)          | |evx|
    # |qvy| = | sin(ra) * sin(pa) * cos(ya) - cos(ra) * sin(ya),   sin(ra) * sin(pa) * sin(ya) + cos(ra) * cos(ya),  sin(ra) * cos(pa)| |evy|
    # |qvz|   | cos(ra) * sin(pa) * cos(ya) + sin(ra) * sin(ya),   cos(ra) * sin(pa) * sin(ya) - sin(ra) * cos(ya),  cos(pa) * cos(ra)| |evz|
    #
    #===============================================================================================
    c_pa = math.cos(pa)
    s_pa = math.sin(pa)
    c_ra = math.cos(ra)
    s_ra = math.sin(ra)
    c_ya = math.cos(ya)
    s_ya = math.sin(ya)

    qvx = evx * c_pa * c_ya                        + evy * c_pa * s_ya                        - evz * s_pa
    qvy = evx * (s_ra * s_pa * c_ya - c_ra * s_ya) + evy * (s_ra * s_pa * s_ya + c_ra * c_ya) + evz * s_ra * c_pa
    qvz = evx * (c_ra * s_pa * c_ya + s_ra * s_ya) + evy * (c_ra * s_pa * s_ya - s_ra * c_ya) + evz * c_pa * c_ra

    return qvx, qvy, qvz


####################################################################################################
#
# Initialize hardware PWM
#
####################################################################################################
RPIO_DMA_CHANNEL = 1

def PWMInit():
    #-----------------------------------------------------------------------------------------------
    # Set up the globally shared single PWM channel
    #-----------------------------------------------------------------------------------------------
    PWM.set_loglevel(PWM.LOG_LEVEL_ERRORS)
    PWM.setup(1)                                    # 1us resolution pulses
    PWM.init_channel(RPIO_DMA_CHANNEL, 3000)        # pulse every 3ms


####################################################################################################
#
# Cleanup hardware PWM
#
####################################################################################################
def PWMTerm():
    PWM.cleanup()


####################################################################################################
#
# GPIO pins initialization for MPU6050 FIFO overflow interrupt
#
####################################################################################################
def GPIOInit(FIFOOverflowISR):
    GPIO.setmode(GPIO.BCM)

    GPIO.setup(GPIO_FIFO_OVERFLOW_INTERRUPT, GPIO.IN, GPIO.PUD_OFF)
    GPIO.add_event_detect(GPIO_FIFO_OVERFLOW_INTERRUPT, GPIO.RISING) #, FIFOOverflowISR)

#AB:    GPIO.setup(GPIO_POWER_BROWN_OUT_INTERRUPT, GPIO.IN, GPIO.PUD_OFF)
#AB:    GPIO.add_event_detect(GPIO_POWER_BROWN_OUT_INTERRUPT, GPIO.FALLING)

    GPIO.setup(GPIO_GARMIN_BUSY, GPIO.IN, GPIO.PUD_DOWN)
#AB:    GPIO.add_event_detect(GPIO_GARMIN_BUSY, GPIO.FALLING)

    GPIO.setup(GPIO_BUTTON, GPIO.IN, GPIO.PUD_UP)

    GPIO.setup(GPIO_LED, GPIO.OUT)
    GPIO.output(GPIO_LED, GPIO.LOW)


####################################################################################################
#
# GPIO pins cleanup for MPU6050 FIFO overflow interrupt
#
####################################################################################################
def GPIOTerm():
#AB:    GPIO.remove_event_detect(GPIO_FIFO_OVERFLOW_INTERRUPT)
    GPIO.cleanup()


####################################################################################################
#
# Check CLI validity, set calibrate_sensors / fly or sys.exit(1)
#
####################################################################################################
def CheckCLI():
    cli_fly = False
    cli_hover_pwm = 1000

    #-----------------------------------------------------------------------------------------------
    # Other configuration defaults
    #-----------------------------------------------------------------------------------------------
    cli_test_case = 0
    cli_diagnostics = False
    cli_tau = 7.5
    cli_calibrate_0g = False
    cli_fp_filename = ''
    cli_cc_compass = False
    cli_file_control = False
    cli_yaw_control = False
    cli_gps_control = False
    cli_add_waypoint = False
    cli_clear_waypoints = False

    hover_pwm_defaulted = True

    #-----------------------------------------------------------------------------------------------
    # Defaults for vertical distance PIDs
    #-----------------------------------------------------------------------------------------------
    cli_vdp_gain = 1.0
    cli_vdi_gain = 0.0
    cli_vdd_gain = 0.0

    #-----------------------------------------------------------------------------------------------
    # Defaults for horizontal distance PIDs
    #-----------------------------------------------------------------------------------------------
    cli_hdp_gain = 1.0
    cli_hdi_gain = 0.0
    cli_hdd_gain = 0.0

    #-----------------------------------------------------------------------------------------------
    # Defaults for horizontal velocity PIDs
    #-----------------------------------------------------------------------------------------------
    cli_hvp_gain = 1.5 #changed 1.5
    cli_hvi_gain = 0.0
    cli_hvd_gain = 0.0

    #-----------------------------------------------------------------------------------------------
    # Per frame specific values.  This is the only place where PID integrals are used to compansate
    # for stable forces such as gravity, weight imbalance in the frame.  Yaw is included here to
    # account for frame unique momentum for required for rotation; however this does not need a
    # integral as there should not be a constant force that needs to be counteracted.
    #-----------------------------------------------------------------------------------------------

    if i_am_zoe:
        #-------------------------------------------------------------------------------------------
        # Zoe's PID configuration due to using her ESCs / motors / props
        #-------------------------------------------------------------------------------------------
        cli_hover_pwm = 1300 #changed

        #-------------------------------------------------------------------------------------------
        # Defaults for vertical velocity PIDs
        #-------------------------------------------------------------------------------------------
        cli_vvp_gain = 400.0  #changed 400
        cli_vvi_gain = 200.0  #changed 200
        cli_vvd_gain = 0.0

        #-------------------------------------------------------------------------------------------
        # Defaults for pitch rotation rate PIDs
        #-------------------------------------------------------------------------------------------
        cli_prp_gain = 80.0 #changed 80
        cli_pri_gain = 0.0
        cli_prd_gain = 0.0

        #-------------------------------------------------------------------------------------------
        # Defaults for roll rotation rate PIDs
        #-------------------------------------------------------------------------------------------
        cli_rrp_gain = 80.0 #changed
        cli_rri_gain = 0.0
        cli_rrd_gain = 0.0

        #-------------------------------------------------------------------------------------------
        # Defaults for yaw rotation rate PIDs
        #-------------------------------------------------------------------------------------------
        cli_yrp_gain = 160.0
        cli_yri_gain = 0.0
        cli_yrd_gain = 0.0

    #-----------------------------------------------------------------------------------------------
    # Right, let's get on with reading the command line and checking consistency
    #-----------------------------------------------------------------------------------------------
    argv = sys.argv[1:]
    opts, args = getopt.getopt(argv,'df:gh:y', ['cc', 'tc=', 'awp', 'cwp', 'gps', 'tau=', 'vdp=', 'vdi=', 'vdd=', 'vvp=', 'vvi=', 'vvd=', 'hdp=', 'hdi=', 'hdd=', 'hvp=', 'hvi=', 'hvd=', 'prp=', 'pri=', 'prd=', 'rrp=', 'rri=', 'rrd=', 'tau=', 'yrp=', 'yri=', 'yrd='])

    for opt, arg in opts:
        if opt in '-d':
            cli_diagnostics = True

    return cli_fp_filename, cli_calibrate_0g, cli_cc_compass, cli_yaw_control, cli_file_control, cli_gps_control, cli_add_waypoint, cli_clear_waypoints, cli_hover_pwm, cli_vdp_gain, cli_vdi_gain, cli_vdd_gain, cli_vvp_gain, cli_vvi_gain, cli_vvd_gain, cli_hdp_gain, cli_hdi_gain, cli_hdd_gain, cli_hvp_gain, cli_hvi_gain, cli_hvd_gain, cli_prp_gain, cli_pri_gain, cli_prd_gain, cli_rrp_gain, cli_rri_gain, cli_rrd_gain, cli_yrp_gain, cli_yri_gain, cli_yrd_gain, cli_test_case, cli_tau, cli_diagnostics


####################################################################################################
#
# Functions to lock memory to prevent paging, and move child processes in different process groups
# such that a Ctrl-C / SIGINT to one isn't distributed automatically to all children.
#
####################################################################################################
MCL_CURRENT = 1
MCL_FUTURE  = 2
def mlockall(flags = MCL_CURRENT| MCL_FUTURE):
    libc_name = ctypes.util.find_library("c")
    libc = ctypes.CDLL(libc_name, use_errno=True)
    result = libc.mlockall(flags)
    if result != 0:
        raise Exception("cannot lock memory, errno=%s" % ctypes.get_errno())

def munlockall():
    libc_name = ctypes.util.find_library("c")
    libc = ctypes.CDLL(libc_name, use_errno=True)
    result = libc.munlockall()
    if result != 0:
        raise Exception("cannot lock memory, errno=%s" % ctypes.get_errno())

def Daemonize():
    os.setpgrp()

#NEWCODE class for remote connecivity
class Remote (threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.ser = serial.Serial("/dev/ttyS0", 115200, timeout=None)
        self.x = bytearray(21)
        self.ch1 = 0
        self.ch2 = 0
        self.ch3 = 0
        self.ch4 = 0
        self.ch5 = 0
        self.ch6 = 0
        self.ch7 = 0
        self.ch8 = 0
        self.filled = 0
        self.rem_pitch = 0
        self.rem_roll = 0
        self.rem_yaw = 0
        self.rem_throttle = 0
        self.rem_switch = 0
        self.stopped = False

    def run(self):
        #NEWCODE #changed
        while(True):
            #ser.reset_input_buffer()
            self.ser.readinto(self.x)
            if(self.x[0] != 168):  #0xa8
                print "NO" + str(self.x[0])
                self.ser = serial.Serial("/dev/ttyS0", 115200, timeout=None)
                continue
            else:
                self.filled = self.filled + 1
            if(self.stopped):
                break


    def isFilled(self):
        if self.filled > 0:
            return True
        else:
            return False

    def stop(self):
        self.stopped = True
        self.join()
        self.ser.close()

    def turnedOn(self):
        while True:
            while not self.isFilled():
                time.sleep(0.01)
            self.getData()
            if(self.rem_throttle == -1):
                flag = True
                i = 0
                for i in range(0, 100):
                    while not self.isFilled():
                        time.sleep(0.01)
                    self.getData()
                    if(self.rem_switch != 1):
                        flag = False
                if flag == True:
                    return True

    def safetyOff(self):
        while True:
            while not self.isFilled():
                time.sleep(0.001)
            self.getData()
            if self.rem_throttle == 0:
                return True
            elif self.rem_throttle == -1 and self.rem_switch == 1:
                return False

    def getData(self):
        self.ch1 = (self.x[3]<<8|self.x[4])/8
        self.ch2 = (self.x[5]<<8|self.x[6])/8
        self.ch3 = (self.x[7]<<8|self.x[8])/8 #throttle
        self.ch4 = (self.x[9]<<8|self.x[10])/8

        self.ch5 = (self.x[11]<<8|self.x[12])/8
        self.ch6 = (self.x[13]<<8|self.x[14])/8
        self.ch7 = (self.x[15]<<8|self.x[16])/8
        self.ch8 = (self.x[17]<<8|self.x[18])/8

        #print "ch5" + str(self.ch5) + " ch6" + str(self.ch6) + "ch7" + str(self.ch7) + "ch8" + str(self.ch8) 
        self.rem_pitch = (self.ch1 - 1500) / (400) #pitch - range -1 to 1
        self.rem_roll = (self.ch2 - 1500) / (400) #roll - range -1 to 1
        self.rem_throttle = (self.ch3 - 1500) / (400)  #throttle range -1 to 1
        self.rem_yaw = (self.ch4 - 1500) / (400) #yaw range -1 to 1

        self.rem_switch = (self.ch8 - 1500) / 400

        #print "filled: " + str(self.filled)
        self.filled = 0
        return self.rem_pitch, self.rem_roll, self.rem_yaw, self.rem_throttle

#/NEWCODE

####################################################################################################
#
# Class to split initialation, flight startup and flight control
#
####################################################################################################
class Quadcopter:

    MOTOR_LOCATION_FRONT = 0b00000001
    MOTOR_LOCATION_BACK =  0b00000010
    MOTOR_LOCATION_LEFT =  0b00000100
    MOTOR_LOCATION_RIGHT = 0b00001000

    MOTOR_ROTATION_CW = 1
    MOTOR_ROTATION_ACW = 2

    keep_looping = False

    #===============================================================================================
    # One-off initialization
    #===============================================================================================
    def __init__(self):

        #-------------------------------------------------------------------------------------------
        # Who am I?
        #-------------------------------------------------------------------------------------------
        global i_am_zoe
        i_am_zoe = False
        my_name = "zoe"
        if my_name == "zoe.local" or my_name == "zoe":
            print "Hi, I'm Zoe.  Nice to meet you!"
            i_am_zoe = True
        else:
            print "Sorry, I'm not qualified to fly this piDrone."
            return

        #-------------------------------------------------------------------------------------------
        # Set up extra sensors based on quad identify.
        # -  compass_installed can only be used with an MPU9250
        #-------------------------------------------------------------------------------------------
        X8 = False
        if i_am_zoe:
            self.compass_installed = False
            self.camera_installed = False
            self.gll_installed = False
            self.gps_installed = False
            self.sweep_installed = False

        #-------------------------------------------------------------------------------------------
        # Lock code permanently in memory - no swapping to disk
        #-------------------------------------------------------------------------------------------
        mlockall()

        #-------------------------------------------------------------------------------------------
        # Set up the base logging
        #-------------------------------------------------------------------------------------------
        global logger
        logger = logging.getLogger('QC logger')
        logger.setLevel(logging.INFO)

        #-------------------------------------------------------------------------------------------
        # Create file and console logger handlers - the file is written into shared memory and only
        # dumped to disk / SD card at the end of a flight for performance reasons
        #-------------------------------------------------------------------------------------------
        global file_handler
        file_handler = logging.FileHandler("qcstats.csv", 'w')
        file_handler.setLevel(logging.WARNING)

        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.CRITICAL)

        #-------------------------------------------------------------------------------------------
        # Create a formatter and add it to both handlers
        #-------------------------------------------------------------------------------------------
        console_formatter = logging.Formatter('%(message)s')
        console_handler.setFormatter(console_formatter)

        file_formatter = logging.Formatter('[%(levelname)s] (%(threadName)-10s) %(funcName)s %(lineno)d, %(message)s')
        file_handler.setFormatter(file_formatter)

        #-------------------------------------------------------------------------------------------
        # Add both handlers to the logger
        #-------------------------------------------------------------------------------------------
        logger.addHandler(console_handler)
        logger.addHandler(file_handler)

        #-------------------------------------------------------------------------------------------
        # First log, whose flying and under what configuration
        #-------------------------------------------------------------------------------------------
        logger.warning("%s is flying.", "Zoe" if i_am_zoe else "Hermione")

        #-------------------------------------------------------------------------------------------
        # Set the BCM pin assigned to the FIFO overflow
        #-------------------------------------------------------------------------------------------
        global GPIO_POWER_BROWN_OUT_INTERRUPT
        GPIO_POWER_BROWN_OUT_INTERRUPT = 35

        global GPIO_FIFO_OVERFLOW_INTERRUPT
        GPIO_FIFO_OVERFLOW_INTERRUPT = 24 if X8 else 22

        global GPIO_GARMIN_BUSY
        GPIO_GARMIN_BUSY = 5

        global GPIO_BUTTON
        GPIO_BUTTON = 6

        global GPIO_LED
        GPIO_LED = 25

        #-------------------------------------------------------------------------------------------
        # Enable RPIO for ESC PWM.  This must be set up prior to adding the SignalHandler below or it
        # will override what we set thus killing the "Kill Switch"..
        #-------------------------------------------------------------------------------------------
        PWMInit()

        #-------------------------------------------------------------------------------------------
        # Enable GPIO for the FIFO overflow interrupt.
        #-------------------------------------------------------------------------------------------
        GPIOInit(self.fifoOverflowISR)

        #-------------------------------------------------------------------------------------------
        # Set the signal handler here so the core processing loop can be stopped (or not started) by
        # Ctrl-C.
        #-------------------------------------------------------------------------------------------
        signal.signal(signal.SIGINT, self.shutdownSignalHandler)

        #-------------------------------------------------------------------------------------------
        # Zoe is a Quad, Hermione is an X8
        #-------------------------------------------------------------------------------------------
        ESC_BCM_FLT = 0
        ESC_BCM_FRT = 0
        ESC_BCM_BLT = 0
        ESC_BCM_BRT = 0
        ESC_BCM_FLU = 0
        ESC_BCM_FRU = 0
        ESC_BCM_BLU = 0
        ESC_BCM_BRU = 0

        if not X8:
            ESC_BCM_FLT = 27
            ESC_BCM_FRT = 17
            ESC_BCM_BLT = 26
            ESC_BCM_BRT = 19
        else:
            ESC_BCM_FLT = 27
            ESC_BCM_FRT = 17
            ESC_BCM_BLT = 26
            ESC_BCM_BRT = 20
            ESC_BCM_FLU = 22
            ESC_BCM_FRU = 23
            ESC_BCM_BLU = 16
            ESC_BCM_BRU = 19


        pin_list = [ESC_BCM_FLT,
                    ESC_BCM_FRT,
                    ESC_BCM_BLT,
                    ESC_BCM_BRT,
                    ESC_BCM_FLU,
                    ESC_BCM_FRU,
                    ESC_BCM_BLU,
                    ESC_BCM_BRU]


        location_list = [self.MOTOR_LOCATION_FRONT | self.MOTOR_LOCATION_LEFT,
                         self.MOTOR_LOCATION_FRONT | self.MOTOR_LOCATION_RIGHT,
                         self.MOTOR_LOCATION_BACK | self.MOTOR_LOCATION_LEFT,
                         self.MOTOR_LOCATION_BACK | self.MOTOR_LOCATION_RIGHT,
                         self.MOTOR_LOCATION_FRONT | self.MOTOR_LOCATION_LEFT,
                         self.MOTOR_LOCATION_FRONT | self.MOTOR_LOCATION_RIGHT,
                         self.MOTOR_LOCATION_BACK | self.MOTOR_LOCATION_LEFT,
                         self.MOTOR_LOCATION_BACK | self.MOTOR_LOCATION_RIGHT]

        rotation_list = [self.MOTOR_ROTATION_ACW,
                         self.MOTOR_ROTATION_CW,
                         self.MOTOR_ROTATION_CW,
                         self.MOTOR_ROTATION_ACW,
                         self.MOTOR_ROTATION_CW,
                         self.MOTOR_ROTATION_ACW,
                         self.MOTOR_ROTATION_ACW,
                         self.MOTOR_ROTATION_CW]

        name_list = ['front left topside',
                     'front right topside',
                     'back left topside',
                     'back right topside',
                     'front left underside',
                     'front right underside',
                     'back left underside',
                     'back right underside']

        motor_spin_pwm = [1220, #changed
                          1080,  #1080
                          1250,
                          1250,
                          1200,  #arbitrary
                          1200,
                          1200,
                          1200]

        self.min_spin_pwm = min(motor_spin_pwm)

        #-------------------------------------------------------------------------------------------
        # Prime the ESCs to stop their anonying beeping!  All 4 of P, C, H & Z  use the T-motor ESCs
        # with the same ESC firmware so have the same spin_pwm
        #-------------------------------------------------------------------------------------------
        global stfu_pwm
        stfu_pwm = 1000

        self.esc_list = []
        for esc_index in range(8 if X8 else 4):
            esc = ESC(pin_list[esc_index], location_list[esc_index], rotation_list[esc_index], name_list[esc_index], motor_spin_pwm[esc_index]) #changed
            self.esc_list.append(esc)

        #===========================================================================================
        # Globals for the IMU setup
        # adc_frequency      - the sampling rate of the ADC
        # sampling_rate      - the data sampling rate and thus data ready interrupt rate
        #                      motion processing.
        # motion_rate        - the target frequency motion processing occurs under perfect conditions.
        # fusion_rate        - the sampling rate of the GLL and the video frame rate
        # alpf               - the accelerometer low pass filter
        # glpf               - the gyro low pass filter
        #===========================================================================================
        global adc_frequency
        global sampling_rate
        global motion_rate
        global fusion_rate

        adc_frequency = 1000        #AB: defined by dlpf >= 1; DO NOT USE ZERO => 8000 adc_frequency
        fusion_rate = 10

        if self.camera_installed or self.gll_installed:
            if i_am_zoe:
                sampling_rate = 500  # Hz
                motion_rate = 75     # Hz
        else:
            sampling_rate = 1000     # Hz
            motion_rate = 200        # Hz

        glpf = 1                     #AB: 184Hz

        #-------------------------------------------------------------------------------------------
        # This is not for antialiasing: the accelerometer low pass filter happens between the ADC
        # rate and our IMU sampling rate.  ADC rate is 1kHz through this case.  However, I've seen poor
        # behavious in double integration when IMU sampling rate is 500Hz and alpf = 460Hz.
        #-------------------------------------------------------------------------------------------
        if sampling_rate == 1000:   #AB: SRD = 0 (1kHz)
            alpf = 0                #AB: alpf = 460Hz
        elif sampling_rate == 500:  #AB: SRD = 1 (500Hz)
            alpf = 1                #AB: alpf = 184Hz
        elif sampling_rate >= 200:  #AB: SRD = 2, 3, 4 (333, 250, 200Hz)
            alpf = 2                #AB: alpf = 92Hz
        elif sampling_rate >= 100:  #AB: SRD = 5, 6, 7, 8, 9 (166, 143, 125, 111, 100Hz)
            alpf = 3                #ABL alpf = 41Hz
        else:
            #--------------------------------------------------------------------------------------
            # There's no point going less than 100Hz IMU sampling; we need about 100Hz motion
            # processing for some degree of level of stability.
            #--------------------------------------------------------------------------------------
            print "SRD + alpf useless: forget it!"
            return

        global mpu6050
        mpu6050 = MPU6050(0x68, alpf, glpf)

        #-------------------------------------------------------------------------------------------
        # Scheduling parameters defining standard, and critical FIFO block counts
        #
        # FIFO_MINIMUM - The least number of batches of collect and average for running them through
        #                the motion processor
        # FIFO_MAXIMUM - The most number of batches to be allowed through the motion processor; any
        #                higher risks FIFO overflow.
        #
        # 512/12 is the maximum number of batches in the IMU FIFO
        #
        #-------------------------------------------------------------------------------------------
        self.FIFO_MINIMUM = int(round(sampling_rate / motion_rate))
        self.FIFO_MAXIMUM = int(round(512 / 12)) - self.FIFO_MINIMUM

        #-------------------------------------------------------------------------------------------
        # Initialize the compass object.
        #-------------------------------------------------------------------------------------------
        if self.compass_installed:
            mpu6050.initCompass()

    #===============================================================================================
    # Per-flight configuration, initializations and flight control itself
    #===============================================================================================
    def fly(self):
        remote = Remote()
        remote.start()
        while True:
            #NEWCODE
            rem_pitch = 0
            rem_roll = 0
            rem_yaw = 0
            rem_throttle = 0
            #NEWCODE
            while not remote.turnedOn():
                time.sleep(0.01)

            #-------------------------------------------------------------------------------------------
            # Check the command line for calibration or flight parameters
            #-------------------------------------------------------------------------------------------
            print "\nJust checking a few details.  Gimme a few seconds..."
            try:
                fp_filename, calibrate_0g, cc_compass, yaw_control, file_control, gps_control, add_waypoint, clear_waypoints, hover_pwm, vdp_gain, vdi_gain, vdd_gain, vvp_gain, vvi_gain, vvd_gain, hdp_gain, hdi_gain, hdd_gain, hvp_gain, hvi_gain, hvd_gain, prp_gain, pri_gain, prd_gain, rrp_gain, rri_gain, rrd_gain, yrp_gain, yri_gain, yrd_gain, test_case, atau, diagnostics = CheckCLI()
            except ValueError, err:
                print "Command line error: %s" % err
                return

            logger.warning("fp_filename = %s, calibrate_0g = %d, check / calibrate compass = %s, yaw_control = %s, file_control = %s, gps_control = %s, add_waypoint = %s, clear_waypoints = %s, hover_pwm = %d, vdp_gain = %f, vdi_gain = %f, vdd_gain= %f, vvp_gain = %f, vvi_gain = %f, vvd_gain= %f, hdp_gain = %f, hdi_gain = %f, hdd_gain = %f, hvp_gain = %f, hvi_gain = %f, hvd_gain = %f, prp_gain = %f, pri_gain = %f, prd_gain = %f, rrp_gain = %f, rri_gain = %f, rrd_gain = %f, yrp_gain = %f, yri_gain = %f, yrd_gain = %f, test_case = %d, atau = %f, diagnostics = %s",
                    fp_filename, calibrate_0g, cc_compass, yaw_control, file_control, gps_control, add_waypoint, clear_waypoints, hover_pwm, vdp_gain, vdi_gain, vdd_gain, vvp_gain, vvi_gain, vvd_gain, hdp_gain, hdi_gain, hdd_gain, hvp_gain, hvi_gain, hvd_gain, prp_gain, pri_gain, prd_gain, rrp_gain, rri_gain, rrd_gain, yrp_gain, yri_gain, yrd_gain, test_case, atau, diagnostics)


            #-------------------------------------------------------------------------------------------
            # Calibrate compass.
            #-------------------------------------------------------------------------------------------
            if self.compass_installed:
                if cc_compass:
                    if not mpu6050.compassCheckCalibrate():
                        print "Compass check / calibration error, abort"
                    return
                elif not mpu6050.loadCompassCalibration():
                    print "Compass calibration data not found"
                    return
            elif cc_compass:
                print "Compass not installed, check / calibration not possible."
                return

            #===========================================================================================
            # OK, we're in flight mode, better get on with it
            #===========================================================================================
            self.keep_looping = True

            edx_target = 0.0
            edy_target = 0.0
            edz_target = 0.0

            evx_target = 0.0
            evy_target = 0.0
            evz_target = 0.0

            ya_target = 0.0

            qdx_input = 0.0
            qdy_input = 0.0
            qdz_input = 0.0

            qvx_input = 0.0
            qvy_input = 0.0
            qvz_input = 0.0

            edx_fuse = 0.0
            edy_fuse = 0.0
            edz_fuse = 0.0

            evx_fuse = 0.0
            evy_fuse = 0.0
            evz_fuse = 0.0

            qdx_fuse = 0.0
            qdy_fuse = 0.0
            qdz_fuse = 0.0

            qvx_fuse = 0.0
            qvy_fuse = 0.0
            qvz_fuse = 0.0

            #===========================================================================================
            # Tuning: Set up the PID gains - some are hard coded mathematical approximations, some come
            # from the CLI parameters to allow for tuning  - 10 in all
            # - Quad X axis distance
            # - Quad Y axis distance
            # - Quad Z axis distance
            # - Quad X axis velocity
            # - Quad Y axis velocity
            # - Quad Z axis velocity
            # - Pitch rotation rate
            # - Roll Rotation rate
            # - Yaw angle
            # = Yaw Rotation rate
            #===========================================================================================

            #-------------------------------------------------------------------------------------------
            # The quad X axis PID controls fore / aft distance
            #-------------------------------------------------------------------------------------------
            PID_QDX_P_GAIN = hdp_gain
            PID_QDX_I_GAIN = hdi_gain
            PID_QDX_D_GAIN = hdd_gain

            #-------------------------------------------------------------------------------------------
            # The quad Y axis PID controls left / right distance
            #-------------------------------------------------------------------------------------------
            PID_QDY_P_GAIN = hdp_gain
            PID_QDY_I_GAIN = hdi_gain
            PID_QDY_D_GAIN = hdd_gain

            #-------------------------------------------------------------------------------------------
            # The quad Z axis PID controls up / down distance
            #-------------------------------------------------------------------------------------------
            PID_QDZ_P_GAIN = vdp_gain
            PID_QDZ_I_GAIN = vdi_gain
            PID_QDZ_D_GAIN = vdd_gain

            #-------------------------------------------------------------------------------------------
            # The quad X axis speed controls fore / aft speed
            #-------------------------------------------------------------------------------------------
            PID_QVX_P_GAIN = hvp_gain
            PID_QVX_I_GAIN = hvi_gain
            PID_QVX_D_GAIN = hvd_gain

            #-------------------------------------------------------------------------------------------
            # The quad Y axis speed PID controls left / right speed
            #-------------------------------------------------------------------------------------------
            PID_QVY_P_GAIN = hvp_gain
            PID_QVY_I_GAIN = hvi_gain
            PID_QVY_D_GAIN = hvd_gain

            #-------------------------------------------------------------------------------------------
            # The quad Z axis speed PID controls up / down speed
            #-------------------------------------------------------------------------------------------
            PID_QVZ_P_GAIN = vvp_gain
            PID_QVZ_I_GAIN = vvi_gain
            PID_QVZ_D_GAIN = vvd_gain

            #-------------------------------------------------------------------------------------------
            # The roll angle PID controls stable angles around the X-axis
            #-------------------------------------------------------------------------------------------
            PID_PA_P_GAIN = 2.0 # pap_gain #changed 2.0
            PID_PA_I_GAIN = 0.0 # pai_gain
            PID_PA_D_GAIN = 0.0 # pad_gain

            #-------------------------------------------------------------------------------------------
            # The pitch rate PID controls stable rotation rate around the Y-axis
            #-------------------------------------------------------------------------------------------
            PID_PR_P_GAIN = prp_gain
            PID_PR_I_GAIN = pri_gain
            PID_PR_D_GAIN = prd_gain

            #-------------------------------------------------------------------------------------------
            # The roll angle PID controls stable angles around the X-axis
            #-------------------------------------------------------------------------------------------
            PID_RA_P_GAIN = 2.0 # rap_gain #changed 2.0
            PID_RA_I_GAIN = 0.0 # rai_gain
            PID_RA_D_GAIN = 0.0 # rad_gain

            #-------------------------------------------------------------------------------------------
            # The roll rate PID controls stable rotation rate around the X-axis
            #-------------------------------------------------------------------------------------------
            PID_RR_P_GAIN = rrp_gain
            PID_RR_I_GAIN = rri_gain
            PID_RR_D_GAIN = rrd_gain

            #-------------------------------------------------------------------------------------------
            # The yaw angle PID controls stable angles around the Z-axis
            #-------------------------------------------------------------------------------------------
            PID_YA_P_GAIN = 6.0 # yap_gain #changed 6.0
            PID_YA_I_GAIN = 0.0 # yai_gain
            PID_YA_D_GAIN = 0.0 # yad_gain

            #-------------------------------------------------------------------------------------------
            # The yaw rate PID controls stable rotation speed around the Z-axis
            #-------------------------------------------------------------------------------------------
            PID_YR_P_GAIN = yrp_gain
            PID_YR_I_GAIN = yri_gain
            PID_YR_D_GAIN = yrd_gain

            #-------------------------------------------------------------------------------------------
            # Start the X, Y (horizontal) and Z (vertical) distance PID
            #-------------------------------------------------------------------------------------------
            qdx_pid = PID(PID_QDX_P_GAIN, PID_QDX_I_GAIN, PID_QDX_D_GAIN)
            qdy_pid = PID(PID_QDY_P_GAIN, PID_QDY_I_GAIN, PID_QDY_D_GAIN)
            qdz_pid = PID(PID_QDZ_P_GAIN, PID_QDZ_I_GAIN, PID_QDZ_D_GAIN)

            #-------------------------------------------------------------------------------------------
            # Start the X, Y (horizontal) and Z (vertical) velocity PIDs
            #-------------------------------------------------------------------------------------------
            qvx_pid = PID(PID_QVX_P_GAIN, PID_QVX_I_GAIN, PID_QVX_D_GAIN)
            qvy_pid = PID(PID_QVY_P_GAIN, PID_QVY_I_GAIN, PID_QVY_D_GAIN)
            qvz_pid = PID(PID_QVZ_P_GAIN, PID_QVZ_I_GAIN, PID_QVZ_D_GAIN)

            #-------------------------------------------------------------------------------------------
            # Start the pitch, roll and yaw angle PID - note the different PID class for yaw.
            #-------------------------------------------------------------------------------------------
            pa_pid = PID(PID_PA_P_GAIN, PID_PA_I_GAIN, PID_PA_D_GAIN)
            ra_pid = PID(PID_RA_P_GAIN, PID_RA_I_GAIN, PID_RA_D_GAIN)
            ya_pid = YAW_PID(PID_YA_P_GAIN, PID_YA_I_GAIN, PID_YA_D_GAIN)

            #-------------------------------------------------------------------------------------------
            # Start the pitch, roll and yaw rotation rate PIDs
            #-------------------------------------------------------------------------------------------
            pr_pid = PID(PID_PR_P_GAIN, PID_PR_I_GAIN, PID_PR_D_GAIN)
            rr_pid = PID(PID_RR_P_GAIN, PID_RR_I_GAIN, PID_RR_D_GAIN)
            yr_pid = PID(PID_YR_P_GAIN, PID_YR_I_GAIN, PID_YR_D_GAIN)

            #------------------------------------------------------------------------------------------
            # Set the props spinning at their base rate to ensure initial kick-start doesn't get spotted
            # by the sensors messing up the flight thereafter. spin_pwm is determined by running testcase 1
            # multiple times incrementing -h slowly until a level of PWM is found where all props just spin.
            # This depends on the firmware in the ESCs
            #------------------------------------------------------------------------------------------
            print "Starting up the motors..."

            for esc in self.esc_list:
                esc.set(esc.spin)

            #-------------------------------------------------------------------------------------------
            # Initialize the base setting of earth frame take-off height - i.e. the vertical distance from
            # the height sensor or the take-off platform / leg height if no sensor is available.
            #-------------------------------------------------------------------------------------------
            eftoh = 0.0

            #-------------------------------------------------------------------------------------------
            # Get an initial take-off height
            #-------------------------------------------------------------------------------------------
            g_dist = 0.0

            #--------------------------------------------------------------------------------------------
            # Last chance to change your mind about the flight if all's ok so far
            #--------------------------------------------------------------------------------------------
            time.sleep(1)
            if not remote.safetyOff():
                for esc in self.esc_list:
                    esc.set(stfu_pwm)
                continue

            print ""
            print "################################################################################"
            print "#                                                                              #"
            print "#                             Thunderbirds are go!                             #"
            print "#                                                                              #"
            print "################################################################################"
            print ""

            ################################### INITIAL IMU READINGS ###################################


            #-------------------------------------------------------------------------------------------
            # Get IMU takeoff info.
            #-------------------------------------------------------------------------------------------
            mpu6050.flushFIFO()

            temp = mpu6050.readTemperature()
            logger.critical("IMU core temp (start): ,%f", temp / 333.86 + 21.0)

            time.sleep(20 / sampling_rate) # 20 < 0.5 x FIFO_SIZE
            nfb = mpu6050.numFIFOBatches()
            qax, qay, qaz, qrx, qry, qrz, dt = mpu6050.readFIFO(nfb)

            #-------------------------------------------------------------------------------------------
            # Feed back the gyro offset calibration
            #-------------------------------------------------------------------------------------------
            mpu6050.setGyroOffsets(qrx, qry, qrz)

            #-------------------------------------------------------------------------------------------
            # Read the IMU acceleration to obtain angles and gravity.
            #-------------------------------------------------------------------------------------------
            qax, qay, qaz, qrx, qry, qrz = mpu6050.scaleSensors(qax, qay, qaz, qrx, qry, qrz)

            #-------------------------------------------------------------------------------------------
            # Calculate the angles - ideally takeoff should be on a horizontal surface but a few degrees
            # here or there won't matter.
            #-------------------------------------------------------------------------------------------
            pa, ra = GetRotationAngles(qax, qay, qaz)
            ya = 0.0
            ya_fused = 0.0

            apa, ara = GetAbsoluteAngles(qax, qay, qaz)
            aya = 0.0

            pqrx = qrx
            pqry = qry
            pqrz = qrz

            #-------------------------------------------------------------------------------------------
            # Get the value for gravity.
            #-------------------------------------------------------------------------------------------
            egx, egy, egz = RotateVector(qax, qay, qaz, -pa, -ra, -ya)

            #-------------------------------------------------------------------------------------------
            # The tilt ratio is used to compensate sensor height (and thus velocity) for the fact the
            # sensors are leaning.
            #
            # tilt ratio is derived from cos(tilt angle);
            # - tilt angle a = arctan(sqrt(x*x + y*y) / z)
            # - cos(arctan(a)) = 1 / (sqrt(1 + a*a))
            # This all collapses down to the following.  0 <= Tilt ratio <= 1
            #-------------------------------------------------------------------------------------------
            tilt_ratio = qaz / egz
            eftoh *= tilt_ratio

            #-------------------------------------------------------------------------------------------
            # Log the critical parameters from this warm-up: the take-off surface tilt, and gravity.
            # Note that some of the variables used above are used in the main processing loop.  Messing
            # with the above code can have very unexpected effects in flight.
            #-------------------------------------------------------------------------------------------
            logger.warning("pitch, %f, roll, %f", math.degrees(pa), math.degrees(ra))
            logger.warning("egx, %f, egy, %f, egz %f", egx, egy, egz)
            logger.warning("based upon %d samples", dt * sampling_rate)
            logger.warning("EFTOH:, %f", eftoh)

            #-------------------------------------------------------------------------------------------
            # Prime the direction vector of the earth's magnetic core to provide long term yaw stability.
            #-------------------------------------------------------------------------------------------
            mgx = 0.0
            mgy = 0.0
            mgz = 0.0
            cya_prev = 0.0

            if self.compass_installed:
                #---------------------------------------------------------------------------------------
                # Take 100 samples at the sampling rate
                #---------------------------------------------------------------------------------------
                mgx_ave = 0.0
                mgy_ave = 0.0
                mgz_ave = 0.0
                for ii in range(100):
                    mgx, mgy, mgz = mpu6050.readCompass()
                    mgx_ave += mgx
                    mgy_ave += mgy
                    mgz_ave += mgz

                    time.sleep(1 / sampling_rate)

                mgx = mgx_ave / 100
                mgy = mgy_ave / 100
                mgz = mgz_ave / 100

                #---------------------------------------------------------------------------------------
                # Rotate compass readings back to earth plane and tweak to be 0 - 2 pi radians.
                #---------------------------------------------------------------------------------------
                cax, cay, caz = RotateVector(mgx, mgy, mgz, -pa, -ra, 0)
                cya_prev = -(math.atan2(cax, cay) + 2 * math.pi) % (2 * math.pi)
                logger.critical("Initial orientation:, %f." % (math.degrees(cya_prev)))


            ######################################### GO GO GO! ########################################


            #-------------------------------------------------------------------------------------------
            # Set up the various timing constants and stats.
            #-------------------------------------------------------------------------------------------
            start_flight = time.time()
            motion_dt = 0.0
            fusion_dt = 0.0
            sampling_loops = 0
            motion_loops = 0
            fusion_loops = 0
            garmin_loops = 0
            video_loops = 0
            autopilot_loops = 0

            #-------------------------------------------------------------------------------------------
            # Diagnostic log header
            #-------------------------------------------------------------------------------------------
            if diagnostics:

                pwm_header = "FL PWM, FR PWM, BL PWM, BR PWM" if i_am_zoe else "FLT PWM, FRT PWM, BLT PWM, BRT PWM, FLB PWM, FRB PWM, BLB PWM, BRB PWM"
                logger.warning("time, dt, loops, " +
                               "temperature, " +
    #                           "mgx, mgy, mgz, cya, " + #changed
                               "edx_fuse, edy_fuse, edz_fuse, " +
                               "evx_fuse, evy_fuse, evz_fuse, " +
    #                           "qdx_fuse, qdy_fuse, qdz_fuse, " +
    #                           "qvx_fuse, qvy_fuse, qvz_fuse, " +
                               "edx_target, edy_target, edz_target, " +
                               "evx_target, evy_target, evz_target, " +
                               "qrx, qry, qrz, " +
                               "qax, qay, qaz, " +
                               "qgx, qgy, qgz, " +
                               "pitch, roll, yaw, yaw2, " +

                               "qdx_input, qdy_input, qdz_input, " +
                               "qdx_target, qdy_target, qdz_target, " +
                               "qvx_input, qvy_input, qvz_input, " +
                               "qvx_target, qvy_target, qvz_target, " +
    #                           "qvz_out, " +
                               "YOLO, " +
                               "pa_input, ra_input, ya_input, " +
                               "pa_target, ra_target, ya_target, " +
                               "pr_input, rr_input, yr_input, " +
                               "pr_target, rr_target, yr_target, " +
                               "pr_out, rr_out, yr_out, " +

                                "qdx_input, qdx_target, qvx_input, qvx_target, pa_input, pa_target, pr_input, pr_target, pr_out, " +
                                "qdy_input, qdy_target, qvy_input, qvy_target, ra_input, ra_target, rr_input, rr_target, rr_out, " +
                                "qdz_input, qdz_target, qvz_input, qvz_target, qaz_out, " +
                                "ya_input, ya_target, yr_input, yr_target, yr_out, " +
                               pwm_header)
            #-------------------------------------------------------------------------------------------
            # Flush the IMU FIFO and enable the FIFO overflow interrupt
            #-------------------------------------------------------------------------------------------
            GPIO.event_detected(GPIO_FIFO_OVERFLOW_INTERRUPT)
            mpu6050.flushFIFO()
            mpu6050.enableFIFOOverflowISR()

            #===========================================================================================
            #
            # Motion and PID processing loop naming conventions
            #
            # qd* = quad frame distance
            # qv* = quad frame velocity
            # qa? = quad frame acceleration
            # qg? = quad frame gravity
            # qr? = quad frame rotation
            # ea? = earth frame acceleration
            # eg? = earth frame gravity
            # ua? = euler angles between frames
            # ur? = euler rotation between frames
            # a?a = absoluted angles between frames
            #
            #===========================================================================================

            #prev_time = time.time()
            #now_time = time.time()
            #max_time = 0
            #min_time = 1000

            #newcode
            #start_time_yolo = time.time()
            while self.keep_looping:
                #newcode
                #p_t = time.time()
                ############################### SENSOR INPUT SCHEDULING ################################
                #---------------------------------------------------------------------------------------
                # Check on the number of IMU batches already stashed in the FIFO, and if not enough,
                # check autopilot and video, and ultimate sleep.
                #---------------------------------------------------------------------------------------
                nfb = mpu6050.numFIFOBatches()
                #newcode
                #print "no.fifo" + str(nfb)
                if nfb >= self.FIFO_MAXIMUM:
                    logger.critical("ABORT: FIFO too full risking overflow: %d.", nfb)
                    if vmp != None:
                        logger.critical("       Next VFP phase: %d", vmp.phase)
                    #newcode
                    #break

                #INFORMATION
                #FIFO_MINIMUM : 10
                #FIFO_MAXIMIM : 33
                
                if nfb < self.FIFO_MINIMUM:
                    #-----------------------------------------------------------------------------------
                    # Nowt else to do; sleep until either we timeout, or more data comes in from GPS
                    # video, or autopilot.
                    #-----------------------------------------------------------------------------------
                    timeout = (self.FIFO_MINIMUM - nfb) / sampling_rate

                    #-----------------------------------------------------------------------------------
                    # We had free time, do we still?  Better check.
                    #-----------------------------------------------------------------------------------
                    #newcode
                    continue


                ####################################### IMU FIFO #######################################


                #---------------------------------------------------------------------------------------
                # Before proceeding further, check the FIFO overflow interrupt to ensure we didn't sleep
                # too long
                #---------------------------------------------------------------------------------------
                #newcode
                if GPIO.event_detected(GPIO_FIFO_OVERFLOW_INTERRUPT):
                #    logger.critical("ABORT: FIFO overflow.")
                    print "OVERFLOW"
                #    break

                #---------------------------------------------------------------------------------------
                # Power brownout check - doesn't work on 3B onwards
                #---------------------------------------------------------------------------------------
                '''
                #AB! if GPIO.event_detected(GPIO_POWER_BROWN_OUT_INTERRUPT):
                #AB!     logger.critical("BROWN-OUT, ABORT!")
                #AB!     break
                '''

                
                #CHECKPOINT1
                #---------------------------------------------------------------------------------------
                # Now get the batch of averaged data from the FIFO.
                #---------------------------------------------------------------------------------------
                try:
                    qax, qay, qaz, qrx, qry, qrz, motion_dt = mpu6050.readFIFO(nfb)
                except IOError as err:
                    logger.critical("ABORT:")
                    for arg in err.args:
                        logger.critical("%s", arg)
                    break
                

                #CHECKPOINT2 ckpt1 -> ckpt2 approx 5ms
                
                #---------------------------------------------------------------------------------------
                # Sort out units and calibration for the incoming data
                #---------------------------------------------------------------------------------------
                qax, qay, qaz, qrx, qry, qrz = mpu6050.scaleSensors(qax,
                                                                    qay,
                                                                    qaz,
                                                                    qrx,
                                                                    qry,
                                                                    qrz)

                #---------------------------------------------------------------------------------------
                # Track the number of motion loops and sampling loops.  motion_dt on which their are based
                # are the core timing provided by the IMU and are used for all timing events later such
                # as integration and PID Intergral and Differential factors.
                #---------------------------------------------------------------------------------------
                motion_loops += 1
                #print motion_dt
                sampling_loops += motion_dt * sampling_rate


                ################################## ANGLES PROCESSING ###################################


                #---------------------------------------------------------------------------------------
                # Euler angle fusion: Merge the 'integral' of the previous euler rotation rates with
                # the noisy accelermeter current values.  Keep yaw within +/- pi radians
                #---------------------------------------------------------------------------------------
                urp, urr, ury = Body2EulerRates(qry, qrx, qrz, pa, ra)
                pa += urp * motion_dt
                ra += urr * motion_dt
                ya += ury * motion_dt
                ya = (ya + math.pi) % (2 * math.pi) - math.pi

                upa, ura = GetRotationAngles(qax, qay, qaz)

                atau_fraction = atau / (atau + motion_dt)
                pa = atau_fraction * pa + (1 - atau_fraction) * upa
                ra = atau_fraction * ra + (1 - atau_fraction) * ura

                #---------------------------------------------------------------------------------------
                # Absolute angle fusion: Merge the 'integral' of the gyro rotation rates with
                # the noisy accelermeter current values.  Keep yaw within +/- pi radians
                #---------------------------------------------------------------------------------------
                apa += qry * motion_dt
                ara += qrx * motion_dt
                aya += qrz * motion_dt
                aya = (aya + math.pi) % (2 * math.pi) - math.pi

                upa, ura = GetAbsoluteAngles(qax, qay, qaz)

                atau_fraction = atau / (atau + motion_dt)
                apa = atau_fraction * apa + (1 - atau_fraction) * upa
                ara = atau_fraction * ara + (1 - atau_fraction) * ura

                '''
                #---------------------------------------------------------------------------------------
                # The temperature drifts throughout the flight, and if below about 20 degrees, this drift
                # causes major problems with double integration of accelerometer readings vs gravity.
                # However, because the flights are mostly fixed velocity = zero acceleration, a simple
                # complementary filter can keep gravity in sync with this drift.  Worth also checking
                # Butterworth used in past?
                #---------------------------------------------------------------------------------------
                eax, eay, eaz = RotateVector(qax, qay, qaz, -pa, -ra, -ya)
                gtau = 10 # seconds
                gtau_fraction = gtau / (gtau + motion_dt)
                egx = atau_fraction * egx + (1 - atau_fraction) * eax
                egy = atau_fraction * egy + (1 - atau_fraction) * eay
                egz = atau_fraction * egz + (1 - atau_fraction) * eaz
                '''

                ############################### IMU VELOCITY / DISTANCE ################################


                #---------------------------------------------------------------------------------------
                # Rotate gravity to the new quadframe
                #---------------------------------------------------------------------------------------
                qgx, qgy, qgz = RotateVector(egx, egy, egz, pa, ra, ya)

                #---------------------------------------------------------------------------------------
                # The tilt ratio is the ratio of gravity measured in the quad-frame Z axis and total gravity.
                # It's used to compensate for LiDAR height (and thus velocity) for the fact the laser may
                # not be pointing directly vertically down.
                #
                # - tilt angle a = arctan(sqrt(x*x + y*y) / z)
                # - compensated height = cos(arctan(a)) = 1 / (sqrt(1 + a*a))
                #
                # http://www.rapidtables.com/math/trigonometry/arctan/cos-of-arctan.htm
                #
                #---------------------------------------------------------------------------------------
                tilt_ratio = qgz / egz

                #==================== Velocity and Distance Increment processing =======================

                #---------------------------------------------------------------------------------------
                # Delete reorientated gravity from raw accelerometer readings and integrate over time
                # to make velocity all in quad frame.
                #---------------------------------------------------------------------------------------
                qvx_increment = (qax - qgx) * GRAV_ACCEL * motion_dt
                qvy_increment = (qay - qgy) * GRAV_ACCEL * motion_dt
                qvz_increment = (qaz - qgz) * GRAV_ACCEL * motion_dt

                qvx_input += qvx_increment
                qvy_input += qvy_increment
                qvz_input += qvz_increment

                #---------------------------------------------------------------------------------------
                # Integrate again the velocities to get distance.
                #---------------------------------------------------------------------------------------
                qdx_increment = qvx_input * motion_dt
                qdy_increment = qvy_input * motion_dt
                qdz_increment = qvz_input * motion_dt

                qdx_input += qdx_increment
                qdy_input += qdy_increment
                qdz_input += qdz_increment


                ######################## ABSOLUTE DISTANCE / ORIENTATION SENSORS #######################


                #---------------------------------------------------------------------------------------
                # Read the compass to determine yaw and orientation.
                #---------------------------------------------------------------------------------------
                if self.compass_installed:
                    mgx, mgy, mgz = mpu6050.readCompass()

                    #-----------------------------------------------------------------------------------
                    # Rotate compass readings back to earth plane and rescale to 0 - 2 * pi
                    #-----------------------------------------------------------------------------------
                    cax, cay, caz = RotateVector(mgx, mgy, mgz, -pa, -ra, 0)
                    cya = (-math.atan2(cax, cay) + 2 * math.pi) % (2 * math.pi)
                    cya_increment = cya - cya_prev
                    cya_prev = cya

                    #-----------------------------------------------------------------------------------
                    # Fuse the gyro and compass increments so that gyro yaw dominates short term and compass
                    # long term.
                    #-----------------------------------------------------------------------------------
                    yaw_tau = 1
                    yaw_fraction = yaw_tau / (yaw_tau + motion_dt)
                    yaw_increment = yaw_fraction * qrz * motion_dt + (1 - yaw_fraction) * cya_increment
                    ya_fused += yaw_increment


                ########################### VELOCITY / DISTANCE PID TARGETS ############################


                if edz_fuse > edz_target + 0.5:
                    logger.critical("ABORT: Height breach! %f target, %f actual", edz_target, edz_fuse)
                    break

                #---------------------------------------------------------------------------------------
                # Convert earth-frame distance targets to quadcopter frame.
                #---------------------------------------------------------------------------------------
                edx_target += evx_target * motion_dt
                edy_target += evy_target * motion_dt
                edz_target += evz_target * motion_dt
                qdx_target, qdy_target, qdz_target = RotateVector(edx_target, edy_target, edz_target, pa, ra, ya)

                '''
                #---------------------------------------------------------------------------------------
                # If we're doing yaw control, then from the iDrone POV, she's always flying forward, and
                # it's the responsibility of the yaw target to keep her facing in the right direction.
                #---------------------------------------------------------------------------------------
                if yaw_control:
                    qdx_target = math.pow(math.pow(qdx_target, 2) + math.pow(qdy_target, 2), 0.5)
                    qdy_target = 0.0
                '''


                ########### QUAD FRAME VELOCITY / DISTANCE / ANGLE / ROTATION PID PROCESSING ###########


                #=======================================================================================
                # Distance PIDs
                #=======================================================================================
                [p_out, i_out, d_out] = qdx_pid.Compute(qdx_input, qdx_target, motion_dt)
                qvx_target = p_out + i_out + d_out

                [p_out, i_out, d_out] = qdy_pid.Compute(qdy_input, qdy_target, motion_dt)
                qvy_target = p_out + i_out + d_out

                [p_out, i_out, d_out] = qdz_pid.Compute(qdz_input, qdz_target, motion_dt)
                qvz_target = p_out + i_out + d_out




                #NEWCODE #changed
                #INSERT CODE FOR READING PITCH, ROLL FROM REMOTE
                #NEWCODE #changed
                if remote.isFilled() == True:
                    rem_pitch, rem_roll, rem_yaw, rem_throttle = remote.getData()
                
                #else:
                #   print "NOT filled"
                #/NEWCODE
                qvx_target = rem_pitch
                qvy_target = -rem_roll
                qvz_target = 0

                '''
                '''
                #---------------------------------------------------------------------------------------
                # Constrain the target velocity to 1m/s.
                #---------------------------------------------------------------------------------------
                MAX_VEL = 1.0
                qvx_target = qvx_target if abs(qvx_target) < MAX_VEL else (qvx_target / abs(qvx_target) * MAX_VEL)
                qvy_target = qvy_target if abs(qvy_target) < MAX_VEL else (qvy_target / abs(qvy_target) * MAX_VEL)
                qvz_target = qvz_target if abs(qvz_target) < MAX_VEL else (qvz_target / abs(qvz_target) * MAX_VEL)
                '''
                '''

                #=======================================================================================
                # Velocity PIDs
                #=======================================================================================
                [p_out, i_out, d_out] = qvx_pid.Compute(qvx_input, qvx_target, motion_dt)
                qax_target = p_out + i_out + d_out

                [p_out, i_out, d_out] = qvy_pid.Compute(qvy_input, qvy_target, motion_dt)
                qay_target =  p_out + i_out + d_out

                [p_out, i_out, d_out] = qvz_pid.Compute(qvz_input, qvz_target, motion_dt)
                qaz_out = p_out + i_out + d_out

                #---------------------------------------------------------------------------------------
                # We now need to convert desired acceleration to desired angles before running the angular
                # PIDs.  Via the right hand rule:
                #
                # A positive x-axis acceleration (fore) needs a nose-down lean which is a positive
                # rotation around the y axis
                # A positive y-axis acceleration (port) needs a port-down lean which is a negative
                # rotation around the x axis
                #
                # If yaw control is enabled, the yaw angle target is set such that she's facing the way
                # she should be travelling based upon the earth frame velocity targets.  If these
                # targets are zero, then no yaw happens.
                #
                # If yaw control is disabled, the yaw angle target is zero - she always points in the
                # direction she took off in.
                #
                # Note this must use atan2 to safely handle division by 0.
                #---------------------------------------------------------------------------------------
                pa_target = math.atan(qax_target)
                ra_target = -math.atan(qay_target)
                ya_target = ya_target if not yaw_control else (ya_target if (abs(evx_target) + abs(evy_target)) == 0 else math.atan2(evy_target, evx_target))

                '''
                '''
                #---------------------------------------------------------------------------------------
                # Contrain the target angle to 30 degrees.  Note yaw is deliberately not included as it
                # needs full rotation to track the direction of flight when yaw_control = True.
                #---------------------------------------------------------------------------------------
                MAX_ANGLE = math.radians(30)
                pa_target = pa_target if abs(pa_target) < MAX_ANGLE else (pa_target / abs(pa_target) * MAX_ANGLE)
                ra_target = ra_target if abs(ra_target) < MAX_ANGLE else (ra_target / abs(ra_target) * MAX_ANGLE)
                '''
                '''

                #======================================================================================
                # Angle PIDs
                #======================================================================================
                [p_out, i_out, d_out] = pa_pid.Compute(apa, pa_target, motion_dt)
                pr_target = p_out + i_out + d_out

                [p_out, i_out, d_out] = ra_pid.Compute(ara, ra_target, motion_dt)
                rr_target = p_out + i_out + d_out

                #changed
                #[p_out, i_out, d_out] = ya_pid.Compute(aya, ya_target, motion_dt)
                #yr_target = p_out + i_out + d_out

                [p_out, i_out, d_out] = ya_pid.Compute(aya, rem_yaw * 2, motion_dt)
                yr_target = p_out + i_out + d_out

                '''
                #---------------------------------------------------------------------------------------
                # Constrain the target rotation rate to pi / second.
                # Additionally, if we're under yaw control, and there has been a significant course change
                # more than 20 degrees, then limit it to 30 degrees / second.
                #---------------------------------------------------------------------------------------
                MAX_RATE = math.pi
                if yaw_control and abs(ya_target - aya) > math.pi / 9:
                    YAW_MAX_RATE = math.pi / 6
                else:
                    YAW_MAX_RATE = MAX_RATE

                pr_target = pr_target if abs(pr_target) < MAX_RATE else (pr_target / abs(pr_target) * MAX_RATE)
                rr_target = rr_target if abs(rr_target) < MAX_RATE else (rr_target / abs(rr_target) * MAX_RATE)
                yr_target = yr_target if abs(yr_target) < YAW_MAX_RATE else (yr_target / abs(yr_target) * YAW_MAX_RATE)
                '''
                
                #print "pitch : " + str(apa) + ", roll : " + str(ara) + ", yaw : " + str(aya) + ", throttle" + str(0)
                #print "pitch : " + str(rem_pitch) + ", roll : " + str(rem_roll) + ", yaw : " + str(rem_yaw) + ", throttle" + str(rem_throttle)
                
                """
                #NEWCODE #changed
                #INSERT CODE FOR READING PITCH, ROLL FROM REMOTE
                #NEWCODE #changed
                if remote.isFilled() == True:
                    rem_pitch, rem_roll, rem_yaw, rem_throttle = remote.getData()
                #else:
                #   print "NOT filled"
                #/NEWCODE
                #NEWCODE #changed
                
                [p_out, i_out, d_out] = pa_pid.Compute(apa, rem_pitch * 2.5, motion_dt) #changed
                pr_target = p_out + i_out + d_out
     
                [p_out, i_out, d_out] = ra_pid.Compute(ara, rem_roll * 2.5, motion_dt) #changed
                rr_target = p_out + i_out + d_out

                [p_out, i_out, d_out] = ya_pid.Compute(aya, rem_yaw * 2.5, motion_dt) #changed
                yr_target = p_out + i_out + d_out
                #/NEWCODE #changed
                """

                #=======================================================================================
                # Rotation rate PIDs
                #=======================================================================================

                [p_out, i_out, d_out] = pr_pid.Compute(qry, pr_target, motion_dt)
                pr_out = p_out + i_out + d_out

                [p_out, i_out, d_out] = rr_pid.Compute(qrx, rr_target, motion_dt)
                rr_out = p_out + i_out + d_out

                [p_out, i_out, d_out] = yr_pid.Compute(qrz, yr_target, motion_dt)
                yr_out = p_out + i_out + d_out

                ################################## PID OUTPUT -> PWM CONVERSION ########################

                #---------------------------------------------------------------------------------------
                # Convert the vertical velocity PID output direct to ESC input PWM pulse width.
                #---------------------------------------------------------------------------------------
                #vert_out = hover_pwm + qaz_out #changed
                vert_out = hover_pwm + (rem_throttle * 500) #changed

                #---------------------------------------------------------------------------------------
                # Convert the rotation rate PID outputs direct to ESC input PWM pulse width
                #---------------------------------------------------------------------------------------
                pr_out /= 2
                rr_out /= 2
                yr_out /= 2


                #=======================================================================================
                # PID output distribution: Walk through the ESCs, and apply the PID outputs i.e. the
                # updates PWM pulse widths according to where the ESC is sited on the frame
                #=======================================================================================
                for esc in self.esc_list:

                    #-----------------------------------------------------------------------------------
                    # Update all blades' power in accordance with the z error
                    #-----------------------------------------------------------------------------------
                    pulse_width = vert_out + esc.spin - self.min_spin_pwm

                    #-----------------------------------------------------------------------------------
                    # For a left downwards roll, the x gyro goes negative, so the PID error is positive,
                    # meaning PID output is positive, meaning this needs to be added to the left blades
                    # and subtracted from the right.
                    #-----------------------------------------------------------------------------------
                    if esc.motor_location & self.MOTOR_LOCATION_RIGHT:
                        pulse_width -= rr_out
                    else:
                        pulse_width += rr_out

                    #-----------------------------------------------------------------------------------
                    # For a forward downwards pitch, the y gyro goes positive The PID error is negative as a
                    # result, meaning PID output is negative, meaning this needs to be subtracted from the
                    # front blades and added to the back.
                    #-----------------------------------------------------------------------------------
                    if esc.motor_location & self.MOTOR_LOCATION_BACK:
                        pulse_width += pr_out
                    else:
                        pulse_width -= pr_out

                    #-----------------------------------------------------------------------------------
                    # For CW yaw, the z gyro goes negative, so the PID error is postitive, meaning PID
                    # output is positive, meaning this need to be added to the ACW (FL and BR) blades and
                    # subtracted from the CW (FR & BL) blades.
                    #-----------------------------------------------------------------------------------
                    if esc.motor_rotation == self.MOTOR_ROTATION_CW:
                        pulse_width += yr_out
                    else:
                        pulse_width -= yr_out

                    #-----------------------------------------------------------------------------------
                    # Apply the blended outputs to the esc PWM signal
                    #-----------------------------------------------------------------------------------
                    esc.set(int(round(pulse_width)))

                #---------------------------------------------------------------------------------------
                # Diagnostic log - every motion loop
                #---------------------------------------------------------------------------------------
                if diagnostics:
                    temp = mpu6050.readTemperature()

                    pwm_data = "%d, %d, %d, %d" % (self.esc_list[0].pulse_width,
                                                   self.esc_list[1].pulse_width,
                                                   self.esc_list[2].pulse_width,
                                                   self.esc_list[3].pulse_width) if i_am_zoe else "%d, %d, %d, %d, %d, %d, %d, %d" % (self.esc_list[0].pulse_width,
                                                                                                                                      self.esc_list[1].pulse_width,
                                                                                                                                      self.esc_list[2].pulse_width,
                                                                                                                                      self.esc_list[3].pulse_width,
                                                                                                                                      self.esc_list[4].pulse_width,
                                                                                                                                      self.esc_list[5].pulse_width,
                                                                                                                                      self.esc_list[6].pulse_width,
                                                                                                                                      self.esc_list[7].pulse_width)
                    logger.warning("%f, %f, %d, " % (sampling_loops / sampling_rate, motion_dt, sampling_loops) +
                                   "%f, " % (temp / 333.86 + 21) +
    #                               "%f, %f, %f, %f, " % (mgx, mgy, mgz, math.degrees(cya)) + #changed
                                   "%f, %f, %f, " % (edx_fuse, edy_fuse, edz_fuse) +
                                   "%f, %f, %f, " % (evx_fuse, evy_fuse, evz_fuse) +
    #                               "%f, %f, %f, " % (qdx_fuse, qdy_fuse, qdz_fuse) +
    #                               "%f, %f, %f, " % (qvx_fuse, qvy_fuse, qvz_fuse) +
                                   "%f, %f, %f, " % (edx_target, edy_target, edz_target) +
                                   "%f, %f, %f, " % (evx_target, evy_target, evz_target) +
                                   "%f, %f, %f, " % (qrx, qry, qrz) +
                                   "%f, %f, %f, " % (qax, qay, qaz) +
                                   "%f, %f, %f, " % (qgx, qgy, qgz) +
                                   "%f, %f, %f, %f, " % (math.degrees(pa), math.degrees(ra), math.degrees(ya), math.degrees(ya_fused)) +

                                   "%f, %f, %f, " % (qdx_input, qdy_input, qdz_input) +
                                   "%f, %f, %f, " % (qdx_target, qdy_target, qdz_target) +
                                   "%f, %f, %f, " % (qvx_input, qvy_input, qvz_input) +
                                   "%f, %f, %f, " % (qvx_target, qvy_target, qvz_target) +
    #                               "%f, " % qvz_out +
                                   "YOLO," +
                                   "%f, %f, %f, " % (math.degrees(apa), math.degrees(ara), math.degrees(aya)) +
                                   "%f, %f, %f, " % (math.degrees(pa_target), math.degrees(ra_target), math.degrees(ya_target)) +
                                   "%f, %f, %f, " % (math.degrees(qry), math.degrees(qrx), math.degrees(qrz)) +
                                   "%f, %f, %f, " % (math.degrees(pr_target), math.degrees(rr_target), math.degrees(yr_target)) +
                                   "%f, %f, %f, " % (math.degrees(pr_out), math.degrees(rr_out), math.degrees(yr_out)) +

                                   "%f, %f, %f, %f, %f, %f, %f, %f, %d, " % (qdx_input, qdx_target, qvx_input, qvx_target, math.degrees(apa), math.degrees(pa_target), math.degrees(qry), math.degrees(pr_target), pr_out) +
                                   "%f, %f, %f, %f, %f, %f, %f, %f, %d, " % (qdy_input, qdy_target, qvy_input, qvy_target, math.degrees(ara), math.degrees(ra_target), math.degrees(qrx), math.degrees(rr_target), rr_out) +
                                   "%f, %f, %f, %f, %d, " % (qdz_input, qdz_target, qvz_input, qvz_target, qaz_out) +
                                   "%f, %f, %f, %f, %d, " % (math.degrees(aya), math.degrees(ya_target), math.degrees(qrz), math.degrees(yr_target), yr_out) +
                                   pwm_data)


                #prev_time = now_time
                #n_t = now_time = time.time()
                #dtt = n_t - p_t
                #dt = now_time - prev_time
                #if(dt > max_time):
                #    max_time = dt
                #if(dt < min_time):
                #    min_time = dt
                #print "dt : " + str((dt) * 1000)
                #print "                         dtt : " + str((dtt) * 1000)

                if(remote.rem_throttle == -1 and remote.rem_switch == 1):
                    break

            #stop_time_yolo = time.time()
            #print "min_time : " + str(min_time*1000)
            #print "max_time : " + str(max_time*1000)
            #print "FLIGHT TIME : " + str(stop_time_yolo - start_time_yolo)

            logger.critical("Flight time %f", time.time() - start_flight)
            logger.critical("Sampling loops: %d", sampling_loops)
            logger.critical("Motion processing loops: %d", motion_loops)
            logger.critical("Fusion processing loops: %d", fusion_loops)
            logger.critical("LiDAR processing loops: %d", garmin_loops)
            logger.critical("Autopilot processing loops: %d.", autopilot_loops)
            if sampling_loops != 0:
                logger.critical("Video frame rate: %f", video_loops * sampling_rate / sampling_loops )

            temp = mpu6050.readTemperature()
            logger.critical("IMU core temp (end): ,%f", temp / 333.86 + 21.0)
            max_az, min_az, max_gx, min_gx, max_gy, min_gy, max_gz, min_gz, = mpu6050.getStats()
            logger.critical("Max Z acceleration: %f", max_az)
            logger.critical("Min Z acceleration: %f", min_az)
            logger.critical("Max X gyrometer: %f", max_gx)
            logger.critical("Min X gyrometer: %f", min_gx)
            logger.critical("Max Y gyrometer: %f", max_gy)
            logger.critical("Min Y gyrometer: %f", min_gy)
            logger.critical("Max Z gyrometer: %f", max_gz)
            logger.critical("Min Z gyrometer: %f", min_gz)

            #-------------------------------------------------------------------------------------------
            # Stop the PWM and FIFO overflow interrupt between flights
            #-------------------------------------------------------------------------------------------
            for esc in self.esc_list:
                esc.set(0)
            mpu6050.disableFIFOOverflowISR()

        #newcode
        remote.stop()


    ################################################################################################
    #
    # Shutdown triggered by early Ctrl-C or end of script
    #
    ################################################################################################
    def shutdown(self):

        #-------------------------------------------------------------------------------------------
        # Stop the signal handler
        #-------------------------------------------------------------------------------------------
        signal.signal(signal.SIGINT, signal.SIG_IGN)

        #-------------------------------------------------------------------------------------------
        # Stop the blades spinning
        #-------------------------------------------------------------------------------------------
        for esc in self.esc_list:
            esc.set(stfu_pwm)

        #-------------------------------------------------------------------------------------------
        # Copy logs from /dev/shm (shared / virtual memory) to disk.
        #-------------------------------------------------------------------------------------------
        file_handler.close()

        #-------------------------------------------------------------------------------------------
        # Unlock memory we've used from RAM
        #-------------------------------------------------------------------------------------------
        munlockall()

        #-------------------------------------------------------------------------------------------
        # Clean up PWM / GPIO, but pause beforehand to give the ESCs time to stop properly
        #-------------------------------------------------------------------------------------------
        time.sleep(1.0)
        PWMTerm()

        #-------------------------------------------------------------------------------------------
        # Clean up the GPIO FIFO Overflow ISR
        #-------------------------------------------------------------------------------------------
        GPIOTerm()

        #-------------------------------------------------------------------------------------------
        # Reset the signal handler to default
        #-------------------------------------------------------------------------------------------
        signal.signal(signal.SIGINT, signal.SIG_DFL)

        sys.exit(0)

    ####################################################################################################
    #
    # Signal handler for Ctrl-C => abort cleanly; should really be just a "try: except KeyboardInterrupt:"
    #
    ####################################################################################################
    def shutdownSignalHandler(self, signal, frame):
        if not self.keep_looping:
            self.shutdown()
        self.keep_looping = False

    ####################################################################################################
    #
    # Interrupt Service Routine for FIFO overflow => abort flight cleanly - RETIRED, JUST POLL NOW
    #
    ####################################################################################################
    def fifoOverflowISR(self, pin):
        if self.keep_looping:
            print "FIFO OVERFLOW, ABORT"
            self.keep_looping = False

os.nice(-10)

if __name__ == '__main__':
    #-------------------------------------------------------------------------------------
    # Off we go!
    #-------------------------------------------------------------------------------------
    Quadcopter().fly()
