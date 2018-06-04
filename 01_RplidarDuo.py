# 01_RplidarDuo.py - run two RplidarA2M8 Simultaneously

# Rplidar A2M8 - Py on Raspberry Pi 3
# Python 3 only

import random
import logging
import sys
import time
import codecs
import serial
import struct
import numpy as np

from collections import namedtuple
from multiprocessing import Process
from multiprocessing import Queue



def lidarLeft_Iter(whichLidar):
    # lidar - LEFT
    # ================================================================
    # index subscripts
    idx_NewScan     = 0
    idx_QOL         = 1
    idx_AngleDeg    = 2
    idx_DistMm      = 3

    SYNC_BYTE       = b'\xA5'
    SYNC_BYTE2      = b'\x5A'
    GET_INFO_BYTE   = b'\x50'
    GET_HEALTH_BYTE = b'\x52'
    STOP_BYTE       = b'\x25'
    RESET_BYTE      = b'\x40'

    _SCAN_TYPE = {
        'normal': {'byte': b'\x20', 'response': 129, 'size': 5},
        'force': {'byte': b'\x21', 'response': 129, 'size': 5},
        'express': {'byte': b'\x82', 'response': 130, 'size': 84},
    }

    DESCRIPTOR_LEN = 7
    INFO_LEN = 20
    HEALTH_LEN = 3

    INFO_TYPE = 4
    HEALTH_TYPE = 6

    # Constants & Command to start A2 motor
    MAX_MOTOR_PWM = 1023
    DEFAULT_MOTOR_PWM = 660
    SET_PWM_BYTE = b'\xF0'

    _HEALTH_STATUSES = {
        0: 'Good',
        1: 'Warning',
        2: 'Error',
    }

    class RPLidarException(Exception):
        '''Basic exception class for RPLidar'''    

    def _b2i(byte):
        '''Converts byte to integer (for Python 2 compatability)'''
        return byte if int(sys.version[0]) == 3 else ord(byte)

    def _showhex(signal):
        '''Converts string bytes to hex representation (useful for debugging)'''
        return [format(_b2i(b), '#02x') for b in signal]

    def _process_scan(raw):
        '''Processes input raw data and returns measurement data'''
        new_scan = bool(_b2i(raw[0]) & 0b1)
        inversed_new_scan = bool((_b2i(raw[0]) >> 1) & 0b1)
        quality = _b2i(raw[0]) >> 2
        if new_scan == inversed_new_scan:
            raise RPLidarException('New scan flags mismatch')
        check_bit = _b2i(raw[1]) & 0b1
        if check_bit != 1:
            raise RPLidarException('Check bit not equal to 1')
        angle = ((_b2i(raw[1]) >> 1) + (_b2i(raw[2]) << 7)) / 64.
        distance = (_b2i(raw[3]) + (_b2i(raw[4]) << 8)) / 4.
        return new_scan, quality, angle, distance


    def _process_express_scan(data, new_angle, trame):
        new_scan = (new_angle < data.start_angle) & (trame == 1)
        angle = (data.start_angle + (
                (new_angle - data.start_angle) % 360
                )/32*trame - data.angle[trame-1]) % 360
        distance = data.distance[trame-1]
        return new_scan, None, angle, distance


    class RPLidar(object):
        '''Class for communicating with RPLidar rangefinder scanners'''

        def __init__(self, port, baudrate=115200, timeout=1):
            '''Initilize RPLidar object for communicating with the sensor.

            Parameters
            ----------
            port : str
                Serial port name to which sensor is connected
                
            baudrate : int, optional
                Baudrate for serial connection (the default is 115200)
                
            timeout : float, optional
                Serial port connection timeout in seconds (the default is 1)
            
            '''
            self._serial = None
            self.port = port
            self.baudrate = baudrate
            self.timeout = timeout
            self._motor_speed = DEFAULT_MOTOR_PWM
            self.scanning = [False, 0, 'normal']
            self.express_trame = 32
            self.express_data = False
            self.motor_running = None
            #logging.basicConfig(filename='logger.txt',level=logging.DEBUG)
            logging.basicConfig(filename='A2Mx_LidarLeft_Log.txt',level=logging.CRITICAL)
     
            self.connect()

        def connect(self):
            '''Connects to the serial port with the name `self.port`. If it was
            connected to another serial port disconnects from it first.'''
            if self._serial is not None:
                self.disconnect()
                logging.info('Serial port in use, disconnecting first')
            try:
                self._serial = serial.Serial(
                    self.port, self.baudrate,
                    parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                    timeout=self.timeout)
                logging.info('Connecting to Serial')
            except serial.SerialException as err:
                raise RPLidarException('Failed to connect to the sensor '
                                       'due to: %s' % err)
                logging.info('Failed to connect to sensor due to {}'.format(err))

        def disconnect(self):
            '''Disconnects from the serial port'''
            if self._serial is None:
                return
            self._serial.close()
            logging.info('sensor is disconnected')

        def _set_pwm(self, pwm):
            payload = struct.pack("<H", pwm)
            self._send_payload_cmd(SET_PWM_BYTE, payload)
            #logging.debug('PWM cmd byte is {}, payload is {}'.format(SET_PWM_BYTE, payload))

        @property
        def motor_speed(self):        
            return self._motor_speed

        @motor_speed.setter
        def motor_speed(self, pwm):
            assert(0 <= pwm <= MAX_MOTOR_PWM)
            self._motor_speed = pwm
            if self.motor_running:
                self._set_pwm(self._motor_speed)

        def start_motor(self):
            '''Starts sensor motor'''
            logging.info('Starting motor')
            # For A1
            self._serial.setDTR(False)

            # For A2
            self._set_pwm(self._motor_speed)
            self.motor_running = True

        def stop_motor(self):
            '''Stops sensor motor'''
            logging.info('Stoping motor')
            # For A2
            self._set_pwm(0)
            time.sleep(.001)
            # For A1
            self._serial.setDTR(True)
            self.motor_running = False

        def _send_payload_cmd(self, cmd, payload):
            '''Sends `cmd` command with `payload` to the sensor'''
            size = struct.pack('B', len(payload))
            req = SYNC_BYTE + cmd + size + payload
            checksum = 0
            for v in struct.unpack('B'*len(req), req):
                checksum ^= v
            req += struct.pack('B', checksum)
            self._serial.write(req)
            #logging.debug('Command sent: {}'.format(_showhex(req)))

        def _send_cmd(self, cmd):
            '''Sends `cmd` command to the sensor'''
            req = SYNC_BYTE + cmd
            self._serial.write(req)
            #logging.debug('Command sent: {}'.format(_showhex(req)))

        def _read_descriptor(self):
            '''Reads descriptor packet'''
            descriptor = self._serial.read(DESCRIPTOR_LEN)
            #logging.debug('Received descriptor: {}'.format(_showhex(descriptor)))
            if len(descriptor) != DESCRIPTOR_LEN:
                raise RPLidarException('Descriptor length mismatch')
            elif not descriptor.startswith(SYNC_BYTE + SYNC_BYTE2):
                raise RPLidarException('Incorrect descriptor starting bytes')
            is_single = _b2i(descriptor[-2]) == 0
            return _b2i(descriptor[2]), is_single, _b2i(descriptor[-1])

        def _read_response(self, dsize):
            '''Reads response packet with length of `dsize` bytes'''
            #logging.debug('Trying to read response: {} bytes'.format(dsize))
            while self._serial.inWaiting() < dsize:
                time.sleep(0.001)
            data = self._serial.read(dsize)
            #logging.debug('Received data: {}'.format(_showhex(data)))
            return data

        def get_info(self):
            '''Get device information

            Returns
            -------
            dict
                Dictionary with the sensor information
            '''
            if self._serial.inWaiting() > 0:
                return ('Data in buffer, you can\'t have info ! '
                        'Run clean_input() to emptied the buffer.')
            self._send_cmd(GET_INFO_BYTE)
            dsize, is_single, dtype = self._read_descriptor()
            if dsize != INFO_LEN:
                raise RPLidarException('Wrong get_info reply length')
            if not is_single:
                raise RPLidarException('Not a single response mode')
            if dtype != INFO_TYPE:
                raise RPLidarException('Wrong response data type')
            raw = self._read_response(dsize)
            serialnumber = codecs.encode(raw[4:], 'hex').upper()
            serialnumber = codecs.decode(serialnumber, 'ascii')
            data = {
                'model': _b2i(raw[0]),
                'firmware': (_b2i(raw[2]), _b2i(raw[1])),
                'hardware': _b2i(raw[3]),
                'serialnumber': serialnumber,
            }
            return data

        def get_health(self):
            '''Get device health state. When the core system detects some
            potential risk that may cause hardware failure in the future,
            the returned status value will be 'Warning'. But sensor can still work
            as normal. When sensor is in the Protection Stop state, the returned
            status value will be 'Error'. In case of warning or error statuses
            non-zero error code will be returned.

            Returns
            -------
            status : str
                'Good', 'Warning' or 'Error' statuses
            error_code : int
                The related error code that caused a warning/error.
            '''
            if self._serial.inWaiting() > 0:
                return ('Data in buffer, you can\'t have info ! '
                        'Run clean_input() to emptied the buffer.')
            logging.info('Asking for health')
            self._send_cmd(GET_HEALTH_BYTE)
            dsize, is_single, dtype = self._read_descriptor()
            if dsize != HEALTH_LEN:
                raise RPLidarException('Wrong get_info reply length')
            if not is_single:
                raise RPLidarException('Not a single response mode')
            if dtype != HEALTH_TYPE:
                raise RPLidarException('Wrong response data type')
            raw = self._read_response(dsize)
            status = _HEALTH_STATUSES[_b2i(raw[0])]
            error_code = (_b2i(raw[1]) << 8) + _b2i(raw[2])
            return status, error_code

        def clean_input(self):
            '''Clean input buffer by reading all available data'''
            if self.scanning[0]:
                return 'Cleanning not allowed during scanning process active !'
            self._serial.flushInput()
            self.express_trame = 32
            self.express_data = False

        def stop(self):
            '''Stops scanning process, disables laser diode and the measurement
            system, moves sensor to the idle state.'''
            logging.info('Stopping scanning')
            self._send_cmd(STOP_BYTE)
            time.sleep(.1)
            self.scanning[0] = False
            self.clean_input()

        def start(self, scan_type='normal'):
            '''Start the scanning process

            Parameters
            ----------
            scan : normal, force or express.
            '''
            if self.scanning[0]:
                return 'Scanning already running !'
            '''Start the scanning process, enable laser diode and the
            measurement system'''
            status, error_code = self.get_health()
            #logging.debug('Health status: {}, {}'.format(status, error_code))
            if status == _HEALTH_STATUSES[2]:
                logging.warning('Trying to reset sensor due to the error code: {}'.format(error_code))
                self.reset()
                status, error_code = self.get_health()
                if status == _HEALTH_STATUSES[2]:
                    raise RPLidarException('RPLidar hardware failure. '
                                           'Error code: %d' % error_code)
            elif status == _HEALTH_STATUSES[1]:
                logging.warning('Warning sensor status detected, code {}'.format(error_code))

            cmd = _SCAN_TYPE[scan_type]['byte']
            logging.info('starting scan process in {} mode'.format(scan_type))

            if scan_type == 'express':
                self._send_payload_cmd(cmd, b'\x00\x00\x00\x00\x00')
            else:
                self._send_cmd(cmd)

            dsize, is_single, dtype = self._read_descriptor()
            if dsize != _SCAN_TYPE[scan_type]['size']:
                raise RPLidarException('Wrong get_info reply length')
            if is_single:
                raise RPLidarException('Not a multiple response mode')
            if dtype != _SCAN_TYPE[scan_type]['response']:
                raise RPLidarException('Wrong response data type')
            self.scanning = [True, dsize, scan_type]

        def reset(self):
            '''Resets sensor core, reverting it to a similar state as it has
            just been powered up.'''
            logging.info('Resetting the sensor')
            self._send_cmd(RESET_BYTE)
            time.sleep(2)
            self.clean_input()

        def iter_measures(self, scan_type='normal', max_buf_meas=3000):
            '''Iterate over measures. Note that consumer must be fast enough,
            otherwise data will be accumulated inside buffer and consumer will get
            data with increasing lag.

            Parameters
            ----------
            max_buf_meas : int or False if you want unlimited buffer
                Maximum number of bytes to be stored inside the buffer. Once
                numbe exceeds this limit buffer will be emptied out.

            Yields
            ------
            new_scan : bool
                True if measures belongs to a new scan
            quality : int
                Reflected laser pulse strength
            angle : float
                The measure heading angle in degree unit [0, 360)
            distance : float
                Measured object distance related to the sensor's rotation center.
                In millimeter unit. Set to 0 when measure is invalid.
            '''
            self.start_motor()
            if not self.scanning[0]:
                self.start(scan_type)
            while True:
                dsize = self.scanning[1]
                if max_buf_meas:
                    data_in_buf = self._serial.inWaiting()
                    if data_in_buf > max_buf_meas:
                        logging.warning('Too many bytes in the input buffer, cleaning buffer: {} / {}'.format(data_in_buf, max_buf_meas))
                        self.stop()
                        self.start(self.scanning[2])

                if self.scanning[2] == 'normal':
                    raw = self._read_response(dsize)
                    yield _process_scan(raw)
                if self.scanning[2] == 'express':
                    if self.express_trame == 32:
                        self.express_trame = 0
                        if not self.express_data:
                            self.express_data = ExpressPacket.from_string(
                                                self._read_response(dsize))

                        self.express_old_data = self.express_data
                        self.express_data = ExpressPacket.from_string(
                                            self._read_response(dsize))
     
                    self.express_trame += 1
                    yield _process_express_scan(self.express_old_data,
                                                self.express_data.start_angle,
                                                self.express_trame)

        def iter_scans(self, scan_type='normal', max_buf_meas=3000, min_len=5):
            '''Iterate over scans. Note that consumer must be fast enough,
            otherwise data will be accumulated inside buffer and consumer will get
            data with increasing lag.

            Parameters
            ----------
            max_buf_meas : int
                Maximum number of measures to be stored inside the buffer. Once
                numbe exceeds this limit buffer will be emptied out.
            min_len : int
                Minimum number of measures in the scan for it to be yelded.

            Yields
            ------
            scan : list
                List of the measures. Each measurment is tuple with following
                format: (quality, angle, distance). For values description please
                refer to `iter_measures` method's documentation.
            '''
            scan_list = []
            iterator = self.iter_measures(scan_type, max_buf_meas)
            for new_scan, quality, angle, distance in iterator:
                if new_scan:
                    if len(scan_list) > min_len:
                        yield scan_list
                    scan_list = []
                if distance > 0:
                    scan_list.append((quality, angle, distance))


    class ExpressPacket(namedtuple('express_packet',
                                   'distance angle new_scan start_angle')):
        sync1 = 0xa
        sync2 = 0x5
        sign = {0: 1, 1: -1}

        @classmethod
        def from_string(cls, data):
            packet = bytearray(data)

            if (packet[0] >> 4) != cls.sync1 or (packet[1] >> 4) != cls.sync2:
                raise ValueError('try to parse corrupted data ({})'.format(packet))

            checksum = 0
            for b in packet[2:]:
                checksum ^= b
            if checksum != (packet[0] & 0b00001111) + ((
                            packet[1] & 0b00001111) << 4):
                raise ValueError('Invalid checksum ({})'.format(packet))

            new_scan = packet[3] >> 7
            start_angle = (packet[2] + ((packet[3] & 0b01111111) << 8)) / 64

            d = a = ()
            for i in range(0,80,5):
                d += ((packet[i+4] >> 2) + (packet[i+5] << 6),)
                a += (((packet[i+8] & 0b00001111) + ((
                        packet[i+4] & 0b00000001) << 4))/8*cls.sign[(
                         packet[i+4] & 0b00000010) >> 1],)
                d += ((packet[i+6] >> 2) + (packet[i+7] << 6),)
                a += (((packet[i+8] >> 4) + (
                    (packet[i+6] & 0b00000001) << 4))/8*cls.sign[(
                        packet[i+6] & 0b00000010) >> 1],)
            return cls(d, a, new_scan, start_angle)

    # todo: temp: create a random fn that delivers 'real' obs# to the queue (both lidars)
    try:
        lidar = RPLidar(whichLidar)
    except:
        print('\nRplidarA2M8\nUnable to connect to {}'.format(whichLidar))
        sys.exit()     # todo: after debugging, uncomment this line
        
    while True:
        for measurement in lidar.iter_measures(max_buf_meas=1000):
            #print('L -> {}'.format(measurement[idx_DistMm]))
            obsD_Q.put('L - '+ str(measurement[idx_DistMm]))
    


def lidarRight_Iter(whichLidar):
    # lidar RIGHT
    # ================================================================
    # index subscripts
    idx_NewScan     = 0
    idx_QOL         = 1
    idx_AngleDeg    = 2
    idx_DistMm      = 3

    SYNC_BYTE       = b'\xA5'
    SYNC_BYTE2      = b'\x5A'
    GET_INFO_BYTE   = b'\x50'
    GET_HEALTH_BYTE = b'\x52'
    STOP_BYTE       = b'\x25'
    RESET_BYTE      = b'\x40'

    _SCAN_TYPE = {
        'normal': {'byte': b'\x20', 'response': 129, 'size': 5},
        'force': {'byte': b'\x21', 'response': 129, 'size': 5},
        'express': {'byte': b'\x82', 'response': 130, 'size': 84},
    }

    DESCRIPTOR_LEN = 7
    INFO_LEN = 20
    HEALTH_LEN = 3

    INFO_TYPE = 4
    HEALTH_TYPE = 6

    # Constants & Command to start A2 motor
    MAX_MOTOR_PWM = 1023
    DEFAULT_MOTOR_PWM = 660
    SET_PWM_BYTE = b'\xF0'

    _HEALTH_STATUSES = {
        0: 'Good',
        1: 'Warning',
        2: 'Error',
    }

    class RPLidarException(Exception):
        '''Basic exception class for RPLidar'''    

    def _b2i(byte):
        '''Converts byte to integer (for Python 2 compatability)'''
        return byte if int(sys.version[0]) == 3 else ord(byte)

    def _showhex(signal):
        '''Converts string bytes to hex representation (useful for debugging)'''
        return [format(_b2i(b), '#02x') for b in signal]

    def _process_scan(raw):
        '''Processes input raw data and returns measurement data'''
        new_scan = bool(_b2i(raw[0]) & 0b1)
        inversed_new_scan = bool((_b2i(raw[0]) >> 1) & 0b1)
        quality = _b2i(raw[0]) >> 2
        if new_scan == inversed_new_scan:
            raise RPLidarException('New scan flags mismatch')
        check_bit = _b2i(raw[1]) & 0b1
        if check_bit != 1:
            raise RPLidarException('Check bit not equal to 1')
        angle = ((_b2i(raw[1]) >> 1) + (_b2i(raw[2]) << 7)) / 64.
        distance = (_b2i(raw[3]) + (_b2i(raw[4]) << 8)) / 4.
        return new_scan, quality, angle, distance


    def _process_express_scan(data, new_angle, trame):
        new_scan = (new_angle < data.start_angle) & (trame == 1)
        angle = (data.start_angle + (
                (new_angle - data.start_angle) % 360
                )/32*trame - data.angle[trame-1]) % 360
        distance = data.distance[trame-1]
        return new_scan, None, angle, distance


    class RPLidar(object):
        '''Class for communicating with RPLidar rangefinder scanners'''

        def __init__(self, port, baudrate=115200, timeout=1):
            '''Initilize RPLidar object for communicating with the sensor.

            Parameters
            ----------
            port : str
                Serial port name to which sensor is connected
                
            baudrate : int, optional
                Baudrate for serial connection (the default is 115200)
                
            timeout : float, optional
                Serial port connection timeout in seconds (the default is 1)
            
            '''
            self._serial = None
            self.port = port
            self.baudrate = baudrate
            self.timeout = timeout
            self._motor_speed = DEFAULT_MOTOR_PWM
            self.scanning = [False, 0, 'normal']
            self.express_trame = 32
            self.express_data = False
            self.motor_running = None
            #logging.basicConfig(filename='logger.txt',level=logging.DEBUG)
            logging.basicConfig(filename='A2Mx_LidarRight_Log.txt',level=logging.CRITICAL)
     
            self.connect()

        def connect(self):
            '''Connects to the serial port with the name `self.port`. If it was
            connected to another serial port disconnects from it first.'''
            if self._serial is not None:
                self.disconnect()
                logging.info('Serial port in use, disconnecting first')
            try:
                self._serial = serial.Serial(
                    self.port, self.baudrate,
                    parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                    timeout=self.timeout)
                logging.info('Connecting to Serial')
            except serial.SerialException as err:
                raise RPLidarException('Failed to connect to the sensor '
                                       'due to: %s' % err)
                logging.info('Failed to connect to sensor due to {}'.format(err))

        def disconnect(self):
            '''Disconnects from the serial port'''
            if self._serial is None:
                return
            self._serial.close()
            logging.info('sensor is disconnected')

        def _set_pwm(self, pwm):
            payload = struct.pack("<H", pwm)
            self._send_payload_cmd(SET_PWM_BYTE, payload)
            #logging.debug('PWM cmd byte is {}, payload is {}'.format(SET_PWM_BYTE, payload))

        @property
        def motor_speed(self):        
            return self._motor_speed

        @motor_speed.setter
        def motor_speed(self, pwm):
            assert(0 <= pwm <= MAX_MOTOR_PWM)
            self._motor_speed = pwm
            if self.motor_running:
                self._set_pwm(self._motor_speed)

        def start_motor(self):
            '''Starts sensor motor'''
            logging.info('Starting motor')
            # For A1
            self._serial.setDTR(False)

            # For A2
            self._set_pwm(self._motor_speed)
            self.motor_running = True

        def stop_motor(self):
            '''Stops sensor motor'''
            logging.info('Stoping motor')
            # For A2
            self._set_pwm(0)
            time.sleep(.001)
            # For A1
            self._serial.setDTR(True)
            self.motor_running = False

        def _send_payload_cmd(self, cmd, payload):
            '''Sends `cmd` command with `payload` to the sensor'''
            size = struct.pack('B', len(payload))
            req = SYNC_BYTE + cmd + size + payload
            checksum = 0
            for v in struct.unpack('B'*len(req), req):
                checksum ^= v
            req += struct.pack('B', checksum)
            self._serial.write(req)
            #logging.debug('Command sent: {}'.format(_showhex(req)))

        def _send_cmd(self, cmd):
            '''Sends `cmd` command to the sensor'''
            req = SYNC_BYTE + cmd
            self._serial.write(req)
            #logging.debug('Command sent: {}'.format(_showhex(req)))

        def _read_descriptor(self):
            '''Reads descriptor packet'''
            descriptor = self._serial.read(DESCRIPTOR_LEN)
            #logging.debug('Received descriptor: {}'.format(_showhex(descriptor)))
            if len(descriptor) != DESCRIPTOR_LEN:
                raise RPLidarException('Descriptor length mismatch')
            elif not descriptor.startswith(SYNC_BYTE + SYNC_BYTE2):
                raise RPLidarException('Incorrect descriptor starting bytes')
            is_single = _b2i(descriptor[-2]) == 0
            return _b2i(descriptor[2]), is_single, _b2i(descriptor[-1])

        def _read_response(self, dsize):
            '''Reads response packet with length of `dsize` bytes'''
            #logging.debug('Trying to read response: {} bytes'.format(dsize))
            while self._serial.inWaiting() < dsize:
                time.sleep(0.001)
            data = self._serial.read(dsize)
            #logging.debug('Received data: {}'.format(_showhex(data)))
            return data

        def get_info(self):
            '''Get device information

            Returns
            -------
            dict
                Dictionary with the sensor information
            '''
            if self._serial.inWaiting() > 0:
                return ('Data in buffer, you can\'t have info ! '
                        'Run clean_input() to emptied the buffer.')
            self._send_cmd(GET_INFO_BYTE)
            dsize, is_single, dtype = self._read_descriptor()
            if dsize != INFO_LEN:
                raise RPLidarException('Wrong get_info reply length')
            if not is_single:
                raise RPLidarException('Not a single response mode')
            if dtype != INFO_TYPE:
                raise RPLidarException('Wrong response data type')
            raw = self._read_response(dsize)
            serialnumber = codecs.encode(raw[4:], 'hex').upper()
            serialnumber = codecs.decode(serialnumber, 'ascii')
            data = {
                'model': _b2i(raw[0]),
                'firmware': (_b2i(raw[2]), _b2i(raw[1])),
                'hardware': _b2i(raw[3]),
                'serialnumber': serialnumber,
            }
            return data

        def get_health(self):
            '''Get device health state. When the core system detects some
            potential risk that may cause hardware failure in the future,
            the returned status value will be 'Warning'. But sensor can still work
            as normal. When sensor is in the Protection Stop state, the returned
            status value will be 'Error'. In case of warning or error statuses
            non-zero error code will be returned.

            Returns
            -------
            status : str
                'Good', 'Warning' or 'Error' statuses
            error_code : int
                The related error code that caused a warning/error.
            '''
            if self._serial.inWaiting() > 0:
                return ('Data in buffer, you can\'t have info ! '
                        'Run clean_input() to emptied the buffer.')
            logging.info('Asking for health')
            self._send_cmd(GET_HEALTH_BYTE)
            dsize, is_single, dtype = self._read_descriptor()
            if dsize != HEALTH_LEN:
                raise RPLidarException('Wrong get_info reply length')
            if not is_single:
                raise RPLidarException('Not a single response mode')
            if dtype != HEALTH_TYPE:
                raise RPLidarException('Wrong response data type')
            raw = self._read_response(dsize)
            status = _HEALTH_STATUSES[_b2i(raw[0])]
            error_code = (_b2i(raw[1]) << 8) + _b2i(raw[2])
            return status, error_code

        def clean_input(self):
            '''Clean input buffer by reading all available data'''
            if self.scanning[0]:
                return 'Cleanning not allowed during scanning process active !'
            self._serial.flushInput()
            self.express_trame = 32
            self.express_data = False

        def stop(self):
            '''Stops scanning process, disables laser diode and the measurement
            system, moves sensor to the idle state.'''
            logging.info('Stopping scanning')
            self._send_cmd(STOP_BYTE)
            time.sleep(.1)
            self.scanning[0] = False
            self.clean_input()

        def start(self, scan_type='normal'):
            '''Start the scanning process

            Parameters
            ----------
            scan : normal, force or express.
            '''
            if self.scanning[0]:
                return 'Scanning already running !'
            '''Start the scanning process, enable laser diode and the
            measurement system'''
            status, error_code = self.get_health()
            #logging.debug('Health status: {}, {}'.format(status, error_code))
            if status == _HEALTH_STATUSES[2]:
                logging.warning('Trying to reset sensor due to the error code: {}'.format(error_code))
                self.reset()
                status, error_code = self.get_health()
                if status == _HEALTH_STATUSES[2]:
                    raise RPLidarException('RPLidar hardware failure. '
                                           'Error code: %d' % error_code)
            elif status == _HEALTH_STATUSES[1]:
                logging.warning('Warning sensor status detected, code {}'.format(error_code))

            cmd = _SCAN_TYPE[scan_type]['byte']
            logging.info('starting scan process in {} mode'.format(scan_type))

            if scan_type == 'express':
                self._send_payload_cmd(cmd, b'\x00\x00\x00\x00\x00')
            else:
                self._send_cmd(cmd)

            dsize, is_single, dtype = self._read_descriptor()
            if dsize != _SCAN_TYPE[scan_type]['size']:
                raise RPLidarException('Wrong get_info reply length')
            if is_single:
                raise RPLidarException('Not a multiple response mode')
            if dtype != _SCAN_TYPE[scan_type]['response']:
                raise RPLidarException('Wrong response data type')
            self.scanning = [True, dsize, scan_type]

        def reset(self):
            '''Resets sensor core, reverting it to a similar state as it has
            just been powered up.'''
            logging.info('Resetting the sensor')
            self._send_cmd(RESET_BYTE)
            time.sleep(2)
            self.clean_input()

        def iter_measures(self, scan_type='normal', max_buf_meas=3000):
            '''Iterate over measures. Note that consumer must be fast enough,
            otherwise data will be accumulated inside buffer and consumer will get
            data with increasing lag.

            Parameters
            ----------
            max_buf_meas : int or False if you want unlimited buffer
                Maximum number of bytes to be stored inside the buffer. Once
                numbe exceeds this limit buffer will be emptied out.

            Yields
            ------
            new_scan : bool
                True if measures belongs to a new scan
            quality : int
                Reflected laser pulse strength
            angle : float
                The measure heading angle in degree unit [0, 360)
            distance : float
                Measured object distance related to the sensor's rotation center.
                In millimeter unit. Set to 0 when measure is invalid.
            '''
            self.start_motor()
            if not self.scanning[0]:
                self.start(scan_type)
            while True:
                dsize = self.scanning[1]
                if max_buf_meas:
                    data_in_buf = self._serial.inWaiting()
                    if data_in_buf > max_buf_meas:
                        logging.warning('Too many bytes in the input buffer, cleaning buffer: {} / {}'.format(data_in_buf, max_buf_meas))
                        self.stop()
                        self.start(self.scanning[2])

                if self.scanning[2] == 'normal':
                    raw = self._read_response(dsize)
                    yield _process_scan(raw)
                if self.scanning[2] == 'express':
                    if self.express_trame == 32:
                        self.express_trame = 0
                        if not self.express_data:
                            self.express_data = ExpressPacket.from_string(
                                                self._read_response(dsize))

                        self.express_old_data = self.express_data
                        self.express_data = ExpressPacket.from_string(
                                            self._read_response(dsize))
     
                    self.express_trame += 1
                    yield _process_express_scan(self.express_old_data,
                                                self.express_data.start_angle,
                                                self.express_trame)

        def iter_scans(self, scan_type='normal', max_buf_meas=3000, min_len=5):
            '''Iterate over scans. Note that consumer must be fast enough,
            otherwise data will be accumulated inside buffer and consumer will get
            data with increasing lag.

            Parameters
            ----------
            max_buf_meas : int
                Maximum number of measures to be stored inside the buffer. Once
                numbe exceeds this limit buffer will be emptied out.
            min_len : int
                Minimum number of measures in the scan for it to be yelded.

            Yields
            ------
            scan : list
                List of the measures. Each measurment is tuple with following
                format: (quality, angle, distance). For values description please
                refer to `iter_measures` method's documentation.
            '''
            scan_list = []
            iterator = self.iter_measures(scan_type, max_buf_meas)
            for new_scan, quality, angle, distance in iterator:
                if new_scan:
                    if len(scan_list) > min_len:
                        yield scan_list
                    scan_list = []
                if distance > 0:
                    scan_list.append((quality, angle, distance))


    class ExpressPacket(namedtuple('express_packet',
                                   'distance angle new_scan start_angle')):
        sync1 = 0xa
        sync2 = 0x5
        sign = {0: 1, 1: -1}

        @classmethod
        def from_string(cls, data):
            packet = bytearray(data)

            if (packet[0] >> 4) != cls.sync1 or (packet[1] >> 4) != cls.sync2:
                raise ValueError('try to parse corrupted data ({})'.format(packet))

            checksum = 0
            for b in packet[2:]:
                checksum ^= b
            if checksum != (packet[0] & 0b00001111) + ((
                            packet[1] & 0b00001111) << 4):
                raise ValueError('Invalid checksum ({})'.format(packet))

            new_scan = packet[3] >> 7
            start_angle = (packet[2] + ((packet[3] & 0b01111111) << 8)) / 64

            d = a = ()
            for i in range(0,80,5):
                d += ((packet[i+4] >> 2) + (packet[i+5] << 6),)
                a += (((packet[i+8] & 0b00001111) + ((
                        packet[i+4] & 0b00000001) << 4))/8*cls.sign[(
                         packet[i+4] & 0b00000010) >> 1],)
                d += ((packet[i+6] >> 2) + (packet[i+7] << 6),)
                a += (((packet[i+8] >> 4) + (
                    (packet[i+6] & 0b00000001) << 4))/8*cls.sign[(
                        packet[i+6] & 0b00000010) >> 1],)
            return cls(d, a, new_scan, start_angle)

    # todo: temp: create a random fn that delivers 'real' obs# to the queue (both lidars)
    try:
        lidar = RPLidar(whichLidar)
    except:
        print('\nRplidarA2M8\nUnable to connect to {}'.format(whichLidar))
        sys.exit()     # todo: after debugging, uncomment this line
        
    while True:
        for measurement in lidar.iter_measures(max_buf_meas=1000):
            #print('R -> {}'.format(measurement[idx_DistMm]))
            obsD_Q.put('R '+ str(measurement[idx_DistMm]))



def notifyArduino(whichArduino):
    # this fn reads the obstacle queue and send the zone_Num to the
    # arduino via serial (raspi USB to Arduino ucb type B)

    # todo: pop from queue and send to arduino, plug into robot and check if aKomodo is receiving it
    # ... chk via android app

    try:        
        ser = serial.Serial(arduinoPort,57600,timeout=0.1)    # can also try 115200 if req
        ser.reset_input_buffer()
    except:
        ser = None
        print('\nArduino MCU\nunable to access serial port.')
        sys.exit()     # after debugging, uncomment this line

    while True:
        #print('\n\nI am arduino --- {}'.format(whichArduino))
        #print('\n\nI am arduino --- {}'.format(whichArduino))
        while not obsD_Q.empty():
            print(obsD_Q.get())
    


# ==============================================================
# ==============================================================
#                               MAIN
# ==============================================================
# ==============================================================

# for testing, maybe need to add one more process that tracks the time and then signals the process to stop
# for actual deployment, maybe now need, since it runs forever until power off
# but this proc can still be useful, if we want the Lidar to stop spinning when robot is not moving.
# so this proc will be like a control system taking cmds from another brain

if __name__ == "__main__":
    logging.basicConfig(filename='duoLidar_Main.txt',level=logging.CRITICAL) # .INFO, .DEBUG, .CRITICAL
    logging.info('. starting rplidarA2M8 Duo lidars ... ')   

    # debugging stuff
    # ---------------
    #random.seed()    


    # USB Serial
    # ----------    
    #lidarPort = '/dev/ttyUSB0'
    #lidarPort = '/dev/ttyUSB1'
    #lidarPort_Left = '/dev/tty-lidarLeft'  
    #lidarsPort_Right = '/dev/tty-lidarRight'

    rcLidars = ['/dev/tty-lidarLeft','/dev/tty-lidarRight']
    
    #arduinoPort = '/dev/ttyACM0'
    #arduinoPort = '/dev/ttyACM1'
    arduinoPort = '/dev/tty-arduino'


    # delay 2s to let arduino power up first, else MCU's UART buffer may overflow (theory, untested)
    time.sleep(2)

    # Proc and Queue - Vars
    # ------------
    procs = []    
    obsD_Q      = Queue()   # instantiating the obstacle detected queue object 
    obsD_Q_Cnt  = 0         # init obstacle Detected Queue Counter

    # start multiprocessing
    # ---------------------
    # arduino
    proc = Process(target=notifyArduino, args=(arduinoPort,))
    procs.append(proc)     
    proc.start()

    # lidar - Left
    proc = Process(target=lidarLeft_Iter, args=(rcLidars[0],))
    procs.append(proc)     
    proc.start()

    # lidar - Right
    proc = Process(target=lidarRight_Iter, args=(rcLidars[1],))
    procs.append(proc)     
    proc.start()
    

    # time.sleep(3)     # temp pause to read exception msgs

    # complete the processes - processes join back the main thread
    # for the actual duo lidar code, there could be a infinite loop, since the lidar doesn't shut down
    # until powered down, so it never proc.join()

    # RC notes: how come main never ends?
    # If proc() is just a short finite fn, _main_ has ended will execute
    # but if proc() has a while True loop in it, _main_ has ended never executes.
    # may we need another "Timer Proc" that check the time
##    prgStop_Treshold = 5   # in seconds
##    prgTime_Start = 0
##    prgTime_Start = time.time()
##    while((time.time() - prgTime_Start)< prgStop_Treshold):
##        pass
##    else:         
##        for proc in procs:
##            proc.join()
##        print('\n__main__ has ended\n')



'''
    FIFO
    to push items in queue, obsD_Q.put(item)
    to pop items in queue,  obsD_Q.get(item)

    while not queue.empty():
        print(queue.get())    

'''




