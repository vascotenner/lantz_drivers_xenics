# -*- coding: utf-8 -*-
"""
    lantz.drivers.xenics.xcamera
    ~~~~~~~~~~~~~~~~~~~~~~~~~~

    Implements Xenics XControl driver for Xenics cameras


    Implementation Notes
    --------------------

    This wrapper uses the "old" Xcontrol driver.
    Only tested under CCD under Windows XP

    ---

    :copyright: 2015, see AUTHORS for more details.
    :license: GPL, see LICENSE for more details.
"""

import time
import os.path
import ctypes as ct
import threading

import numpy as np
from numpy.ctypeslib import ndpointer

from lantz import Action, Feat
from lantz.errors import InstrumentError
from lantz.foreign import LibraryDriver


class Status(object):
    READ_BUSY = 1
    BUFFER_EMPTY = 2
    COC_RUNNING = 4
    BUFFERS_FULL = 8

_ERRORS = {
    0: 'I_OK',
    10000: 'E_BUG',
    10001: 'E_NOINIT',
    10002: 'E_LOGICLOADFAILED',
    10003: 'E_OUT_OF_RANGE',
    10004: 'E_NOT_SUPPORTED',
    10005: 'E_NOT_FOUND',
    10006: 'E_FILTER_DONE',
    10007: 'E_SAVE_ERROR',
    10008: 'E_MISMATCHED',
    10009: 'E_BUSY',
    10010: 'E_TIMEOUT'}

# List of functions that should not be checked for errorcodes
_IGNORE_ERR = ['XC_OpenCamera',
               'XC_IsInitialised',
               'XC_GetHeight',
               'XC_GetWidth',
               'XC_GetFrameSizeInBytes',
               'XC_CloseCamera'
              ]

def is_running(func):
    """This decorator checks if the camera is still running"""
    # argnames = func.func_code.co_varnames[:func.func_code.co_argcount]
    try:
        fname = func.__name__
    except AttributeError:
        fname = 'Unkown'

    def checker(*args, **kwargs):
        if args[0].connected:
            return func(*args, **kwargs)
        else:
            args[0].log_info('Camera not running. Not running function %s'
                             % fname)

            def func2(*args, **kwargs):
                pass

            return func2

    return checker


class Xcamera(LibraryDriver):
    """Xenics Xcontrol Xcamera driver for Xenics CCD cameras
    """
    # 'c:/Program Files/X-Control/xcamera.dll'
    LIBRARY_NAME = 'xcamera.dll'
    readlock = threading.Lock()

    def __init__(self, camera_num=0,
                 #config_file='C:\Documents and Settings\MONA\Desktop\\vis\Xlin16BitFvB3.xcf',
                 #config_file=r'C:\Program Files\X-Control\Settings\Xlin16BitFvB3.xcf',
                 config_file=None, #r'C:\Program Files\X-Control\Settings\20130701_Vasco.xcf',
                 *args, **kwargs):
        """
        @params

        :type camera_num: int, The usb device index: 0,1,..
        :type config_file: string, Path of the config file for the camera
        """
        super().__init__(*args, **kwargs)
        self.camera_num = camera_num
        self.config_file = config_file
        self.connected = False

    def _add_types(self):
        self.lib.XC_OpenCamera.argtypes = [ct.c_uint]
        self.lib.XC_CloseCamera.argtypes = [ct.c_uint]
        self.lib.XC_IsInitialised.argtypes = [ct.c_uint]
        self.lib.XC_LoadFromFile.argtypes = [ct.c_uint, ct.c_char_p]
        self.lib.XC_SetGainCamera.argtypes = [ct.c_uint, ct.c_double]
        self.lib.XC_SetIntegrationTime.argtypes = [ct.c_uint, ct.c_ulong]
        self.lib.XC_SetCoolingTemperature.argtypes = [ct.c_uint, ct.c_short]
        self.lib.XC_SetFan.argtypes = [ct.c_uint, ct.c_bool]
        self.lib.XC_StartCapture.argtypes = [ct.c_uint]
        self.lib.XC_StopCapture.argtypes = [ct.c_uint]
        self.lib.XC_GetFrameSizeInBytes.argtypes = [ct.c_uint]
        self.lib.XC_CloseCamera.restype = ct.c_void_p
        self.lib.XC_StopCapture.restype = ct.c_void_p

    def _return_handler(self, func_name, ret_value):
        if self.lib.__getattribute__(func_name).restype != ct.c_void_p and (func_name not in _IGNORE_ERR):
            if ret_value > 0:
                raise InstrumentError(_ERRORS[ret_value])
        return ret_value

    def initialize(self):
        """ Connect the camera."""
        self.connected = False
        self.capturing = False
        self.wait = 0.02

        self.log_debug("Connecting to Camera")

        self.xcamera_id = self.lib.XC_OpenCamera(self.camera_num)
        if self.xcamera_id == None:
            self.log_critical("Could not connect to camera {}".format(self.camera_num))
        else:
            self.log_debug("Connected. xcamera_id = %d" % self.xcamera_id)
            self.connected = True
            if self.lib.XC_IsInitialised(self.xcamera_id) == 0:
                self.log_error("Camera could not be initialized. Retry.")
                self.finalize()
                self.initialize()
            else:
                self.log_debug("Camera initialized.")

            self.__load_config()

            self.__sensor_shape()
            self.__frame_size()
            self.buffer = np.zeros(shape=self.shape, dtype=np.uint16)

    def finalize(self):
        self.log_info("Disconnecting Camera")
        if self.connected:
            self.connected = False
            self.lib.XC_CloseCamera(self.xcamera_id)
        else:
            self.log_debug("Already disconnected")

    @Action()
    @is_running
    def __load_config(self):
        if self.config_file:
            if not os.path.exists(self.config_file):
                raise FileExistsError('The specified configfile does not exist: {}'.format(self.config_file))
            self.log_debug("Loading config file: %s" % self.config_file)
            self.lib.XC_LoadFromFile(self.xcamera_id,
                                     self.config_file)

    @Action()
    @is_running
    def __sensor_shape(self):
        """ Read shape of sensor from camera """
        height = self.lib.XC_GetHeight(self.xcamera_id)
        width = self.lib.XC_GetWidth(self.xcamera_id)
        self.shape = (height, width)
        self.log_debug("Read camera shape %s" % str(self.shape))

    @Action()
    @is_running
    def __frame_size(self):
        self.frame_size = self.lib.XC_GetFrameSizeInBytes(self.xcamera_id)
        self.log_debug("Read frame size %d" % self.frame_size)
        self.lib.XC_CopyFrameBuffer.argtypes = [ct.c_uint,
                                                ndpointer(dtype=np.uint16, shape=self.shape),
                                                ct.c_uint]

    @Feat(values={'low': 3, 'medium': 2, 'high': 1, 'super_high': 0})
    @is_running
    def gain(self):
        gain = ct.c_double()
        self.lib.XC_GetGainCamera(self.xcamera_id,
                                                 ct.byref(gain))
        return gain.value

    @gain.setter
    @is_running
    def gain(self, gain):
        (self.lib.XC_SetGainCamera(self.xcamera_id,
                                                 gain))
        self.log_debug("Gain set to %d" % gain)

    @Feat(units='us')
    @is_running
    def exposure_time(self):
        us = ct.c_uint()
        (self.lib.XC_GetIntegrationTime(self.xcamera_id,
                                                      ct.byref(us)))
        self.log_debug('Type us: {}'.format(type(us)))
        return us.value

    @exposure_time.setter
    @is_running
    def exposure_time(self, us):
        (self.lib.XC_SetIntegrationTime(self.xcamera_id,
                                                      int(us)))
        self.log_debug("Integration time set to %d us" % us)

    @Feat(units='K')
    @is_running
    def temperature(self):
        temp = ct.c_short()
        (self.lib.XC_GetTemperature(self.xcamera_id,
                                                  ct.byref(temp)))
        return temp.value

    @temperature.setter
    @is_running
    def temperature(self, k):
        (self.lib.XC_SetCoolingTemperature(self.xcamera_id,
                                                         k))
        self.log_debug("Temperature set to %d K" % k)

    @Feat(values={True: True, False: False})
    @is_running
    def fan(self):
        fan = ct.c_bool()
        (self.lib.XC_GetFan(self.xcamera_id,
                                          ct.byref(fan)))
        return fan.value

    @fan.setter
    @is_running
    def fan(self, k):
        """ Set fan on (True) """
        (self.lib.XC_SetFan(self.xcamera_id,
                                          k))
        self.log_debug("Switch fan %s" % ('on' if k else 'off'))

    @Action()
    @is_running
    def start_capture(self):
        self.lib.XC_StartCapture(self.xcamera_id)
        self.log_debug("Start capture")

    @Action()
    @is_running
    def stop_capture(self):
        self.lib.XC_StopCapture(self.xcamera_id)
        self.log_debug("Stop capture")

    def _read_frame(self):
        """Copy the buffer from the camera"""
        if self.readlock.acquire(blocking=False):
            # make sure that there is only one read 
            # at the time, else the driver will freeze
            (self.lib.XC_CopyFrameBuffer(self.xcamera_id,
                                                       self.buffer,
                                                       self.frame_size))
            self.readlock.release()
            self.log_debug("Captured 1 frame")
            return self.buffer
        else:
            time.sleep(0.001)
            self._read_frame()

    @Action()
    @is_running
    def single_frame(self):
        """ Read a single frame
        """
        time.sleep(self.exposure_time.to('s').magnitude + self.wait)
        time.sleep(0.0002)  # can be very short
        self._read_frame()
        return self.buffer

    @Action()
    @is_running
    def single_shot(self):
        """ Start and stop camera and capture one frame. Takes more than 20ms"""
        self.start_capture()
        self.single_frame()
        self.stop_capture()
        return self.buffer

    @Action(values={True: True, False: False})
    def shutdown(self, fan=False):
        """ Shutdown camera and put fan off

        :param fan: put fan off (False), or leave it on (True)"""
        self.fan = fan
        self.finalize()


if __name__ == '__main__':
    import argparse
    import lantz.log

    parser = argparse.ArgumentParser(description='Test Xcamera')
    parser.add_argument('-i', '--interactive', action='store_true',
                        default=False, help='Show interactive GUI')
    args = parser.parse_args()
    lantz.log.log_to_socket(lantz.log.DEBUG)
    with Xcamera() as cam:
        if args.interactive:
            from lantz.ui.app import start_test_app

            start_test_app(cam)
        else:
            import matplotlib.pyplot as plt

            image = cam.single_shot()

            print(np.min(image), np.max(image), np.mean(image))

            plt.imshow(image, cmap='gray')
            plt.show()
