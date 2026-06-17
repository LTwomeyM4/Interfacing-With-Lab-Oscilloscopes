# import sys
# import os
# import subprocess
# import html
# import webbrowser
# from datetime import datetime
# import time
# import json
# import struct
# import math
# import csv
# from datetime import datetime
# from pathlib import Path
# from PIL import Image
# import io
# import numpy
# import usb.core
# import usb.util

import time
import json
import io
import numpy
import usb.core
import usb.util
from PIL import Image
import matplotlib.pyplot as plt

class Scope_Control(object):
    
    def __init__(self):

        try:
            self.MOD_NAME_STR = "MP72_Lib"
            self.FUNC_NAME = ".Scope_Control()" # use this in exception handling messages
            self.ERR_STATEMENT = "Error: " + self.MOD_NAME_STR + self.FUNC_NAME

            self.VID = 0x5345
            self.PID = 0x1234

            self.EP_IN = 0x81
            self.EP_OUT = 0x03

            self.TIMEOUT_MS = 500

            self.VERTICAL_SCALES = [
                "5mV", "10mV", "20mV", "50mV",
                "100mV", "200mV", "500mV",
                "1V", "2V", "5V",
            ]

            self.TIMEBASES = [
                ("2.0ns", 2e-9), ("5.0ns", 5e-9), ("10ns", 10e-9),
                ("20ns", 20e-9), ("50ns", 50e-9),
                ("100ns", 100e-9), ("200ns", 200e-9), ("500ns", 500e-9),
                ("1.0us", 1e-6), ("2.0us", 2e-6), ("5.0us", 5e-6),
                ("10us", 10e-6), ("20us", 20e-6), ("50us", 50e-6),
                ("100us", 100e-6), ("200us", 200e-6), ("500us", 500e-6),
                ("1.0ms", 1e-3), ("2.0ms", 2e-3), ("5.0ms", 5e-3),
                ("10ms", 10e-3), ("20ms", 20e-3), ("50ms", 50e-3),
                ("100ms", 100e-3), ("200ms", 200e-3), ("500ms", 500e-3),
                ("1.0s", 1.0), ("2.0s", 2.0), ("5.0s", 5.0), ("10s", 10.0),
            ]

            self.dev = usb.core.find(idVendor=self.VID, idProduct=self.PID)
            if self.dev is None:
                raise RuntimeError("No Oscilloscope Found USB 5345:1234.")

            try:
                if self.dev.is_kernel_driver_active(0):
                    self.dev.detach_kernel_driver(0)
            except Exception:
                pass

            self.dev.set_configuration()
            #usb.util.claim_interface(self.dev, 0)

        except TypeError as e:
            print(self.ERR_STATEMENT)
            print(e)


    def __del__(self):
        if self.dev is not None:
            try:
                usb.util.release_interface(self.dev, 0)
            except Exception:
                pass
            usb.util.dispose_resources(self.dev)

    # Probably Unnecessary
    # def drain_input(self):
    #     while True:
    #         try:
    #             self.dev.read(self.EP_IN, 8192, timeout=1000)
    #         except Exception:
    #             break

    def write_cmd(self, cmd, delay=0.1):
        self.dev.write(self.EP_OUT, (cmd + "\n").encode("ascii"), timeout=self.TIMEOUT_MS)
        time.sleep(delay) #THIS LINE BROKE EVERYTHING, UNCOMMENT IF YOU WANT A COIN FLIPPER


    # def read(self, timeout=5000):
    #     data = bytearray()
    #     while True:
    #         try:
    #             chunk = self.dev.read(self.EP_IN, 8192, timeout=timeout)
    #             b = bytes(chunk)
    #             data.extend(b)
    #             # Optional: break if we have a full BMP (known size)
    #             if len(data) >= 4 and data[4:6] == b'BM':
    #                 bmp_size = int.from_bytes(data[6:10], 'little')  # file size from BMP header
    #                 expected_total = 4 + bmp_size
    #                 if len(data) >= expected_total:
    #                     break
    #         except usb.core.USBTimeoutError:
    #             # No more data – exit loop
    #             break
    #     return bytes(data)
    
    def read_response(self, max_total=50_000_000):
        data = bytearray()
        expected_total = None

        while True:
            try:
                chunk = self.dev.read(self.EP_IN, 8192, timeout=self.TIMEOUT_MS)
                b = bytes(chunk)
                data.extend(b)

                if data.endswith(b"->\n") or data.endswith(b"->"):
                    break

                # if len(data) >= 4 and expected_total is None:
                #     n = int.from_bytes(data[:4], "little")
                #     if 0 < n <= max_total:
                #         expected_total = n + 4

                # if expected_total is not None and len(data) >= expected_total:
                #     break

            except usb.core.USBTimeoutError:
                break

        return bytes(data)



    def parse_volts(self, s):
        s = s.strip()
        if s.endswith('mV'):
            return float(s[:-2]) / 1000.0
        elif s.endswith('V'):
            return float(s[:-1])
        else:
            return float(s)


    def parse_time_str(self, s):
        s = s.strip()
        units = {'ms': 1e-3, 'us': 1e-6, 'ns': 1e-9, 's': 1.0}
        for suffix, mult in units.items():
            if s.endswith(suffix):
                return float(s[:-len(suffix)]) * mult
        return float(s)



    # def Set_Scale(self, scale_text):
    #     s = str(scale_text).strip().lower()
    #     if s.endswith("mv"):
    #         return float(s[:-2]) * 1e-3
    #     if s.endswith("v"):
    #         return float(s[:-1])
    #     raise ValueError(f"Vertical scale not understood: {scale_text}")


    def Read_Waveform_plus_Metadata(self):

        V_array_1 = []
        V_array_2 = []
        fft_sig_1 = []
        fft_sig_2 = []
        # Commands to read head data and waverform
        self.write_cmd(":DATA:WAVE:SCREen:HEAD?")
        head = self.read_response()
        # The first 4 bytes are the JSON length
        json_len = int.from_bytes(head[:4], 'little')
        # The JSON string is from byte 4 to 4+json_len
        json_str = head[4:4+json_len].decode('utf-8')
        metadata = json.loads(json_str)
        
        if self.parse_time_str(metadata['TIMEBASE']['SCALE']) < 1e-6:
            Time_array = numpy.linspace(-380, 380, 760) * (15.2* self.parse_time_str(metadata['TIMEBASE']['SCALE']))/760
        else:
            Time_array = numpy.linspace(-760, 760, 1520) * (15.2* self.parse_time_str(metadata['TIMEBASE']['SCALE']))/1520

        if metadata['CHANNEL'][0]['DISPLAY'] == "ON" and metadata['CHANNEL'][1]['DISPLAY'] == "OFF":
            self.write_cmd(":DATA:WAVE:SCREen:CH1?")
            raw_1 = self.read_response()  
            offset_1 = metadata['CHANNEL'][0]['OFFSET'] * 0.001
            samples_1 = numpy.frombuffer(raw_1, dtype=numpy.int16)
            #First 2 values are bad
            voltage_1 = samples_1[2:] * float(metadata['CHANNEL'][0]['Current_Ratio']) * 0.0001 + offset_1
            filename = 'Waveform_Data_Scope_1.txt'
            numpy.savetxt(filename, voltage_1, fmt = '%0.4f', delimiter = '\t')
            V_array_1 = numpy.genfromtxt("Waveform_Data_Scope_1.txt")
            fft_sig_full = numpy.fft.fft(voltage_1)
            fft_sig_1 = numpy.abs(fft_sig_full)
            numpy.savetxt('FFT_Data_Scope.txt', fft_sig_1, fmt = '%0.4f', delimiter = '\t')

        elif metadata['CHANNEL'][1]['DISPLAY'] == "ON" and metadata['CHANNEL'][0]['DISPLAY'] == "OFF":
            self.write_cmd(":DATA:WAVE:SCREen:CH2?")
            raw_2 = self.read_response()  
            offset_2 = metadata['CHANNEL'][1]['OFFSET'] * 0.001
            samples_2 = numpy.frombuffer(raw_2, dtype=numpy.int16)
            #First 2 values are bad
            voltage_2 = samples_2[2:] * float(metadata['CHANNEL'][1]['Current_Ratio']) * 0.0001 + offset_2
            filename = 'Waveform_Data_Scope_2.txt'
            numpy.savetxt(filename, voltage_2, fmt = '%0.4f', delimiter = '\t')
            V_array_2 = numpy.genfromtxt("Waveform_Data_Scope_2.txt")
            fft_sig_full = numpy.fft.fft(voltage_2)
            fft_sig_2 = numpy.abs(fft_sig_full)
            numpy.savetxt('FFT_Data_Scope.txt', fft_sig_2, fmt = '%0.4f', delimiter = '\t')

        elif metadata['CHANNEL'][1]['DISPLAY'] == "ON" and metadata['CHANNEL'][0]['DISPLAY'] == "ON":
            self.write_cmd(":DATA:WAVE:SCREen:CH1?")
            raw_1 = self.read_response()  
            offset_1 = metadata['CHANNEL'][0]['OFFSET'] * 0.001
            samples_1 = numpy.frombuffer(raw_1, dtype=numpy.int16)
            #First 2 values are bad
            voltage_1 = samples_1[2:] * float(metadata['CHANNEL'][0]['Current_Ratio']) * 0.0001 + offset_1
            filename = 'Waveform_Data_Scope_1.txt'
            numpy.savetxt(filename, voltage_1, fmt = '%0.4f', delimiter = '\t')
            V_array_1 = numpy.genfromtxt("Waveform_Data_Scope_1.txt")
            fft_sig_full = numpy.fft.fft(voltage_1)
            fft_sig_1 = numpy.abs(fft_sig_full)
            numpy.savetxt('FFT_Data_Scope.txt', fft_sig_1, fmt = '%0.4f', delimiter = '\t')
            self.write_cmd(":DATA:WAVE:SCREen:CH2?")
            raw_2 = self.read_response()  
            offset_2 = metadata['CHANNEL'][1]['OFFSET'] * 0.001
            samples_2 = numpy.frombuffer(raw_2, dtype=numpy.int16)
            #First 2 values are bad
            voltage_2 = samples_2[2:] * float(metadata['CHANNEL'][1]['Current_Ratio']) * 0.0001 + offset_2
            filename = 'Waveform_Data_Scope_2.txt'
            numpy.savetxt(filename, voltage_2, fmt = '%0.4f', delimiter = '\t')
            V_array_2 = numpy.genfromtxt("Waveform_Data_Scope_2.txt")
            fft_sig_full = numpy.fft.fft(voltage_2)
            fft_sig_2 = numpy.abs(fft_sig_full)
            numpy.savetxt('FFT_Data_Scope.txt', fft_sig_2, fmt = '%0.4f', delimiter = '\t')
        else:
            pass

        

        data_list = [metadata, Time_array, V_array_1, V_array_2 ,fft_sig_1, fft_sig_2]

        return data_list
    
    def Read_plus_Plot(self):
        all_data = self.Read_Waveform_plus_Metadata()
        plt.style.use('dark_background')

        fig, ax1= plt.subplots()
        if len(all_data[2]) == 0:
            pass
        else:
            ax1.plot(all_data[1], all_data[2], color='y')
        ax1.grid(True)
        ax1.minorticks_on()
        ax1.set_ylabel("Voltage CH1 (V)")
        ax1.set_ylim(-4*self.parse_volts(all_data[0]['CHANNEL'][0]['SCALE']), 4*self.parse_volts(all_data[0]['CHANNEL'][0]['SCALE']))

        ax2 = ax1.twinx()
        if len(all_data[3]) == 0:
            pass
        else:
            ax2.plot(all_data[1], all_data[3], color='b')
        ax2.grid(True)
        ax2.minorticks_on()
        ax2.set_ylabel("Voltage CH2 (V)")
        ax2.set_ylim(-4*self.parse_volts(all_data[0]['CHANNEL'][1]['SCALE']), 4*self.parse_volts(all_data[0]['CHANNEL'][1]['SCALE']))

        plt.title("Oscilloscope CH1+CH2 Waveform")
        plt.xlabel("Time")
        plt.savefig("Scope_Screen.png")
        plt.show()

    def Read_plus_FFT(self, ch = 1):
        all_data = self.Read_Waveform_plus_Metadata()
        plt.style.use('dark_background')
        if ch == 1 or ch == 2:
            plt.plot(all_data[ch + 3])
        else:
            print("ERROR: Channel out of range, please chose channel 1 or 2")
            pass
        plt.title("Oscilloscope CH1 FFT")
        # plt.xlabel("Time")
        # plt.ylabel("Voltage (V)")
        # plt.ylim(-4*self.parse_volts(all_data[0]['CHANNEL'][0]['SCALE']), 4*self.parse_volts(all_data[0]['CHANNEL'][0]['SCALE']))
        plt.grid(True)
        plt.minorticks_on()
        plt.savefig("Scope_FFT.png")
        plt.show()




    def convert_bmp_to_image(self, raw_data):
        if not raw_data:
            return None
        
        if raw_data.endswith(b'->\n'):
            raw_data = raw_data[:-3]
        elif raw_data.endswith(b'->'):
            raw_data = raw_data[:-2]
        
        if len(raw_data) > 4 and raw_data[4:6] == b'BM':
            raw_data = raw_data[4:]
        
        if raw_data.startswith(b'#'):
            try:
                num_digits = int(chr(raw_data[1]))
                header_len = 2 + num_digits
                raw_data = raw_data[header_len:]
            except:
                pass
        
        try:
            image = Image.open(io.BytesIO(raw_data))
            return image
        except Exception as e:
            print(f"Cannot identify image: {e}")
            # Save for inspection
            with open("debug_raw.bin", "wb") as f:
                f.write(raw_data)
            return None
    

    def command_list(self):
        # Find Descriptions Here:
        # https://files.owon.com.cn/software/Application/XDS400_Series_Oscilloscopes_SCPI_Protocol.pdf
        help_list = [
            "Ocilloscope SCPI commands:",
            "\t-:ACQuire Command Subsystem:",
            "\t\t:ACQuire:MODE",
            "\t\t:ACQuire:AVERage:NUM <count>",
            "\t\t:ACQuire:DEPMEM <mdep>",

            "\t-:HORIzontal Command Subsystem:",
            "\t\t:HORIzontal:SCALe",
            "\t\t:HORIzontal:OFFset ",

            "\t-:CH Command Subsystem: ",
            "\t\t:CH<n>:DISPlay ",
            "\t\t:CH<n>:COUPling",
            "\t\t:CH<n>:PROBe",
            "\t\t:CH<n>:SCALe ",
            "\t\t:CH<n>:OFFSet ",
            "\t\t:CH<n>:INVErse",
            "\t\t:CH<n>:TERmination",

            "\t-:MEASUrement Command Subsystem:",
            "\t\t:MEASUrement:DISPlay",
            "\t\t:MEASUrement:CH<n>:<items>",
            "\t\t:MEASUrement:<items>? <cha>,<chb>",
            "\t\t:MEASUrement:CH<n>",
            "\t\t:MEASUrement:ALL ",

            "\t-:TRIGger Command Subsystem:",
            "\t\t:TRIGger:STATus?",
            "\t\t:TRIGger:TYPE <type>",
            "\t\t:TRIGger:SINGle",
            "\t\t:TRIGger:SINGle:SWEEp <mode> ",
            "\t\t:TRIGger:SINGle:HOLDoff",

            "\t-:Data Command Subsystem:",
            "\t\t:DATA:WAVE:SCREen:HEAD?",
            "\t\t:DATA:WAVE:SCREen:CH<x>? NOTE:MUST RUN HEAD BEFORE THIS" ,
            "\t\t:DATA:WAVE:SCREen:BMP? ",
            "\t\t:DATA:WAVE:DEPMem:All? ",

            "\t-Other Commands:",
            "\t\t:AUTOset ON",
            "\t\t:AUTOscale ",
            "\t\t:RUNning \n",

            "Arbitrary Function Generator SCPI commands:",
            "\t-:FUNCtion Command Subsystem: ",
            "\t\t:FUNCtion ",
            "\t\t:FUNCtion:FREQuency",
            "\t\t:FUNCtion:PERiod ",
            "\t\t:FUNCtion:PHASe ",
            "\t\t:FUNCtion:ALIGnphase ",
            "\t\t:FUNCtion:AMPLitude ",
            "\t\t:FUNCtion:OFFSet ",
            "\t\t:FUNCtion:HIGHt",
            "\t\t:FUNCtion:LOW",
            "\t\t:FUNCtion:RAMP:SYMMetry ",
            "\t\t:FUNCtion:PULSe:WIDTh",
            "\t\t:FUNCtion:PULSe:DTYCycle ",
            "\t\t:FUNCtion:ARB:BUILtinwform ",
            "\t\t:FUNCtion:ARB:FILE",

            "\t-:FILE Command Subsystem:",
            "\t\t:FILE:DOWNload",
            "\t\t:FILE:UPLoad",
            "\t\t:FILE:DELete",

            "\t-:CHANnel Command Subsystem:",
            "\t\t:CHANnel",
            "\t\t:CHANnel:CH1",
            "\t\t:CHANnel:CH2\n",
            "For Desciptions of each command:",
            "https://files.owon.com.cn/software/Application/XDS400_Series_Oscilloscopes_SCPI_Protocol.pdf"
        ]
        for i in help_list:
            print(i)

    def help(self):
        self.command_list()
