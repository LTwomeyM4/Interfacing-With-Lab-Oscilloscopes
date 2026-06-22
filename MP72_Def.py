
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

            self.TIMEOUT_MS = 200

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


    def write_cmd(self, cmd, delay=0.1):
        self.dev.write(self.EP_OUT, (cmd + "\n").encode("ascii"), timeout=self.TIMEOUT_MS)
        time.sleep(delay)


    
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

                # Necessary for Windows
                if len(data) >= 4 and expected_total is None:
                    n = int.from_bytes(data[:4], "little")
                    if 0 < n <= max_total:
                        expected_total = n + 4

                if expected_total is not None and len(data) >= expected_total:
                    break

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
    
    def parse_sample_rate(self, s):
        s = s.strip()
        units = {'KS/s)': 1e3, 'MS/s)': 1e6, 'S/s)': 1.0}
        for suffix, mult in units.items():
            if s.endswith(suffix):
                return float(s[1:-len(suffix)]) * mult
        return float(s)
    
    def parse_frequency(self, s):
        s = s.strip()
        units = {b'MHz->': 1e6, b'kHz->': 1e3, b'Hz->': 1.0}
        for suffix, mult in units.items():
            if s.endswith(suffix):
                return float(s[3:-len(suffix)]) * mult
        return float(s)
    
    def parse_oscilloscope_data(self, raw_bytes: bytes) -> list[dict]:
        """Extract JSON from binary payload and return list of measurement entries."""
        # Find the start of the JSON part (the first '{')
        json_start = raw_bytes.find(b'{')
        if json_start == -1:
            raise ValueError("No JSON data found in payload")
        
        json_bytes = raw_bytes[json_start:]
        data = json.loads(json_bytes)
        
        entries = []
        for channel, measurements in data.items():
            for meas, raw_value in measurements.items():
                # Split into value string and status (ON/OFF)
                if ',' in raw_value:
                    value_part, status = raw_value.rsplit(',', 1)
                else:
                    value_part, status = raw_value, ''
                
                entries.append({
                    "Channel": channel,
                    "Measurement": meas,
                    "Value": value_part.strip(),
                    "Status": status.strip()
                })
        return entries
    
    def Measurements_Data(self):
        self.write_cmd(":MEASUrement:ALL?")
        mments = self.read_response()
        measurements = self.parse_oscilloscope_data(mments)
        # Nicely formatted display
        print(f"{'Channel':<8} {'Measurement':<18} {'Value':<16} {'Status':<6}")
        print("-" * 50)
        for m in measurements:
            print(f"{m['Channel']:<8} {m['Measurement']:<18} {m['Value']:<16} {m['Status']:<6}")
        return measurements

    def Read_Waveform_plus_Metadata(self):

        # Creating empty arrays
        V_array_1 = []
        V_array_2 = []
        fft_sig_1 = []
        fft_sig_2 = []

        # Commands to read head data
        self.write_cmd(':DATA:WAVE:SCREen:HEAD?')
        head = self.read_response()
        
        # Converting Head Data into as JSON string so values can be read from it
        # for plot calibration purposes.
        json_len = int.from_bytes(head[:4], 'little')
        json_str = head[4:4+json_len].decode('utf-8')
        metadata = json.loads(json_str)
        

        # Checking for which channel is on, doesnt waste time recording data of channels not being displayed
        if metadata['CHANNEL'][0]['DISPLAY'] == 'ON' \
           and metadata['CHANNEL'][1]['DISPLAY'] == 'OFF':
            self.write_cmd(':DATA:WAVE:SCREen:CH1?')
            raw_1 = self.read_response()  
            offset_1 = (metadata['CHANNEL'][0]['OFFSET'] 
                        * 0.001)
            samples_1 = numpy.frombuffer(raw_1, dtype=numpy.int16)
            #First 2 values are bad. Using metadata here to convert ADC screen data to voltage values
            voltage_1 = (samples_1[2:] 
                        * float(metadata['CHANNEL'][0]['Current_Ratio'])
                        * 0.0001 
                        + offset_1)
            t_size = numpy.size(voltage_1)
            Time_array = (numpy.linspace(-t_size/2, t_size/2, t_size) 
                          * (15.2
                             * self.parse_time_str(metadata['TIMEBASE']['SCALE']))
                          / t_size)

            # Saving to txt file
            filename = 'Waveform_Data_Scope_1.txt'
            numpy.savetxt(filename, voltage_1, fmt = '%0.4f', delimiter = '\t')
            V_array_1 = numpy.genfromtxt("Waveform_Data_Scope_1.txt")

        elif metadata['CHANNEL'][1]['DISPLAY'] == "ON" \
             and metadata['CHANNEL'][0]['DISPLAY'] == "OFF":
            self.write_cmd(":DATA:WAVE:SCREen:CH2?")
            raw_2 = self.read_response()  
            offset_2 = (metadata['CHANNEL'][1]['OFFSET'] 
                        * 0.001)
            samples_2 = numpy.frombuffer(raw_2, dtype=numpy.int16)
            #First 2 values are bad. Using metadata here to convert ADC screen data to voltage values
            voltage_2 = (samples_2[2:] 
                         * float(metadata['CHANNEL'][1]['Current_Ratio']) 
                         * 0.0001 
                         + offset_2)
            t_size = numpy.size(voltage_2)
            Time_array = (numpy.linspace(-t_size/2, t_size/2, t_size) 
                          * (15.2
                             * self.parse_time_str(metadata['TIMEBASE']['SCALE']))
                          / t_size)
            
            # Saving to txt file
            filename = 'Waveform_Data_Scope_2.txt'
            numpy.savetxt(filename, voltage_2, fmt = '%0.4f', delimiter = '\t')
            V_array_2 = numpy.genfromtxt("Waveform_Data_Scope_2.txt")

        elif metadata['CHANNEL'][1]['DISPLAY'] == "ON" \
             and metadata['CHANNEL'][0]['DISPLAY'] == "ON":
            self.write_cmd(":DATA:WAVE:SCREen:CH1?")
            raw_1 = self.read_response()  
            self.write_cmd(":DATA:WAVE:SCREen:CH2?")
            raw_2 = self.read_response()  
            offset_1 = (metadata['CHANNEL'][0]['OFFSET'] 
                        * 0.001)
            offset_2 = (metadata['CHANNEL'][1]['OFFSET'] 
                        * 0.001)
            
            samples_1 = numpy.frombuffer(raw_1, dtype=numpy.int16)
            samples_2 = numpy.frombuffer(raw_2, dtype=numpy.int16)
            #First 2 values are bad. Using metadata here to convert ADC screen data to voltage values
            voltage_1 = (samples_1[2:] 
                         * float(metadata['CHANNEL'][0]['Current_Ratio']) 
                         * 0.0001 
                         + offset_1)
            voltage_2 = (samples_2[2:] 
                         * float(metadata['CHANNEL'][1]['Current_Ratio']) 
                         * 0.0001 
                         + offset_2)
            t_size = numpy.size(voltage_1)
            Time_array = (numpy.linspace(-t_size/2, t_size/2, t_size) 
                          * (15.2
                             * self.parse_time_str(metadata['TIMEBASE']['SCALE']))
                          / t_size)

            # Saving to txt files
            filename = 'Waveform_Data_Scope_1.txt'
            numpy.savetxt(filename, voltage_1, fmt = '%0.4f', delimiter = '\t')
            V_array_1 = numpy.genfromtxt("Waveform_Data_Scope_1.txt")
            filename = 'Waveform_Data_Scope_2.txt'
            numpy.savetxt(filename, voltage_2, fmt = '%0.4f', delimiter = '\t')
            V_array_2 = numpy.genfromtxt("Waveform_Data_Scope_2.txt")

        else:
            pass

        
        # Creating data list
        data_list = [metadata, Time_array, V_array_1, V_array_2 ,fft_sig_1, fft_sig_2]

        return data_list
    
    def Read_plus_Plot(self):
        all_data = self.Read_Waveform_plus_Metadata()
        plt.style.use('dark_background')

        fig, ax1= plt.subplots()
        # Skips plot if voltage array is empty (channel 1 not displayed)
        if len(all_data[2]) == 0:
            pass
        else:
            ax1.plot(all_data[1], all_data[2], color='y')
        ax1.grid(True)
        ax1.minorticks_on()
        ax1.set_ylabel("Voltage CH1 (V)")
        # Sets Vertical scale to match oscilloscope's scale
        ax1.set_ylim(-4*self.parse_volts(all_data[0]['CHANNEL'][0]['SCALE']),
                      4*self.parse_volts(all_data[0]['CHANNEL'][0]['SCALE']))

        # Plot secoind channel on same plot with seperate axis
        ax2 = ax1.twinx()
        # Skips plot if voltage array is empty (channel 2 not displayed)
        if len(all_data[3]) == 0:
            pass
        else:
            ax2.plot(all_data[1], all_data[3], color='b')
        ax2.grid(True)
        ax2.minorticks_on()
        ax2.set_ylabel("Voltage CH2 (V)")
        ax2.set_ylim(-4*self.parse_volts(all_data[0]['CHANNEL'][1]['SCALE']), 
                      4*self.parse_volts(all_data[0]['CHANNEL'][1]['SCALE']))

        plt.title("Oscilloscope Screen Waveforms")
        plt.xlabel("Time")
        plt.savefig("Scope_Screen.png")
        plt.show()

    def Read_plus_FFT(self, ch = 1, xlim = None):
        '''
        Get signal frequency for FFT improvement.
            - If the full screen is captured, the FFT becomes noisier as there is a non-integer
              number of periods on screen. With the frequency known, a check is done to find the 
              fractional number of periods which fit on screen. This number is then rounded down 
              (numpy.floor()), and the voltage array is reduced in size such that the number of 
              periods on screen is close to an integer number. Try commenting out the 
              lines 335 and 336 to see the difference. (following 2 lines)
        '''
        self.write_cmd(":MEASUrement:CH1:FREQuency?")
        fr = self.read_response()
        all_data = self.Read_Waveform_plus_Metadata()
        plt.style.use('dark_background')
        # Check for known frequency. If timescale is too high or low,
        # then 'self.write_cmd(":MEASUrement:CH1:FREQuency?")' returns '?'
        try:
            fr = self.parse_frequency(fr)
            if xlim == None:
                xlim = (fr * 10)
            
            P = 1/fr
            P_f = numpy.floor((self.parse_time_str(all_data[0]['TIMEBASE']['SCALE'])
                            *(15.2))/P)
            P_t = (self.parse_time_str(all_data[0]['TIMEBASE']['SCALE'])
                *(15.2))/P

            new_size = ((P_f 
                         * numpy.size(all_data[1])) 
                        / P_t)
            all_data[1][:int(new_size)]
            # Check for amount of periods on screen. Accuracy is reduced if 
            # there are to many or too few
            if P_t >= 1 and P_t <= 40:
                pass
            else:
                print("WARNING: Too many or too few periods on screen. " \
                      "Reccomend adjusting Time Scale for increased accuracy.")
        except:
            print("WARNING:Frequency unkown. Accuracy of FFT will be Reduced. " \
                  "Reccomend Adjusting Time Scale/ Voltage Scale")
            new_size = numpy.size(all_data[1])


       
    
        if ch == 1 or ch == 2:
            # try:
            N = numpy.size(all_data[1][:int(new_size)])
            '''
            This breaks below 5us. Must figure out why and change. 
            Something to do with sample rate not updating on screen?
            '''
            Sr = 0
            if self.parse_time_str(all_data[0]['TIMEBASE']['SCALE']) >= 4.9e-6:
                Sr = self.parse_sample_rate(all_data[0]["SAMPLE"]["SAMPLERATE"])
            elif self.parse_time_str(all_data[0]['TIMEBASE']['SCALE']) >= 1.9e-6:
                Sr = (5/2)*self.parse_sample_rate(all_data[0]["SAMPLE"]["SAMPLERATE"])
            elif self.parse_time_str(all_data[0]['TIMEBASE']['SCALE']) >= 0.9e-6:
                Sr = 2*(5/2)*self.parse_sample_rate(all_data[0]["SAMPLE"]["SAMPLERATE"])
            elif self.parse_time_str(all_data[0]['TIMEBASE']['SCALE']) >= 4.9e-7:
                Sr = (2**1)*(5/2)*self.parse_sample_rate(all_data[0]["SAMPLE"]["SAMPLERATE"])
            elif self.parse_time_str(all_data[0]['TIMEBASE']['SCALE']) >= 1.9e-7:
                Sr = (2**2)*((5/2)**2)*self.parse_sample_rate(all_data[0]["SAMPLE"]["SAMPLERATE"])
            elif self.parse_time_str(all_data[0]['TIMEBASE']['SCALE']) >= 0.9e-7:
                Sr = (2**2)*((5/2)**2)*self.parse_sample_rate(all_data[0]["SAMPLE"]["SAMPLERATE"])
            elif self.parse_time_str(all_data[0]['TIMEBASE']['SCALE']) >= 4.9e-8:
                Sr = (2**3)*((5/2)**2)*self.parse_sample_rate(all_data[0]["SAMPLE"]["SAMPLERATE"])
            elif self.parse_time_str(all_data[0]['TIMEBASE']['SCALE']) >= 1.9e-8:
                Sr = (2**3)*((5/2)**3)*self.parse_sample_rate(all_data[0]["SAMPLE"]["SAMPLERATE"])
            elif self.parse_time_str(all_data[0]['TIMEBASE']['SCALE']) >= 0.9e-8:
                Sr = (2**4)*((5/2)**3)*self.parse_sample_rate(all_data[0]["SAMPLE"]["SAMPLERATE"])
            elif self.parse_time_str(all_data[0]['TIMEBASE']['SCALE']) >= 4.9e-9:
                Sr = (2**5)*((5/2)**3)*self.parse_sample_rate(all_data[0]["SAMPLE"]["SAMPLERATE"])
            else:
                print("ERROR: Sample rate not recognised")

            freqs = numpy.fft.fftfreq(N, 1/(Sr))
            fft = numpy.fft.fft(all_data[ch + 1][:int(new_size)])
            positive_freqs = freqs[:N//2]
            positive_mag = numpy.abs(fft)[:N//2]

            # Plots only positive side of FFT
            positive_freqs = freqs[:N//2]/5
            positive_mag = numpy.abs(fft)[:N//2]
            plt.stem(positive_freqs, positive_mag, basefmt=" ", markerfmt=" ")
            plt.title(f"FFT of Signal on Channel {ch}")
            plt.xlabel("Frequency (Hz)")
            plt.xlim(0, xlim)
            plt.grid(True)
            plt.minorticks_on()
            plt.savefig("Scope_FFT.png")
            plt.show()
        
            numpy.savetxt('FFT_Data_Scope.txt', fft, fmt = '%0.4f', delimiter = '\t')
            fft_data = [fft, positive_freqs,  positive_mag]
            return fft_data
            # except:
            #     print(f"ERROR: Chosen Channel not displayed on Oscilloscope. Please turn on Channel {ch} display")
        else:
            print("ERROR: Channel out of range, please chose channel 1 or 2")
            pass



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
            "\t\t:FUNCtion:ALIGnumpyhase ",
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
