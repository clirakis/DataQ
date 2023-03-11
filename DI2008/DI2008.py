"""
Python3 code for interfacing to DATAQ DI-2008 DAQ module. The code pulled
from the repository isn't quite working.

Modified    BY    Reason
--------    --    ------
10-Mar-23   CBL   Original
11-Mar-23   CBL   Thermocoulple only, class only

References:
    -- https://raw.githubusercontent.com/dataq-instruments/Python/master/binary_comm/DI-2008/di_2008.py
    -- https://www.dataq.com/resources/pdfs/misc/di-2008%20protocol.pdf
    -- https://www.dataq.com/products/di-2008/#software

Python modules used:
    -- https://pyserial.readthedocs.io/en/latest/pyserial.html

Notes from original code:
This program works only with model:
DI-2008

Any other instrument model should be disconnected from the PC
to prevent the program from detecting a device with a DATAQ
Instruments VID and attempting to use it. 
Such attempts will fail.


If you don't see a device in /dev/ttyACM0 then hold the button as you plug
in the device. Documention here:
https://www.dataq.com/blog/data-acquisition/usb-daq-products-support-libusb-cdc/

"""
import serial
import serial.tools.list_ports
import time
import logging
from   datetime import datetime

"""
Change slist tuple to vary analog channel configuration.
Refer to the protocol for details.
Bit pattern for channels 0-7:
    15:13 - Unused
    12    - mode
    11    - Range
    10:8  - Full scale value or TC type.
    7:4   - unused
    3:0   - channel

                 Range=0   Range=1
    10   9   8   Mode =0   Mode =0     Mode 1
     0   0   0   +/- 500mV +/-50V      B Thermocouple
     0   0   1   +/- 250mV +/-25V      E Thermocouple
     0   1   0   +/- 100mV +/-10V      J Thermocouple
     0   1   1   +/-  50mV +/- 5V      K Thermocouple
     1   0   0   +/-  25mV +/-2.5V     N Thermocouple
     1   0   1   +/-  10mV +/-1V       R Thermocouple
     1   1   0   N/A       N/A         S Thermocopule
     1   1   1   N/A       N/A         T Thermocouple  - copper/Constantan


Channel
0-7 Analog inputs
8 Digital in
9 Rate (DI2)
10 Count (DI3)

slist Tuple example interpretation (from protocol)
    0x0A00 = Channel 0 +/- 10V range
    0x0B01 = Channel 1 +/-  5V range
    0x1702 = Channel 2, T - type thermocouple
    0x1303 = Channel 3, K - type thermocouple
    0x0709 = Rate channel, 500Hz range
    0x000A = Count channel
    0x0008 = Digital inputs

    Form ours for all 7 channels as T input Thermocouples
"""
slist = [0x1700,0x1701,0x1702,0x1703,0x1704,0x1705,0x1706, 0x1707]

"""
Define analog_ranges tuple to contain an ordered list of analog measurement
ranges supported by the DI-2008. This tuple begins with gain code
0 (+/- 500 mV) and ends gain code 0xD (+/- 1 V) and is padded with 0 values
as place holders for undefined codes (see protocol.)
"""
analog_ranges = [.5, 0.25, 0.1, .05, .025, .01, 0, 0, 50 ,25, 10, 5, 2.5, 1, 0, 0]


"""
Define a tuple that contains an ordered list of rate measurement
ranges supported by the DI-2008. The first item in the list is the
lowest gain code (e.g. 50 kHz range = gain code 1).
"""
rate_ranges = tuple((50000,20000,10000,5000,2000,1000,500,200,100,50,20,10))


"""
m and b TC scaling constants in TC type order: B, E, J, K, N, R, S, T
See protocol, these are fixed. 
"""
tc_m = [0.023956,0.018311,0.021515,0.023987,0.022888,0.02774,0.02774,0.009155]
tc_b = [1035,400,495,586,550,859,859,100]

"""
Define a list of analog voltage and rate ranges to apply in slist order.
Value 0 is appended as a placeholder for enabled TC and dig-in channels. 
This list is populated in the config_scn_lst() routine based upon 
slist contents.
"""
range_table = list(())

class DI2008:
    def __init__(self):
        
        # initially no errors. 
        self.error         = False
        # Define flag to indicate if acquiring is active 
        self.acquiring     = False
        self.slist_pointer = 0
        self.output_string = ""   # initial string is null.
        self.Decimation    = 1
        self.Verbose       = True
        
        # setup logging to file. Filename is based on date/time
        now = datetime.now()
        fname = now.strftime('%Y%m%d_%H%M%S.log')
        logging.basicConfig(filename=fname,encoding='utf-8',
                            level=logging.DEBUG)

        toLog = 'Program starts on: ' + now.strftime('%Y-%m-%d %H:%M:%S')
        logging.info(toLog)
        

        # Open the serial method
        if (self.discovery()):
            self.Setup()
        else:
            self.error = True

    def discovery(self):
        """
        Discover DATAQ Instruments devices and models.
        Note that if multiple devices are connected, only the 
        device discovered first is used. We leave it to you to
        ensure that the device is a model DI-2008

        Tested 10-Mar-23, operational
        """
        # Get a list of active com ports to scan for possible DATAQ
        # Instruments devices
        available_ports = list(serial.tools.list_ports.comports())
    
        # Will eventually hold the com port of the detected device, if any
        hooked_port = "" 
        for p in available_ports:
            toLog = 'available ports: ' + str(p)
            logging.info(toLog)
            # Do we have a DATAQ Instruments device?
            if ("VID:PID=0683" in p.hwid):
                # Yes!  Dectect and assign the hooked com port
                hooked_port = p.device
                break

        if hooked_port:
            logging.info("Found a DATAQ Instruments device on" + hooked_port)
            self.ser = serial.Serial()
            self.ser.timeout  = 0
            self.ser.port     = hooked_port
            # initial baudrate
            self.ser.baudrate = '115200'
            self.ser.open()
            self.error = False
            return (True)
        else:
            # Get here if no DATAQ Instruments devices are detected
            print("Please connect a DATAQ Instruments device")
            self.error = True
            return (False)

    def send_cmd(self, command):
        """
        Sends a passed command string after appending <cr>
        @param command - ASCII command to send. 
        """
        if (self.Verbose):
            print('send_cmd: ', command)
            
        s = ""
        self.ser.write((command+'\r').encode())
        time.sleep(.1)
        if not(self.acquiring):
            # Echo commands if not acquiring
            while True:
                # wait for something to be available in the serial port
                if(self.ser.inWaiting() > 0):
                    while True:
                        try:
                            rl = self.ser.readline()
                            # Put the string into something we
                            # can understand. 
                            s = rl.decode()
                            s = s.strip('\n')
                            s = s.strip('\r')
                            s = s.strip(chr(0))
                            break
                        except:
                            continue
                # if the data is non-null then print out the result. 
                if (s != "") and (self.Verbose):
                    print ('send_cmd result: ', s)
                    return s
                    break

    def Read(self):
        """
        Read any waiting bytes, assuming character values
        back and report to user.
        Not sure this method makes sense. 
        """
        s = ""
        while True:
            # wait for something to be available in the serial port
            if(self.ser.inWaiting() > 0):
                while True:
                    try:
                        rl = self.ser.readline()
                        # Put the string into something we
                        # can understand. 
                        s = rl.decode()
                        s = s.strip('\n')
                        s = s.strip('\r')
                        s = s.strip(chr(0))
                        break
                    except:
                        continue
                    # if the data is non-null then print out the result. 
                    if s != "":
                        print ('read result: ', s)
                        break
        return s

    def config_scn_lst(self):
        """
        Configure the instrment's scan list based on the values
        in slist. 
        """
        # Scan list position must start with 0 and increment sequentially
        position = 0 
        for item in slist:
            self.send_cmd("slist "+ str(position ) + " " + str(item))
            position += 1
            
            # Update the Range table - 
            if (item & 0xf < 8) and (item & 0x1000 == 0):
                # This is a voltage channel.
                range_table.append(analog_ranges[item >> 8])

            elif (item & 0xf < 8) and (item & 0x1000 != 0):
                # This is a TC channel. Append 0 as a placeholder
                range_table.append(0)

            elif item & 0xf == 8:
                # This is a dig in channel. No measurement range support. 
                # Append 0 as a placeholder
                range_table.append(0) 

            elif item & 0xf == 9:
                """
                This is a rate channel
                Rate ranges begin with 1, so subtract 1 to 
                maintain zero-based index in the rate_ranges tuple
                """
                range_table.append(rate_ranges[(item >> 8)-1]) 

            else:
                """
                This is a count channel. No measurement range support. 
                Append 0 as a placeholder
                """
                range_table.append(0)

    def SampleRate(self, Hz):
        """
        Allow the user to specify the sample rate in Hz and
        figure out the scan rate based on number channels and
        decimation rate. 
        """
        SR = 800.0/(self.Hz * self.Decimation)

    def ScanRate(self,ScanRate):
        """
        Setup the overall sample rate based on decimation and srate values.
        The equation for sample rate is
        Sample Rate (Hz) = 800/(Decimation * Scan Rate)
        @param ScanRate   {4:2232}

        Note that this is the rate that the device steps through all channels.
        The actual data rate is divided by the number of channels. 
        """
        toSend = 'srate ' + str(ScanRate)
        self.send_cmd(toSend)  # scanning rate 
        
    def Filter(self, channel, val):
        """
        @channel {0:7} Which channel to apply filter to. if > 7 then set all.
        @param val
            0 - last point
            1 - Average
            2 - Maximum
            3 - Minimum
        """
        # clip the limits on the inputs. 
        if (val>3):
            val = 0
        if (channel>7):
            # set them all to a single value. 
            toSend = 'filter * ' + str(val)
        else:
            toSend = 'filter ' + str(channel) + ' ' + str(val)
        self.send_cmd(toSend)

    def MovingAverage(self, value):
        """
        set the window for the moving average.
        If the rate channel is enable in the instruments scan list using
        the slist command, a moving average may be applied.

        @param value - the moving average window (number of samples) {1:64}
        """
        if (value<1):
            value = 1
        if (value>64):
            value=64
        toSend = 'ffl ' + str(value)
        self.send_cmd(toSend)

        
    def SetDecimation(self, value):
        """
        Set the decimation factor based on the filter setting.
        @param value {1:32767}
        """
        if (value < 1):
            value = 1
        if (value>32767):
            value = 32767
        self.Decimation = value
        toSend = 'dec ' + str(value)
        self.send_cmd(toSend)

    def Setup(self):
        # Stop in case Device was left running
        self.Stop()
        # Keep the packet size small for responsiveness
        self.send_cmd("ps 0")
        # Configure the instrument's scan list
        self.config_scn_lst()
        self.Filter(8,1)        # set all channels to current value, 
        self.SetDecimation(20)  # filter for 20 samples
        self.ScanRate(4)        # set scan rate to 40 

    def Start(self):
        """
        Start the scan based on the setup parameters in  config_scn_lst
        This command does not echo and once issued, no subsequent command
        echos. 
        """
        print('START')
        self.acquiring = True
        self.send_cmd("start")

    def Stop(self):
        """
        Stop and flush the serial buffer.
        After this point, echos continue. 
        """
        self.send_cmd("stop")
        self.acquiring = False
        time.sleep(1)
        print ("")
        print ("Stopped")
        self.ser.flushInput()

    def ResetCounter(self):
        logging.info('# Reset Counter')
        self.send_cmd("reset 1")

    def Info(self):
        """
        Tell me about the connected device.
        """
        self.send_cmd('info 1')   # part id
        self.send_cmd('info 2')   # firmware version
        self.send_cmd('info 6')   # serial number
        self.send_cmd('info 9')   # sample rate divisor

    def DIO_Configure(self, value):
        """
        7 bits of I/O
        0 - means input
        1 - means output
        eg: a value of 127 means all outputs. a value of 0 means all inputs
        @param value - decimal number representing the bit string of how
                       to configure the input/output 
        """
        command = 'endo ' + str(value)
        self.send_cmd(command)

    def DIO_Out(self, value):
        """
        @param value - decimal encoded bit value to output on the DIO
        register. 
        """
        command = 'dout ' + str(value)

    def DIO_In(self):
        """
        Get the input from the digital inputs.
        read all inputs. Can specify value for read in.
        """
        instr = self.send_cmd('din')
        # need to separate out the command string from the value
        # has a space in it.
        values = instr.split(" ")
        return int(values[1])
    
    def Do(self):
        """
        Get some data
        """
        print(' DO ')
        print('nbytes: ', self.ser.inWaiting())
 
        while (self.ser.inWaiting() > (2 * len(slist))):
            for i in range(len(slist)):
                # The four LSBs of slist determine measurement function
                function = slist[self.slist_pointer] & 0xf
                mode_bit = slist[self.slist_pointer] & 0x1000
                # Always two bytes per sample...read them
                bytes = self.ser.read(2)
                
                if (function < 8) and (not(mode_bit)):
                    # Working with a Voltage input channel. Scale accordingly.
                    result = range_table[self.slist_pointer] * int.from_bytes(bytes,byteorder='little', signed=True) / 32768
                    self.output_string = self.output_string + "{: 3.3f}, ".format(result)
                elif (function < 8) and (mode_bit):
                    """
                    Working with a TC channel.
                    Convert to temperature if no errors.
                    First, test for TC error conditions.
                    """
                    result = int.from_bytes(bytes,byteorder='little', signed=True)
                    if result == 32767:
                        self.output_string = self.output_string + "cjc error, "
                        
                    elif result == -32768:
                        self.output_string = self.output_string + "open, "
                        
                    else:
                        # Get here if no errors, so isolate TC type
                        tc_type = slist[self.slist_pointer] & 0x0700
                        # Move TC type into 3 LSBs to form an index we'll use to select m & b scaling constants
                        tc_type = tc_type >> 8
                        result = tc_m[tc_type] * result + tc_b[tc_type]
                        self.output_string = self.output_string + "{: 3.3f}, ".format(result)

                elif function == 8:
                    # Working with the Digital input channel 
                    result = (int.from_bytes(bytes,byteorder='big', signed=False)) & (0x007f)
                    self.output_string = self.output_string + "{: 3d}, ".format(result)

                elif function == 9:
                    # Working with the Rate input channel
                    result = (int.from_bytes(bytes,byteorder='little', signed=True) + 32768) / 65535 * (range_table[self.slist_pointer])
                    self.output_string = self.output_string + "{: 3.1f}, ".format(result)

                else:
                    # Working with the Counter input channel
                    result = (int.from_bytes(bytes,byteorder='little', signed=True)) + 32768
                    self.output_string = self.output_string + "{: 1d}, ".format(result)

                # Get the next position in slist
                self.slist_pointer += 1

            print(' slist_pointer:', self.slist_pointer)
            print(' output = ', self.output_string)
            
            if (self.slist_pointer + 1) > (len(slist)):
                # End of a pass through slist items...output, reset, continue
                print(self.output_string.rstrip(", ") + "             ", end="\r") 
                self.output_string = ""
                self.slist_pointer = 0
        time.sleep(1)
        print('DO done.')
        
    def TestDIO(self):
        """
        Setup and test the DIO stream. 
        """
        self.DIO_Configure(0) # all input
        print('Result: ', self.DIO_In())

