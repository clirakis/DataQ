"""
Python3 code for interfacing to DATAQ DI-2008 DAQ module. 


Modified    BY    Reason
--------    --    ------
11-Mar-23   CBL   Thermocoulple only 

"""
from DI2008   import DI2008
from Logger import MyLogger
import time

def main():
    print ('Program starts')
    tslist = [0x1700,0x1701,0x1702,0x1703,0x1704,0x1705,0x1706, 0x1707]

    DATAQ = DI2008(tslist)         # open class, pass in slist parameters.
    # set a data rate of 1 per 15 seconds
    DATAQ.SampleRate(15)
    DATAQ.TotalSamplesTime(3600) # one hour
    
    log   = MyLogger("TC")
    log.LogData(['DATIME','TC0','TC1','TC2','TC3','TC4','TC5','TC6','TC7'])
    print(" Log thermocouple data.")
    
    count = 0
    DATAQ.Start();
    while (count < DATAQ.NSamples()):
        n,strtime, x = DATAQ.Do()
        if (n>0):
            count = count + 1
            #print(count, ' ', strtime, ' values: ', x)
            log.LogDataTime(strtime,x)
        time.sleep(0.1)
    DATAQ.Stop()
            

if __name__ == '__main__':
    main()
