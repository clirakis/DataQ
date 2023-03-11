"""
Python3 code for interfacing to DATAQ DI-2008 DAQ module. 


Modified    BY    Reason
--------    --    ------
11-Mar-23   CBL   Thermocoulple only 

"""
from DI2008 import DI2008
import time

def main():
    print ('Program starts')
    DATAQ = DI2008()
    
    print("")
    print("Ready to acquire...")
    print ("")
    print("Press <g> to go, <s> to stop, <r> resets counter channel, and <q> to quit:")
    
    # This is the slist position pointer. Ranges from 0 (first position)
    # to len(slist)
    slist_pointer = 0
    # This is the constructed output string
    output_string = ""
    count = 0
    
    while True:
        data = input('command?  ')
        res = data.upper()
        # If key 'SPACE' start scanning
        if (res == 'G'):
            DATAQ.Start();
            while (count < 10):
                DATAQ.Do()
                count = count + 1
                time.sleep(0.1)
            DATAQ.Stop()
            
        elif (res == 'I'):
            DATAQ.Info()

        elif (res == 'Q'):
            DATAQ.Stop()
            break
        # If key 'r' reset counter 
        elif (res == 'R'):
            DATAQ.ResetCounter()
        elif (res == 'T'):
            DATAQ.TestDIO()

if __name__ == '__main__':
    main()
