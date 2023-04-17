"""
Logger.py

Modified  By  Reason
--------  --  ------
11-Mar-23 CBL Original - log data in CSV format

"""
from   datetime import datetime
import logging

class MyLogger:
    def __init__(self, fn):
        now = datetime.now()
        fname = now.strftime('%Y%m%d_%H%M%S') + fn + '.csv'
        logging.info('Logging data to: ' + fname)
        self.fd = open(fname, 'w')

    def __del__(self):
        self.fd.close()

    def LogData(self, values):
        """
        @strtime - time of acqusition
        @param values - an array defining the columns of data
        """
        toLog = ''
        count = 0
        for col in values:
            toLog = toLog + str(col)
            count = count + 1
            if(count<len(values)):
                toLog = toLog + ','
        self.fd.write(toLog + '\n')
        
    def LogDataTime(self, strtime, values):
        """
        @strtime - time of acqusition
        @param values - an array defining the columns of data
        """
        toLog = strtime
        for col in values:
            toLog = toLog + ',' + "{: 3.3f}".format(col)
        self.fd.write(toLog + '\n')
