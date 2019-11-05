#!venv/bin/python3

from can_message import *


class canMessage_GPS(canMessage):
    def __init__(self):
        canMessage.__init__(self)

        self.log_filename = "0x500_GPS.csv"
        self.log_header = "Hour;Minutes;Day;Month;Year;Timestamp [ms]\n"

    def read_bytes(self, message):
        day = int(message[0], 16)
        month = int(message[1], 16)
        year = int(message[2], 16) + 2000

        hour = int(message[3], 16)
        minute = int(message[4], 16)

        return(';'.join(str(x) for x in (hour, minute, day, month, year)))
