import collections
import serial

# BK Precision Example Code is hosted here:
# https://github.com/BKPrecisionCorp/9129B/blob/master/python3/simple.py
# It's not much.
#
# Programming Manual is available here:
# https://bkpmedia.s3.amazonaws.com/downloads/programming_manuals/en-us/9129B_programming_manual.pdf
#
# Note:
#
# TODO: Everything
# 

class BK_9129B:
	pass

if __name__ == "__main__":
	ser = serial.Serial('COM6')  # open serial port
	ser.write("SYST:REM\n".encode())
	ser.write("OUTP:STAT 0\n".encode()) # Turns all Outputs on
	ser.write("SYST:VERS?\n".encode())
	print(ser.readline())