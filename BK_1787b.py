import collections
import serial
import time

# BK Precision Reference Implementation is hosted here:
# https://github.com/BKPrecisionCorp/1785B_Series/blob/master/Python%203/lib1785b.py
# Consider using it, before using this library
#
# Programming Manual is available here:
# http://ridl.cfd.rit.edu/products/manuals/BK%20Precision/1786B_manual.pdf
#
# Note:
# Supply reporting of current is not accurate to display readout
#
# TODO: Calibration commands...
# LOL NO. If you know what they are for, this implementation should be
# sufficient to implement them yourself. I have no desire for that functionality.

class BK_1787b:
	# Serial Port
	_serial = None # Serial Object
	# Serial Configuration
	_BAUD_RATE = [4800, 9600, 19200, 38400] # 9600 Baud is default
	_DEFAULT_ADDRESS = 0
	_PARITY = None
	_STOP_BIT = 1
	# Command table
	_COMMANDS = {
		'CalMode'      : 0x27, # Enter calibration mode
		'CalReadI'     : 0x2b, # Calibrate current value
		'CalReadInfo'  : 0x2f, # Read calibration information
		'CalReadV'     : 0x29, # Calibrate voltage value
		'CalReset'     : 0x32, # Restore factor default calibration data
		'CalSaveData'  : 0x2d, # Save calibration data to EEPROM
		'CalSendI'     : 0x2c, # Send the output current to calibration program
		'CalSendV'     : 0x2a, # Send the output voltage to calibration program
		'CalSetInfo'   : 0x2e, # Setting calibration information
		'CalState'     : 0x28, # Read calibration mode state
		'EnaLocalKey'  : 0x37, # Enable the local key
		'OutputEnable' : 0x21, # Setting the output on/off state
		'ReadConfig'   : 0x26, # Read the present current/voltage, maximum voltage,
							   # setup voltage/current, operation states
		'ReadID'       : 0x31, # Read product's model-, series-, and version- information
		'RemoteEnable' : 0x20, # Setting the remote control mode
		'ReturnInfo'   : 0x12, # Return information of command operation  	
		'SetAddress'   : 0x25, # Setting the communication address
		'SetLimitV'    : 0x22, # Setting maximum output voltage
		'SetOutputI'   : 0x24, # Setting the output current
		'SetOutputV'   : 0x23, # Setting the output voltage
		'StartByte'    : 0xaa, # Start byte
	}
	
	def __init__(self, com, address = 0):
		self._serial = com
		self._address = address
	def _assemble_bytes(self, Bytes):
		# Bytes : List of Bytes to be communicated in send order
		# Frame Format
		#	Start 0xAA (Byte)
		#	Address 0x00 (Byte)
		cmd = bytearray(26)
		cmd[0] = self._COMMANDS['StartByte']
		cmd[1] = self._address
		#   Command (Byte)
		#   Information (4-25) bytes
		ii = 2
		for bt in Bytes:
			cmd[ii] = bt
			ii = ii + 1
		#   Check sum; sum of former 25 bytes
		sum = 0
		for ch in cmd: sum = sum + ch
		cmd[25] = sum % 256
		return cmd
	def _send_cmd(self, cmd, max_retry = 10):
		# Implemented separately for modularity
		# Sends Command, then verifies supply response
		# In case of 
		out_bytes = self._assemble_bytes(cmd)
		success = False
		ii = 0
		err = 0xFF # Not Specified in User Manual
		while( (success == False) and (ii < max_retry) ):
			self._serial.write(out_bytes)
			resp = self._receive_msg()
			# Verify Response Packet
			if((resp[0] == 0xAA) and (resp[1] == self._address) and (resp[2] == 0x12)):
				err = resp[3]
			# If Command Succeeds, exit loop
			if(err == 0x80): success = True
			# Increment send count
			ii = ii + 1
		ret = False
		if( (ii >= max_retry) and (err != 0x80) ):
			print('Transmit Error: Failed to Send Command')
		else:
			ret = True
		return ret
	def _receive_msg(self, max_bytes=64):
		# Implemented separately for modularity
		# Max_bytes sets the max number of additional bytes to search
		# for a valid message string.
		rx_buf = collections.deque(maxlen=26)
		rx_count = 0;
		# Initial read
		for byte in self._serial.read(size=26):
			rx_buf.append(byte)
		# Continue reading until start byte detected
		while((rx_buf[0] != 0xaa) and (rx_count < max_bytes)):
			next_byte = int.from_bytes(self._serial.read(size=1), 'little')
			rx_buf.append(next_byte)
			rx_count = rx_count + 1
		# Translate Result to byte array
		msg_bytes = bytearray(26)
		if(rx_buf[0] == 0xaa):
			for ii in range(26): msg_bytes[ii] = rx_buf[ii]
		# Return the resulting array
		return msg_bytes
	@staticmethod
	def _num_to_bytes(value, len):
		val = int(round(value))
		return [bt for bt in val.to_bytes(len, 'little')]
	def close(self):
		self.output_disable()
		self.remote_control(enable = False)
	def get_status(self, retry_lim=10):
		# Queries the device for present values
		# Expects a response containing the following data:
		# {Output Current, Output Voltage, Supply State, Current Limit,
		# Maximum Voltage, (Target) Output Voltage}
		cmd = bytearray(1)
		cmd[0] = self._COMMANDS['ReadConfig']
		retry = True
		ii = 0

		status = {}
		while( (ii < retry_lim) and retry ): 
			# Return is unique to status command; implemented separately
			self._serial.write(self._assemble_bytes(cmd))

			# Decode the Response
			resp = self._receive_msg()

			# Verify Checksum
			sum = 0
			for ch in resp[0:-1]:
				sum = sum + ch # Sum all by the checksum
			chk = (sum % 256)

			if( (resp[0] == 0xAA) and (resp[1] == self._address) and (chk == resp[25])):
				retry = False
				status['address'] = resp[1]
				status['command'] = resp[2]
				# State Variable
				state = resp[9]
				status['output_enable'] = True if((state & 0x01) == 0x01) else False
				status['over_heat'] = True if((state & 0x02) == 0x02) else False
				# Output Mode
				output_mode = (state & 0x0c) >> 2
				if(output_mode == 1): status['output_mode'] = 'CV'
				elif(output_mode == 2): status['output_mode'] = 'CC'
				else: status['output_mode'] = 'UR'
				status['fan_speed'] = (state & 0x70) >> 4
				status['operation_state'] = 'RC' if ((state & 0x80) == 0x80) else 'FP'
				# Output_* reports the presently measured value
				status['output_current'] = 0.001*int.from_bytes(resp[3:5], 'little')
					# ==== THIS QUERY IS INCORRECT ==== #

				status['output_voltage'] = 0.001*int.from_bytes(resp[5:9], 'little')
				# Target_* reports the specified setpoint (desired value)
				status['target_current'] = 0.001*int.from_bytes(resp[10:12], 'little')
				status['target_voltage'] = 0.001*int.from_bytes(resp[16:20], 'little')
				# Maxumum Values - I DON'T KNOW WHAT THESE MEAN
				status['maximum_voltage'] = 0.001*int.from_bytes(resp[12:16], 'little')

			ii = ii + 1
		return status
	def output_enable(self, enable = True):
		# Enables the output (enable = True, default)
		# for (enable = False) output is disabled
		cmd = bytearray(2)
		cmd[0] = self._COMMANDS['OutputEnable']
		cmd[1] = 0x01 if enable else 0x00
		self._send_cmd(cmd)
	def output_disable(self):
		self.output_enable(False)
	def remote_control(self, enable = True):
		# Enables remote control. Must be set prior to use.
		cmd = bytearray(2)
		cmd[0] = self._COMMANDS['RemoteEnable']
		cmd[1] = 0x01 if enable else 0x00
		self._send_cmd(cmd)
	def set_maximum_voltage(self, voltage):
		# Sets the (maximum) output voltage, for voltage is specified in volts
		# For regular usage use set_output_voltage()
		cmd = bytearray(5)
		cmd[0] = self._COMMANDS['SetLimitV']
		cmd[1:5] = int(voltage*1000).to_bytes(4, 'little')
		self._send_cmd(cmd)
	def set_output_voltage(self, voltage):
		# Sets the output voltage, for voltage is specified in volts
		cmd = bytearray(5)
		cmd[0] = self._COMMANDS['SetOutputV']
		cmd[1:5] = int(voltage*1000).to_bytes(4, 'little')
		self._send_cmd(cmd)
	def set_output_current(self, current):
		# Sets the (maximum) output current, for current specified in amps
		cmd = bytearray(3)
		cmd[0] = self._COMMANDS['SetOutputI']
		cmd[1:3] = int(current*1000).to_bytes(2, 'little')
		self._send_cmd(cmd)

if __name__ == "__main__":
	# EXAMPLE USAGE
	ser = serial.Serial('COM8')
	PSU = BK_1787b(ser)
	PSU.remote_control()
	PSU.set_output_voltage(12)
	PSU.set_output_current(0.2)
	PSU.output_enable()
	print(PSU.get_status())
	time.sleep(5)
	PSU.close()