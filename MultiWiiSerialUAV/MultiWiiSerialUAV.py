import struct
import serial
import threading
import time

class MultiwiiSerialUAV(threading.Thread):
	IDENT            = 100
	STATUS           = 101
	RAW_IMU          = 102
	SERVO            = 103
	MOTOR            = 104
	RC               = 105
	RAW_GPS          = 106
	COMP_GPS         = 107
	ATTITUDE         = 108
	ALTITUDE         = 109
	ANALOG           = 110
	RC_TUNING        = 111
	PID              = 112
	BOX              = 113
	MISC             = 114
	MOTOR_PINS       = 115
	BOXNAMES         = 116
	PIDNAMES         = 117
	WP               = 118
	BOXIDS           = 119
	RC_RAW_IMU       = 121
	SET_RAW_RC       = 200
	SET_RAW_GPS      = 201
	SET_PID          = 202
	SET_BOX          = 203
	SET_RC_TUNING    = 204
	ACC_CALIBRATION  = 205
	MAG_CALIBRATION  = 206
	SET_MISC         = 207
	RESET_CONF       = 208
	SET_WP           = 209
	SWITCH_RC_SERIAL = 210
	IS_SERIAL        = 211
	DEBUG            = 254

	SerialInterface  = None


	latitude  =  0.0
	longitude =  0.0
	altitude  = -0
	heading   = -0
	timestamp = -0
	gpsString = -0
	numSats   = -0
	accuracy  = -1
	roll      =  0
	pitch     =  0
	yaw       =  0
	throttle  =  0

	magx = 0
	magy = 0
	magz = 0 

	gyrx = 0
	gyry = 0
	gyrz = 0

	accx = 0
	accy = 0
	accz = 0


	RCroll     = 1500
	RCpitch    = 1500
	RCyaw      = 1500
	RCthrottle = 1000
	RCAUX1     = 0
	RCAUX2     = 0
	RCAUX3     = 0
	RCAUX4     = 0

	stopSignal = False
	ready      = False

	def isReady(self):
		return self.ready

	def readPreamble(self):
		state = 1
		while (state != 3):
			if state == 1:
				data = self.SerialInterface.read(1)
				if data == '$':
					state = 2
			elif state == 2:
				data =  self.SerialInterface.read(1)
				if data == 'M':
					state = 3
				elif data == '$':
					state = 2
				else:
					state = 1
		return

	def run(self):
		self.connect("/dev/ttyUSB0")

		while (self.stopSignal == False):
			self.writeRC()
			self.sendCommand(109)
			self.readMessage()

		print ("UAV finished")

	def stop(self):
		self.stopSignal = True

	def getAltitude(self):
		return self.altitude

	def readMessage(self):
		
		self.readPreamble()
		postAmble = self.SerialInterface.read(3)

		if (len(postAmble) == 3):
			if (postAmble[0] == '<'):
				pass
				#print ("Incoming Message")
			else:
				pass
				#print ("Outgoing Message")

			cmd = ord(postAmble[2])
			size = ord(postAmble[1])

			#print ("CMD: " + str(cmd))
			data = self.SerialInterface.read(size)
			chksum = self.SerialInterface.read(1)
			
			chksumVerify = self.getChkSum(postAmble[1] + postAmble[2] + data)#Verify Checksum
			if( ord(chksum) == chksumVerify):
				#pass to interpreter
				if(cmd == 102):
					pass
				elif (cmd == 103):
					pass
				elif (cmd == self.ALTITUDE):
					self.parseALTITUDE(data)
				else:
					pass
			else:
				print ("CHKSUM failed")


	def parseALTITUDE(self,data):
		unpacked = struct.unpack('<ih',data)
		self.altitude = unpacked[0]
		#print ("Altitude: " + str(self.altitude))
		pass

	def calibrate(self):
		print("Calibrating")
		self.sendCommand(205)
		self.sendCommand(206)
		time.sleep(12)
		self.ready = True
		print("Calibrating finished")

	def sendCommand(self,command):
		msg = struct.pack('<3c3B','$','M','<',0,command,command)
		self.SerialInterface.write(msg)
		self.SerialInterface.flush()
		pass

	def arm(self):
		self.setRC(1500,1500,2000,1000,0,0,0,0)

	
	def disarm(self):
		self.setRC(1500,1500,1000,1000,0,0,0,0)


	def sendDataCommand(command,data):
		pass

	def setRC(self,Lroll,Lpitch,Lyaw,Lthrottle,Laux1,Laux2,Laux3,Laux4):
		self.RCroll       = Lroll
		self.RCpitch      = Lpitch
		self.RCyaw        = Lyaw
		self.RCthrottle   = Lthrottle
		self.RCAUX1       = Laux1
		self.RCAUX2       = Laux2
		self.RCAUX3       = Laux3
		self.RCAUX4       = Laux4

	def writeRC(self):
		self.MSP_SET_RAW_RC(self.RCroll,self.RCpitch,self.RCyaw,self.RCthrottle, self.RCAUX1,self.RCAUX2,self.RCAUX3,self.RCAUX4)

	def MSP_SET_RAW_RC(self,Lroll,Lpitch,Lyaw,Lthrottle,Laux1,Laux2,Laux3,Laux4):

		self.SerialInterface.write(struct.pack('<3c','$','M','<'))
		restByte= struct.pack('<2B8H',16,200, Lroll,Lpitch,Lyaw,Lthrottle,Laux1,Laux2,Laux3,Laux4)
		self.SerialInterface.write(restByte)
		self.SerialInterface.write(struct.pack('<B',self.getChkSum(restByte)))


	def getChkSum(self,message):
		checksum = 0
		for i in range(len(message)):
			checksum = checksum ^ ord(message[i])

		return checksum


	def connect(self,interface):
		self.SerialInterface=serial.Serial()
		self.SerialInterface.port=interface
		self.SerialInterface.baudrate=115200
		self.SerialInterface.parity=serial.PARITY_NONE
		self.SerialInterface.bytesize=serial.EIGHTBITS
		self.SerialInterface.stopbits=serial.STOPBITS_ONE
		self.SerialInterface.xonxoff=False
		self.SerialInterface.rtscts=False
		self.SerialInterface.dsrdtr=False
		self.SerialInterface.writeTimeout=2
		self.SerialInterface.readTimeout=2

		try:
			self.SerialInterface.open()

		except Exception,e:
			print("Could not open serial interface: "+str(e))
			exit()

		self.SerialInterface.flushInput()
		self.SerialInterface.flushOutput()

		time.sleep(10)
		self.calibrate()
		print("SerialInterface opened")









