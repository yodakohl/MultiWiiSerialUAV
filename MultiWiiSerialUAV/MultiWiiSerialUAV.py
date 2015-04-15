import struct
import serial
import threading
import time

#If Copter is manually controlled and no heartbeat is send within 2 seconds
#the failsafe routine is executed

#On low battery the failsafe is executed



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

	altitude  = -0
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
	RCAUX1     = 1000
	RCAUX2     = 1000
	RCAUX3     = 1000
	RCAUX4     = 1000

	GPS_FIX           = 0
	GPS_numSat        = 0
	GPS_latitude      = 0
	GPS_longitude     = 0
	GPS_altitude      = 0
	GPS_speed         = 0
	GPS_ground_course = 0

	stopSignal = False
	ready      = False
	armed      = False
	failsafe   = False

	lastHeartbeat = None

	interface = "/dev/ttyUSB0"

	def isArmed(self):
		return self.armed

	def isReady(self):
		return self.ready


	#if no heartbeat is sent each second, the failsafe will be triggered
	def heartbeat(self):
		self.lastHeartbeat = time.time()


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
		self.connect(self.interface)

		while (self.stopSignal == False):
			try:
				self.writeRC()
				self.readMessage()
				self.sendCommand(self.ALTITUDE)
				self.readMessage()
				time.sleep(0.1)

				if((lastHeartbeat!=None) and( time.time() > lastHeartbeat +1)):
					self.setFailsafe()

			except Exception,e:
				print("Bad: " + str(e))
				pass

		print ("UAV finished")


	def setInterface(self,interfaceName):
		self.interface = interfaceName

	def stop(self):
		self.stopSignal = True

	def getAltitude(self):
		return self.altitude

	def getLongitude(self):
		return self.longitude

	def getLatitude(self):
		return self.latitude

	def setFailsafe(self):
		print("FAILSAFE")
		self.failsafe = True


	def readMessage(self):
		
		self.readPreamble()
		postAmble = self.SerialInterface.read(3)

		if (len(postAmble) == 3):
			cmd = ord(postAmble[2])
			size = ord(postAmble[1])
			data = self.SerialInterface.read(size)
			chksum = self.SerialInterface.read(1)
			
			chksumVerify = self.getChkSum(postAmble[1] + postAmble[2] + data)
			if( ord(chksum) == chksumVerify):

				if(cmd == self.RAW_IMU):
					pass
				elif (cmd == self.SERVO):
					pass
				elif (cmd == self.ALTITUDE):
					self.parseALTITUDE(data)
				elif (cmd == self.RAW_GPS):
					self.parseRAW_GPS(data)
				else:
					pass
					#print ("Unknown Command: " + str(cmd))
			else:
				print ("CHKSUM failed")


	def parseALTITUDE(self,data):
		unpacked = struct.unpack('<ih',data)
		self.altitude = unpacked[0]

	def parseRAW_GPS(self,data):
		unpacked = struct.unpack('<bbiihhh',data)
		GPS_FIX           = unpacked[0]
		GPS_numSat        = unpacked[1]
		GPS_latitude      = unpacked[2]
		GPS_longitude     = unpacked[3]
		GPS_altitude      = unpacked[4]
		GPS_speed         = unpacked[5]
		GPS_ground_course = unpacked[6]

	def calibrate(self):
		print("Calibrating")
		self.sendCommand(self.ACC_CALIBRATION)
		self.sendCommand(self.MAG_CALIBRATION)
		time.sleep(12)
		self.ready = True
		print("Calibrating finished")

	def sendCommand(self,command):
		msg = struct.pack('<3c3B','$','M','<',0,command,command)
		self.SerialInterface.write(msg)
		self.SerialInterface.flush()

	def arm(self):
		self.setRC(1500,1500,2000,1000,self.RCAUX1,self.RCAUX2,self.RCAUX3,self.RCAUX4)
		self.armed = True

	def disarm(self):
		self.setRC(1500,1500,1000,1000,self.RCAUX1,self.RCAUX2,self.RCAUX3,self.RCAUX4)
		self.armed = False

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
		if (self.failsafe == False):
			self.MSP_SET_RAW_RC(self.RCroll,self.RCpitch,self.RCyaw,self.RCthrottle, self.RCAUX1,self.RCAUX2,self.RCAUX3,self.RCAUX4)
		else:
			self.MSP_SET_RAW_RC(1500,1500,1500,1350, 1000,1000,1000,1000)


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










