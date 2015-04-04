from MultiWiiSerialUAV import MultiwiiSerialUAV

import sys
import time



def main():


	UAV = MultiwiiSerialUAV()
	UAV.daemon = True
	try:
		UAV.start()
		while UAV.isReady() == False:
			time.sleep(1)
		print("Arming")
		UAV.arm()
		time.sleep(4)
		print("Armed, running Idle")
		UAV.setRC(1500,1500,1500,1650,0,0,0,0)

		while True:
			if (UAV.getAltitude() > 90):
				UAV.setRC(1500,1500,1500,1000,2000,0,0,0)
				pass
			elif(UAV.getAltitude() <50):
				#UAV.setRC(1500,1500,1500,1400,1000,0,0,0)
				pass
			time.sleep(0.1)

	except (KeyboardInterrupt, SystemExit):
		print("Recieved Stop Request")
		UAV.stop()
		sys.exit()

if __name__=="__main__":
	main()
