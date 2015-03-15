import kivy
kivy.require('1.0.6') # replace with your current kivy version !

import time

from kivy.app import App
from kivy.uix.button import Button
from kivy.uix.widget import Widget
from kivy.core.window import Window
from MultiwiiSerialUAV import MultiwiiSerialUAV


class Remote(Widget):

	UAV = MultiwiiSerialUAV()
	UAV.daemon = True
	throttle = 1000
	roll = 1500
	pitch = 1500
	yaw = 1500

	def __init__(self, **kwargs):
		super(Remote, self).__init__(**kwargs)
		self._keyboard = Window.request_keyboard(self._keyboard_closed, self)
		self._keyboard.bind(on_key_down=self._on_keyboard_down)
		self._keyboard.bind(on_key_up=self._on_keyboard_up)

	def _keyboard_closed(self):
		self._keyboard.unbind(on_key_down=self._on_keyboard_down)
		self._keyboard.unbind(on_key_up=self._on_keyboard_up)
		self._keyboard = None


	def _failsafe(self):
		self.throttle = 1000
		self.roll = 1500
		self.pitch = 1500
		self.yaw = 1500
		self._setRC()
		self.UAV.disarm()

	def _setRC(self):
		self.UAV.setRC(self.roll,self.pitch,self.yaw,self.throttle,0,2000,0,0)

	def _throttleUp(self):
		self.throttle += 50
		if (self.throttle > 2000):
			self.throttle = 2000
		self._setRC()


	def _throttleDown(self):
		self.throttle -= 100
		if (self.throttle < 1000):
			self.throttle = 1000
		self._setRC()

	def _start(self):
		print("Starting UAV")
		self.UAV.start()
		while self.UAV.isReady() == False:
			time.sleep(1)
		print("Arming")
		self.UAV.arm()
		time.sleep(4)
		print("Armed, running Idle")

	def _moveLeft(self):
		pass

	def _moveRight(self):
		pass


	def _moveForward(self):
		pass

	def _moveBackward(self):
		pass


	def _on_keyboard_down(self, keyboard, keycode, text, modifiers):
		if keycode[1] == 'v':
			print("W")
			self._throttleUp()
		elif keycode[1] == 'i':
			self._throttleDown()
		elif keycode[1] == 'up':
			self.pitch = 1650
			self._setRC()
		elif keycode[1] == 'u':
			self.yaw = 1250
			self._setRC()
		elif keycode[1] == 'a':
			self.yaw = 1750
			self._setRC()
		elif keycode[1] == 'down':
			self.pitch = 1350 
			self._setRC()
		elif keycode[1] == 'left':
			self.roll = 1350
			self._setRC()
		elif keycode[1] == 'right':
			self.roll = 1650
			self._setRC()
			print("RIGHT")
		elif keycode[1] == 'spacebar':
			self._failsafe()
		elif keycode[1] == 'enter':
			self._start()
		return True

	def _on_keyboard_up(self,keyboard, keycode):
		print "UP: " + keycode[1]
		if keycode[1] == 'v':
			pass
		elif keycode[1] == 'i':
			pass
		elif keycode[1] == 'u':
			self.yaw = 1500
			self._setRC()
		elif keycode[1] == 'a':
			self.yaw = 1500
			self._setRC()
		elif keycode[1] == 'up':
			self.pitch = 1500
			self._setRC()
		elif keycode[1] == 'down':
			self.pitch = 1500
			self._setRC()
		elif keycode[1] == 'left':
			self.roll = 1500
			self._setRC()
		elif keycode[1] == 'right':
			self.roll = 1500
			self._setRC()
			print("RIGHT")
		elif keycode[1] == 'spacebar':
			self._failsafe()

		return True


class RemoteApp(App):
	def build(self):
		return Remote()





if __name__ == '__main__':
	RemoteApp().run()
