import RPi.GPIO as gpio
import time

class MotorDriver(object):

	def __init__(self):

		self.IN1 = 17 #Wheel1 Forward
		self.IN2 = 22 #Wheel1 Reverse
		self.IN3 = 23 #Wheel2 Forward
		self.IN4 = 24 #Wheel2 Reverse
        self.IN5 = 5 #Wheel1 Forward
		self.IN6 = 6 #Wheel1 Reverse
		self.IN7 = 13 #Wheel2 Forward
		self.IN8 = 19 #Wheel2 Reverse

		#+ For other two wheels

		gpio.setmode(gpio.BCM)
		gpio.setup(self.IN1,gpio.OUT)
		gpio.setup(self.IN2,gpio.OUT)
		gpio.setup(self.IN3,gpio.OUT)
		gpio.setup(self.IN4,gpio.OUT)

	def set_motor(self,A1,A2,B1,B2):
		
		gpio.output(self.IN1, A1)
		gpio.output(self.IN2, A2)
		gpio.output(self.IN3, B1)
		gpio.output(self.IN4, B2)

	def forward(self):
		self.set_motor(True,False,True,False)
	

	def stop(self):
		self.set_motor(0,0,0,0)

	def reverse(self):
		self.set_motor(0,1,0,1)

	def right(self):
		self.set_motor(0,0,1,0)

	def left(self):
		self.set_motor(1,0,0,0)


def motor_test():
	motor = MotorDriver()
	
	print("forward")
	motor.forward()
	time.sleep(1)

	print("stop")
	motor.stop()
	time.sleep(1)

	print("reverse")
	motor.reverse()
	time.sleep(1)

	print("stop")
	motor.stop()
	time.sleep(1)

	print("right")
	motor.right()
	time.sleep(0.5)
	
	print("left")
	motor.left()
	time.sleep(0.5)
		
	print("stop")
	motor.stop()
	


if __name__=="__main__":
	print("Starting Robot Test")
	while True:
		print("Testing bot")
		motor_test()
	GPIO.cleanup()
	print("END!!!! JAI SHRI RAM")	