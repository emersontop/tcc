from gpiozero import AngularServo
from time import sleep

servo = AngularServo(22, min_pulse_width=1/1000, max_pulse_width=19/1000)

while (True):
	servo.angle = 90
	sleep(2)
	servo.angle = 0
	sleep(2)
	servo.angle = -90
	sleep(2)