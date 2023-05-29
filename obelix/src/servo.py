import RPi.GPIO as GPIO
import time


#setup servo raspy
GPIO.setmode(GPIO.BOARD)
GPIO.setup(12,GPIO.OUT)
servo = GPIO.PWM(12,50)

servo.start(0)
time.sleep(2)

while True:
    try:
        print("Frente 7")
        servo.ChangeDutyCycle(7)
        time.sleep(2)

        print("Esquerda 1")
        servo.ChangeDutyCycle(2)
        time.sleep(2)

        print("Direita 12")
        servo.ChangeDutyCycle(12)
        time.sleep(2)

        print("Frente 7")
        servo.ChangeDutyCycle(7)
        time.sleep(2)

    except (KeyboardInterrupt):
        break

print("Fim")

servo.stop()
GPIO.cleanup()