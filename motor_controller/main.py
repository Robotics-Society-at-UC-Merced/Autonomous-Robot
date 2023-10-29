import time
import board
import pwmio
import digitalio

en = digitalio.DigitalInOut(board.D2)
en.direction = digitalio.Direction.OUTPUT

dir = digitalio.DigitalInOut(board.D4)
dir.direction = digitalio.Direction.OUTPUT

motor = pwmio.PWMOut(board.D3, frequency=60000, duty_cycle=0)

en.value = True
dir.value = True

while True:
    print("hello")
    motor.duty_cycle = 10000
