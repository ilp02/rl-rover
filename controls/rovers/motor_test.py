import RPi.GPIO as GPIO
import time

# BCM numbering
GPIO.setmode(GPIO.BCM)

# Define motor control GPIOs
motor_pins = [[17, 27], [23, 22]] #AIN1, AIN2, BIN1, BIN2

print(1)
#Set pins
for motor in motor_pins:
    for pin in motor:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)

print(2)
for i in range(5):
    print(3)
    #ccw
    for motor in motor_pins:
        GPIO.output(motor[0], GPIO.HIGH)
        GPIO.output(motor[1], GPIO.LOW)

    time.sleep(5)
    break
    print(4)
    #cw
    for motor in motor_pins:
        GPIO.output(motor[0], GPIO.LOW)
        GPIO.output(motor[1], GPIO.HIGH)

    time.sleep(1)

# Cleanup
GPIO.cleanup()
print("Test complete. GPIO cleaned up.")