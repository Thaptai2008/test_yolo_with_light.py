import RPi.GPIO as GPIO
import time

# GPIO ที่เชื่อมกับ MDDS10
PWM1 = 18
DIR1 = 23
PWM2 = 24
DIR2 = 25

GPIO.setmode(GPIO.BCM)
GPIO.setup([PWM1, DIR1, PWM2, DIR2], GPIO.OUT)

# สร้าง PWM
pwm_left = GPIO.PWM(PWM1, 1000)
pwm_right = GPIO.PWM(PWM2, 1000)
pwm_left.start(0)
pwm_right.start(0)

def move_forward(speed=70, duration=2):
    GPIO.output(DIR1, GPIO.HIGH)
    GPIO.output(DIR2, GPIO.HIGH)
    pwm_left.ChangeDutyCycle(speed)
    pwm_right.ChangeDutyCycle(speed)
    time.sleep(duration)

def move_backward(speed=70, duration=2):
    GPIO.output(DIR1, GPIO.LOW)
    GPIO.output(DIR2, GPIO.LOW)
    pwm_left.ChangeDutyCycle(speed)
    pwm_right.ChangeDutyCycle(speed)
    time.sleep(duration)

def turn_left(speed=70, duration=1):
    GPIO.output(DIR1, GPIO.HIGH)
    GPIO.output(DIR2, GPIO.HIGH)
    pwm_left.ChangeDutyCycle(0)
    pwm_right.ChangeDutyCycle(speed)
    time.sleep(duration)

def turn_right(speed=70, duration=1):
    GPIO.output(DIR1, GPIO.HIGH)
    GPIO.output(DIR2, GPIO.HIGH)
    pwm_left.ChangeDutyCycle(speed)
    pwm_right.ChangeDutyCycle(0)
    time.sleep(duration)

def stop():
    pwm_left.ChangeDutyCycle(0)
    pwm_right.ChangeDutyCycle(0)

try:
    print("เดินหน้า")
    move_forward()

    print("เลี้ยวซ้าย")
    turn_left()

    print("เลี้ยวขวา")
    turn_right()

    print("ถอยหลัง")
    move_backward()

    print("หยุด")
    stop()

except KeyboardInterrupt:
    pass

finally:
    pwm_left.stop()
    pwm_right.stop()
    GPIO.cleanup()
