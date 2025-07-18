from ultralytics import YOLO
import cv2
import RPi.GPIO as GPIO
import time

# GPIO Pin Definitions
IN1 = 17   # มอเตอร์ซ้าย
IN2 = 27
IN3 = 22   # มอเตอร์ขวา
IN4 = 23
ENA = 18   # PWM ซ้าย
ENB = 24   # PWM ขวา

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup([IN1, IN2, IN3, IN4, ENA, ENB], GPIO.OUT)

# PWM Setup
pwm_left = GPIO.PWM(ENA, 1000)
pwm_right = GPIO.PWM(ENB, 1000)
pwm_left.start(0)
pwm_right.start(0)

# Load YOLOv8 model
model = YOLO("yolov8n.pt")

# เปิดกล้อง
cap = cv2.VideoCapture(0)

def motor_forward(speed=70):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_left.ChangeDutyCycle(speed)
    pwm_right.ChangeDutyCycle(speed)

def motor_stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm_left.ChangeDutyCycle(0)
    pwm_right.ChangeDutyCycle(0)

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        results = model(frame, imgsz=640, conf=0.5)

        bottle_found = False
        for box in results[0].boxes:
            class_id = int(box.cls[0])
            if class_id == 39:
                bottle_found = True
                break

        if bottle_found:
            print("พบขวดน้ำ → เดินหน้า")
            motor_forward()
        else:
            print("ไม่พบขวด → หยุด")
            motor_stop()

        annotated_frame = results[0].plot()
        cv2.imshow('YOLO Detection', annotated_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("ปิดโปรแกรม")

finally:
    cap.release()
    cv2.destroyAllWindows()
    motor_stop()
    GPIO.cleanup()
