from ultralytics import YOLO
import cv2
import RPi.GPIO as GPIO
import time

# กำหนด PIN มอเตอร์
LEFT_MOTOR_FORWARD = 5
LEFT_MOTOR_BACKWARD = 6
RIGHT_MOTOR_FORWARD = 13
RIGHT_MOTOR_BACKWARD = 19

# PWM Speed
PWM_FREQ = 100  # Hz
SEARCH_SPEED = 30  # searching speed
MOVE_SPEED = 70    # move forward speed

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup([LEFT_MOTOR_FORWARD, LEFT_MOTOR_BACKWARD, RIGHT_MOTOR_FORWARD, RIGHT_MOTOR_BACKWARD], GPIO.OUT)

# Setup PWM
pwm_left_forward = GPIO.PWM(LEFT_MOTOR_FORWARD, PWM_FREQ)
pwm_left_backward = GPIO.PWM(LEFT_MOTOR_BACKWARD, PWM_FREQ)
pwm_right_forward = GPIO.PWM(RIGHT_MOTOR_FORWARD, PWM_FREQ)
pwm_right_backward = GPIO.PWM(RIGHT_MOTOR_BACKWARD, PWM_FREQ)

pwm_left_forward.start(0)
pwm_left_backward.start(0)
pwm_right_forward.start(0)
pwm_right_backward.start(0)

# โหลดโมเดล Custom ที่คุณจะเทรนภายหลัง
model = YOLO("custom_model.pt")  # เปลี่ยนชื่อเมื่อคุณเทรนเสร็จ

# เปิดกล้อง
cap = cv2.VideoCapture(0)

def stop_motors():
    pwm_left_forward.ChangeDutyCycle(0)
    pwm_left_backward.ChangeDutyCycle(0)
    pwm_right_forward.ChangeDutyCycle(0)
    pwm_right_backward.ChangeDutyCycle(0)

def move_forward(speed):
    pwm_left_forward.ChangeDutyCycle(speed)
    pwm_right_forward.ChangeDutyCycle(speed)
    pwm_left_backward.ChangeDutyCycle(0)
    pwm_right_backward.ChangeDutyCycle(0)

def turn_right_and_forward(speed):
    pwm_left_forward.ChangeDutyCycle(speed)
    pwm_right_forward.ChangeDutyCycle(speed // 3)  # หมุนขวาช้าๆ
    pwm_left_backward.ChangeDutyCycle(0)
    pwm_right_backward.ChangeDutyCycle(0)

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        results = model(frame, imgsz=640, conf=0.5)

        # ตรวจว่ามี class ขยะ (สมมุติ class_id = 0)
        garbage_found = False
        for box in results[0].boxes:
            class_id = int(box.cls[0])
            if class_id == 0:   # class 0 คือ "ขยะ" ที่คุณ train เอง
                garbage_found = True
                break

        if garbage_found:
            print("🚮 พบขยะ → เดินหน้า")
            move_forward(MOVE_SPEED)
        else:
            print("🔄 ไม่พบขยะ → หมุนหา + เดินหน้าช้า")
            turn_right_and_forward(SEARCH_SPEED)

        # แสดงภาพ
        annotated_frame = results[0].plot()
        cv2.imshow('Detection', annotated_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("🛑 หยุดระบบ")

finally:
    stop_motors()
    cap.release()
    cv2.destroyAllWindows()
    GPIO.cleanup()
