from ultralytics import YOLO
import cv2
import RPi.GPIO as GPIO
import time

# LED pin
LEFT_LED = 17
RIGHT_LED = 18

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(LEFT_LED, GPIO.OUT)
GPIO.setup(RIGHT_LED, GPIO.OUT)

# Load YOLOv8 model
model = YOLO("yolov8n.pt")   # ใช้โมเดลเล็กสุด (n = nano)

# เปิดกล้อง
cap = cv2.VideoCapture(0)

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        # ตรวจจับวัตถุในภาพ
        results = model(frame, imgsz=640, conf=0.5)

        # ตรวจว่ามี class "bottle" หรือไม่
        bottle_found = False

        for box in results[0].boxes:
            class_id = int(box.cls[0])
            if class_id == 39:   # 39 = bottle ใน COCO dataset
                bottle_found = True
                break

        if bottle_found:
            GPIO.output(LEFT_LED, GPIO.HIGH)
            GPIO.output(RIGHT_LED, GPIO.HIGH)
            print("พบขวดน้ำ → เปิดไฟทั้งสองดวง")
        else:
            GPIO.output(LEFT_LED, GPIO.LOW)
            GPIO.output(RIGHT_LED, GPIO.LOW)
            print("ไม่พบขวดน้ำ → ดับไฟ")

        # แสดงภาพด้วยกล่องวัตถุ
        annotated_frame = results[0].plot()
        cv2.imshow('YOLO Detection', annotated_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("ปิดโปรแกรม")

finally:
    cap.release()
    cv2.destroyAllWindows()
    GPIO.cleanup()
