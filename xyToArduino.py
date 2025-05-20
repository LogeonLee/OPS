import serial
import time
import math

# 아두이노 포트 설정
arduino = serial.Serial(port='COM6', baudrate=9600, timeout=1)
time.sleep(2)

# 링크 길이 40
L1 = -40.0
L2 = 115.0

def compute_angles(x, y):
    r2 = x**2 + y**2
    c2 = (r2 - L1**2 - L2**2) / (2 * L1 * L2)
    c2 = max(min(c2, 1.0), -1.0)
    theta2 = math.acos(c2)

    k1 = L1 + L2 * math.cos(theta2)
    k2 = L2 * math.sin(theta2)
    theta1 = math.atan2(y, x) - math.atan2(k2, k1)

    angle1 = int(math.degrees(theta1))
    angle2 = int(math.degrees(theta2))
    print(angle1, angle2)
    # 보정 및 제한
    angle1 = max(min(angle1, 180), 0)
    angle2 = max(min(angle2, 180), 0)

    return angle1, angle2

def send_angles(angle1, angle2, angle3):
    arduino.reset_input_buffer()
    arduino.write(f"{angle1},{angle2},{angle3}\n".encode())
    print(f"Sent: {angle1}, {angle2}, {angle3}")

# 메인 루프
while True:
    try:
        x, y = map(float, input("Enter X Y (mm): ").split())
        a1, a2 = compute_angles(x, y)
        a3 = 90  # 기본 각도 or 다른 로직
        send_angles(a1, a2, 180-a3)
    except Exception as e:
        print("Invalid input or unreachable target:", e)
