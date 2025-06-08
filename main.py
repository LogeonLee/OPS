import cv2
import mediapipe as mp
import numpy as np
import serial
import time
import math
import threading
from plyer import notification

# Mediapipe 초기화
mp_pose = mp.solutions.pose
pose = mp_pose.Pose()
mp_drawing = mp.solutions.drawing_utils

# 알림 상태 추적
popup_displayed = False
face_popup_displayed = False
last_popup_time = 0
popup_cooldown_sec = 5
last_face_popup_time = 0
face_popup_cooldown_sec = 5

# 캘리브레이션 저장 변수
shoulder_distances = []
movements_cm = []
last_shoulder_distance = None
initial_distance_cm = None
center_x_ref = None
left_x = None
right_x = None
px_per_cm = None
x_calibration_step = 0
last_l_shoulder = None
last_r_shoulder = None
current_center_x = None
calibration_done = False
real_distance = None
nowposx = 0
nowposy = 0

# 버튼 위치 (x1, y1, x2, y2)
button_position = (20, 60, 200, 110)

# 아두이노 포트 설정
arduino = serial.Serial(
    port='COM3',
    baudrate=9600,
    timeout=1,
    dsrdtr=False,   # DTR line 비활성화
    rtscts=False
)
arduino.setDTR(False)
time.sleep(2)


# 링크 길이 40
L1 = -160.0
L2 = 200.0

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

def send_location(x, y):
    try:
        nowposx = x
        nowposy = y
        a1, a2 = compute_angles(x, y)
        a3 = 90  # 기본 각도 or 다른 로직
        send_angles(a1, a2, 180-a3)
        time.sleep(1)
    except Exception as e:
        print("Invalid input or unreachable target:", e)


# 거리 계산 함수
def calculate_distance(p1, p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

# 캘리브레이션 계산 함수
def estimate_initial_distance_4points(p, d):
    if len(p) != 4 or len(d) != 4:
        raise ValueError("4개의 픽셀 및 거리 정보가 필요합니다.")
    try:
        x1 = abs((p[1] * d[1]) / (p[0] - p[1]))
        x2 = abs((p[2] * d[2]) / (p[0] - p[2]))
        x3 = abs((p[3] * d[3]) / (p[0] - p[3]))
        return (x1 + x2 + x3) / 3
    except ZeroDivisionError:
        return float('inf')

# 회전 각도 추정 함수 (어깨)
def estimate_shoulder_rotation_angle(landmarks):
    l_sh = landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER]
    r_sh = landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER]
    dx = l_sh.x - r_sh.x
    dz = l_sh.z - r_sh.z
    angle_rad = np.arctan2(dz, dx)
    return np.degrees(angle_rad)

# 회전 각도 추정 함수 (얼굴)
def estimate_face_rotation_angle(landmarks):
    l_eye = landmarks[mp_pose.PoseLandmark.LEFT_EYE]
    r_eye = landmarks[mp_pose.PoseLandmark.RIGHT_EYE]
    dx = l_eye.x - r_eye.x
    dz = l_eye.z - r_eye.z
    angle_rad = np.arctan2(dz, dx)
    return np.degrees(angle_rad)

def calibrate_distance():
    global shoulder_distances, movements_cm, last_shoulder_distance
    global x_calibration_step, center_x_ref, left_x, right_x, px_per_cm
    global initial_distance_cm

    shoulder_distances = []
    movements_cm = []
    send_location(-60, 0)
    for i in [60, 120, 140, 180]:
        if last_shoulder_distance is not None:
            move = float((i/10)-6)
            send_location(-i, 0)
            time.sleep(8)
            shoulder_distances.append(last_shoulder_distance)
            movements_cm.append(move)
            print(f"[{len(shoulder_distances)}] 저장: {last_shoulder_distance:.2f}px, 이동: {move}cm")
            print(shoulder_distances)
        
    # 캘리브레이션 완료 시 거리 계산
    if len(shoulder_distances) == 4:
        initial_distance_cm = estimate_initial_distance_4points(shoulder_distances, movements_cm)
        print(f"\n📏 추정된 초기 거리 (X): {initial_distance_cm:.2f} cm")
    else:
        print("❌ 캘리브레이션 데이터 부족 (3개 필요)")
    """
    for  i in [0, 1, 2]:
        current_center_x = (last_l_shoulder[0] + last_r_shoulder[0]) // 2
        if x_calibration_step == 0:
            send_location(40,0)
            time.sleep(4);
            center_x_ref = current_center_x
        elif x_calibration_step == 1:
            send_location(40, 10)
            time.sleep(1);
            left_x = current_center_x
        elif x_calibration_step == 2:
            send_location(40,-10)
            time.sleep(1);
            right_x = current_center_x
    
            # 캘리브레이션 완료
            px_per_cm = ((center_x_ref - left_x) + (right_x - center_x_ref)) / 2
            print(f"✅ 좌우 px_per_cm: {px_per_cm:.2f} px/cm")
            calibration_done = True
        
        x_calibration_step += 1
    """

# 마우스 콜백 함수
def mouse_callback(event, x, y, flags, param):
    x1, y1, x2, y2 = button_position
    if event == cv2.EVENT_LBUTTONDOWN and x1 <= x <= x2 and y1 <= y <= y2:
        thread = threading.Thread(target = calibrate_distance)
        thread.start()
        
# 비디오 캡처 설정
cap = cv2.VideoCapture(0)
cv2.namedWindow("Posture Detection")
cv2.setMouseCallback("Posture Detection", mouse_callback)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = pose.process(frame_rgb)

    if results.pose_landmarks:
        landmarks = results.pose_landmarks.landmark
        h, w, _ = frame.shape

        # 어깨 거리 계산
        l_sh = landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER]
        r_sh = landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER]
        
        # 전역 좌표 갱신
        last_l_shoulder = (int(l_sh.x * w), int(l_sh.y * h))
        last_r_shoulder = (int(r_sh.x * w), int(r_sh.y * h))
        
        # 어깨 간 거리
        last_shoulder_distance = calculate_distance(last_l_shoulder, last_r_shoulder)

        # 회전 각도 계산
        shoulder_angle = estimate_shoulder_rotation_angle(landmarks)
        face_angle = estimate_face_rotation_angle(landmarks)

        # 실시간 거리 계산 (캘리브레이션 완료 후)
        if initial_distance_cm is not None and last_shoulder_distance > 0:
            real_distance = initial_distance_cm * shoulder_distances[0] / last_shoulder_distance
            cv2.putText(frame, f"Distance: {real_distance:.1f} cm", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 200), 2)
            if real_distance > 55 or real_distance < 50:
                nowposx -= (real_distance - 50)
                if (nowposx > -60):
                    nowposx = -60
                send_location(nowposx, nowposy)

        text = f"Shoulder Angle: {shoulder_angle:.1f}°, Face Angle: {face_angle:.1f}°"
        cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        """
        if calibration_done and px_per_cm is not None:
        # 현재 어깨 중심 위치
            current_center_x = (last_l_shoulder[0] + last_r_shoulder[0]) // 2
    
            # 화면 중심
            screen_center_x = frame.shape[1] // 2
        
            # 픽셀 오차 → 보정된 px_per_cm (거리 기반 보정 포함)
            scaled_px_per_cm = px_per_cm * (initial_distance_cm / real_distance)
        
            # 픽셀 차이 → cm로 환산
            x_offset_px = current_center_x - screen_center_x
            x_offset_cm_from_screen = x_offset_px / scaled_px_per_cm
    
        # 표시
            cv2.putText(frame, f"From Center: {x_offset_cm_from_screen:.1f} cm",
                    (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 180, 255), 2)
        """
        # 알림 조건
        if abs(shoulder_angle) > 15 and time.time() - last_popup_time > popup_cooldown_sec:
            notification.notify(
                title='어깨 경고',
                message='몸을 정면으로 맞춰주세요!',
                timeout=1
            )
            last_popup_time = time.time()
        """
        if abs(face_angle) > 15 and time.time() - last_face_popup_time > face_popup_cooldown_sec:
            notification.notify(
                title='얼굴 경고',
                message='얼굴을 정면으로 맞춰주세요!',
                timeout=1
            )
            last_face_popup_time = time.time()
        """
        mp_drawing.draw_landmarks(frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

    # 캘리브레이션 버튼 그리기
    x1, y1, x2, y2 = button_position
    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 128, 255), -1)
    cv2.putText(frame, "Save Point", (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)

    cv2.imshow("Posture Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()


