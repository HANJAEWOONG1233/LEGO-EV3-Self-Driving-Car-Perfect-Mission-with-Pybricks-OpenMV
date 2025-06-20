#!/usr/bin/env pybricks-micropython

# 버전 1.61 (최종 부스트를 1초 시간제한으로 변경)

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from mindsensorsPYB import LSA
from pybricks.iodevices import UARTDevice # UART 통신을 위해 추가

import os
import sys
import time

# --- 설정 상수 ---
SPEAKER_MUTED = False

# EV3 브릭 및 장치 초기화
ev3 = EV3Brick()
try:
    lsa = LSA(Port.S1, 0x14) # LSA 포트 및 주소 확인
except Exception as e:
    ev3.screen.print("LSA Init Error"); wait(2000); exit()

run_motor = Motor(Port.A) # 주행 모터 포트 확인
steering_motor = Motor(Port.D) # 조향 모터 포트 확인

try:
    color_sensor = ColorSensor(Port.S4) # 컬러 센서 포트 확인
except Exception as e: # 컬러센서가 S4에 없으면 OpenMV용으로 UART를 S4에 연결 시도
    ev3.screen.print("ColorSensor S4 Err")
    wait(1000)

try:
    openmv_uart = UARTDevice(Port.S3, baudrate=115200, timeout=10) # timeout 추가
    ev3.screen.print("OpenMV UART on S3 OK")
    wait(500)
except Exception as e:
    ev3.screen.clear(); ev3.screen.print("OpenMV UART Error:"); ev3.speaker.beep(); wait(5000);
    openmv_uart = None

try:
    ultrasonic_sensor = UltrasonicSensor(Port.S2) # 초음파 센서 포트 확인
except Exception as e:
    ev3.screen.print("Ultrasonic S2 Err"); wait(2000)
    ultrasonic_sensor = None


# --- RGB 색상 판단 임계값 ---
RED_R_MIN = 12; RED_R_MAX = 22; RED_G_MAX = 8; RED_B_MAX = 10
YELLOW_R_MIN = 16; YELLOW_R_MAX = 28; YELLOW_G_MIN = 19; YELLOW_G_MAX = 33; YELLOW_B_MAX = 18
# -------------------------------------------------------------------------

# 라인 트레이싱 매개변수
DRIVE_SPEED_LINETRACE = 200
DRIVE_SPEED_MAX_LINETRACE = 2000
STEERING_KP_LINETRACE = -7.0
HIGH_SPEED_KP_FACTOR = 0.1 
STEERING_MOTOR_SPEED_LINETRACE = 2500

# 시간제 부스트 하이퍼파라미터
SECOND_RED_BOOST_DURATION_MS = 2600

# --- [수정됨] 4번째 빨간 선 이후 연속 동작 설정 ---
FOURTH_RED_INITIAL_BOOST_MS = 1500 # 4번째 선 감지 직후 부스트 시간 (1.5초)
POST_BOOST_WAIT_DURATION_MS = 12500 # 이후 일반 속도 대기 시간 (12초)
FINAL_BOOST_DURATION_MS = 500      # 마지막 부스트 시간 (1초)
# ----------------------------------------------------

# 3번째 빨간 선 감지 후 후진 설정
WHEEL_CIRCUMFERENCE_CM = 25.0
REVERSE_DISTANCE_CM = 10.0
REVERSE_FOR_SIGNAL_SPEED = 800
REVERSE_FOR_SIGNAL_DEG = (REVERSE_DISTANCE_CM / WHEEL_CIRCUMFERENCE_CM) * 360.0

# (주차, 점멸등, 경적 상수는 이전과 동일)
PARKING_DRIVE_SPEED = 80; PARKING_STEERING_SPEED = 300
MIN_OPEN_SPACE_FOR_PARKING_MM = 350
DIST_BETWEEN_SCAN_SPOTS_MOTOR_DEG = 360 * 2.2
DRIVE_PAST_SPOT_MOTOR_DEG = 180
STEERING_ANGLE_INTO_SPOT_MOTOR_DEG = 70
REVERSE_DIAGONAL_MOTOR_DEG = 360 * 1.15
STEERING_ANGLE_STRAIGHTEN_MOTOR_DEG = -80
REVERSE_STRAIGHT_MOTOR_DEG = 360 * 0.7
FLASHING_LIGHT_COLORS = [Color.RED, Color.YELLOW, Color.GREEN]
FLASHING_LIGHT_INTERVAL_MS = 1000
HORN_FREQUENCY = 600; HORN_DURATION_MS = 150
RED_LINE_BEEP_FREQUENCY = 440; RED_LINE_BEEP_DURATION = 100
PARKING_MODE_ACTIVATE_BUTTON = Button.UP
PARKING_SCAN_START_BUTTON = Button.DOWN

# --- 로봇 상태 정의 ---
class RobotState:
    INITIALIZING = "Initializing"
    WAITING_TO_START = "Waiting to Start"
    LINE_TRACING = "Line Tracing"
    TRAFFIC_LIGHT_WAITING = "Traffic Light Waiting"
    PARKING_SCAN_PROMPT = "Parking Scan Prompt"
    PARKING_SCANNING = "Parking Scanning"
    PARKING_EXECUTING = "Parking Executing"
    MISSION_COMPLETE = "Mission Complete"
    ERROR = "Error"

# --- 전역 변수 ---
current_state = RobotState.INITIALIZING
red_line_count = 0; robot_is_on_red_patch = False
third_red_line_detected_this_lap = False; green_light_sim_received = False
parking_spot_selected = None; parking_attempted_this_lap = False
school_zone_action_completed_this_segment = False; in_high_speed_mode = False

# 특수 주행 모드용 변수
post_yellow_mode = "NONE"; post_yellow_timer = StopWatch()
second_red_boost_active = False; second_red_boost_timer = StopWatch()
post_fourth_red_mode = "NONE"; post_fourth_red_timer = StopWatch()


# --- 도우미 함수 ---
def speak(text): pass
def beep_custom(f=HORN_FREQUENCY, d=HORN_DURATION_MS):
    if not SPEAKER_MUTED: ev3.speaker.beep(f, d)
def start_origin_calibration():
    ev3.screen.clear(); ev3.screen.print("Origin Calibrating...")
    steering_motor.run(100)
    while True:
        a = steering_motor.angle(); wait(100); b = steering_motor.angle()
        if a == b: break
    steering_motor.stop()
    steering_motor.run_angle(-100, 100, then=Stop.HOLD, wait=True)
    steering_motor.reset_angle(0)
    steering_motor.run_target(PARKING_STEERING_SPEED, 0, then=Stop.HOLD, wait=True)
    ev3.screen.clear(); ev3.screen.print("Origin Calibrated!"); wait(500)
def stop_robot_motion(): run_motor.brake(); steering_motor.brake()
def start_line_tracing_motion(use_max_speed=False):
    ev3.screen.clear()
    speed = DRIVE_SPEED_MAX_LINETRACE if use_max_speed else DRIVE_SPEED_LINETRACE
    msg = "Line Tracing (MAX)" if use_max_speed else "Line Tracing Start..."
    ev3.screen.print(msg); run_motor.run(speed)
def activate_flashing_lights():
    for c in FLASHING_LIGHT_COLORS: ev3.light.on(c); wait(FLASHING_LIGHT_INTERVAL_MS)
    ev3.light.off()
def sound_parking_horn():
    for _ in range(2): beep_custom(); wait(HORN_DURATION_MS // 2)

def play_completion_tune():
    if not SPEAKER_MUTED:
        ev3.screen.print("Playing victory tune!")
        notes = ['C5/8', 'E5/8', 'G5/4', 'C6/2']
        ev3.speaker.play_notes(notes, tempo=160)

def read_openmv_signal():
    if openmv_uart is None: return ""
    try:
        if openmv_uart.waiting() > 0: return openmv_uart.read_all().decode().strip()
        else: return ""
    except Exception: return ""

# --- 핵심 로직 함수 ---
def handle_color_detections():
    global red_line_count,robot_is_on_red_patch,third_red_line_detected_this_lap,current_state, school_zone_action_completed_this_segment,in_high_speed_mode
    global post_yellow_mode, post_yellow_timer
    global second_red_boost_active, second_red_boost_timer
    global post_fourth_red_mode, post_fourth_red_timer

    if color_sensor is None: return False 

    r,g,b=color_sensor.rgb()
    is_red=(r>=RED_R_MIN and r<=RED_R_MAX and g<=RED_G_MAX and b<=RED_B_MAX and r>(g+3)and r>(b+3))
    is_yellow=(r>=YELLOW_R_MIN and r<=YELLOW_R_MAX and g>=YELLOW_G_MIN and g<=YELLOW_G_MAX and b<=YELLOW_B_MAX and g>r and g>(b+5)and r>(b+3))
    if is_red:
        school_zone_action_completed_this_segment=False
        if not robot_is_on_red_patch:
            red_line_count+=1;robot_is_on_red_patch=True
            if not SPEAKER_MUTED:ev3.speaker.beep(RED_LINE_BEEP_FREQUENCY,RED_LINE_BEEP_DURATION)
            
            ev3.screen.clear(); ev3.screen.print("Red Line Count: " + str(red_line_count)); wait(500)

            if red_line_count==2 and not third_red_line_detected_this_lap:
                ev3.screen.clear(); ev3.screen.print("2nd Red: Boost!")
                in_high_speed_mode = True
                second_red_boost_active = True
                second_red_boost_timer.reset()
                run_motor.run(DRIVE_SPEED_MAX_LINETRACE)
                return False
            
            elif red_line_count==3 and not third_red_line_detected_this_lap:
                third_red_line_detected_this_lap=True;in_high_speed_mode=False
                ev3.screen.clear();ev3.screen.print("3rd Red: Stop!")
                stop_robot_motion()
                ev3.screen.print("Reversing " + str(REVERSE_DISTANCE_CM) + "cm...")
                run_motor.run_angle(-REVERSE_FOR_SIGNAL_SPEED, REVERSE_FOR_SIGNAL_DEG, Stop.BRAKE, True)
                wait(200)
                ev3.screen.print("Wait OMV Signal")
                current_state=RobotState.TRAFFIC_LIGHT_WAITING
                return True
            
            elif red_line_count == 4:
                ev3.screen.clear(); ev3.screen.print("4th Red: Initial Boost!")
                in_high_speed_mode = True
                post_fourth_red_mode = "INITIAL_BOOST" # 상태머신 시작
                post_fourth_red_timer.reset()
                run_motor.run(DRIVE_SPEED_MAX_LINETRACE)
                return False

            elif red_line_count >= 5:
                ev3.screen.clear(); ev3.screen.print("5th Red: Finish Line!"); stop_robot_motion(); current_state = RobotState.MISSION_COMPLETE; return True
        return False
    elif is_yellow:
        robot_is_on_red_patch=False
        if current_state==RobotState.LINE_TRACING and not school_zone_action_completed_this_segment:
            if in_high_speed_mode:in_high_speed_mode=False;run_motor.run(DRIVE_SPEED_LINETRACE);wait(50)
            
            ev3.screen.clear();ev3.screen.print("School Zone!"); stop_robot_motion()
            activate_flashing_lights()
            
            ev3.screen.print("Phase 1: 7s Normal Speed")
            post_yellow_mode = "NORMAL"
            post_yellow_timer.reset()
            school_zone_action_completed_this_segment = True
            
            start_line_tracing_motion(use_max_speed=False)
            return True
        return False
    else:robot_is_on_red_patch=False;return False

def scan_and_select_parking_spot():
    global parking_spot_selected
    if ultrasonic_sensor is None: ev3.screen.print("Ultrasonic Error!"); return None
    ev3.screen.clear();ev3.screen.print("Scanning Parking...");dist1=ultrasonic_sensor.distance();wait(1000)
    run_motor.run_angle(PARKING_DRIVE_SPEED,DIST_BETWEEN_SCAN_SPOTS_MOTOR_DEG,Stop.BRAKE,True);wait(500)
    dist2=ultrasonic_sensor.distance();wait(1000)
    good1=dist1>MIN_OPEN_SPACE_FOR_PARKING_MM;good2=dist2>MIN_OPEN_SPACE_FOR_PARKING_MM;name=None
    if good1 and(not good2 or dist1>=dist2):ev3.screen.print("Select: Spot 1");run_motor.run_angle(-PARKING_DRIVE_SPEED,DIST_BETWEEN_SCAN_SPOTS_MOTOR_DEG,Stop.BRAKE,True);wait(500);name="Spot 1"
    elif good2:ev3.screen.print("Select: Spot 2");name="Spot 2"
    else:ev3.screen.print("No Suitable Spot.")
    parking_spot_selected=name;return name is not None
def perform_parallel_parking():
    global parking_attempted_this_lap
    if parking_spot_selected is None:ev3.screen.print("Parking Cancel (No Spot)");parking_attempted_this_lap=True;return False
    ev3.screen.clear();ev3.screen.print("Parking: "+str(parking_spot_selected));wait(500);sound_parking_horn();wait(300)
    activate_flashing_lights();wait(200);run_motor.run_angle(PARKING_DRIVE_SPEED,DRIVE_PAST_SPOT_MOTOR_DEG,Stop.BRAKE,True)
    steering_motor.run_target(PARKING_STEERING_SPEED,STEERING_ANGLE_INTO_SPOT_MOTOR_DEG,Stop.HOLD,True);activate_flashing_lights();wait(200)
    run_motor.run_angle(-PARKING_DRIVE_SPEED,REVERSE_DIAGONAL_MOTOR_DEG,Stop.BRAKE,True)
    steering_motor.run_target(PARKING_STEERING_SPEED,STEERING_ANGLE_STRAIGHTEN_MOTOR_DEG,Stop.HOLD,True);activate_flashing_lights();wait(200)
    run_motor.run_angle(-PARKING_DRIVE_SPEED,REVERSE_STRAIGHT_MOTOR_DEG,Stop.BRAKE,True)
    steering_motor.run_target(PARKING_STEERING_SPEED,0,Stop.HOLD,True);ev3.light.off();stop_robot_motion()
    ev3.screen.clear();ev3.screen.print("Parking Complete!");parking_attempted_this_lap=True;return True

# --- 메인 프로그램 ---
try:
    while True:
        pressed_buttons_this_loop = ev3.buttons.pressed()

        if current_state == RobotState.INITIALIZING:
            start_origin_calibration()
            ev3.screen.clear(); ev3.screen.print("Wheels Aligning...")
            steering_motor.run_target(PARKING_STEERING_SPEED, 0, then=Stop.HOLD, wait=True); wait(500)
            current_state = RobotState.WAITING_TO_START

        elif current_state == RobotState.WAITING_TO_START:
            ev3.screen.clear(); ev3.screen.print("Press Center Button"); ev3.screen.print("to Start Mission.")
            while not Button.CENTER in ev3.buttons.pressed(): wait(50)
            ev3.screen.clear(); ev3.screen.print("Center Pressed! Starting..."); ev3.speaker.beep(); wait(500) 
            start_line_tracing_motion(); current_state = RobotState.LINE_TRACING

        elif current_state == RobotState.LINE_TRACING:
            if handle_color_detections(): 
                continue 
            
            if second_red_boost_active:
                if second_red_boost_timer.time() > SECOND_RED_BOOST_DURATION_MS:
                    ev3.screen.clear(); ev3.screen.print("2nd Red Boost End.")
                    second_red_boost_active = False
                    in_high_speed_mode = False
                    run_motor.run(DRIVE_SPEED_LINETRACE)
                    wait(500)
            
            # --- [수정됨] 4번째 빨간 선 이후 연속 동작 처리 ---
            if post_fourth_red_mode == "INITIAL_BOOST":
                if post_fourth_red_timer.time() > FOURTH_RED_INITIAL_BOOST_MS:
                    ev3.screen.clear(); ev3.screen.print("Waiting 12s...")
                    post_fourth_red_mode = "WAITING"
                    in_high_speed_mode = False
                    run_motor.run(DRIVE_SPEED_LINETRACE)
                    post_fourth_red_timer.reset()

            elif post_fourth_red_mode == "WAITING":
                if post_fourth_red_timer.time() > POST_BOOST_WAIT_DURATION_MS:
                    ev3.screen.clear(); ev3.screen.print("Final 1s Boost!")
                    post_fourth_red_mode = "FINAL_BOOST"
                    in_high_speed_mode = True
                    run_motor.run(DRIVE_SPEED_MAX_LINETRACE)
                    post_fourth_red_timer.reset()

            elif post_fourth_red_mode == "FINAL_BOOST":
                if post_fourth_red_timer.time() > FINAL_BOOST_DURATION_MS:
                    ev3.screen.clear(); ev3.screen.print("All boosts complete.")
                    post_fourth_red_mode = "NONE" # 모든 동작 완료
                    in_high_speed_mode = False
                    run_motor.run(DRIVE_SPEED_LINETRACE)
                    wait(500)
            # ----------------------------------------------------

            if post_yellow_mode == "NORMAL":
                if post_yellow_timer.time() > 7000:
                    ev3.screen.clear(); ev3.screen.print("Phase 2: 3s High Speed")
                    post_yellow_mode = "BOOST"
                    in_high_speed_mode = True
                    run_motor.run(DRIVE_SPEED_MAX_LINETRACE)
                    post_yellow_timer.reset()
            elif post_yellow_mode == "BOOST":
                if post_yellow_timer.time() > 3000:
                    ev3.screen.clear(); ev3.screen.print("School Zone clear.")
                    ev3.screen.print("Resuming normal trace.")
                    post_yellow_mode = "NONE"
                    in_high_speed_mode = False
                    run_motor.run(DRIVE_SPEED_LINETRACE)
                    wait(500)
            
            if green_light_sim_received and not parking_attempted_this_lap:
                if PARKING_MODE_ACTIVATE_BUTTON in pressed_buttons_this_loop:
                    if in_high_speed_mode: # 모든 부스트 케이스를 한번에 처리
                        in_high_speed_mode=False
                        # 모든 특수 모드 플래그/상태 초기화
                        second_red_boost_active=False
                        post_fourth_red_mode="NONE" 
                        run_motor.run(DRIVE_SPEED_LINETRACE)
                    ev3.screen.clear();ev3.screen.print("Parking Mode Activated");stop_robot_motion();current_state=RobotState.PARKING_SCAN_PROMPT;continue
            try:
                hap=list(lsa.ReadRaw_Calibrated())
                if hap and len(hap)>=8:
                    left_dark=sum(100-r_val for r_val in hap[0:3])/3;right_dark=sum(100-r_val for r_val in hap[5:8])/3
                    error=left_dark-right_dark;current_kp=STEERING_KP_LINETRACE*HIGH_SPEED_KP_FACTOR if in_high_speed_mode else STEERING_KP_LINETRACE
                    correction=current_kp*error;max_correction=90;correction=max(-max_correction,min(max_correction,correction))
                    steering_motor.run_target(STEERING_MOTOR_SPEED_LINETRACE,correction,Stop.HOLD,False)
                else:pass
            except Exception as e:pass

        elif current_state == RobotState.TRAFFIC_LIGHT_WAITING:
            signal = read_openmv_signal()
            if signal == "two":
                ev3.screen.clear(); ev3.screen.print("OMV: Green! Go.")
                beep_custom(880, 200); wait(200)
                green_light_sim_received = True
                
                start_line_tracing_motion(use_max_speed=False)
                
                current_state = RobotState.LINE_TRACING
            elif signal == "one": ev3.screen.print("OMV: Red. Still Waiting...")
            elif signal == "three": ev3.screen.print("OMV: Yellow. Still Waiting...")
            elif signal: ev3.screen.print("OMV Sig: " + signal)
            wait(100)

        elif current_state == RobotState.PARKING_SCAN_PROMPT:
            ev3.screen.clear(); ev3.screen.print("Parking: Align, then Down btn")
            scan_trigger_pressed = False
            while not scan_trigger_pressed:
                if PARKING_SCAN_START_BUTTON in ev3.buttons.pressed():
                    ev3.screen.print("Down Button for Scan!"); ev3.speaker.beep(); wait(500); scan_trigger_pressed = True
                wait(50)
            current_state = RobotState.PARKING_SCANNING
        elif current_state == RobotState.PARKING_SCANNING:
            if scan_and_select_parking_spot(): current_state = RobotState.PARKING_EXECUTING
            else: 
                ev3.screen.print("No spot. Resume trace."); wait(1000)
                parking_attempted_this_lap = True; start_line_tracing_motion(); current_state = RobotState.LINE_TRACING
        
        elif current_state == RobotState.PARKING_EXECUTING:
            if perform_parallel_parking():
                ev3.screen.clear()
                ev3.screen.print("Parking successful!")
                ev3.screen.print("Resuming line tracing...")
                wait(2000)
                start_line_tracing_motion()
                current_state = RobotState.LINE_TRACING
            else:
                ev3.screen.print("Parking failed/canceled."); wait(1000)
                start_line_tracing_motion(); current_state = RobotState.LINE_TRACING
        
        elif current_state == RobotState.MISSION_COMPLETE:
            ev3.screen.clear(); ev3.screen.print("Mission Complete!")
            play_completion_tune() 
            stop_robot_motion(); ev3.light.off(); wait(3000); break
        
        elif current_state == RobotState.ERROR:
            ev3.screen.clear(); ev3.screen.print("Error Occurred!"); stop_robot_motion(); ev3.light.on(Color.RED); wait(5000); break
        wait(20)
except KeyboardInterrupt:
    ev3.screen.clear(); ev3.screen.print("Program Stopped by User.")
except Exception as e:
    ev3.screen.clear(); ev3.screen.print("Runtime Error!"); print("Runtime Error:", e); current_state = RobotState.ERROR
finally:
    stop_robot_motion(); ev3.light.off(); ev3.screen.clear()
    if current_state != RobotState.MISSION_COMPLETE and current_state != RobotState.ERROR: ev3.screen.print("Program Ended.")
    elif current_state == RobotState.ERROR: ev3.screen.print("Ended due to Error.")
    wait(2000)


# # 버전 1.58 (4번째 빨간 선 부스트 시간 1.5초로 변경)

# from pybricks.hubs import EV3Brick
# from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
#                                  InfraredSensor, UltrasonicSensor, GyroSensor)
# from pybricks.parameters import Port, Stop, Direction, Button, Color
# from pybricks.tools import wait, StopWatch, DataLog
# from pybricks.robotics import DriveBase
# from mindsensorsPYB import LSA
# from pybricks.iodevices import UARTDevice # UART 통신을 위해 추가

# import os
# import sys
# import time

# # --- 설정 상수 ---
# SPEAKER_MUTED = False

# # EV3 브릭 및 장치 초기화
# ev3 = EV3Brick()
# try:
#     lsa = LSA(Port.S1, 0x14) # LSA 포트 및 주소 확인
# except Exception as e:
#     ev3.screen.print("LSA Init Error"); wait(2000); exit()

# run_motor = Motor(Port.A) # 주행 모터 포트 확인
# steering_motor = Motor(Port.D) # 조향 모터 포트 확인

# try:
#     color_sensor = ColorSensor(Port.S4) # 컬러 센서 포트 확인
# except Exception as e: # 컬러센서가 S4에 없으면 OpenMV용으로 UART를 S4에 연결 시도
#     ev3.screen.print("ColorSensor S4 Err")
#     wait(1000)
#     # 이 경우 컬러센서가 다른 포트에 있거나 사용하지 않는다고 가정.

# try:
#     # OpenMV는 Port.S3에 연결되었다고 가정 (컬러센서와 같은 포트 사용 시 주의)
#     openmv_uart = UARTDevice(Port.S3, baudrate=115200, timeout=10) # timeout 추가
#     ev3.screen.print("OpenMV UART on S3 OK")
#     wait(500)
# except Exception as e:
#     ev3.screen.clear()
#     ev3.screen.print("OpenMV UART Error:")
#     error_message = str(e)
#     max_chars_per_line = 18
#     for i in range(0, len(error_message), max_chars_per_line):
#         ev3.screen.print(error_message[i:i+max_chars_per_line])
#     ev3.speaker.beep(); wait(5000);
#     openmv_uart = None

# try:
#     ultrasonic_sensor = UltrasonicSensor(Port.S2) # 초음파 센서 포트 확인
# except Exception as e:
#     ev3.screen.print("Ultrasonic S2 Err"); wait(2000)
#     ultrasonic_sensor = None


# # --- RGB 색상 판단 임계값 ---
# RED_R_MIN = 12; RED_R_MAX = 22; RED_G_MAX = 8; RED_B_MAX = 10
# YELLOW_R_MIN = 16; YELLOW_R_MAX = 28; YELLOW_G_MIN = 19; YELLOW_G_MAX = 33; YELLOW_B_MAX = 18
# # -------------------------------------------------------------------------

# # 라인 트레이싱 매개변수
# DRIVE_SPEED_LINETRACE = 200
# DRIVE_SPEED_MAX_LINETRACE = 2000
# STEERING_KP_LINETRACE = -7.0
# HIGH_SPEED_KP_FACTOR = 0.1 
# STEERING_MOTOR_SPEED_LINETRACE = 2500

# # 시간제 부스트 하이퍼파라미터
# SECOND_RED_BOOST_DURATION_MS = 2600
# # --- [수정됨] 4번째 빨간 선 부스트 시간을 1.5초로 변경 ---
# FOURTH_RED_BOOST_DURATION_MS = 1500 # 4번째 빨간 선 통과 후 부스트 지속 시간 (1.5초)

# # 3번째 빨간 선 감지 후 후진 설정
# WHEEL_CIRCUMFERENCE_CM = 25.0
# REVERSE_DISTANCE_CM = 10.0
# REVERSE_FOR_SIGNAL_SPEED = 800
# REVERSE_FOR_SIGNAL_DEG = (REVERSE_DISTANCE_CM / WHEEL_CIRCUMFERENCE_CM) * 360.0

# # (주차, 점멸등, 경적 상수는 이전과 동일)
# PARKING_DRIVE_SPEED = 80; PARKING_STEERING_SPEED = 300
# MIN_OPEN_SPACE_FOR_PARKING_MM = 350
# DIST_BETWEEN_SCAN_SPOTS_MOTOR_DEG = 360 * 2.2
# DRIVE_PAST_SPOT_MOTOR_DEG = 180
# STEERING_ANGLE_INTO_SPOT_MOTOR_DEG = 70
# REVERSE_DIAGONAL_MOTOR_DEG = 360 * 1.1
# STEERING_ANGLE_STRAIGHTEN_MOTOR_DEG = -80
# REVERSE_STRAIGHT_MOTOR_DEG = 360 * 0.7
# FLASHING_LIGHT_COLORS = [Color.RED, Color.YELLOW, Color.GREEN]
# FLASHING_LIGHT_INTERVAL_MS = 1000
# HORN_FREQUENCY = 600; HORN_DURATION_MS = 150
# RED_LINE_BEEP_FREQUENCY = 440; RED_LINE_BEEP_DURATION = 100
# PARKING_MODE_ACTIVATE_BUTTON = Button.UP
# PARKING_SCAN_START_BUTTON = Button.DOWN

# # --- 로봇 상태 정의 ---
# class RobotState:
#     INITIALIZING = "Initializing"
#     WAITING_TO_START = "Waiting to Start"
#     LINE_TRACING = "Line Tracing"
#     TRAFFIC_LIGHT_WAITING = "Traffic Light Waiting"
#     PARKING_SCAN_PROMPT = "Parking Scan Prompt"
#     PARKING_SCANNING = "Parking Scanning"
#     PARKING_EXECUTING = "Parking Executing"
#     MISSION_COMPLETE = "Mission Complete"
#     ERROR = "Error"

# # --- 전역 변수 ---
# current_state = RobotState.INITIALIZING
# red_line_count = 0; robot_is_on_red_patch = False
# third_red_line_detected_this_lap = False; green_light_sim_received = False
# parking_spot_selected = None; parking_attempted_this_lap = False
# school_zone_action_completed_this_segment = False; in_high_speed_mode = False

# # 특수 주행 모드용 변수
# post_yellow_mode = "NONE"; post_yellow_timer = StopWatch()
# second_red_boost_active = False; second_red_boost_timer = StopWatch()
# fourth_red_boost_active = False; fourth_red_boost_timer = StopWatch() # 4번째 선 부스트용


# # --- 도우미 함수 ---
# def speak(text): pass
# def beep_custom(f=HORN_FREQUENCY, d=HORN_DURATION_MS):
#     if not SPEAKER_MUTED: ev3.speaker.beep(f, d)
# def start_origin_calibration():
#     ev3.screen.clear(); ev3.screen.print("Origin Calibrating...")
#     steering_motor.run(100)
#     while True:
#         a = steering_motor.angle(); wait(100); b = steering_motor.angle()
#         if a == b: break
#     steering_motor.stop()
#     steering_motor.run_angle(-100, 100, then=Stop.HOLD, wait=True)
#     steering_motor.reset_angle(0)
#     steering_motor.run_target(PARKING_STEERING_SPEED, 0, then=Stop.HOLD, wait=True)
#     ev3.screen.clear(); ev3.screen.print("Origin Calibrated!"); wait(500)
# def stop_robot_motion(): run_motor.brake(); steering_motor.brake()
# def start_line_tracing_motion(use_max_speed=False):
#     ev3.screen.clear()
#     speed = DRIVE_SPEED_MAX_LINETRACE if use_max_speed else DRIVE_SPEED_LINETRACE
#     msg = "Line Tracing (MAX)" if use_max_speed else "Line Tracing Start..."
#     ev3.screen.print(msg); run_motor.run(speed)
# def activate_flashing_lights():
#     for c in FLASHING_LIGHT_COLORS: ev3.light.on(c); wait(FLASHING_LIGHT_INTERVAL_MS)
#     ev3.light.off()
# def sound_parking_horn():
#     for _ in range(2): beep_custom(); wait(HORN_DURATION_MS // 2)

# def play_completion_tune():
#     if not SPEAKER_MUTED:
#         ev3.screen.print("Playing victory tune!")
#         notes = ['C5/8', 'E5/8', 'G5/4', 'C6/2']
#         ev3.speaker.play_notes(notes, tempo=160)

# def read_openmv_signal():
#     if openmv_uart is None: return ""
#     try:
#         if openmv_uart.waiting() > 0: return openmv_uart.read_all().decode().strip()
#         else: return ""
#     except Exception: return ""

# # --- 핵심 로직 함수 ---
# def handle_color_detections():
#     global red_line_count,robot_is_on_red_patch,third_red_line_detected_this_lap,current_state, school_zone_action_completed_this_segment,in_high_speed_mode
#     global post_yellow_mode, post_yellow_timer
#     global second_red_boost_active, second_red_boost_timer
#     global fourth_red_boost_active, fourth_red_boost_timer # 전역 변수 선언 추가

#     if color_sensor is None: return False 

#     r,g,b=color_sensor.rgb()
#     is_red=(r>=RED_R_MIN and r<=RED_R_MAX and g<=RED_G_MAX and b<=RED_B_MAX and r>(g+3)and r>(b+3))
#     is_yellow=(r>=YELLOW_R_MIN and r<=YELLOW_R_MAX and g>=YELLOW_G_MIN and g<=YELLOW_G_MAX and b<=YELLOW_B_MAX and g>r and g>(b+5)and r>(b+3))
#     if is_red:
#         school_zone_action_completed_this_segment=False
#         if not robot_is_on_red_patch:
#             red_line_count+=1;robot_is_on_red_patch=True
#             if not SPEAKER_MUTED:ev3.speaker.beep(RED_LINE_BEEP_FREQUENCY,RED_LINE_BEEP_DURATION)
            
#             ev3.screen.clear(); ev3.screen.print("Red Line Count: " + str(red_line_count)); wait(500)

#             if red_line_count==2 and not third_red_line_detected_this_lap:
#                 ev3.screen.clear(); ev3.screen.print("2nd Red: Boost!")
#                 in_high_speed_mode = True
#                 second_red_boost_active = True
#                 second_red_boost_timer.reset()
#                 run_motor.run(DRIVE_SPEED_MAX_LINETRACE)
#                 return False
            
#             elif red_line_count==3 and not third_red_line_detected_this_lap:
#                 third_red_line_detected_this_lap=True;in_high_speed_mode=False
#                 ev3.screen.clear();ev3.screen.print("3rd Red: Stop!")
#                 stop_robot_motion()
#                 ev3.screen.print("Reversing " + str(REVERSE_DISTANCE_CM) + "cm...")
#                 run_motor.run_angle(-REVERSE_FOR_SIGNAL_SPEED, REVERSE_FOR_SIGNAL_DEG, Stop.BRAKE, True)
#                 wait(200)
#                 ev3.screen.print("Wait OMV Signal")
#                 current_state=RobotState.TRAFFIC_LIGHT_WAITING
#                 return True
            
#             elif red_line_count == 4:
#                 ev3.screen.clear(); ev3.screen.print("4th Red: 1.5s Boost!")
#                 in_high_speed_mode = True
#                 fourth_red_boost_active = True
#                 fourth_red_boost_timer.reset()
#                 run_motor.run(DRIVE_SPEED_MAX_LINETRACE)
#                 return False # 라인 트레이싱 계속

#             elif red_line_count >= 5:
#                 ev3.screen.clear(); ev3.screen.print("5th Red: Finish Line!"); stop_robot_motion(); current_state = RobotState.MISSION_COMPLETE; return True
#         return False
#     elif is_yellow:
#         robot_is_on_red_patch=False
#         if current_state==RobotState.LINE_TRACING and not school_zone_action_completed_this_segment:
#             if in_high_speed_mode:in_high_speed_mode=False;run_motor.run(DRIVE_SPEED_LINETRACE);wait(50)
            
#             ev3.screen.clear();ev3.screen.print("School Zone!"); stop_robot_motion()
#             activate_flashing_lights()
            
#             ev3.screen.print("Phase 1: 7s Normal Speed")
#             post_yellow_mode = "NORMAL"
#             post_yellow_timer.reset()
#             school_zone_action_completed_this_segment = True
            
#             start_line_tracing_motion(use_max_speed=False)
#             return True
#         return False
#     else:robot_is_on_red_patch=False;return False

# def scan_and_select_parking_spot():
#     global parking_spot_selected
#     if ultrasonic_sensor is None: ev3.screen.print("Ultrasonic Error!"); return None
#     ev3.screen.clear();ev3.screen.print("Scanning Parking...");dist1=ultrasonic_sensor.distance();wait(1000)
#     run_motor.run_angle(PARKING_DRIVE_SPEED,DIST_BETWEEN_SCAN_SPOTS_MOTOR_DEG,Stop.BRAKE,True);wait(500)
#     dist2=ultrasonic_sensor.distance();wait(1000)
#     good1=dist1>MIN_OPEN_SPACE_FOR_PARKING_MM;good2=dist2>MIN_OPEN_SPACE_FOR_PARKING_MM;name=None
#     if good1 and(not good2 or dist1>=dist2):ev3.screen.print("Select: Spot 1");run_motor.run_angle(-PARKING_DRIVE_SPEED,DIST_BETWEEN_SCAN_SPOTS_MOTOR_DEG,Stop.BRAKE,True);wait(500);name="Spot 1"
#     elif good2:ev3.screen.print("Select: Spot 2");name="Spot 2"
#     else:ev3.screen.print("No Suitable Spot.")
#     parking_spot_selected=name;return name is not None
# def perform_parallel_parking():
#     global parking_attempted_this_lap
#     if parking_spot_selected is None:ev3.screen.print("Parking Cancel (No Spot)");parking_attempted_this_lap=True;return False
#     ev3.screen.clear();ev3.screen.print("Parking: "+str(parking_spot_selected));wait(500);sound_parking_horn();wait(300)
#     activate_flashing_lights();wait(200);run_motor.run_angle(PARKING_DRIVE_SPEED,DRIVE_PAST_SPOT_MOTOR_DEG,Stop.BRAKE,True)
#     steering_motor.run_target(PARKING_STEERING_SPEED,STEERING_ANGLE_INTO_SPOT_MOTOR_DEG,Stop.HOLD,True);activate_flashing_lights();wait(200)
#     run_motor.run_angle(-PARKING_DRIVE_SPEED,REVERSE_DIAGONAL_MOTOR_DEG,Stop.BRAKE,True)
#     steering_motor.run_target(PARKING_STEERING_SPEED,STEERING_ANGLE_STRAIGHTEN_MOTOR_DEG,Stop.HOLD,True);activate_flashing_lights();wait(200)
#     run_motor.run_angle(-PARKING_DRIVE_SPEED,REVERSE_STRAIGHT_MOTOR_DEG,Stop.BRAKE,True)
#     steering_motor.run_target(PARKING_STEERING_SPEED,0,Stop.HOLD,True);ev3.light.off();stop_robot_motion()
#     ev3.screen.clear();ev3.screen.print("Parking Complete!");parking_attempted_this_lap=True;return True

# # --- 메인 프로그램 ---
# try:
#     while True:
#         pressed_buttons_this_loop = ev3.buttons.pressed()

#         if current_state == RobotState.INITIALIZING:
#             start_origin_calibration()
#             ev3.screen.clear(); ev3.screen.print("Wheels Aligning...")
#             steering_motor.run_target(PARKING_STEERING_SPEED, 0, then=Stop.HOLD, wait=True); wait(500)
#             current_state = RobotState.WAITING_TO_START

#         elif current_state == RobotState.WAITING_TO_START:
#             ev3.screen.clear(); ev3.screen.print("Press Center Button"); ev3.screen.print("to Start Mission.")
#             while not Button.CENTER in ev3.buttons.pressed(): wait(50)
#             ev3.screen.clear(); ev3.screen.print("Center Pressed! Starting..."); ev3.speaker.beep(); wait(500) 
#             start_line_tracing_motion(); current_state = RobotState.LINE_TRACING

#         elif current_state == RobotState.LINE_TRACING:
#             if handle_color_detections(): 
#                 continue 
            
#             if second_red_boost_active:
#                 if second_red_boost_timer.time() > SECOND_RED_BOOST_DURATION_MS:
#                     ev3.screen.clear(); ev3.screen.print("2nd Red Boost End.")
#                     second_red_boost_active = False
#                     in_high_speed_mode = False
#                     run_motor.run(DRIVE_SPEED_LINETRACE)
#                     wait(500)
            
#             if fourth_red_boost_active:
#                 if fourth_red_boost_timer.time() > FOURTH_RED_BOOST_DURATION_MS:
#                     ev3.screen.clear(); ev3.screen.print("4th Red Boost End.")
#                     fourth_red_boost_active = False
#                     in_high_speed_mode = False
#                     run_motor.run(DRIVE_SPEED_LINETRACE)
#                     wait(500)

#             if post_yellow_mode == "NORMAL":
#                 if post_yellow_timer.time() > 7000:
#                     ev3.screen.clear(); ev3.screen.print("Phase 2: 3s High Speed")
#                     post_yellow_mode = "BOOST"
#                     in_high_speed_mode = True
#                     run_motor.run(DRIVE_SPEED_MAX_LINETRACE)
#                     post_yellow_timer.reset()
#             elif post_yellow_mode == "BOOST":
#                 if post_yellow_timer.time() > 3000:
#                     ev3.screen.clear(); ev3.screen.print("School Zone clear.")
#                     ev3.screen.print("Resuming normal trace.")
#                     post_yellow_mode = "NONE"
#                     in_high_speed_mode = False
#                     run_motor.run(DRIVE_SPEED_LINETRACE)
#                     wait(500)
            
#             if green_light_sim_received and not parking_attempted_this_lap:
#                 if PARKING_MODE_ACTIVATE_BUTTON in pressed_buttons_this_loop:
#                     if in_high_speed_mode or second_red_boost_active or fourth_red_boost_active:
#                         in_high_speed_mode=False; second_red_boost_active=False; fourth_red_boost_active=False
#                         run_motor.run(DRIVE_SPEED_LINETRACE)
#                     ev3.screen.clear();ev3.screen.print("Parking Mode Activated");stop_robot_motion();current_state=RobotState.PARKING_SCAN_PROMPT;continue
#             try:
#                 hap=list(lsa.ReadRaw_Calibrated())
#                 if hap and len(hap)>=8:
#                     left_dark=sum(100-r_val for r_val in hap[0:3])/3;right_dark=sum(100-r_val for r_val in hap[5:8])/3
#                     error=left_dark-right_dark;current_kp=STEERING_KP_LINETRACE*HIGH_SPEED_KP_FACTOR if in_high_speed_mode else STEERING_KP_LINETRACE
#                     correction=current_kp*error;max_correction=90;correction=max(-max_correction,min(max_correction,correction))
#                     steering_motor.run_target(STEERING_MOTOR_SPEED_LINETRACE,correction,Stop.HOLD,False)
#                 else:pass
#             except Exception as e:pass

#         elif current_state == RobotState.TRAFFIC_LIGHT_WAITING:
#             signal = read_openmv_signal()
#             if signal == "two":
#                 ev3.screen.clear(); ev3.screen.print("OMV: Green! Go.")
#                 beep_custom(880, 200); wait(200)
#                 green_light_sim_received = True
                
#                 start_line_tracing_motion(use_max_speed=False)
                
#                 current_state = RobotState.LINE_TRACING
#             elif signal == "one": ev3.screen.print("OMV: Red. Still Waiting...")
#             elif signal == "three": ev3.screen.print("OMV: Yellow. Still Waiting...")
#             elif signal: ev3.screen.print("OMV Sig: " + signal)
#             wait(100)

#         elif current_state == RobotState.PARKING_SCAN_PROMPT:
#             ev3.screen.clear(); ev3.screen.print("Parking: Align, then Down btn")
#             scan_trigger_pressed = False
#             while not scan_trigger_pressed:
#                 if PARKING_SCAN_START_BUTTON in ev3.buttons.pressed():
#                     ev3.screen.print("Down Button for Scan!"); ev3.speaker.beep(); wait(500); scan_trigger_pressed = True
#                 wait(50)
#             current_state = RobotState.PARKING_SCANNING
#         elif current_state == RobotState.PARKING_SCANNING:
#             if scan_and_select_parking_spot(): current_state = RobotState.PARKING_EXECUTING
#             else: 
#                 ev3.screen.print("No spot. Resume trace."); wait(1000)
#                 parking_attempted_this_lap = True; start_line_tracing_motion(); current_state = RobotState.LINE_TRACING
        
#         elif current_state == RobotState.PARKING_EXECUTING:
#             if perform_parallel_parking():
#                 ev3.screen.clear()
#                 ev3.screen.print("Parking successful!")
#                 ev3.screen.print("Resuming line tracing...")
#                 wait(2000)
#                 start_line_tracing_motion()
#                 current_state = RobotState.LINE_TRACING
#             else:
#                 ev3.screen.print("Parking failed/canceled."); wait(1000)
#                 start_line_tracing_motion(); current_state = RobotState.LINE_TRACING
        
#         elif current_state == RobotState.MISSION_COMPLETE:
#             ev3.screen.clear(); ev3.screen.print("Mission Complete!")
#             play_completion_tune() 
#             stop_robot_motion(); ev3.light.off(); wait(3000); break
        
#         elif current_state == RobotState.ERROR:
#             ev3.screen.clear(); ev3.screen.print("Error Occurred!"); stop_robot_motion(); ev3.light.on(Color.RED); wait(5000); break
#         wait(20)
# except KeyboardInterrupt:
#     ev3.screen.clear(); ev3.screen.print("Program Stopped by User.")
# except Exception as e:
#     ev3.screen.clear(); ev3.screen.print("Runtime Error!"); print("Runtime Error:", e); current_state = RobotState.ERROR
# finally:
#     stop_robot_motion(); ev3.light.off(); ev3.screen.clear()
#     if current_state != RobotState.MISSION_COMPLETE and current_state != RobotState.ERROR: ev3.screen.print("Program Ended.")
#     elif current_state == RobotState.ERROR: ev3.screen.print("Ended due to Error.")
#     wait(2000)




# #!/usr/bin/env pybricks-micropython

# # 버전 1.44 (제공된 OpenMV 데이터 읽기 함수 적용)

# from pybricks.hubs import EV3Brick
# from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
#                                  InfraredSensor, UltrasonicSensor, GyroSensor)
# from pybricks.parameters import Port, Stop, Direction, Button, Color
# from pybricks.tools import wait, StopWatch, DataLog
# from pybricks.robotics import DriveBase
# from mindsensorsPYB import LSA
# from pybricks.iodevices import UARTDevice # UART 통신을 위해 추가

# import os
# import sys
# import time

# # --- 설정 상수 ---
# SPEAKER_MUTED = False

# # EV3 브릭 및 장치 초기화
# ev3 = EV3Brick()
# try:
#     lsa = LSA(Port.S1, 0x14) # LSA 포트 및 주소 확인
# except Exception as e:
#     ev3.screen.print("LSA Init Error"); wait(2000); exit()

# run_motor = Motor(Port.A) # 주행 모터 포트 확인
# steering_motor = Motor(Port.D) # 조향 모터 포트 확인

# try:
#     color_sensor = ColorSensor(Port.S4) # 컬러 센서 포트 확인
# except Exception as e: # 컬러센서가 S4에 없으면 OpenMV용으로 UART를 S4에 연결 시도
#     ev3.screen.print("ColorSensor S4 Err")
#     wait(1000)
#     # 이 경우 컬러센서가 다른 포트에 있거나 사용하지 않는다고 가정.
#     # 만약 컬러센서도 필요하다면 포트 충돌을 해결해야 함.

# try:
#     # OpenMV는 Port.S4에 연결되었다고 가정 (컬러센서와 같은 포트 사용 시 주의)
#     # 만약 컬러센서가 Port.S4를 사용한다면, OpenMV는 다른 포트(예: Port.S3)를 사용해야 합니다.
#     # 여기서는 일단 Port.S4로 시도하고, 컬러센서와 충돌 시 메시지를 남깁니다.
#     openmv_uart = UARTDevice(Port.S3, baudrate=115200, timeout=10) # timeout 추가 (non-blocking 읽기 시 도움)
#     ev3.screen.print("OpenMV UART on S4 OK")
#     wait(500)
# except Exception as e:
#     ev3.screen.clear()
#     ev3.screen.print("OpenMV UART Error:")
#     error_message = str(e)
#     max_chars_per_line = 18
#     for i in range(0, len(error_message), max_chars_per_line):
#         ev3.screen.print(error_message[i:i+max_chars_per_line])
#     ev3.speaker.beep(); wait(5000); # OpenMV 연결 실패 시 계속 진행할지 결정 필요
#     openmv_uart = None # 오류 발생 시 None으로 설정하여 이후 코드에서 확인 가능

# try:
#     ultrasonic_sensor = UltrasonicSensor(Port.S2) # 초음파 센서 포트 확인
# except Exception as e:
#     ev3.screen.print("Ultrasonic S2 Err"); wait(2000)
#     ultrasonic_sensor = None


# # --- RGB 색상 판단 임계값 ---
# RED_R_MIN = 12; RED_R_MAX = 22; RED_G_MAX = 8; RED_B_MAX = 10
# YELLOW_R_MIN = 16; YELLOW_R_MAX = 28; YELLOW_G_MIN = 19; YELLOW_G_MAX = 33; YELLOW_B_MAX = 18
# # -------------------------------------------------------------------------

# # 라인 트레이싱 매개변수
# DRIVE_SPEED_LINETRACE = 200
# DRIVE_SPEED_MAX_LINETRACE = 2000
# STEERING_KP_LINETRACE = -7.0
# HIGH_SPEED_KP_FACTOR = 0.1 
# STEERING_MOTOR_SPEED_LINETRACE = 2500

# # (주차, 점멸등, 경적, 스쿨존 상수는 이전과 동일)
# PARKING_DRIVE_SPEED = 80; PARKING_STEERING_SPEED = 300
# MIN_OPEN_SPACE_FOR_PARKING_MM = 350
# DIST_BETWEEN_SCAN_SPOTS_MOTOR_DEG = 360 * 2.2
# DRIVE_PAST_SPOT_MOTOR_DEG = 180
# STEERING_ANGLE_INTO_SPOT_MOTOR_DEG = 70
# REVERSE_DIAGONAL_MOTOR_DEG = 360 * 1.1
# STEERING_ANGLE_STRAIGHTEN_MOTOR_DEG = -80
# REVERSE_STRAIGHT_MOTOR_DEG = 360 * 0.7
# FLASHING_LIGHT_COLORS = [Color.RED, Color.YELLOW, Color.GREEN]
# FLASHING_LIGHT_INTERVAL_MS = 1000
# HORN_FREQUENCY = 600; HORN_DURATION_MS = 150
# RED_LINE_BEEP_FREQUENCY = 440; RED_LINE_BEEP_DURATION = 100
# # GREEN_LIGHT_BUTTON, RED_LIGHT_BUTTON_ANY_OTHER는 OpenMV 신호로 대체됨
# PARKING_MODE_ACTIVATE_BUTTON = Button.UP
# PARKING_SCAN_START_BUTTON = Button.DOWN
# SCHOOL_ZONE_CLEAR_DRIVE_DEG = 360 * 0.1 
# SCHOOL_ZONE_CLEAR_DRIVE_SPEED = 50    

# # --- 로봇 상태 정의 ---
# class RobotState:
#     INITIALIZING = "Initializing"
#     WAITING_TO_START = "Waiting to Start"
#     LINE_TRACING = "Line Tracing"
#     SCHOOL_ZONE_ACTION = "School Zone Action"
#     TRAFFIC_LIGHT_WAITING = "Traffic Light Waiting"
#     PARKING_SCAN_PROMPT = "Parking Scan Prompt"
#     PARKING_SCANNING = "Parking Scanning"
#     PARKING_EXECUTING = "Parking Executing"
#     MISSION_COMPLETE = "Mission Complete"
#     ERROR = "Error"

# # --- 전역 변수 ---
# current_state = RobotState.INITIALIZING
# red_line_count = 0; robot_is_on_red_patch = False
# third_red_line_detected_this_lap = False; green_light_sim_received = False
# parking_spot_selected = None; parking_attempted_this_lap = False
# school_zone_action_completed_this_segment = False; in_high_speed_mode = False

# # --- 도우미 함수 ---
# def speak(text): pass
# def beep_custom(f=HORN_FREQUENCY, d=HORN_DURATION_MS):
#     if not SPEAKER_MUTED: ev3.speaker.beep(f, d)
# def start_origin_calibration():
#     ev3.screen.clear(); ev3.screen.print("Origin Calibrating...")
#     steering_motor.run(100)
#     while True:
#         a = steering_motor.angle(); wait(100); b = steering_motor.angle()
#         if a == b: break
#     steering_motor.stop()
#     steering_motor.run_angle(-100, 100, then=Stop.HOLD, wait=True)
#     steering_motor.reset_angle(0)
#     steering_motor.run_target(PARKING_STEERING_SPEED, 0, then=Stop.HOLD, wait=True)
#     ev3.screen.clear(); ev3.screen.print("Origin Calibrated!"); wait(500)
# def stop_robot_motion(): run_motor.brake(); steering_motor.brake()
# def start_line_tracing_motion(use_max_speed=False):
#     ev3.screen.clear()
#     speed = DRIVE_SPEED_MAX_LINETRACE if use_max_speed else DRIVE_SPEED_LINETRACE
#     msg = "Line Tracing Start (MAX)" if use_max_speed else "Line Tracing Start..."
#     ev3.screen.print(msg); run_motor.run(speed)
# def activate_flashing_lights():
#     for c in FLASHING_LIGHT_COLORS: ev3.light.on(c); wait(FLASHING_LIGHT_INTERVAL_MS)
#     ev3.light.off()
# def sound_parking_horn():
#     for _ in range(2): beep_custom(); wait(HORN_DURATION_MS // 2)

# # --- OpenMV 신호 읽기 함수 (제공된 함수 사용) ---
# def read_openmv_signal():
#     if openmv_uart is None: # UART 초기화 실패 시
#         # ev3.screen.print("OMV UART not init") # 너무 자주 출력될 수 있음
#         return "" # 빈 문자열 또는 None 반환

#     try:
#         # UART 버퍼에 데이터가 있는지 확인 (non-blocking)
#         if openmv_uart.waiting() > 0:
#             data = openmv_uart.read_all() # 모든 대기 중인 데이터 읽기
#             return data.decode().strip()  # 디코딩 및 공백 제거
#         else:
#             return "" # 읽을 데이터 없음
#     except Exception as e:
#         # ev3.screen.print("OMV Read Error") # 디버깅 시 사용
#         # print("OMV Read Exception:", e)
#         return "" # 오류 발생 시 빈 문자열 반환

# # --- 핵심 로직 함수 ---
# def handle_color_detections():
#     global red_line_count,robot_is_on_red_patch,third_red_line_detected_this_lap,current_state, school_zone_action_completed_this_segment,in_high_speed_mode
#     # 컬러 센서 사용 가능 여부 확인
#     if color_sensor is None: return False 

#     r,g,b=color_sensor.rgb()
#     is_red=(r>=RED_R_MIN and r<=RED_R_MAX and g<=RED_G_MAX and b<=RED_B_MAX and r>(g+3)and r>(b+3))
#     is_yellow=(r>=YELLOW_R_MIN and r<=YELLOW_R_MAX and g>=YELLOW_G_MIN and g<=YELLOW_G_MAX and b<=YELLOW_B_MAX and g>r and g>(b+5)and r>(b+3))
#     if is_red:
#         school_zone_action_completed_this_segment=False
#         if not robot_is_on_red_patch:
#             red_line_count+=1;robot_is_on_red_patch=True
#             if not SPEAKER_MUTED:ev3.speaker.beep(RED_LINE_BEEP_FREQUENCY,RED_LINE_BEEP_DURATION)
#             if red_line_count==2 and not third_red_line_detected_this_lap:
#                 ev3.screen.clear();ev3.screen.print("2nd Red: High Speed!");in_high_speed_mode=True;run_motor.run(DRIVE_SPEED_MAX_LINETRACE);return False
#             elif red_line_count==3 and not third_red_line_detected_this_lap:
#                 third_red_line_detected_this_lap=True;in_high_speed_mode=False
#                 ev3.screen.clear();ev3.screen.print("3rd Red: Stop!");ev3.screen.print("Wait OMV Signal");stop_robot_motion();current_state=RobotState.TRAFFIC_LIGHT_WAITING;return True
#         return False
#     elif is_yellow:
#         robot_is_on_red_patch=False
#         if current_state==RobotState.LINE_TRACING and not school_zone_action_completed_this_segment:
#             if in_high_speed_mode:in_high_speed_mode=False;run_motor.run(DRIVE_SPEED_LINETRACE);wait(50)
#             ev3.screen.clear();ev3.screen.print("School Zone!");stop_robot_motion();current_state=RobotState.SCHOOL_ZONE_ACTION;return True
#         return False
#     else:robot_is_on_red_patch=False;return False

# # (scan_and_select_parking_spot, perform_parallel_parking 함수는 이전과 동일)
# def scan_and_select_parking_spot():
#     global parking_spot_selected
#     if ultrasonic_sensor is None: ev3.screen.print("Ultrasonic Error!"); return None
#     ev3.screen.clear();ev3.screen.print("Scanning Parking...");dist1=ultrasonic_sensor.distance();wait(1000)
#     run_motor.run_angle(PARKING_DRIVE_SPEED,DIST_BETWEEN_SCAN_SPOTS_MOTOR_DEG,Stop.BRAKE,True);wait(500)
#     dist2=ultrasonic_sensor.distance();wait(1000)
#     good1=dist1>MIN_OPEN_SPACE_FOR_PARKING_MM;good2=dist2>MIN_OPEN_SPACE_FOR_PARKING_MM;name=None
#     if good1 and(not good2 or dist1>=dist2):ev3.screen.print("Select: Spot 1");run_motor.run_angle(-PARKING_DRIVE_SPEED,DIST_BETWEEN_SCAN_SPOTS_MOTOR_DEG,Stop.BRAKE,True);wait(500);name="Spot 1"
#     elif good2:ev3.screen.print("Select: Spot 2");name="Spot 2"
#     else:ev3.screen.print("No Suitable Spot.")
#     parking_spot_selected=name;return name is not None
# def perform_parallel_parking():
#     global parking_attempted_this_lap
#     if parking_spot_selected is None:ev3.screen.print("Parking Cancel (No Spot)");parking_attempted_this_lap=True;return False
#     ev3.screen.clear();ev3.screen.print("Parking: "+str(parking_spot_selected));wait(500);sound_parking_horn();wait(300)
#     activate_flashing_lights();wait(200);run_motor.run_angle(PARKING_DRIVE_SPEED,DRIVE_PAST_SPOT_MOTOR_DEG,Stop.BRAKE,True)
#     steering_motor.run_target(PARKING_STEERING_SPEED,STEERING_ANGLE_INTO_SPOT_MOTOR_DEG,Stop.HOLD,True);activate_flashing_lights();wait(200)
#     run_motor.run_angle(-PARKING_DRIVE_SPEED,REVERSE_DIAGONAL_MOTOR_DEG,Stop.BRAKE,True)
#     steering_motor.run_target(PARKING_STEERING_SPEED,STEERING_ANGLE_STRAIGHTEN_MOTOR_DEG,Stop.HOLD,True);activate_flashing_lights();wait(200)
#     run_motor.run_angle(-PARKING_DRIVE_SPEED,REVERSE_STRAIGHT_MOTOR_DEG,Stop.BRAKE,True)
#     steering_motor.run_target(PARKING_STEERING_SPEED,0,Stop.HOLD,True);ev3.light.off();stop_robot_motion()
#     ev3.screen.clear();ev3.screen.print("Parking Complete!");parking_attempted_this_lap=True;return True

# # --- 메인 프로그램 ---
# try:
#     while True:
#         pressed_buttons_this_loop = ev3.buttons.pressed()

#         if current_state == RobotState.INITIALIZING:
#             start_origin_calibration()
#             ev3.screen.clear(); ev3.screen.print("Wheels Aligning...")
#             steering_motor.run_target(PARKING_STEERING_SPEED, 0, then=Stop.HOLD, wait=True); wait(500)
#             current_state = RobotState.WAITING_TO_START

#         elif current_state == RobotState.WAITING_TO_START:
#             ev3.screen.clear(); ev3.screen.print("Press Center Button"); ev3.screen.print("to Start Mission.")
#             while not Button.CENTER in ev3.buttons.pressed(): wait(50) # 직접 읽음
#             ev3.screen.clear(); ev3.screen.print("Center Pressed! Starting..."); ev3.speaker.beep(); wait(500) 
#             start_line_tracing_motion(); current_state = RobotState.LINE_TRACING

#         elif current_state == RobotState.LINE_TRACING:
#             if handle_color_detections(): 
#                 continue 
#             if green_light_sim_received and not parking_attempted_this_lap:
#                 if PARKING_MODE_ACTIVATE_BUTTON in pressed_buttons_this_loop:
#                     if in_high_speed_mode:in_high_speed_mode=False;run_motor.run(DRIVE_SPEED_LINETRACE)
#                     ev3.screen.clear();ev3.screen.print("Parking Mode Activated");stop_robot_motion();current_state=RobotState.PARKING_SCAN_PROMPT;continue
#             try:
#                 hap=list(lsa.ReadRaw_Calibrated())
#                 if hap and len(hap)>=8:
#                     left_dark=sum(100-r_val for r_val in hap[0:3])/3;right_dark=sum(100-r_val for r_val in hap[5:8])/3
#                     error=left_dark-right_dark;current_kp=STEERING_KP_LINETRACE*HIGH_SPEED_KP_FACTOR if in_high_speed_mode else STEERING_KP_LINETRACE
#                     correction=current_kp*error;max_correction=90;correction=max(-max_correction,min(max_correction,correction))
#                     steering_motor.run_target(STEERING_MOTOR_SPEED_LINETRACE,correction,Stop.HOLD,False)
#                 else:pass
#             except Exception as e:pass

#         elif current_state == RobotState.SCHOOL_ZONE_ACTION:
#             ev3.screen.clear();ev3.screen.print("School Zone Action...");activate_flashing_lights()
#             ev3.screen.print("Clearing yellow zone...");steering_motor.run_target(STEERING_MOTOR_SPEED_LINETRACE,0,Stop.HOLD,True)
#             run_motor.run_angle(SCHOOL_ZONE_CLEAR_DRIVE_SPEED,SCHOOL_ZONE_CLEAR_DRIVE_DEG,Stop.BRAKE,True)
#             school_zone_action_completed_this_segment=True;ev3.screen.print("Re-aligning wheels...")
#             steering_motor.run_target(PARKING_STEERING_SPEED,0,Stop.HOLD,True);wait(200)
#             ev3.screen.print("Resuming line trace.");start_line_tracing_motion();current_state=RobotState.LINE_TRACING

#         elif current_state == RobotState.TRAFFIC_LIGHT_WAITING: # OpenMV 신호 대기
#             ev3.screen.clear(); ev3.screen.print("3rd Red: Waiting OMV Signal")
#             # ev3.screen.print("(Use EV3 Btns for Sim)") # 테스트용 안내
            
#             signal = read_openmv_signal() # OpenMV로부터 신호 읽기

#             if signal == "two": # OpenMV가 "two"를 보내면 초록불로 간주
#                 ev3.screen.clear(); ev3.screen.print("OMV: Green detected! Go!")
#                 beep_custom(880, 200); wait(500)
#                 green_light_sim_received = True 
#                 start_line_tracing_motion(); current_state = RobotState.LINE_TRACING
#             elif signal == "one": # 예시: 빨간불 신호
#                 ev3.screen.print("OMV: Red. Still Waiting...")
#                 # wait(100) # 너무 자주 화면 바꾸지 않도록
#             elif signal == "three": # 예시: 노란불 신호
#                 ev3.screen.print("OMV: Yellow. Still Waiting...")
#                 # wait(100)
#             elif signal: # 빈 문자열이 아닌 다른 알 수 없는 신호 수신 시
#                 ev3.screen.print("OMV Sig: " + signal) # 수신된 신호 표시
#                 # wait(100)
#             # else: 신호가 없으면 ("") 계속 대기 (아무것도 안 함)
            
#             wait(100) # OpenMV 신호 폴링 간격

#         # (이하 PARKING_SCAN_PROMPT 등 다른 상태들은 이전과 동일)
#         elif current_state == RobotState.PARKING_SCAN_PROMPT:
#             ev3.screen.clear(); ev3.screen.print("Parking: Align, then Down btn")
#             scan_trigger_pressed = False
#             while not scan_trigger_pressed:
#                 current_scan_buttons = ev3.buttons.pressed()
#                 if PARKING_SCAN_START_BUTTON in current_scan_buttons:
#                     ev3.screen.print("Down Button for Scan!"); ev3.speaker.beep(); wait(500); scan_trigger_pressed = True
#                 wait(50)
#             current_state = RobotState.PARKING_SCANNING
#         elif current_state == RobotState.PARKING_SCANNING:
#             if scan_and_select_parking_spot(): current_state = RobotState.PARKING_EXECUTING
#             else: 
#                 ev3.screen.print("No spot. Resume trace."); wait(1000)
#                 parking_attempted_this_lap = True; start_line_tracing_motion(); current_state = RobotState.LINE_TRACING
#         elif current_state == RobotState.PARKING_EXECUTING:
#             if perform_parallel_parking(): current_state = RobotState.MISSION_COMPLETE
#             else:
#                 ev3.screen.print("Parking failed/canceled."); wait(1000)
#                 start_line_tracing_motion(); current_state = RobotState.LINE_TRACING
#         elif current_state == RobotState.MISSION_COMPLETE:
#             ev3.screen.clear(); ev3.screen.print("Mission Complete!"); stop_robot_motion(); ev3.light.off(); wait(5000); break
#         elif current_state == RobotState.ERROR:
#             ev3.screen.clear(); ev3.screen.print("Error Occurred!"); stop_robot_motion(); ev3.light.on(Color.RED); wait(5000); break
#         wait(20)
# except KeyboardInterrupt:
#     ev3.screen.clear(); ev3.screen.print("Program Stopped by User.")
# except Exception as e:
#     ev3.screen.clear(); ev3.screen.print("Runtime Error!"); print("Runtime Error:", e); current_state = RobotState.ERROR
# finally:
#     stop_robot_motion(); ev3.light.off(); ev3.screen.clear()
#     if current_state != RobotState.MISSION_COMPLETE and current_state != RobotState.ERROR: ev3.screen.print("Program Ended.")
#     elif current_state == RobotState.ERROR: ev3.screen.print("Ended due to Error.")
#     wait(2000)













































# #!/usr/bin/env pybricks-micropython

# # 버전 1.45 (첫 번째 빨간선부터 고속 주행 적용)

# from pybricks.hubs import EV3Brick
# from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
#                                  InfraredSensor, UltrasonicSensor, GyroSensor)
# from pybricks.parameters import Port, Stop, Direction, Button, Color
# from pybricks.tools import wait, StopWatch, DataLog
# from pybricks.robotics import DriveBase
# from mindsensorsPYB import LSA
# from pybricks.iodevices import UARTDevice # UART 통신을 위해 추가

# import os
# import sys
# import time

# # --- 설정 상수 ---
# SPEAKER_MUTED = False

# # EV3 브릭 및 장치 초기화
# ev3 = EV3Brick()
# try:
#     lsa = LSA(Port.S1, 0x14) # LSA 포트 및 주소 확인
# except Exception as e:
#     ev3.screen.print("LSA Init Error"); wait(2000); exit()

# run_motor = Motor(Port.A) # 주행 모터 포트 확인
# steering_motor = Motor(Port.D) # 조향 모터 포트 확인

# try:
#     color_sensor = ColorSensor(Port.S4) # 컬러 센서 포트 확인
# except Exception as e: # 컬러센서가 S4에 없으면 OpenMV용으로 UART를 S4에 연결 시도
#     ev3.screen.print("ColorSensor S4 Err")
#     wait(1000)
#     # 이 경우 컬러센서가 다른 포트에 있거나 사용하지 않는다고 가정.
#     # 만약 컬러센서도 필요하다면 포트 충돌을 해결해야 함.

# try:
#     # OpenMV는 Port.S3에 연결되었다고 가정
#     openmv_uart = UARTDevice(Port.S3, baudrate=115200, timeout=10) # timeout 추가 (non-blocking 읽기 시 도움)
#     ev3.screen.print("OpenMV UART on S3 OK") # 포트 번호 수정
#     wait(500)
# except Exception as e:
#     ev3.screen.clear()
#     ev3.screen.print("OpenMV UART Error:")
#     error_message = str(e)
#     max_chars_per_line = 18
#     for i in range(0, len(error_message), max_chars_per_line):
#         ev3.screen.print(error_message[i:i+max_chars_per_line])
#     ev3.speaker.beep(); wait(5000); # OpenMV 연결 실패 시 계속 진행할지 결정 필요
#     openmv_uart = None # 오류 발생 시 None으로 설정하여 이후 코드에서 확인 가능

# try:
#     ultrasonic_sensor = UltrasonicSensor(Port.S2) # 초음파 센서 포트 확인
# except Exception as e:
#     ev3.screen.print("Ultrasonic S2 Err"); wait(2000)
#     ultrasonic_sensor = None


# # --- RGB 색상 판단 임계값 ---
# RED_R_MIN = 12; RED_R_MAX = 22; RED_G_MAX = 8; RED_B_MAX = 10
# YELLOW_R_MIN = 16; YELLOW_R_MAX = 28; YELLOW_G_MIN = 19; YELLOW_G_MAX = 33; YELLOW_B_MAX = 18
# # -------------------------------------------------------------------------

# # 라인 트레이싱 매개변수
# DRIVE_SPEED_LINETRACE = 200
# DRIVE_SPEED_MAX_LINETRACE = 2000
# STEERING_KP_LINETRACE = -7.0
# HIGH_SPEED_KP_FACTOR = 0.1 
# STEERING_MOTOR_SPEED_LINETRACE = 2500

# # (주차, 점멸등, 경적, 스쿨존 상수는 이전과 동일)
# PARKING_DRIVE_SPEED = 80; PARKING_STEERING_SPEED = 300
# MIN_OPEN_SPACE_FOR_PARKING_MM = 350
# DIST_BETWEEN_SCAN_SPOTS_MOTOR_DEG = 360 * 2.2
# DRIVE_PAST_SPOT_MOTOR_DEG = 180
# STEERING_ANGLE_INTO_SPOT_MOTOR_DEG = 70
# REVERSE_DIAGONAL_MOTOR_DEG = 360 * 1.1
# STEERING_ANGLE_STRAIGHTEN_MOTOR_DEG = -80
# REVERSE_STRAIGHT_MOTOR_DEG = 360 * 0.7
# FLASHING_LIGHT_COLORS = [Color.RED, Color.YELLOW, Color.GREEN]
# FLASHING_LIGHT_INTERVAL_MS = 1000
# HORN_FREQUENCY = 600; HORN_DURATION_MS = 150
# RED_LINE_BEEP_FREQUENCY = 440; RED_LINE_BEEP_DURATION = 100
# # GREEN_LIGHT_BUTTON, RED_LIGHT_BUTTON_ANY_OTHER는 OpenMV 신호로 대체됨
# PARKING_MODE_ACTIVATE_BUTTON = Button.UP
# PARKING_SCAN_START_BUTTON = Button.DOWN
# SCHOOL_ZONE_CLEAR_DRIVE_DEG = 360 * 0.1 
# SCHOOL_ZONE_CLEAR_DRIVE_SPEED = 50    

# # --- 로봇 상태 정의 ---
# class RobotState:
#     INITIALIZING = "Initializing"
#     WAITING_TO_START = "Waiting to Start"
#     LINE_TRACING = "Line Tracing"
#     SCHOOL_ZONE_ACTION = "School Zone Action"
#     TRAFFIC_LIGHT_WAITING = "Traffic Light Waiting"
#     PARKING_SCAN_PROMPT = "Parking Scan Prompt"
#     PARKING_SCANNING = "Parking Scanning"
#     PARKING_EXECUTING = "Parking Executing"
#     MISSION_COMPLETE = "Mission Complete"
#     ERROR = "Error"

# # --- 전역 변수 ---
# current_state = RobotState.INITIALIZING
# red_line_count = 0; robot_is_on_red_patch = False
# third_red_line_detected_this_lap = False; green_light_sim_received = False
# parking_spot_selected = None; parking_attempted_this_lap = False
# school_zone_action_completed_this_segment = False; in_high_speed_mode = False

# # --- 도우미 함수 ---
# def speak(text): pass
# def beep_custom(f=HORN_FREQUENCY, d=HORN_DURATION_MS):
#     if not SPEAKER_MUTED: ev3.speaker.beep(f, d)
# def start_origin_calibration():
#     ev3.screen.clear(); ev3.screen.print("Origin Calibrating...")
#     steering_motor.run(100)
#     while True:
#         a = steering_motor.angle(); wait(100); b = steering_motor.angle()
#         if a == b: break
#     steering_motor.stop()
#     steering_motor.run_angle(-100, 100, then=Stop.HOLD, wait=True)
#     steering_motor.reset_angle(0)
#     steering_motor.run_target(PARKING_STEERING_SPEED, 0, then=Stop.HOLD, wait=True)
#     ev3.screen.clear(); ev3.screen.print("Origin Calibrated!"); wait(500)
# def stop_robot_motion(): run_motor.brake(); steering_motor.brake()
# def start_line_tracing_motion(use_max_speed=False):
#     ev3.screen.clear()
#     speed = DRIVE_SPEED_MAX_LINETRACE if use_max_speed else DRIVE_SPEED_LINETRACE
#     msg = "Line Tracing Start (MAX)" if use_max_speed else "Line Tracing Start..."
#     ev3.screen.print(msg); run_motor.run(speed)
# def activate_flashing_lights():
#     for c in FLASHING_LIGHT_COLORS: ev3.light.on(c); wait(FLASHING_LIGHT_INTERVAL_MS)
#     ev3.light.off()
# def sound_parking_horn():
#     for _ in range(2): beep_custom(); wait(HORN_DURATION_MS // 2)

# # --- OpenMV 신호 읽기 함수 (제공된 함수 사용) ---
# def read_openmv_signal():
#     if openmv_uart is None: # UART 초기화 실패 시
#         # ev3.screen.print("OMV UART not init") # 너무 자주 출력될 수 있음
#         return "" # 빈 문자열 또는 None 반환

#     try:
#         # UART 버퍼에 데이터가 있는지 확인 (non-blocking)
#         if openmv_uart.waiting() > 0:
#             data = openmv_uart.read_all() # 모든 대기 중인 데이터 읽기
#             return data.decode().strip()  # 디코딩 및 공백 제거
#         else:
#             return "" # 읽을 데이터 없음
#     except Exception as e:
#         # ev3.screen.print("OMV Read Error") # 디버깅 시 사용
#         # print("OMV Read Exception:", e)
#         return "" # 오류 발생 시 빈 문자열 반환

# # --- 핵심 로직 함수 ---
# def handle_color_detections():
#     global red_line_count,robot_is_on_red_patch,third_red_line_detected_this_lap,current_state, school_zone_action_completed_this_segment,in_high_speed_mode
#     # 컬러 센서 사용 가능 여부 확인
#     if color_sensor is None: return False 

#     r,g,b=color_sensor.rgb()
#     is_red=(r>=RED_R_MIN and r<=RED_R_MAX and g<=RED_G_MAX and b<=RED_B_MAX and r>(g+3)and r>(b+3))
#     is_yellow=(r>=YELLOW_R_MIN and r<=YELLOW_R_MAX and g>=YELLOW_G_MIN and g<=YELLOW_G_MAX and b<=YELLOW_B_MAX and g>r and g>(b+5)and r>(b+3))
#     if is_red:
#         school_zone_action_completed_this_segment=False
#         if not robot_is_on_red_patch:
#             red_line_count+=1;robot_is_on_red_patch=True
#             if not SPEAKER_MUTED:ev3.speaker.beep(RED_LINE_BEEP_FREQUENCY,RED_LINE_BEEP_DURATION)
            
#             # ###################### 변경된 부분 시작 ######################
#             # 첫 번째 빨간 선을 감지하면 고속 모드로 전환
#             if red_line_count == 1 and not third_red_line_detected_this_lap:
#                 ev3.screen.clear();ev3.screen.print("1st Red: High Speed!")
#                 in_high_speed_mode=True
#                 run_motor.run(DRIVE_SPEED_MAX_LINETRACE)
#                 return False
#             # ###################### 변경된 부분 끝 ########################
            
#             # 두 번째 빨간 선에서는 아무것도 하지 않고 고속 주행을 유지합니다.
            
#             # 세 번째 빨간 선에서는 정지
#             elif red_line_count==3 and not third_red_line_detected_this_lap:
#                 third_red_line_detected_this_lap=True;in_high_speed_mode=False
#                 ev3.screen.clear();ev3.screen.print("3rd Red: Stop!");ev3.screen.print("Wait OMV Signal");stop_robot_motion();current_state=RobotState.TRAFFIC_LIGHT_WAITING;return True
#         return False
#     elif is_yellow:
#         robot_is_on_red_patch=False
#         # 노란 선(스쿨존)을 만나면 고속 모드를 해제
#         if current_state==RobotState.LINE_TRACING and not school_zone_action_completed_this_segment:
#             if in_high_speed_mode:
#                 in_high_speed_mode=False
#                 run_motor.run(DRIVE_SPEED_LINETRACE)
#                 wait(50)
#             ev3.screen.clear();ev3.screen.print("School Zone!");stop_robot_motion();current_state=RobotState.SCHOOL_ZONE_ACTION;return True
#         return False
#     else:robot_is_on_red_patch=False;return False

# # (scan_and_select_parking_spot, perform_parallel_parking 함수는 이전과 동일)
# def scan_and_select_parking_spot():
#     global parking_spot_selected
#     if ultrasonic_sensor is None: ev3.screen.print("Ultrasonic Error!"); return None
#     ev3.screen.clear();ev3.screen.print("Scanning Parking...");dist1=ultrasonic_sensor.distance();wait(1000)
#     run_motor.run_angle(PARKING_DRIVE_SPEED,DIST_BETWEEN_SCAN_SPOTS_MOTOR_DEG,Stop.BRAKE,True);wait(500)
#     dist2=ultrasonic_sensor.distance();wait(1000)
#     good1=dist1>MIN_OPEN_SPACE_FOR_PARKING_MM;good2=dist2>MIN_OPEN_SPACE_FOR_PARKING_MM;name=None
#     if good1 and(not good2 or dist1>=dist2):ev3.screen.print("Select: Spot 1");run_motor.run_angle(-PARKING_DRIVE_SPEED,DIST_BETWEEN_SCAN_SPOTS_MOTOR_DEG,Stop.BRAKE,True);wait(500);name="Spot 1"
#     elif good2:ev3.screen.print("Select: Spot 2");name="Spot 2"
#     else:ev3.screen.print("No Suitable Spot.")
#     parking_spot_selected=name;return name is not None
# def perform_parallel_parking():
#     global parking_attempted_this_lap
#     if parking_spot_selected is None:ev3.screen.print("Parking Cancel (No Spot)");parking_attempted_this_lap=True;return False
#     ev3.screen.clear();ev3.screen.print("Parking: "+str(parking_spot_selected));wait(500);sound_parking_horn();wait(300)
#     activate_flashing_lights();wait(200);run_motor.run_angle(PARKING_DRIVE_SPEED,DRIVE_PAST_SPOT_MOTOR_DEG,Stop.BRAKE,True)
#     steering_motor.run_target(PARKING_STEERING_SPEED,STEERING_ANGLE_INTO_SPOT_MOTOR_DEG,Stop.HOLD,True);activate_flashing_lights();wait(200)
#     run_motor.run_angle(-PARKING_DRIVE_SPEED,REVERSE_DIAGONAL_MOTOR_DEG,Stop.BRAKE,True)
#     steering_motor.run_target(PARKING_STEERING_SPEED,STEERING_ANGLE_STRAIGHTEN_MOTOR_DEG,Stop.HOLD,True);activate_flashing_lights();wait(200)
#     run_motor.run_angle(-PARKING_DRIVE_SPEED,REVERSE_STRAIGHT_MOTOR_DEG,Stop.BRAKE,True)
#     steering_motor.run_target(PARKING_STEERING_SPEED,0,Stop.HOLD,True);ev3.light.off();stop_robot_motion()
#     ev3.screen.clear();ev3.screen.print("Parking Complete!");parking_attempted_this_lap=True;return True

# # --- 메인 프로그램 ---
# try:
#     while True:
#         pressed_buttons_this_loop = ev3.buttons.pressed()

#         if current_state == RobotState.INITIALIZING:
#             start_origin_calibration()
#             ev3.screen.clear(); ev3.screen.print("Wheels Aligning...")
#             steering_motor.run_target(PARKING_STEERING_SPEED, 0, then=Stop.HOLD, wait=True); wait(500)
#             current_state = RobotState.WAITING_TO_START

#         elif current_state == RobotState.WAITING_TO_START:
#             ev3.screen.clear(); ev3.screen.print("Press Center Button"); ev3.screen.print("to Start Mission.")
#             while not Button.CENTER in ev3.buttons.pressed(): wait(50) # 직접 읽음
#             ev3.screen.clear(); ev3.screen.print("Center Pressed! Starting..."); ev3.speaker.beep(); wait(500) 
#             start_line_tracing_motion(); current_state = RobotState.LINE_TRACING

#         elif current_state == RobotState.LINE_TRACING:
#             if handle_color_detections(): 
#                 continue 
#             if green_light_sim_received and not parking_attempted_this_lap:
#                 if PARKING_MODE_ACTIVATE_BUTTON in pressed_buttons_this_loop:
#                     if in_high_speed_mode:in_high_speed_mode=False;run_motor.run(DRIVE_SPEED_LINETRACE)
#                     ev3.screen.clear();ev3.screen.print("Parking Mode Activated");stop_robot_motion();current_state=RobotState.PARKING_SCAN_PROMPT;continue
#             try:
#                 hap=list(lsa.ReadRaw_Calibrated())
#                 if hap and len(hap)>=8:
#                     left_dark=sum(100-r_val for r_val in hap[0:3])/3;right_dark=sum(100-r_val for r_val in hap[5:8])/3
#                     error=left_dark-right_dark;current_kp=STEERING_KP_LINETRACE*HIGH_SPEED_KP_FACTOR if in_high_speed_mode else STEERING_KP_LINETRACE
#                     correction=current_kp*error;max_correction=90;correction=max(-max_correction,min(max_correction,correction))
#                     steering_motor.run_target(STEERING_MOTOR_SPEED_LINETRACE,correction,Stop.HOLD,False)
#                 else:pass
#             except Exception as e:pass

#         elif current_state == RobotState.SCHOOL_ZONE_ACTION:
#             ev3.screen.clear();ev3.screen.print("School Zone Action...");activate_flashing_lights()
#             ev3.screen.print("Clearing yellow zone...");steering_motor.run_target(STEERING_MOTOR_SPEED_LINETRACE,0,Stop.HOLD,True)
#             run_motor.run_angle(SCHOOL_ZONE_CLEAR_DRIVE_SPEED,SCHOOL_ZONE_CLEAR_DRIVE_DEG,Stop.BRAKE,True)
#             school_zone_action_completed_this_segment=True;ev3.screen.print("Re-aligning wheels...")
#             steering_motor.run_target(PARKING_STEERING_SPEED,0,Stop.HOLD,True);wait(200)
#             ev3.screen.print("Resuming line trace.");start_line_tracing_motion();current_state=RobotState.LINE_TRACING

#         elif current_state == RobotState.TRAFFIC_LIGHT_WAITING: # OpenMV 신호 대기
#             ev3.screen.clear(); ev3.screen.print("3rd Red: Waiting OMV Signal")
            
#             signal = read_openmv_signal() # OpenMV로부터 신호 읽기

#             if signal == "two": # OpenMV가 "two"를 보내면 초록불로 간주
#                 ev3.screen.clear(); ev3.screen.print("OMV: Green detected! Go!")
#                 beep_custom(880, 200); wait(500)
#                 green_light_sim_received = True 
#                 start_line_tracing_motion(); current_state = RobotState.LINE_TRACING
#             elif signal == "one": # 예시: 빨간불 신호
#                 ev3.screen.print("OMV: Red. Still Waiting...")
#             elif signal == "three": # 예시: 노란불 신호
#                 ev3.screen.print("OMV: Yellow. Still Waiting...")
#             elif signal: # 빈 문자열이 아닌 다른 알 수 없는 신호 수신 시
#                 ev3.screen.print("OMV Sig: " + signal) # 수신된 신호 표시
            
#             wait(100) # OpenMV 신호 폴링 간격

#         # (이하 PARKING_SCAN_PROMPT 등 다른 상태들은 이전과 동일)
#         elif current_state == RobotState.PARKING_SCAN_PROMPT:
#             ev3.screen.clear(); ev3.screen.print("Parking: Align, then Down btn")
#             scan_trigger_pressed = False
#             while not scan_trigger_pressed:
#                 current_scan_buttons = ev3.buttons.pressed()
#                 if PARKING_SCAN_START_BUTTON in current_scan_buttons:
#                     ev3.screen.print("Down Button for Scan!"); ev3.speaker.beep(); wait(500); scan_trigger_pressed = True
#                 wait(50)
#             current_state = RobotState.PARKING_SCANNING
#         elif current_state == RobotState.PARKING_SCANNING:
#             if scan_and_select_parking_spot(): current_state = RobotState.PARKING_EXECUTING
#             else: 
#                 ev3.screen.print("No spot. Resume trace."); wait(1000)
#                 parking_attempted_this_lap = True; start_line_tracing_motion(); current_state = RobotState.LINE_TRACING
#         elif current_state == RobotState.PARKING_EXECUTING:
#             if perform_parallel_parking(): current_state = RobotState.MISSION_COMPLETE
#             else:
#                 ev3.screen.print("Parking failed/canceled."); wait(1000)
#                 start_line_tracing_motion(); current_state = RobotState.LINE_TRACING
#         elif current_state == RobotState.MISSION_COMPLETE:
#             ev3.screen.clear(); ev3.screen.print("Mission Complete!"); stop_robot_motion(); ev3.light.off(); wait(5000); break
#         elif current_state == RobotState.ERROR:
#             ev3.screen.clear(); ev3.screen.print("Error Occurred!"); stop_robot_motion(); ev3.light.on(Color.RED); wait(5000); break
#         wait(20)
# except KeyboardInterrupt:
#     ev3.screen.clear(); ev3.screen.print("Program Stopped by User.")
# except Exception as e:
#     ev3.screen.clear(); ev3.screen.print("Runtime Error!"); print("Runtime Error:", e); current_state = RobotState.ERROR
# finally:
#     stop_robot_motion(); ev3.light.off(); ev3.screen.clear()
#     if current_state != RobotState.MISSION_COMPLETE and current_state != RobotState.ERROR: ev3.screen.print("Program Ended.")
#     elif current_state == RobotState.ERROR: ev3.screen.print("Ended due to Error.")
#     wait(2000)









































# #!/usr/bin/env pybricks-micropython

# # 버전 1.47 (고속 모드 속도 이론적 최대치로 설정)

# from pybricks.hubs import EV3Brick
# from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
#                                  InfraredSensor, UltrasonicSensor, GyroSensor)
# from pybricks.parameters import Port, Stop, Direction, Button, Color
# from pybricks.tools import wait, StopWatch, DataLog
# from pybricks.robotics import DriveBase
# from mindsensorsPYB import LSA
# from pybricks.iodevices import UARTDevice

# import os
# import sys
# import time

# # --- 설정 상수 ---
# SPEAKER_MUTED = False

# # EV3 브릭 및 장치 초기화
# ev3 = EV3Brick()
# try: lsa = LSA(Port.S1, 0x14)
# except Exception as e: ev3.screen.print("LSA Init Error"); wait(2000); exit()
# run_motor = Motor(Port.A)
# steering_motor = Motor(Port.D)
# COLOR_SENSOR_PORT = Port.S3 # 예시
# OPENMV_UART_PORT = Port.S4  # 예시
# try: color_sensor = ColorSensor(COLOR_SENSOR_PORT)
# except Exception as e: ev3.screen.print("ColorSensor Err"); wait(1000); color_sensor = None
# try: openmv_uart = UARTDevice(OPENMV_UART_PORT, baudrate=115200, timeout=10)
# except Exception as e: ev3.screen.clear(); ev3.screen.print("OpenMV UART Error"); ev3.speaker.beep(); wait(5000); openmv_uart = None
# try: ultrasonic_sensor = UltrasonicSensor(Port.S2)
# except Exception as e: ev3.screen.print("Ultrasonic S2 Err"); wait(2000); ultrasonic_sensor = None

# # --- RGB 색상 판단 임계값 ---
# RED_R_MIN = 12; RED_R_MAX = 22; RED_G_MAX = 8; RED_B_MAX = 10
# YELLOW_R_MIN = 16; YELLOW_R_MAX = 28; YELLOW_G_MIN = 19; YELLOW_G_MAX = 33; YELLOW_B_MAX = 18
# # -------------------------------------------------------------------------

# # 라인 트레이싱 매개변수
# DRIVE_SPEED_LINETRACE = 200       # 일반 라인 트레이싱 속도
# DRIVE_SPEED_MAX_LINETRACE = 900 # 2~3번째 빨간선 사이 "최대" 속도 (deg/s, 예시로 매우 높게 설정)
# STEERING_KP_LINETRACE = -7.0
# HIGH_SPEED_KP_FACTOR = 0.5    # 고속 주행 시 P 게인 감소 비율 (더 낮춰볼 수 있음)
# STEERING_MOTOR_SPEED_LINETRACE = 2500

# # (주차, 점멸등, 경적, 스쿨존 상수는 이전과 동일)
# PARKING_DRIVE_SPEED = 80; PARKING_STEERING_SPEED = 300
# MIN_OPEN_SPACE_FOR_PARKING_MM = 350; DIST_BETWEEN_SCAN_SPOTS_MOTOR_DEG = 360 * 2.2
# DRIVE_PAST_SPOT_MOTOR_DEG = 180; STEERING_ANGLE_INTO_SPOT_MOTOR_DEG = 70
# REVERSE_DIAGONAL_MOTOR_DEG = 360 * 1.1; STEERING_ANGLE_STRAIGHTEN_MOTOR_DEG = -80
# REVERSE_STRAIGHT_MOTOR_DEG = 360 * 0.7
# FLASHING_LIGHT_COLORS = [Color.RED, Color.YELLOW, Color.GREEN]; FLASHING_LIGHT_INTERVAL_MS = 1000
# HORN_FREQUENCY = 600; HORN_DURATION_MS = 150
# RED_LINE_BEEP_FREQUENCY = 440; RED_LINE_BEEP_DURATION = 100
# PARKING_MODE_ACTIVATE_BUTTON = Button.UP; PARKING_SCAN_START_BUTTON = Button.DOWN
# SCHOOL_ZONE_CLEAR_DRIVE_DEG = 360 * 0.1; SCHOOL_ZONE_CLEAR_DRIVE_SPEED = 50    

# # --- 로봇 상태 정의 ---
# class RobotState:
#     INITIALIZING = "Initializing"; WAITING_TO_START = "Waiting to Start"
#     LINE_TRACING = "Line Tracing"; SCHOOL_ZONE_ACTION = "School Zone Action"
#     TRAFFIC_LIGHT_WAITING = "Traffic Light Waiting"
#     PARKING_SCAN_PROMPT = "Parking Scan Prompt"; PARKING_SCANNING = "Parking Scanning"
#     PARKING_EXECUTING = "Parking Executing"; MISSION_COMPLETE = "Mission Complete"; ERROR = "Error"

# # --- 전역 변수 ---
# current_state = RobotState.INITIALIZING
# red_line_count = 0; robot_is_on_red_patch = False
# third_red_line_detected_this_lap = False; green_light_sim_received = False
# parking_spot_selected = None; parking_attempted_this_lap = False
# school_zone_action_completed_this_segment = False; in_high_speed_mode = False

# # --- 도우미 함수 ---
# def speak(text): pass
# def beep_custom(f=HORN_FREQUENCY, d=HORN_DURATION_MS):
#     if not SPEAKER_MUTED: ev3.speaker.beep(f, d)
# def start_origin_calibration():
#     ev3.screen.clear(); ev3.screen.print("Origin Calibrating...")
#     steering_motor.run(100)
#     while True:
#         a = steering_motor.angle(); wait(100); b = steering_motor.angle()
#         if a == b: break
#     steering_motor.stop()
#     steering_motor.run_angle(-100, 100,then=Stop.HOLD,wait=True)
#     steering_motor.reset_angle(0)
#     steering_motor.run_target(PARKING_STEERING_SPEED,0,then=Stop.HOLD,wait=True)
#     ev3.screen.clear(); ev3.screen.print("Origin Calibrated!"); wait(500)
# def stop_robot_motion(): run_motor.brake(); steering_motor.brake()
# def start_line_tracing_motion(use_max_speed=False): # 이 함수는 이제 직접 속도를 결정
#     ev3.screen.clear()
#     # in_high_speed_mode 전역 변수를 직접 참조하여 속도 결정
#     # (use_max_speed 인자는 이제 handle_color_detections에서 직접 속도 설정하므로 덜 중요해짐)
#     final_speed = DRIVE_SPEED_LINETRACE 
#     message_suffix = ""
#     if in_high_speed_mode: # 현재 고속 모드 상태라면
#         final_speed = DRIVE_SPEED_MAX_LINETRACE
#         message_suffix = " (MAX SPEED)"
    
#     ev3.screen.print("Line Tracing Start..." + message_suffix)
#     run_motor.run(final_speed)

# def activate_flashing_lights():
#     for c in FLASHING_LIGHT_COLORS: ev3.light.on(c); wait(FLASHING_LIGHT_INTERVAL_MS)
#     ev3.light.off()
# def sound_parking_horn():
#     for _ in range(2): beep_custom(); wait(HORN_DURATION_MS // 2)

# def read_openmv_signal(): # 이전과 동일
#     if openmv_uart is None: return ""
#     try:
#         if openmv_uart.waiting() > 0: data = openmv_uart.read_all(); return data.decode().strip()
#         else: return ""
#     except Exception as e: return ""

# # --- 핵심 로직 함수 ---
# def handle_color_detections(): # 이전과 동일
#     global red_line_count,robot_is_on_red_patch,third_red_line_detected_this_lap,current_state, school_zone_action_completed_this_segment,in_high_speed_mode
#     if color_sensor is None: return False 
#     r,g,b=color_sensor.rgb()
#     is_red=(r>=RED_R_MIN and r<=RED_R_MAX and g<=RED_G_MAX and b<=RED_B_MAX and r>(g+3)and r>(b+3))
#     is_yellow=(r>=YELLOW_R_MIN and r<=YELLOW_R_MAX and g>=YELLOW_G_MIN and g<=YELLOW_G_MAX and b<=YELLOW_B_MAX and g>r and g>(b+5)and r>(b+3))
#     if is_red:
#         school_zone_action_completed_this_segment=False
#         if not robot_is_on_red_patch:
#             red_line_count+=1;robot_is_on_red_patch=True
#             if not SPEAKER_MUTED:ev3.speaker.beep(RED_LINE_BEEP_FREQUENCY,RED_LINE_BEEP_DURATION)
#             if red_line_count==2 and not third_red_line_detected_this_lap:
#                 ev3.screen.clear();ev3.screen.print("2nd Red: High Speed!");in_high_speed_mode=True
#                 run_motor.run(DRIVE_SPEED_MAX_LINETRACE) # 최대 속도로 즉시 변경
#                 return False # 상태 변경 없이 라인 트레이싱 지속 (고속으로)
#             elif red_line_count==3 and not third_red_line_detected_this_lap:
#                 third_red_line_detected_this_lap=True;in_high_speed_mode=False
#                 run_motor.run(DRIVE_SPEED_LINETRACE) # 3번째 선 감지 시 일반 속도로 (어차피 멈춤)
#                 ev3.screen.clear();ev3.screen.print("3rd Red: Stop!");ev3.screen.print("Wait OMV Signal");stop_robot_motion();current_state=RobotState.TRAFFIC_LIGHT_WAITING;return True
#         return False
#     elif is_yellow:
#         robot_is_on_red_patch=False
#         if current_state==RobotState.LINE_TRACING and not school_zone_action_completed_this_segment:
#             if in_high_speed_mode:in_high_speed_mode=False;run_motor.run(DRIVE_SPEED_LINETRACE);wait(50)
#             ev3.screen.clear();ev3.screen.print("School Zone!");stop_robot_motion();current_state=RobotState.SCHOOL_ZONE_ACTION;return True
#         return False
#     else:robot_is_on_red_patch=False;return False

# # (scan_and_select_parking_spot, perform_parallel_parking 함수는 이전과 동일)
# def scan_and_select_parking_spot():
#     global parking_spot_selected
#     if ultrasonic_sensor is None: ev3.screen.print("Ultrasonic Error!"); return None
#     ev3.screen.clear();ev3.screen.print("Scanning Parking...");dist1=ultrasonic_sensor.distance();wait(1000)
#     run_motor.run_angle(PARKING_DRIVE_SPEED,DIST_BETWEEN_SCAN_SPOTS_MOTOR_DEG,Stop.BRAKE,True);wait(500)
#     dist2=ultrasonic_sensor.distance();wait(1000)
#     good1=dist1>MIN_OPEN_SPACE_FOR_PARKING_MM;good2=dist2>MIN_OPEN_SPACE_FOR_PARKING_MM;name=None
#     if good1 and(not good2 or dist1>=dist2):ev3.screen.print("Select: Spot 1");run_motor.run_angle(-PARKING_DRIVE_SPEED,DIST_BETWEEN_SCAN_SPOTS_MOTOR_DEG,Stop.BRAKE,True);wait(500);name="Spot 1"
#     elif good2:ev3.screen.print("Select: Spot 2");name="Spot 2"
#     else:ev3.screen.print("No Suitable Spot.")
#     parking_spot_selected=name;return name is not None
# def perform_parallel_parking():
#     global parking_attempted_this_lap
#     if parking_spot_selected is None:ev3.screen.print("Parking Cancel (No Spot)");parking_attempted_this_lap=True;return False
#     ev3.screen.clear();ev3.screen.print("Parking: "+str(parking_spot_selected));wait(500);sound_parking_horn();wait(300)
#     activate_flashing_lights();wait(200);run_motor.run_angle(PARKING_DRIVE_SPEED,DRIVE_PAST_SPOT_MOTOR_DEG,Stop.BRAKE,True)
#     steering_motor.run_target(PARKING_STEERING_SPEED,STEERING_ANGLE_INTO_SPOT_MOTOR_DEG,Stop.HOLD,True);activate_flashing_lights();wait(200)
#     run_motor.run_angle(-PARKING_DRIVE_SPEED,REVERSE_DIAGONAL_MOTOR_DEG,Stop.BRAKE,True)
#     steering_motor.run_target(PARKING_STEERING_SPEED,STEERING_ANGLE_STRAIGHTEN_MOTOR_DEG,Stop.HOLD,True);activate_flashing_lights();wait(200)
#     run_motor.run_angle(-PARKING_DRIVE_SPEED,REVERSE_STRAIGHT_MOTOR_DEG,Stop.BRAKE,True)
#     steering_motor.run_target(PARKING_STEERING_SPEED,0,Stop.HOLD,True);ev3.light.off();stop_robot_motion()
#     ev3.screen.clear();ev3.screen.print("Parking Complete!");parking_attempted_this_lap=True;return True

# # --- 메인 프로그램 ---
# try:
#     while True:
#         pressed_buttons_this_loop = ev3.buttons.pressed()

#         if current_state == RobotState.INITIALIZING:
#             start_origin_calibration()
#             ev3.screen.clear(); ev3.screen.print("Wheels Aligning...")
#             steering_motor.run_target(PARKING_STEERING_SPEED, 0, then=Stop.HOLD, wait=True); wait(500)
#             current_state = RobotState.WAITING_TO_START

#         elif current_state == RobotState.WAITING_TO_START:
#             ev3.screen.clear(); ev3.screen.print("Press Center Button"); ev3.screen.print("to Start Mission.")
#             current_start_buttons = ev3.buttons.pressed() 
#             if Button.CENTER in current_start_buttons:
#                 ev3.screen.clear(); ev3.screen.print("Center Pressed! Starting..."); ev3.speaker.beep(); wait(500) 
#                 start_line_tracing_motion() # 일반 속도로 시작
#                 current_state = RobotState.LINE_TRACING
#             else: wait(50) 

#         elif current_state == RobotState.LINE_TRACING:
#             if handle_color_detections(): continue 
#             # 고속 모드 중에는 handle_color_detections에서 이미 run_motor.run(DRIVE_SPEED_MAX_LINETRACE)가 호출됨.
#             # LSA 로직은 조향만 담당.
#             if green_light_sim_received and not parking_attempted_this_lap:
#                 if PARKING_MODE_ACTIVATE_BUTTON in pressed_buttons_this_loop:
#                     if in_high_speed_mode:in_high_speed_mode=False;run_motor.run(DRIVE_SPEED_LINETRACE) # 일반 속도로 복귀
#                     ev3.screen.clear();ev3.screen.print("Parking Mode Activated");stop_robot_motion();current_state=RobotState.PARKING_SCAN_PROMPT;continue
#             try:
#                 hap=list(lsa.ReadRaw_Calibrated())
#                 if hap and len(hap)>=8:
#                     left_dark=sum(100-r_val for r_val in hap[0:3])/3;right_dark=sum(100-r_val for r_val in hap[5:8])/3
#                     error=left_dark-right_dark
#                     current_kp = STEERING_KP_LINETRACE * HIGH_SPEED_KP_FACTOR if in_high_speed_mode else STEERING_KP_LINETRACE
#                     correction=current_kp*error;max_correction=90;correction=max(-max_correction,min(max_correction,correction))
#                     steering_motor.run_target(STEERING_MOTOR_SPEED_LINETRACE,correction,Stop.HOLD,False)
#                 else:pass
#             except Exception as e:pass

#         elif current_state == RobotState.SCHOOL_ZONE_ACTION:
#             ev3.screen.clear();ev3.screen.print("School Zone Action...");activate_flashing_lights()
#             ev3.screen.print("Clearing yellow zone...");steering_motor.run_target(STEERING_MOTOR_SPEED_LINETRACE,0,Stop.HOLD,True)
#             run_motor.run_angle(SCHOOL_ZONE_CLEAR_DRIVE_SPEED,SCHOOL_ZONE_CLEAR_DRIVE_DEG,Stop.BRAKE,True)
#             school_zone_action_completed_this_segment=True;ev3.screen.print("Re-aligning wheels...")
#             steering_motor.run_target(PARKING_STEERING_SPEED,0,Stop.HOLD,True);wait(200)
#             ev3.screen.print("Resuming line trace.");start_line_tracing_motion();current_state=RobotState.LINE_TRACING

#         elif current_state == RobotState.TRAFFIC_LIGHT_WAITING:
#             ev3.screen.clear(); ev3.screen.print("3rd Red: Waiting OMV Signal")
#             signal = read_openmv_signal() 
#             if signal == "two":
#                 ev3.screen.clear(); ev3.screen.print("OMV: Green! Go!"); beep_custom(880, 200); wait(500)
#                 green_light_sim_received = True; start_line_tracing_motion(); current_state = RobotState.LINE_TRACING
#             elif signal == "one": ev3.screen.print("OMV: Red. Waiting...")
#             elif signal: ev3.screen.print("OMV Sig: [" + signal + "]")
#             wait(100)

#         # (이하 PARKING_SCAN_PROMPT 등 다른 상태들은 이전과 동일)
#         elif current_state == RobotState.PARKING_SCAN_PROMPT:
#             ev3.screen.clear(); ev3.screen.print("Parking: Align, then Down btn")
#             scan_trigger_pressed = False
#             while not scan_trigger_pressed:
#                 current_scan_buttons = ev3.buttons.pressed()
#                 if PARKING_SCAN_START_BUTTON in current_scan_buttons:
#                     ev3.screen.print("Down Button for Scan!"); ev3.speaker.beep(); wait(500); scan_trigger_pressed = True
#                 wait(50)
#             current_state = RobotState.PARKING_SCANNING
#         elif current_state == RobotState.PARKING_SCANNING:
#             if scan_and_select_parking_spot(): current_state = RobotState.PARKING_EXECUTING
#             else: 
#                 ev3.screen.print("No spot. Resume trace."); wait(1000)
#                 parking_attempted_this_lap = True; start_line_tracing_motion(); current_state = RobotState.LINE_TRACING
#         elif current_state == RobotState.PARKING_EXECUTING:
#             if perform_parallel_parking(): current_state = RobotState.MISSION_COMPLETE
#             else:
#                 ev3.screen.print("Parking failed/canceled."); wait(1000)
#                 start_line_tracing_motion(); current_state = RobotState.LINE_TRACING
#         elif current_state == RobotState.MISSION_COMPLETE:
#             ev3.screen.clear(); ev3.screen.print("Mission Complete!"); stop_robot_motion(); ev3.light.off(); wait(5000); break
#         elif current_state == RobotState.ERROR:
#             ev3.screen.clear(); ev3.screen.print("Error Occurred!"); stop_robot_motion(); ev3.light.on(Color.RED); wait(5000); break
        
#         if current_state != RobotState.WAITING_TO_START: # WAITING_TO_START 상태가 아닐 때만 이 wait 실행
#              wait(20)

# except KeyboardInterrupt:
#     ev3.screen.clear(); ev3.screen.print("Program Stopped by User.")
# except Exception as e:
#     ev3.screen.clear(); ev3.screen.print("Runtime Error!"); print("Runtime Error:", e); current_state = RobotState.ERROR
# finally:
#     stop_robot_motion(); ev3.light.off(); ev3.screen.clear()
#     if current_state != RobotState.MISSION_COMPLETE and current_state != RobotState.ERROR: ev3.screen.print("Program Ended.")
#     elif current_state == RobotState.ERROR: ev3.screen.print("Ended due to Error.")
#     wait(2000)












































# #!/usr/bin/env pybricks-micropython

# # 버전 1.44 (제공된 OpenMV 데이터 읽기 함수 적용)

# from pybricks.hubs import EV3Brick
# from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
#                                  InfraredSensor, UltrasonicSensor, GyroSensor)
# from pybricks.parameters import Port, Stop, Direction, Button, Color
# from pybricks.tools import wait, StopWatch, DataLog
# from pybricks.robotics import DriveBase
# from mindsensorsPYB import LSA
# from pybricks.iodevices import UARTDevice # UART 통신을 위해 추가

# import os
# import sys
# import time

# # --- 설정 상수 ---
# SPEAKER_MUTED = False

# # EV3 브릭 및 장치 초기화
# ev3 = EV3Brick()
# try:
#     lsa = LSA(Port.S1, 0x14) # LSA 포트 및 주소 확인
# except Exception as e:
#     ev3.screen.print("LSA Init Error"); wait(2000); exit()

# run_motor = Motor(Port.A) # 주행 모터 포트 확인
# steering_motor = Motor(Port.D) # 조향 모터 포트 확인

# try:
#     color_sensor = ColorSensor(Port.S4) # 컬러 센서 포트 확인
# except Exception as e: # 컬러센서가 S4에 없으면 OpenMV용으로 UART를 S4에 연결 시도
#     ev3.screen.print("ColorSensor S4 Err")
#     wait(1000)
#     # 이 경우 컬러센서가 다른 포트에 있거나 사용하지 않는다고 가정.
#     # 만약 컬러센서도 필요하다면 포트 충돌을 해결해야 함.

# try:
#     # OpenMV는 Port.S4에 연결되었다고 가정 (컬러센서와 같은 포트 사용 시 주의)
#     # 만약 컬러센서가 Port.S4를 사용한다면, OpenMV는 다른 포트(예: Port.S3)를 사용해야 합니다.
#     # 여기서는 일단 Port.S4로 시도하고, 컬러센서와 충돌 시 메시지를 남깁니다.
#     openmv_uart = UARTDevice(Port.S3, baudrate=115200, timeout=10) # timeout 추가 (non-blocking 읽기 시 도움)
#     ev3.screen.print("OpenMV UART on S4 OK")
#     wait(500)
# except Exception as e:
#     ev3.screen.clear()
#     ev3.screen.print("OpenMV UART Error:")
#     error_message = str(e)
#     max_chars_per_line = 18
#     for i in range(0, len(error_message), max_chars_per_line):
#         ev3.screen.print(error_message[i:i+max_chars_per_line])
#     ev3.speaker.beep(); wait(5000); # OpenMV 연결 실패 시 계속 진행할지 결정 필요
#     openmv_uart = None # 오류 발생 시 None으로 설정하여 이후 코드에서 확인 가능

# try:
#     ultrasonic_sensor = UltrasonicSensor(Port.S2) # 초음파 센서 포트 확인
# except Exception as e:
#     ev3.screen.print("Ultrasonic S2 Err"); wait(2000)
#     ultrasonic_sensor = None


# # --- RGB 색상 판단 임계값 ---
# RED_R_MIN = 12; RED_R_MAX = 22; RED_G_MAX = 8; RED_B_MAX = 10
# YELLOW_R_MIN = 16; YELLOW_R_MAX = 28; YELLOW_G_MIN = 19; YELLOW_G_MAX = 33; YELLOW_B_MAX = 18
# # -------------------------------------------------------------------------

# # 라인 트레이싱 매개변수
# DRIVE_SPEED_LINETRACE = 200
# DRIVE_SPEED_MAX_LINETRACE = 350
# STEERING_KP_LINETRACE = -7.0
# HIGH_SPEED_KP_FACTOR = 0.6 
# STEERING_MOTOR_SPEED_LINETRACE = 2500

# # (주차, 점멸등, 경적, 스쿨존 상수는 이전과 동일)
# PARKING_DRIVE_SPEED = 80; PARKING_STEERING_SPEED = 300
# MIN_OPEN_SPACE_FOR_PARKING_MM = 350
# DIST_BETWEEN_SCAN_SPOTS_MOTOR_DEG = 360 * 2.2
# DRIVE_PAST_SPOT_MOTOR_DEG = 180
# STEERING_ANGLE_INTO_SPOT_MOTOR_DEG = 70
# REVERSE_DIAGONAL_MOTOR_DEG = 360 * 1.1
# STEERING_ANGLE_STRAIGHTEN_MOTOR_DEG = -80
# REVERSE_STRAIGHT_MOTOR_DEG = 360 * 0.7
# FLASHING_LIGHT_COLORS = [Color.RED, Color.YELLOW, Color.GREEN]
# FLASHING_LIGHT_INTERVAL_MS = 1000
# HORN_FREQUENCY = 600; HORN_DURATION_MS = 150
# RED_LINE_BEEP_FREQUENCY = 440; RED_LINE_BEEP_DURATION = 100
# # GREEN_LIGHT_BUTTON, RED_LIGHT_BUTTON_ANY_OTHER는 OpenMV 신호로 대체됨
# PARKING_MODE_ACTIVATE_BUTTON = Button.UP
# PARKING_SCAN_START_BUTTON = Button.DOWN
# SCHOOL_ZONE_CLEAR_DRIVE_DEG = 360 * 0.1 
# SCHOOL_ZONE_CLEAR_DRIVE_SPEED = 50    

# # --- 로봇 상태 정의 ---
# class RobotState:
#     INITIALIZING = "Initializing"
#     WAITING_TO_START = "Waiting to Start"
#     LINE_TRACING = "Line Tracing"
#     SCHOOL_ZONE_ACTION = "School Zone Action"
#     TRAFFIC_LIGHT_WAITING = "Traffic Light Waiting"
#     PARKING_SCAN_PROMPT = "Parking Scan Prompt"
#     PARKING_SCANNING = "Parking Scanning"
#     PARKING_EXECUTING = "Parking Executing"
#     MISSION_COMPLETE = "Mission Complete"
#     ERROR = "Error"

# # --- 전역 변수 ---
# current_state = RobotState.INITIALIZING
# red_line_count = 0; robot_is_on_red_patch = False
# third_red_line_detected_this_lap = False; green_light_sim_received = False
# parking_spot_selected = None; parking_attempted_this_lap = False
# school_zone_action_completed_this_segment = False; in_high_speed_mode = False

# # --- 도우미 함수 ---
# def speak(text): pass
# def beep_custom(f=HORN_FREQUENCY, d=HORN_DURATION_MS):
#     if not SPEAKER_MUTED: ev3.speaker.beep(f, d)
# def start_origin_calibration():
#     ev3.screen.clear(); ev3.screen.print("Origin Calibrating...")
#     steering_motor.run(100)
#     while True:
#         a = steering_motor.angle(); wait(100); b = steering_motor.angle()
#         if a == b: break
#     steering_motor.stop()
#     steering_motor.run_angle(-100, 100, then=Stop.HOLD, wait=True)
#     steering_motor.reset_angle(0)
#     steering_motor.run_target(PARKING_STEERING_SPEED, 0, then=Stop.HOLD, wait=True)
#     ev3.screen.clear(); ev3.screen.print("Origin Calibrated!"); wait(500)
# def stop_robot_motion(): run_motor.brake(); steering_motor.brake()
# def start_line_tracing_motion(use_max_speed=False):
#     ev3.screen.clear()
#     speed = DRIVE_SPEED_MAX_LINETRACE if use_max_speed else DRIVE_SPEED_LINETRACE
#     msg = "Line Tracing Start (MAX)" if use_max_speed else "Line Tracing Start..."
#     ev3.screen.print(msg); run_motor.run(speed)
# def activate_flashing_lights():
#     for c in FLASHING_LIGHT_COLORS: ev3.light.on(c); wait(FLASHING_LIGHT_INTERVAL_MS)
#     ev3.light.off()
# def sound_parking_horn():
#     for _ in range(2): beep_custom(); wait(HORN_DURATION_MS // 2)

# # --- OpenMV 신호 읽기 함수 (제공된 함수 사용) ---
# def read_openmv_signal():
#     if openmv_uart is None: # UART 초기화 실패 시
#         # ev3.screen.print("OMV UART not init") # 너무 자주 출력될 수 있음
#         return "" # 빈 문자열 또는 None 반환

#     try:
#         # UART 버퍼에 데이터가 있는지 확인 (non-blocking)
#         if openmv_uart.waiting() > 0:
#             data = openmv_uart.read_all() # 모든 대기 중인 데이터 읽기
#             return data.decode().strip()  # 디코딩 및 공백 제거
#         else:
#             return "" # 읽을 데이터 없음
#     except Exception as e:
#         # ev3.screen.print("OMV Read Error") # 디버깅 시 사용
#         # print("OMV Read Exception:", e)
#         return "" # 오류 발생 시 빈 문자열 반환

# # --- 핵심 로직 함수 ---
# def handle_color_detections():
#     global red_line_count,robot_is_on_red_patch,third_red_line_detected_this_lap,current_state, school_zone_action_completed_this_segment,in_high_speed_mode
#     # 컬러 센서 사용 가능 여부 확인
#     if color_sensor is None: return False 

#     r,g,b=color_sensor.rgb()
#     is_red=(r>=RED_R_MIN and r<=RED_R_MAX and g<=RED_G_MAX and b<=RED_B_MAX and r>(g+3)and r>(b+3))
#     is_yellow=(r>=YELLOW_R_MIN and r<=YELLOW_R_MAX and g>=YELLOW_G_MIN and g<=YELLOW_G_MAX and b<=YELLOW_B_MAX and g>r and g>(b+5)and r>(b+3))
#     if is_red:
#         school_zone_action_completed_this_segment=False
#         if not robot_is_on_red_patch:
#             red_line_count+=1;robot_is_on_red_patch=True
#             if not SPEAKER_MUTED:ev3.speaker.beep(RED_LINE_BEEP_FREQUENCY,RED_LINE_BEEP_DURATION)
#             if red_line_count==2 and not third_red_line_detected_this_lap:
#                 ev3.screen.clear();ev3.screen.print("2nd Red: High Speed!");in_high_speed_mode=True;run_motor.run(DRIVE_SPEED_MAX_LINETRACE);return False
#             elif red_line_count==3 and not third_red_line_detected_this_lap:
#                 third_red_line_detected_this_lap=True;in_high_speed_mode=False
#                 ev3.screen.clear();ev3.screen.print("3rd Red: Stop!");ev3.screen.print("Wait OMV Signal");stop_robot_motion();current_state=RobotState.TRAFFIC_LIGHT_WAITING;return True
#         return False
#     elif is_yellow:
#         robot_is_on_red_patch=False
#         if current_state==RobotState.LINE_TRACING and not school_zone_action_completed_this_segment:
#             if in_high_speed_mode:in_high_speed_mode=False;run_motor.run(DRIVE_SPEED_LINETRACE);wait(50)
#             ev3.screen.clear();ev3.screen.print("School Zone!");stop_robot_motion();current_state=RobotState.SCHOOL_ZONE_ACTION;return True
#         return False
#     else:robot_is_on_red_patch=False;return False

# # (scan_and_select_parking_spot, perform_parallel_parking 함수는 이전과 동일)
# def scan_and_select_parking_spot():
#     global parking_spot_selected
#     if ultrasonic_sensor is None: ev3.screen.print("Ultrasonic Error!"); return None
#     ev3.screen.clear();ev3.screen.print("Scanning Parking...");dist1=ultrasonic_sensor.distance();wait(1000)
#     run_motor.run_angle(PARKING_DRIVE_SPEED,DIST_BETWEEN_SCAN_SPOTS_MOTOR_DEG,Stop.BRAKE,True);wait(500)
#     dist2=ultrasonic_sensor.distance();wait(1000)
#     good1=dist1>MIN_OPEN_SPACE_FOR_PARKING_MM;good2=dist2>MIN_OPEN_SPACE_FOR_PARKING_MM;name=None
#     if good1 and(not good2 or dist1>=dist2):ev3.screen.print("Select: Spot 1");run_motor.run_angle(-PARKING_DRIVE_SPEED,DIST_BETWEEN_SCAN_SPOTS_MOTOR_DEG,Stop.BRAKE,True);wait(500);name="Spot 1"
#     elif good2:ev3.screen.print("Select: Spot 2");name="Spot 2"
#     else:ev3.screen.print("No Suitable Spot.")
#     parking_spot_selected=name;return name is not None
# def perform_parallel_parking():
#     global parking_attempted_this_lap
#     if parking_spot_selected is None:ev3.screen.print("Parking Cancel (No Spot)");parking_attempted_this_lap=True;return False
#     ev3.screen.clear();ev3.screen.print("Parking: "+str(parking_spot_selected));wait(500);sound_parking_horn();wait(300)
#     activate_flashing_lights();wait(200);run_motor.run_angle(PARKING_DRIVE_SPEED,DRIVE_PAST_SPOT_MOTOR_DEG,Stop.BRAKE,True)
#     steering_motor.run_target(PARKING_STEERING_SPEED,STEERING_ANGLE_INTO_SPOT_MOTOR_DEG,Stop.HOLD,True);activate_flashing_lights();wait(200)
#     run_motor.run_angle(-PARKING_DRIVE_SPEED,REVERSE_DIAGONAL_MOTOR_DEG,Stop.BRAKE,True)
#     steering_motor.run_target(PARKING_STEERING_SPEED,STEERING_ANGLE_STRAIGHTEN_MOTOR_DEG,Stop.HOLD,True);activate_flashing_lights();wait(200)
#     run_motor.run_angle(-PARKING_DRIVE_SPEED,REVERSE_STRAIGHT_MOTOR_DEG,Stop.BRAKE,True)
#     steering_motor.run_target(PARKING_STEERING_SPEED,0,Stop.HOLD,True);ev3.light.off();stop_robot_motion()
#     ev3.screen.clear();ev3.screen.print("Parking Complete!");parking_attempted_this_lap=True;return True

# # --- 메인 프로그램 ---
# try:
#     while True:
#         pressed_buttons_this_loop = ev3.buttons.pressed()

#         if current_state == RobotState.INITIALIZING:
#             start_origin_calibration()
#             ev3.screen.clear(); ev3.screen.print("Wheels Aligning...")
#             steering_motor.run_target(PARKING_STEERING_SPEED, 0, then=Stop.HOLD, wait=True); wait(500)
#             current_state = RobotState.WAITING_TO_START

#         elif current_state == RobotState.WAITING_TO_START:
#             ev3.screen.clear(); ev3.screen.print("Press Center Button"); ev3.screen.print("to Start Mission.")
#             while not Button.CENTER in ev3.buttons.pressed(): wait(50) # 직접 읽음
#             ev3.screen.clear(); ev3.screen.print("Center Pressed! Starting..."); ev3.speaker.beep(); wait(500) 
#             start_line_tracing_motion(); current_state = RobotState.LINE_TRACING

#         elif current_state == RobotState.LINE_TRACING:
#             if handle_color_detections(): 
#                 continue 
#             if green_light_sim_received and not parking_attempted_this_lap:
#                 if PARKING_MODE_ACTIVATE_BUTTON in pressed_buttons_this_loop:
#                     if in_high_speed_mode:in_high_speed_mode=False;run_motor.run(DRIVE_SPEED_LINETRACE)
#                     ev3.screen.clear();ev3.screen.print("Parking Mode Activated");stop_robot_motion();current_state=RobotState.PARKING_SCAN_PROMPT;continue
#             try:
#                 hap=list(lsa.ReadRaw_Calibrated())
#                 if hap and len(hap)>=8:
#                     left_dark=sum(100-r_val for r_val in hap[0:3])/3;right_dark=sum(100-r_val for r_val in hap[5:8])/3
#                     error=left_dark-right_dark;current_kp=STEERING_KP_LINETRACE*HIGH_SPEED_KP_FACTOR if in_high_speed_mode else STEERING_KP_LINETRACE
#                     correction=current_kp*error;max_correction=90;correction=max(-max_correction,min(max_correction,correction))
#                     steering_motor.run_target(STEERING_MOTOR_SPEED_LINETRACE,correction,Stop.HOLD,False)
#                 else:pass
#             except Exception as e:pass

#         elif current_state == RobotState.SCHOOL_ZONE_ACTION:
#             ev3.screen.clear();ev3.screen.print("School Zone Action...");activate_flashing_lights()
#             ev3.screen.print("Clearing yellow zone...");steering_motor.run_target(STEERING_MOTOR_SPEED_LINETRACE,0,Stop.HOLD,True)
#             run_motor.run_angle(SCHOOL_ZONE_CLEAR_DRIVE_SPEED,SCHOOL_ZONE_CLEAR_DRIVE_DEG,Stop.BRAKE,True)
#             school_zone_action_completed_this_segment=True;ev3.screen.print("Re-aligning wheels...")
#             steering_motor.run_target(PARKING_STEERING_SPEED,0,Stop.HOLD,True);wait(200)
#             ev3.screen.print("Resuming line trace.");start_line_tracing_motion();current_state=RobotState.LINE_TRACING

#         elif current_state == RobotState.TRAFFIC_LIGHT_WAITING: # OpenMV 신호 대기
#             ev3.screen.clear(); ev3.screen.print("3rd Red: Waiting OMV Signal")
#             # ev3.screen.print("(Use EV3 Btns for Sim)") # 테스트용 안내
            
#             signal = read_openmv_signal() # OpenMV로부터 신호 읽기

#             if signal == "two": # OpenMV가 "two"를 보내면 초록불로 간주
#                 ev3.screen.clear(); ev3.screen.print("OMV: Green detected! Go!")
#                 beep_custom(880, 200); wait(500)
#                 green_light_sim_received = True 
#                 start_line_tracing_motion(); current_state = RobotState.LINE_TRACING
#             elif signal == "one": # 예시: 빨간불 신호
#                 ev3.screen.print("OMV: Red. Still Waiting...")
#                 # wait(100) # 너무 자주 화면 바꾸지 않도록
#             elif signal == "three": # 예시: 노란불 신호
#                 ev3.screen.print("OMV: Yellow. Still Waiting...")
#                 # wait(100)
#             elif signal: # 빈 문자열이 아닌 다른 알 수 없는 신호 수신 시
#                 ev3.screen.print("OMV Sig: " + signal) # 수신된 신호 표시
#                 # wait(100)
#             # else: 신호가 없으면 ("") 계속 대기 (아무것도 안 함)
            
#             wait(100) # OpenMV 신호 폴링 간격

#         # (이하 PARKING_SCAN_PROMPT 등 다른 상태들은 이전과 동일)
#         elif current_state == RobotState.PARKING_SCAN_PROMPT:
#             ev3.screen.clear(); ev3.screen.print("Parking: Align, then Down btn")
#             scan_trigger_pressed = False
#             while not scan_trigger_pressed:
#                 current_scan_buttons = ev3.buttons.pressed()
#                 if PARKING_SCAN_START_BUTTON in current_scan_buttons:
#                     ev3.screen.print("Down Button for Scan!"); ev3.speaker.beep(); wait(500); scan_trigger_pressed = True
#                 wait(50)
#             current_state = RobotState.PARKING_SCANNING
#         elif current_state == RobotState.PARKING_SCANNING:
#             if scan_and_select_parking_spot(): current_state = RobotState.PARKING_EXECUTING
#             else: 
#                 ev3.screen.print("No spot. Resume trace."); wait(1000)
#                 parking_attempted_this_lap = True; start_line_tracing_motion(); current_state = RobotState.LINE_TRACING
#         elif current_state == RobotState.PARKING_EXECUTING:
#             if perform_parallel_parking(): current_state = RobotState.MISSION_COMPLETE
#             else:
#                 ev3.screen.print("Parking failed/canceled."); wait(1000)
#                 start_line_tracing_motion(); current_state = RobotState.LINE_TRACING
#         elif current_state == RobotState.MISSION_COMPLETE:
#             ev3.screen.clear(); ev3.screen.print("Mission Complete!"); stop_robot_motion(); ev3.light.off(); wait(5000); break
#         elif current_state == RobotState.ERROR:
#             ev3.screen.clear(); ev3.screen.print("Error Occurred!"); stop_robot_motion(); ev3.light.on(Color.RED); wait(5000); break
#         wait(20)
# except KeyboardInterrupt:
#     ev3.screen.clear(); ev3.screen.print("Program Stopped by User.")
# except Exception as e:
#     ev3.screen.clear(); ev3.screen.print("Runtime Error!"); print("Runtime Error:", e); current_state = RobotState.ERROR
# finally:
#     stop_robot_motion(); ev3.light.off(); ev3.screen.clear()
#     if current_state != RobotState.MISSION_COMPLETE and current_state != RobotState.ERROR: ev3.screen.print("Program Ended.")
#     elif current_state == RobotState.ERROR: ev3.screen.print("Ended due to Error.")
#     wait(2000)


























