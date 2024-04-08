import socket
import json
from robot import Robot
import time
import os
from cv import get_most_probable_direction
from automonous import cv_auto
# from cv import *

SERVER_IP = '0.0.0.0'
SERVER_PORT = 12346

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

server_socket.bind((SERVER_IP, SERVER_PORT))

server_socket.listen(5)

print("Server is listening...")

robot = Robot() 
robot.set_robot_power(0, 0)
speed_mode = 60
arm_power = 40 
direction = None 
flag = False  
def scale_axis_to_power(value, speed_mode=30):
    return int(value * speed_mode)  


try:
    client_socket, client_address = server_socket.accept()
    print("Connected")

    while True:
        data = client_socket.recv(1024).decode('utf-8')
        
        if not data:
            print("Client disconnected")
            break

        try:
            command = json.loads(data)
            axis_0 = command.get("axis", {}).get("axis_0", 0)  # Forward/Backward
            axis_1 = command.get("axis", {}).get("axis_1", 0)  # Left/Right
            axis_3 = command.get("axis", {}).get("axis_3", 0)  # Up/Down
            speed_decide = command.get("buttons", {}).get("button_0", 0)  # Speed mode
            forward_decide = command.get("buttons", {}).get("button_11", 0)  # Forward
            backward_decide = command.get("buttons", {}).get("button_12", 0)  # Backward
            minus_decide = command.get("buttons", {}).get("button_9", 0)  # Minus
            plus_decide = command.get("buttons", {}).get("button_10", 0)  # Plus
            direction_decide = command.get("buttons", {}).get("button_2", 0)  # Direction
            arm_decide = command.get("buttons", {}).get("button_3", 0) 
            voltage_decide = command.get("buttons", {}).get("button_1", 0) 

            if voltage_decide:
                voltage = robot.get_voltage()
                print("Current Voltage: ", voltage)
        
            if direction_decide:
                flag = True
                print("Starting time delay...")
                time.sleep(6.5)
                print("Time delay ended")
                print("Getting most probable direction...")
                most_probable_direction = get_most_probable_direction()
                print("Most probable direction: ", most_probable_direction)
                if direction == "none":
                    print("No direction found")
                else:
                    cv_auto(most_probable_direction)
                flag = False
                


            if minus_decide:
                speed_mode -= 5
                if speed_mode < 10:
                    speed_mode = 10
                time.sleep(0.2)
                print('Speed mode:', speed_mode)
            elif plus_decide:
                speed_mode += 5
                if speed_mode > 150:
                    speed_mode = 150
                time.sleep(0.2)
                print('Speed mode:', speed_mode)

            if speed_decide:
                if(speed_mode == 35):
                    speed_mode = 65
                    time.sleep(0.2)
                    print("Speed mode: 65")
                else:
                    speed_mode = 35
                    time.sleep(0.2)
                    print("Speed mode: 35")
            
            if forward_decide:
                robot.set_robot_power(-speed_mode, -speed_mode)
            elif backward_decide:
                robot.set_robot_power(speed_mode, speed_mode)

            if arm_decide:
                if arm_power == 40:
                    arm_power = 70
                    print("Arm power: 70")
                else:
                    arm_power = 40
                    print("Arm power: 40")
            
                
            
            forward_backward_power = scale_axis_to_power(axis_1, speed_mode)
            left_right_power = scale_axis_to_power(axis_0, speed_mode)

            if axis_3 < -0.1:
                store = -1 * arm_power
                robot.set_arm_power(store)
                continue
            elif axis_3 > 0.1:
                store = arm_power
                robot.set_arm_power(store)
                continue
            else:
                if not flag:
                    store = 0
                    robot.set_arm_power(store)

            left_motor_power = forward_backward_power - left_right_power
            right_motor_power = forward_backward_power + left_right_power

            left_motor_power = max(min(left_motor_power, 100), -100)
            right_motor_power = max(min(right_motor_power, 100), -100)

            if not forward_decide and not backward_decide:
                robot.set_robot_power(left_motor_power, right_motor_power)

        except:
            pass
    
    client_socket.close()
    
finally:
    client_socket.close()
    server_socket.close()
    robot.reset_all()
