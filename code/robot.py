from __future__ import print_function, division
import math
import brickpi3
import time

class Robot:
    def __init__(self):
        # Initialize BrickPi3
        self.BP = brickpi3.BrickPi3()
        
        # Constants
        self.WHEEL_RADIUS = 0.83  # inch
        self.DISTANCE_BETWEEN_WHEELS = 3.9  # inch
        self.PI = math.pi

        self.left_motor = self.BP.PORT_A
        self.right_motor = self.BP.PORT_D
        self.arm_motor = self.BP.PORT_B
        self.light_sensor = self.BP.PORT_1
        self.ultra_sensor = self.BP.PORT_2
        self.sector = 0
        self.oldsector = -1

        self.BP.set_sensor_type(self.light_sensor, self.BP.SENSOR_TYPE.NXT_LIGHT_ON)
        self.BP.set_sensor_type(self.ultra_sensor, self.BP.SENSOR_TYPE.NXT_ULTRASONIC)

        # Initial position and orientation
        self.x, self.y, self.theta = 0, 0, 0
        self.old_left_encoder = 0
        self.old_right_encoder = 0

        self.reset_encoders()
    
    def reset_all(self):
        self.BP.reset_all()
    
    def set_port_power(self, port, power):
        self.BP.set_motor_power(port, power)
    
    def get_voltage(self):
        return self.BP.get_voltage_battery()
    
    def set_arm_power(self, power):
        self.BP.set_motor_power(self.arm_motor, power)

    def reset_encoders(self):
        self.BP.offset_motor_encoder(self.left_motor, self.BP.get_motor_encoder(self.left_motor))
        self.BP.offset_motor_encoder(self.right_motor, self.BP.get_motor_encoder(self.right_motor))
        self.old_left_encoder = 0
        self.old_right_encoder = 0
    
    def get_sector(self):
        return self.sector
    
    def set_robot_power(self, left_power, right_power):
        self.BP.set_motor_power(self.left_motor, left_power)
        self.BP.set_motor_power(self.right_motor, right_power)
    
    def set_motor_dps(self, left_power, right_power):
        self.BP.set_motor_dps(self.left_motor, left_power)
        self.BP.set_motor_dps(self.right_motor, right_power)
    
    def get_light_value(self):
        return self.BP.get_sensor(self.light_sensor)
    
    def get_ultrasonic_value(self):
        return self.BP.get_sensor(self.ultra_sensor)

    def encoder_to_distance(self, encoder_value):
        rotations = encoder_value / 360
        circumference = math.pi * (self.WHEEL_RADIUS * 2)
        distance = rotations * circumference
        return distance

    def calculate_new_position(self, left_encoder, right_encoder):
        left_distance = self.encoder_to_distance(left_encoder)
        right_distance = self.encoder_to_distance(right_encoder)
        
        if left_distance == right_distance:
            distance = left_distance
            delta_x = distance * math.cos(self.theta)
            delta_y = distance * math.sin(self.theta)
            delta_theta = 0
        else:
            radius = self.DISTANCE_BETWEEN_WHEELS / 2 * ((left_distance + right_distance) / (right_distance - left_distance))
            delta_theta = (right_distance - left_distance) / self.DISTANCE_BETWEEN_WHEELS
            delta_x = radius * (math.sin(delta_theta + self.theta) - math.sin(self.theta))
            delta_y = radius * (-math.cos(delta_theta + self.theta) + math.cos(self.theta))
        
        self.x += delta_x
        self.y += delta_y
        self.theta = (self.theta + delta_theta) % (2 * self.PI)
    
    def update_position_continuously(self):
    
        left_encoder = self.BP.get_motor_encoder(self.left_motor)
        right_encoder = self.BP.get_motor_encoder(self.right_motor)

        delta_left = left_encoder - self.old_left_encoder
        delta_right = right_encoder - self.old_right_encoder

        self.old_left_encoder = left_encoder
        self.old_right_encoder = right_encoder

        self.calculate_new_position(delta_left, delta_right)


    def set_motor_dps_with_continuous_measurement(self, left_power, right_power, duration):
        left_encoder_old = 0
        right_encoder_old = 0
        start_time = time.time()
        self.BP.set_motor_dps(self.BP.PORT_B, left_power)
        self.BP.set_motor_dps(self.BP.PORT_C, right_power)
        
        while time.time() - start_time < duration:
            left_encoder = self.BP.get_motor_encoder(self.BP.PORT_B)
            delta_left = left_encoder - left_encoder_old
            left_encoder_old = left_encoder
            
            right_encoder = self.BP.get_motor_encoder(self.BP.PORT_C)
            delta_right = right_encoder - right_encoder_old
            right_encoder_old = right_encoder
            
            self.calculate_new_position(delta_left, delta_right)
        
        self.reset_encoders()
        self.BP.set_motor_dps(self.BP.PORT_B, 0)
        self.BP.set_motor_dps(self.BP.PORT_C, 0)
        time.sleep(1)  # Ensures motors have fully stopped

    
    def update_sector(self):
        theta_degrees = (self.theta * 180 / self.PI) % 360  # Convert radians to degrees and normalize to 0-360 range
        sector_size = 360 / 32  # Divide the circle into 32 sectors
        sector = int(theta_degrees // sector_size)
        if sector > self.oldsector:
            if self.oldsector < 15 and sector > 27: return
            self.oldsector = self.sector    
            self.sector = sector
        else:
            if self.oldsector > 27 and sector < 15: 
                self.oldsector = self.sector
                self.sector = sector

robot = Robot()
robot.set_port_power(robot.right_motor, 20)




    


