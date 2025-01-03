#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import can
import struct
import math
global cnt
cnt = 0
max_speed = 12.3/3.6


class CANController:
    def __init__(self):
        rospy.init_node('can_controller')
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.can_bus = can.interface.Bus(channel='can0', bustype='socketcan')

    def cmd_vel_callback(self, msg):
        speed = msg.linear.x  # m/s
        steering_angle = msg.angular.z  # rad/s

        # 속도 및 조향각을 CAN 메시지로 변환하여 전송
        self.send_can_message(speed, steering_angle)

    def send_can_message(self, speed, steering_angle):
        # 첫 번째 바이트 고정 값: 96
        fixed_byte = 0x96
        global cnt
        cnt = cnt+1
        # 조향각을 1바이트로 변환 (1 값당 0.25도, 양수는 0x00~0x7F, 음수는 0x80~0xFF)
        degree = steering_angle * 180 / math.pi
        
        steering_units = int(steering_angle * 180  * 15 / 3.14159)  # 조향각을 각도로 변환 후 0.25도 단위로 변환
        steering_units = self.min_max(-steering_units)
        steering_value = steering_units &0xFF #self.convert_to_1byte(-steering_units)
        

        # 속도를 1바이트로 변환 (양수는 전진, 음수는 후진, 0x00~0x7F: 양수, 0x80~0xFF: 음수)
        speed_units = int(speed * 127/max_speed)  # 속도를 0.01m/s 단위로 변환
        speed_units = self.min_max(speed_units)
        speed_value = speed_units & 0xFF
        rospy.loginfo(f"steer : {steering_value}, speed  : {speed_value}")
        

        # 나머지 바이트는 0으로 채우기
        remaining_bytes = bytes([0x00] * 5)

        # CAN 메시지 데이터 구성
        data = struct.pack('>BBB5s', fixed_byte, steering_value, speed_value, remaining_bytes)

        # CAN 메시지 생성
        msg = can.Message(arbitration_id=0x005A6401, data=data, is_extended_id=True)

        # CAN 메시지 송신
        try:
            self.can_bus.send(msg)
            #rospy.loginfo(f"Sent CAN message: st_val={steering_units},st_msg={steering_value}")
        except can.CanError:
            rospy.logerr("Failed to send CAN message")

    def min_max(self, steering_units):
        if steering_units >= 127:
            steering_units = 127

        if steering_units <= -128:
            steering_units = -128

        return steering_units

    def convert_to_1byte(self, value):
        """1바이트로 변환하여 양수는 0x00~0x7F, 음수는 0x80~0xFF 사용"""
        if value >= 0:
            return value & 0x7F  # 양수 범위에서 값 조정
        else:
            return value & 0xFF  # 음수 범위에서 값 조정
        

def main():
    controller = CANController()
    rospy.spin()

if __name__ == "__main__":
    main()
