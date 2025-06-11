#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# =====================================================================
# ИНСТРУКЦИЯ ПО ИСПОЛЬЗОВАНИЮ:
# 1. Сделайте скрипт исполняемым: chmod +x simple_obstacle_avoidance.py
# 2. Запустите MAVROS и поднимите дрон в режиме OFFBOARD
# 3. Запустите этот скрипт: rosrun basic_nav simple_obstacle_avoidance.py
#    или через launch-файл: roslaunch basic_nav obstacle_avoidance.launch
# =====================================================================

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from tf.transformations import euler_from_quaternion

class ObstacleAvoidanceFSM:
    def __init__(self):
        rospy.init_node('obstacle_avoidance_fsm')
        
        # Параметры
        self.obstacle_threshold = 3.0  # метры
        self.safe_distance = 3.5       # метры
        self.max_speed = 0.6           # м/с
        self.turn_speed = 0.3          # рад/с
        self.turn_angle = 90           # угол обхода (градусы)
        self.turn_wait = 2.0           # сек, пауза после поворота (увеличено)
        self.side_time = 2.5           # сек, время движения вбок при обходе (больше не используется напрямую)
        self.extra_side_time = 0.7     # сек, запас после обнаружения свободного пути
        self.extra_timer = None        # таймер для запаса
        self.min_side_time = 7.0       # минимальное гарантированное время движения вбок (сек)
        self.side_start_time = None    # время начала движения вбок
        
        # Состояния
        self.state = 'FORWARD'         # FORWARD, TURN_AWAY, SIDE, TURN_BACK, RESUME
        self.scan_data = None
        self.drone_state = None
        self.yaw = 0.0                 # текущий курс
        self.target_yaw = 0.0          # целевой курс
        self.avoid_direction = 1       # 1 - вправо, -1 - влево
        self.turn_start_time = None
        self.turning = False
        self.saved_yaw = None
        
        # Подписчики
        rospy.Subscriber('/drone1/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/mavros/state', State, self.state_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        
        # Публикатор
        self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        
        # Таймер
        self.timer = rospy.Timer(rospy.Duration(0.1), self.loop)
        rospy.loginfo('Obstacle avoidance FSM started')

    def scan_callback(self, msg):
        self.scan_data = msg

    def state_callback(self, msg):
        self.drone_state = msg

    def pose_callback(self, msg):
        q = msg.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.yaw = math.degrees(yaw)
        if self.yaw < 0:
            self.yaw += 360

    def is_obstacle_ahead(self):
        if not self.scan_data:
            return False
        ranges = np.array(self.scan_data.ranges)
        mid = len(ranges) // 2
        sector = ranges[mid-3:mid+3]
        sector = np.nan_to_num(sector, nan=self.scan_data.range_max, posinf=self.scan_data.range_max)
        return np.any(sector < self.obstacle_threshold)

    def is_path_clear(self):
        if not self.scan_data:
            return False
        ranges = np.array(self.scan_data.ranges)
        mid = len(ranges) // 2
        sector = ranges[mid-3:mid+3]
        sector = np.nan_to_num(sector, nan=self.scan_data.range_max, posinf=self.scan_data.range_max)
        return np.all(sector > self.safe_distance)

    def angle_diff(self, a, b):
        d = a - b
        while d > 180:
            d -= 360
        while d < -180:
            d += 360
        return d

    def loop(self, event):
        if not self.drone_state or not self.drone_state.armed or (self.drone_state.mode not in ["OFFBOARD", "GUIDED"]):
            return
        cmd = TwistStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "base_link"

        if self.state == 'FORWARD':
            if self.is_obstacle_ahead():
                rospy.loginfo('Обнаружено препятствие! Останавливаемся и начинаем обход.')
                self.saved_yaw = self.yaw
                self.avoid_direction = 1 if np.random.rand() > 0.5 else -1
                self.target_yaw = (self.yaw + self.avoid_direction * self.turn_angle) % 360
                self.state = 'TURN_AWAY'
                self.turning = True
                self.turn_start_time = rospy.Time.now()
                cmd.twist.linear.x = 0.0
                cmd.twist.angular.z = 0.0
            else:
                cmd.twist.linear.x = self.max_speed
                cmd.twist.angular.z = 0.0

        elif self.state == 'TURN_AWAY':
            # Поворачиваем на 90°
            diff = self.angle_diff(self.target_yaw, self.yaw)
            if abs(diff) > 3:
                cmd.twist.linear.x = 0.0
                cmd.twist.angular.z = self.turn_speed * self.avoid_direction
            else:
                rospy.loginfo('Поворот завершен. Летим вдоль препятствия.')
                self.state = 'SIDE'
                self.turning = False
                self.turn_start_time = rospy.Time.now()
                self.side_start_time = rospy.Time.now()
                self.extra_timer = None
                cmd.twist.linear.x = 0.0
                cmd.twist.linear.y = 0.0
                cmd.twist.angular.z = 0.0

        elif self.state == 'SIDE':
            time_in_side = (rospy.Time.now() - self.side_start_time).to_sec() if self.side_start_time else 0.0
            if self.extra_timer is not None:
                if (rospy.Time.now() - self.extra_timer).to_sec() < self.extra_side_time:
                    cmd.twist.linear.x = 0.0
                    cmd.twist.linear.y = self.max_speed * self.avoid_direction
                    cmd.twist.angular.z = 0.0
                else:
                    rospy.loginfo('Обход завершен. Возвращаемся на исходный курс.')
                    self.target_yaw = self.saved_yaw
                    self.state = 'TURN_BACK'
                    self.turning = True
                    self.turn_start_time = rospy.Time.now()
                    self.side_start_time = None
                    self.extra_timer = None
                    cmd.twist.linear.x = 0.0
                    cmd.twist.linear.y = 0.0
                    cmd.twist.angular.z = 0.0
            elif time_in_side < self.min_side_time:
                # Гарантированно летим вбок минимум min_side_time секунд
                cmd.twist.linear.x = 0.0
                cmd.twist.linear.y = self.max_speed * self.avoid_direction
                cmd.twist.angular.z = 0.0
            elif self.is_path_clear():
                self.extra_timer = rospy.Time.now()
                cmd.twist.linear.x = 0.0
                cmd.twist.linear.y = self.max_speed * self.avoid_direction
                cmd.twist.angular.z = 0.0
            else:
                cmd.twist.linear.x = 0.0
                cmd.twist.linear.y = self.max_speed * self.avoid_direction
                cmd.twist.angular.z = 0.0

        elif self.state == 'TURN_BACK':
            # Возвращаемся на исходный курс
            diff = self.angle_diff(self.target_yaw, self.yaw)
            if abs(diff) > 3:
                cmd.twist.linear.x = 0.0
                cmd.twist.angular.z = self.turn_speed * (-self.avoid_direction)
            else:
                rospy.loginfo('Возврат на исходный курс завершен. Продолжаем движение вперед.')
                self.state = 'RESUME'
                self.turning = False
                self.turn_start_time = rospy.Time.now()
                cmd.twist.linear.x = 0.0
                cmd.twist.angular.z = 0.0

        elif self.state == 'RESUME':
            # Короткая пауза для стабилизации
            if (rospy.Time.now() - self.turn_start_time).to_sec() < self.turn_wait:
                cmd.twist.linear.x = 0.0
                cmd.twist.angular.z = 0.0
            else:
                self.state = 'FORWARD'
                cmd.twist.linear.x = self.max_speed
                cmd.twist.angular.z = 0.0

        self.vel_pub.publish(cmd)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = ObstacleAvoidanceFSM()
        node.run()
    except rospy.ROSInterruptException:
        pass 