#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool

class SimpleObstacleAvoidance:
    def __init__(self):
        rospy.init_node('simple_obstacle_avoidance')
        
        # Параметры
        self.obstacle_threshold = 2.0  # метры
        self.max_speed = 1.0          # м/с
        self.turn_speed = 0.5         # рад/с
        
        # Состояние
        self.scan_data = None
        self.drone_state = None
        self.is_avoiding = False
        self.avoid_direction = 1  # 1 - влево, -1 - вправо
        
        # Подписчики
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/mavros/state', State, self.state_callback)
        
        # Издатели
        self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        
        # Ждем подключения к MAVROS
        rospy.loginfo("Ожидание подключения к MAVROS...")
        while not rospy.is_shutdown() and (self.drone_state is None or not self.drone_state.connected):
            rospy.sleep(0.1)
        
        rospy.loginfo("Подключение к MAVROS установлено")
        
        # Таймер для основного цикла
        self.timer = rospy.Timer(rospy.Duration(0.1), self.navigation_loop)
        
        rospy.loginfo("Система обхода препятствий запущена")
    
    def scan_callback(self, msg):
        """Обработчик данных лидара"""
        self.scan_data = msg
    
    def state_callback(self, msg):
        """Обработчик состояния дрона"""
        self.drone_state = msg
    
    def is_obstacle_ahead(self):
        """Проверяет наличие препятствий впереди"""
        if not self.scan_data:
            return False
            
        # Проверяем сектор ±30° впереди
        ranges = np.array(self.scan_data.ranges)
        angle_min = self.scan_data.angle_min
        angle_max = self.scan_data.angle_max
        angle_increment = self.scan_data.angle_increment
        
        # Определяем индексы для сектора впереди
        front_angle = 30 * math.pi / 180  # 30 градусов в радианах
        front_indices = int(len(ranges) * front_angle / (angle_max - angle_min))
        mid_idx = len(ranges) // 2
        
        front_ranges = ranges[mid_idx-front_indices:mid_idx+front_indices]
        front_ranges = np.nan_to_num(front_ranges, nan=self.scan_data.range_max, posinf=self.scan_data.range_max)
        
        return np.any(front_ranges < self.obstacle_threshold)
    
    def check_left_right_obstacles(self):
        """Проверяет препятствия слева и справа для выбора направления обхода"""
        if not self.scan_data:
            return 1  # По умолчанию влево
            
        ranges = np.array(self.scan_data.ranges)
        
        # Индексы для левого и правого секторов (±45-90°)
        quarter = len(ranges) // 4
        
        # Левый сектор (45-90°)
        left_ranges = ranges[quarter-quarter//2:quarter+quarter//2]
        left_ranges = np.nan_to_num(left_ranges, nan=self.scan_data.range_max, posinf=self.scan_data.range_max)
        left_min = np.min(left_ranges)
        
        # Правый сектор (-45-90°)
        right_ranges = ranges[3*quarter-quarter//2:3*quarter+quarter//2]
        right_ranges = np.nan_to_num(right_ranges, nan=self.scan_data.range_max, posinf=self.scan_data.range_max)
        right_min = np.min(right_ranges)
        
        # Выбираем направление с большим свободным пространством
        return 1 if left_min > right_min else -1
    
    def navigation_loop(self, event):
        """Основной цикл навигации"""
        if not self.drone_state or not self.drone_state.armed or self.drone_state.mode != "OFFBOARD":
            # Дрон не готов к управлению
            return
            
        # Создаем команду скорости
        cmd_vel = TwistStamped()
        cmd_vel.header.stamp = rospy.Time.now()
        cmd_vel.header.frame_id = "base_link"
        
        # Логика обхода препятствий
        if self.is_obstacle_ahead():
            if not self.is_avoiding:
                # Начинаем обход, выбираем направление
                self.avoid_direction = self.check_left_right_obstacles()
                self.is_avoiding = True
                direction_text = "влево" if self.avoid_direction > 0 else "вправо"
                rospy.loginfo(f"Обнаружено препятствие! Обход {direction_text}")
            
            # Выполняем обход
            cmd_vel.twist.linear.x = self.max_speed * 0.5  # Снижаем скорость
            cmd_vel.twist.angular.z = self.turn_speed * self.avoid_direction
        else:
            # Нет препятствий, движемся вперед
            if self.is_avoiding:
                rospy.loginfo("Препятствие пройдено, продолжаем движение")
                self.is_avoiding = False
            
            cmd_vel.twist.linear.x = self.max_speed
        
        # Публикуем команду скорости
        self.vel_pub.publish(cmd_vel)
    
    def run(self):
        """Запускает систему обхода препятствий"""
        rospy.loginfo("Запуск системы обхода препятствий...")
        rospy.spin()

if __name__ == '__main__':
    try:
        obstacle_avoidance = SimpleObstacleAvoidance()
        obstacle_avoidance.run()
    except rospy.ROSInterruptException:
        pass