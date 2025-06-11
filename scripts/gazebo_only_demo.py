#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class GazeboOnlyDemo:
    def __init__(self):
        rospy.init_node('gazebo_only_demo')
        
        # Параметры
        self.obstacle_threshold = 1.5  # метры
        self.max_speed = 0.5          # м/с
        self.turn_speed = 0.5         # рад/с
        
        # Состояние
        self.scan_data = None
        self.is_avoiding = False
        self.avoid_direction = 1  # 1 - влево, -1 - вправо
        self.start_time = rospy.Time.now()
        
        # Подписчики
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        # Издатели
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.status_pub = rospy.Publisher('/drone_status', String, queue_size=10)
        
        # Таймер для основного цикла
        self.timer = rospy.Timer(rospy.Duration(0.1), self.navigation_loop)
        
        rospy.loginfo("Демонстрация обхода препятствий только в Gazebo запущена")
    
    def scan_callback(self, msg):
        """Обработчик данных лидара"""
        self.scan_data = msg
    
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
        angle_min = self.scan_data.angle_min
        angle_max = self.scan_data.angle_max
        
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
        # Создаем команду скорости
        cmd_vel = Twist()
        
        # Проверяем время работы для демонстрации
        elapsed = (rospy.Time.now() - self.start_time).to_sec()
        
        # Простая логика движения для демонстрации
        if elapsed < 5.0:
            # Первые 5 секунд - просто движение вперед
            self.status_pub.publish("Движение вперед")
            cmd_vel.linear.x = self.max_speed
        else:
            # После 5 секунд - проверка препятствий и обход
            if self.is_obstacle_ahead():
                if not self.is_avoiding:
                    # Начинаем обход, выбираем направление
                    self.avoid_direction = self.check_left_right_obstacles()
                    self.is_avoiding = True
                    direction_text = "влево" if self.avoid_direction > 0 else "вправо"
                    rospy.loginfo(f"Обнаружено препятствие! Обход {direction_text}")
                    self.status_pub.publish(f"Обход препятствия {direction_text}")
                
                # Выполняем обход
                cmd_vel.linear.x = self.max_speed * 0.5  # Снижаем скорость
                cmd_vel.angular.z = self.turn_speed * self.avoid_direction
            else:
                # Нет препятствий, движемся вперед
                if self.is_avoiding:
                    rospy.loginfo("Препятствие пройдено, продолжаем движение")
                    self.status_pub.publish("Продолжаем движение")
                    self.is_avoiding = False
                
                cmd_vel.linear.x = self.max_speed
        
        # Публикуем команду скорости
        self.cmd_vel_pub.publish(cmd_vel)
    
    def run(self):
        """Запускает демонстрацию"""
        rospy.loginfo("Запуск демонстрации обхода препятствий...")
        rospy.spin()

if __name__ == '__main__':
    try:
        demo = GazeboOnlyDemo()
        demo.run()
    except rospy.ROSInterruptException:
        pass 