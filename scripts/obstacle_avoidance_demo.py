#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker

class ObstacleAvoidanceDemo:
    def __init__(self):
        rospy.init_node('obstacle_avoidance_demo')
        
        # Параметры
        self.obstacle_threshold = 1.5  # метры
        self.safety_distance = 0.8     # метры
        self.max_speed = 0.5           # м/с
        self.target_altitude = 2.0     # метры
        
        # Целевые точки для демонстрации (x, y)
        self.waypoints = [
            (5.0, 0.0),   # вперед на 5 метров
            (5.0, 5.0),   # вправо на 5 метров
            (0.0, 5.0),   # назад на 5 метров
            (0.0, 0.0)    # влево на 5 метров (возврат)
        ]
        self.current_wp_idx = 0
        
        # Состояние
        self.current_position = None
        self.scan_data = None
        self.current_altitude = 0.0    # Текущая высота дрона
        
        # Подписчики
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        
        # Издатели
        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.marker_pub = rospy.Publisher('/obstacle_avoidance/target', Marker, queue_size=10)
        
        # Сначала отправляем команду удержания текущей позиции
        self.send_hold_position()
        
        # Таймер для основного цикла
        self.timer = rospy.Timer(rospy.Duration(0.1), self.navigation_loop)
        
        rospy.loginfo("Демонстрация обхода препятствий инициализирована")
    
    def send_hold_position(self):
        """Отправляет команду удержания текущей позиции с заданной высотой"""
        rate = rospy.Rate(10)
        
        # Ждем получения текущей позиции
        start_time = rospy.Time.now()
        while not self.current_position and (rospy.Time.now() - start_time).to_sec() < 5.0:
            rospy.loginfo("Ожидание данных о позиции...")
            rate.sleep()
        
        if not self.current_position:
            rospy.logwarn("Не удалось получить данные о позиции, используем нулевую позицию")
            hold_x, hold_y = 0.0, 0.0
        else:
            hold_x = self.current_position.position.x
            hold_y = self.current_position.position.y
            self.current_altitude = self.current_position.position.z
            
        # Если высота слишком мала, используем целевую высоту
        if self.current_altitude < 0.5:
            self.current_altitude = self.target_altitude
            
        rospy.loginfo(f"Удерживаем позицию: x={hold_x}, y={hold_y}, z={self.current_altitude}")
        
        # Отправляем команду удержания позиции несколько раз
        hold_msg = PoseStamped()
        hold_msg.header.frame_id = "map"
        hold_msg.pose.position.x = hold_x
        hold_msg.pose.position.y = hold_y
        hold_msg.pose.position.z = self.current_altitude
        hold_msg.pose.orientation.w = 1.0
        
        for _ in range(20):
            hold_msg.header.stamp = rospy.Time.now()
            self.setpoint_pub.publish(hold_msg)
            rate.sleep()
        
    def scan_callback(self, msg):
        self.scan_data = msg
        
    def pose_callback(self, msg):
        self.current_position = msg.pose
        # Сохраняем текущую высоту для использования в командах
        self.current_altitude = msg.pose.position.z
        
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
        
    def calculate_avoidance_vector(self):
        """Рассчитывает вектор для обхода препятствий"""
        if not self.scan_data:
            return (0.0, 0.0)
            
        ranges = np.array(self.scan_data.ranges)
        angles = np.linspace(self.scan_data.angle_min, self.scan_data.angle_max, len(ranges))
        
        # Заменяем inf и nan на максимальную дальность
        ranges = np.nan_to_num(ranges, nan=self.scan_data.range_max, posinf=self.scan_data.range_max)
        
        # Вычисляем отталкивающие векторы от препятствий
        repulsive_x = 0.0
        repulsive_y = 0.0
        
        for i, r in enumerate(ranges):
            if r < self.obstacle_threshold:
                # Сила отталкивания обратно пропорциональна квадрату расстояния
                force = 1.0 / (r * r) if r > 0.1 else 10.0
                angle = angles[i]
                
                # Компоненты отталкивающего вектора
                repulsive_x -= force * math.cos(angle)
                repulsive_y -= force * math.sin(angle)
        
        # Нормализуем вектор
        magnitude = math.sqrt(repulsive_x**2 + repulsive_y**2)
        if magnitude > 0:
            repulsive_x /= magnitude
            repulsive_y /= magnitude
        
        return (repulsive_x, repulsive_y)
        
    def get_target_position(self):
        """Возвращает текущую целевую точку"""
        if self.current_wp_idx >= len(self.waypoints):
            return None
            
        return self.waypoints[self.current_wp_idx]
        
    def distance_to_waypoint(self, wp):
        """Вычисляет расстояние до заданной точки"""
        if not self.current_position:
            return float('inf')
            
        dx = wp[0] - self.current_position.position.x
        dy = wp[1] - self.current_position.position.y
        return math.sqrt(dx*dx + dy*dy)
        
    def navigation_loop(self, event):
        """Основной цикл навигации"""
        if not self.current_position:
            return
            
        # Получаем текущую целевую точку
        target = self.get_target_position()
        if not target:
            rospy.loginfo("Все точки достигнуты!")
            return
            
        # Проверяем достижение текущей точки
        if self.distance_to_waypoint(target) < 0.5:
            rospy.loginfo("Достигнута точка %d: (%.1f, %.1f)", 
                         self.current_wp_idx, target[0], target[1])
            self.current_wp_idx += 1
            return
            
        # Вычисляем направление к цели
        dx = target[0] - self.current_position.position.x
        dy = target[1] - self.current_position.position.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Нормализуем вектор направления
        if distance > 0:
            dx /= distance
            dy /= distance
            
        # Проверяем наличие препятствий
        if self.is_obstacle_ahead():
            # Вычисляем вектор обхода
            avoid_x, avoid_y = self.calculate_avoidance_vector()
            
            # Комбинируем с направлением к ц