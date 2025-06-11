#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
import tf
import random
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Point, Twist
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA

class SimulationDemo:
    def __init__(self):
        rospy.init_node('simulation_demo')
        
        # Параметры симуляции
        self.update_rate = 10.0  # Гц
        self.drone_speed = 0.5   # м/с
        self.drone_altitude = 2.0  # м
        self.obstacle_threshold = 1.2  # м
        self.obstacle_count = 10  # количество препятствий
        
        # Состояние дрона
        self.drone_position = [0.0, 0.0, self.drone_altitude]
        self.drone_orientation = [0.0, 0.0, 0.0, 1.0]  # кватернион
        self.current_target = [5.0, 0.0]
        
        # Препятствия (x, y, radius)
        self.obstacles = []
        self.generate_obstacles()
        
        # Траектория движения
        self.path_points = []
        self.path_points.append((0.0, 0.0))
        
        # Публикаторы
        self.scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=1)
        self.pose_pub = rospy.Publisher('/mavros/local_position/pose', PoseStamped, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.path_pub = rospy.Publisher('/drone_path', Path, queue_size=1)
        self.obstacles_pub = rospy.Publisher('/obstacles', MarkerArray, queue_size=1)
        self.target_pub = rospy.Publisher('/current_target', Marker, queue_size=1)
        self.avoidance_pub = rospy.Publisher('/avoidance_vector', Marker, queue_size=1)
        
        # TF broadcaster
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        # Таймер для обновления симуляции
        self.timer = rospy.Timer(rospy.Duration(1.0/self.update_rate), self.update_simulation)
        
        rospy.loginfo("Симуляция обхода препятствий запущена")
        
    def generate_obstacles(self):
        """Генерирует случайные препятствия"""
        # Очищаем список препятствий
        self.obstacles = []
        
        # Генерируем препятствия в области 10x10 метров
        for _ in range(self.obstacle_count):
            # Случайная позиция
            x = random.uniform(1.0, 9.0)
            y = random.uniform(-4.0, 4.0)
            
            # Случайный радиус от 0.2 до 0.5 метра
            radius = random.uniform(0.2, 0.5)
            
            self.obstacles.append((x, y, radius))
            
        rospy.loginfo(f"Сгенерировано {len(self.obstacles)} препятствий")
        
    def publish_obstacles(self):
        """Публикует маркеры препятствий для визуализации"""
        marker_array = MarkerArray()
        
        for i, (x, y, radius) in enumerate(self.obstacles):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "obstacles"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = self.drone_altitude / 2.0
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = radius * 2
            marker.scale.y = radius * 2
            marker.scale.z = self.drone_altitude
            
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.7
            
            marker_array.markers.append(marker)
            
        self.obstacles_pub.publish(marker_array)
        
    def publish_target(self):
        """Публикует маркер текущей цели"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "target"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = self.current_target[0]
        marker.pose.position.y = self.current_target[1]
        marker.pose.position.z = self.drone_altitude
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        
        self.target_pub.publish(marker)
        
    def publish_avoidance_vector(self, vector):
        """Публикует маркер вектора обхода препятствий"""
        if not vector or (vector[0] == 0 and vector[1] == 0):
            return
            
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "avoidance"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Начало стрелки в позиции дрона
        marker.points.append(Point(0, 0, 0))
        
        # Конец стрелки в направлении вектора обхода
        end_point = Point()
        end_point.x = vector[0]
        end_point.y = vector[1]
        end_point.z = 0
        marker.points.append(end_point)
        
        marker.scale.x = 0.1  # ширина стрелки
        marker.scale.y = 0.2  # ширина наконечника
        marker.scale.z = 0.2  # длина наконечника
        
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        self.avoidance_pub.publish(marker)
        
    def publish_scan(self):
        """Публикует симулированные данные лидара"""
        scan_msg = LaserScan()
        scan_msg.header.stamp = rospy.Time.now()
        scan_msg.header.frame_id = "lidar_link"
        
        # Параметры лидара
        angle_min = -math.pi
        angle_max = math.pi
        angle_increment = math.pi / 180  # 1 градус
        scan_time = 1.0 / self.update_rate
        range_min = 0.1
        range_max = 10.0
        
        # Количество лучей
        num_readings = int((angle_max - angle_min) / angle_increment) + 1
        
        # Инициализируем все лучи максимальной дальностью
        ranges = [range_max] * num_readings
        
        # Для каждого препятствия вычисляем пересечения с лучами
        for obstacle_x, obstacle_y, obstacle_radius in self.obstacles:
            # Относительная позиция препятствия от дрона
            rel_x = obstacle_x - self.drone_position[0]
            rel_y = obstacle_y - self.drone_position[1]
            
            # Расстояние до центра препятствия
            distance_to_center = math.sqrt(rel_x**2 + rel_y**2)
            
            # Если препятствие слишком далеко, пропускаем
            if distance_to_center > range_max + obstacle_radius:
                continue
                
            # Угол к центру препятствия
            angle_to_center = math.atan2(rel_y, rel_x)
            
            # Угловой размер препятствия
            if distance_to_center > obstacle_radius:
                angular_size = math.asin(obstacle_radius / distance_to_center)
            else:
                # Дрон внутри препятствия
                angular_size = math.pi / 2
                
            # Диапазон углов, в которых видно препятствие
            start_angle = angle_to_center - angular_size
            end_angle = angle_to_center + angular_size
            
            # Индексы лучей, пересекающих препятствие
            start_idx = max(0, min(num_readings-1, int((start_angle - angle_min) / angle_increment)))
            end_idx = max(0, min(num_readings-1, int((end_angle - angle_min) / angle_increment)))
            
            # Обновляем дальности для лучей, пересекающих препятствие
            for idx in range(start_idx, end_idx + 1):
                angle = angle_min + idx * angle_increment
                
                # Угол между лучом и направлением на центр препятствия
                angle_diff = abs(angle - angle_to_center)
                
                # Расстояние от центра луча до центра препятствия
                perpendicular_dist = distance_to_center * math.sin(angle_diff)
                
                # Если луч проходит через препятствие
                if perpendicular_dist < obstacle_radius:
                    # Расстояние до точки пересечения
                    intersection_dist = distance_to_center * math.cos(angle_diff) - math.sqrt(obstacle_radius**2 - perpendicular_dist**2)
                    
                    # Обновляем, если это ближайшее препятствие для данного луча
                    if intersection_dist > 0 and intersection_dist < ranges[idx]:
                        ranges[idx] = intersection_dist
        
        # Заполняем сообщение
        scan_msg.angle_min = angle_min
        scan_msg.angle_max = angle_max
        scan_msg.angle_increment = angle_increment
        scan_msg.time_increment = scan_time / num_readings
        scan_msg.scan_time = scan_time
        scan_msg.range_min = range_min
        scan_msg.range_max = range_max
        scan_msg.ranges = ranges
        
        self.scan_pub.publish(scan_msg)
        
    def publish_pose(self):
        """Публикует текущую позу дрона"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"
        
        pose_msg.pose.position.x = self.drone_position[0]
        pose_msg.pose.position.y = self.drone_position[1]
        pose_msg.pose.position.z = self.drone_position[2]
        
        pose_msg.pose.orientation.x = self.drone_orientation[0]
        pose_msg.pose.orientation.y = self.drone_orientation[1]
        pose_msg.pose.orientation.z = self.drone_orientation[2]
        pose_msg.pose.orientation.w = self.drone_orientation[3]
        
        self.pose_pub.publish(pose_msg)
        
    def publish_path(self):
        """Публикует пройденный путь дрона"""
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"
        
        for x, y in self.path_points:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = self.drone_altitude
            pose.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose)
            
        self.path_pub.publish(path_msg)
        
    def publish_tf(self):
        """Публикует трансформации TF"""
        # map -> base_link
        self.tf_broadcaster.sendTransform(
            (self.drone_position[0], self.drone_position[1], self.drone_position[2]),
            (self.drone_orientation[0], self.drone_orientation[1], self.drone_orientation[2], self.drone_orientation[3]),
            rospy.Time.now(),
            "base_link",
            "map"
        )
        
        # base_link -> lidar_link
        self.tf_broadcaster.sendTransform(
            (0, 0, 0.1),  # лидар немного выше центра дрона
            (0, 0, 0, 1),
            rospy.Time.now(),
            "lidar_link",
            "base_link"
        )
        
    def calculate_avoidance_vector(self):
        """Вычисляет вектор обхода препятствий на основе данных лидара"""
        # Симулируем лидар для внутреннего использования
        angle_min = -math.pi
        angle_max = math.pi
        angle_increment = math.pi / 180  # 1 градус
        num_readings = int((angle_max - angle_min) / angle_increment) + 1
        range_max = 10.0
        
        # Инициализируем все лучи максимальной дальностью
        ranges = [range_max] * num_readings
        
        # Для каждого препятствия вычисляем пересечения с лучами
        for obstacle_x, obstacle_y, obstacle_radius in self.obstacles:
            # Относительная позиция препятствия от дрона
            rel_x = obstacle_x - self.drone_position[0]
            rel_y = obstacle_y - self.drone_position[1]
            
            # Расстояние до центра препятствия
            distance_to_center = math.sqrt(rel_x**2 + rel_y**2)
            
            # Если препятствие слишком далеко, пропускаем
            if distance_to_center > range_max + obstacle_radius:
                continue
                
            # Угол к центру препятствия
            angle_to_center = math.atan2(rel_y, rel_x)
            
            # Угловой размер препятствия
            if distance_to_center > obstacle_radius:
                angular_size = math.asin(obstacle_radius / distance_to_center)
            else:
                # Дрон внутри препятствия
                angular_size = math.pi / 2
                
            # Диапазон углов, в которых видно препятствие
            start_angle = angle_to_center - angular_size
            end_angle = angle_to_center + angular_size
            
            # Индексы лучей, пересекающих препятствие
            start_idx = max(0, min(num_readings-1, int((start_angle - angle_min) / angle_increment)))
            end_idx = max(0, min(num_readings-1, int((end_angle - angle_min) / angle_increment)))
            
            # Обновляем дальности для лучей, пересекающих препятствие
            for idx in range(start_idx, end_idx + 1):
                angle = angle_min + idx * angle_increment
                
                # Угол между лучом и направлением на центр препятствия
                angle_diff = abs(angle - angle_to_center)
                
                # Расстояние от центра луча до центра препятствия
                perpendicular_dist = distance_to_center * math.sin(angle_diff)
                
                # Если луч проходит через препятствие
                if perpendicular_dist < obstacle_radius:
                    # Расстояние до точки пересечения
                    intersection_dist = distance_to_center * math.cos(angle_diff) - math.sqrt(obstacle_radius**2 - perpendicular_dist**2)
                    
                    # Обновляем, если это ближайшее препятствие для данного луча
                    if intersection_dist > 0 and intersection_dist < ranges[idx]:
                        ranges[idx] = intersection_dist
        
        # Вычисляем отталкивающие векторы от препятствий
        repulsive_x = 0.0
        repulsive_y = 0.0
        
        angles = [angle_min + i * angle_increment for i in range(num_readings)]
        
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
        
    def update_drone_position(self):
        """Обновляет позицию дрона на основе целевой точки и препятствий"""
        # Вычисляем направление к цели
        dx = self.current_target[0] - self.drone_position[0]
        dy = self.current_target[1] - self.drone_position[1]
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Если достигли цели, выбираем следующую
        if distance < 0.5:
            rospy.loginfo(f"Достигнута цель: ({self.current_target[0]}, {self.current_target[1]})")
            
            # Циклически меняем цель
            if self.current_target[0] == 5.0 and self.current_target[1] == 0.0:
                self.current_target = [5.0, 5.0]
            elif self.current_target[0] == 5.0 and self.current_target[1] == 5.0:
                self.current_target = [0.0, 5.0]
            elif self.current_target[0] == 0.0 and self.current_target[1] == 5.0:
                self.current_target = [0.0, 0.0]
            else:
                self.current_target = [5.0, 0.0]
                
            rospy.loginfo(f"Новая цель: ({self.current_target[0]}, {self.current_target[1]})")
            return
            
        # Нормализуем вектор направления
        if distance > 0:
            dx /= distance
            dy /= distance
            
        # Проверяем наличие препятствий и вычисляем вектор обхода
        avoidance_vector = self.calculate_avoidance_vector()
        
        # Если есть вектор обхода, комбинируем с направлением к цели
        if avoidance_vector and (avoidance_vector[0] != 0 or avoidance_vector[1] != 0):
            # Комбинируем с направлением к цели (30% цель, 70% обход)
            dx = 0.3 * dx + 0.7 * avoidance_vector[0]
            dy = 0.3 * dy + 0.7 * avoidance_vector[1]
            
            # Нормализуем
            magnitude = math.sqrt(dx*dx + dy*dy)
            if magnitude > 0:
                dx /= magnitude
                dy /= magnitude
                
            rospy.loginfo(f"Обход препятствия: вектор ({dx:.2f}, {dy:.2f})")
            
            # Публикуем вектор обхода для визуализации
            self.publish_avoidance_vector(avoidance_vector)
        
        # Масштабируем вектор движения
        step = self.drone_speed / self.update_rate
        dx *= step
        dy *= step
        
        # Обновляем позицию дрона
        self.drone_position[0] += dx
        self.drone_position[1] += dy
        
        # Обновляем ориентацию дрона (упрощенно)
        yaw = math.atan2(dy, dx)
        self.drone_orientation = tf.transformations.quaternion_from_euler(0, 0, yaw)
        
        # Добавляем точку к пути
        if len(self.path_points) == 0 or math.sqrt((self.path_points[-1][0] - self.drone_position[0])**2 + 
                                                 (self.path_points[-1][1] - self.drone_position[1])**2) > 0.1:
            self.path_points.append((self.drone_position[0], self.drone_position[1]))
        
    def update_simulation(self, event):
        """Основной цикл обновления симуляции"""
        # Обновляем позицию дрона
        self.update_drone_position()
        
        # Публикуем данные
        self.publish_pose()
        self.publish_scan()
        self.publish_tf()
        self.publish_path()
        self.publish_obstacles()
        self.publish_target()
        
        # Публикуем команду скорости для совместимости
        cmd_vel = Twist()
        cmd_vel.linear.x = self.drone_speed
        self.cmd_vel_pub.publish(cmd_vel)
        
    def run(self):
        """Запускает симуляцию"""
        rospy.loginfo("Запуск симуляции обхода препятствий...")
        rospy.spin()

if __name__ == '__main__':
    try:
        demo = SimulationDemo()
        demo.run()
    except rospy.ROSInterruptException:
        pass