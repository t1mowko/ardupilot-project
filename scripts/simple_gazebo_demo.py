#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Twist
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

class SimpleGazeboDemo:
    def __init__(self):
        rospy.init_node('simple_gazebo_demo')
        
        # Параметры
        self.obstacle_threshold = 1.5  # метры
        self.safety_distance = 0.8     # метры
        self.max_linear_speed = 0.5    # м/с
        self.max_angular_speed = 0.5   # рад/с
        self.target_altitude = 1.5     # метры
        
        # Целевые точки для демонстрации (x, y)
        self.waypoints = [
            (5.0, 0.0),   # вперед на 5 метров
            (10.0, 0.0),  # дальше вперед
            (10.0, 5.0),  # вправо
            (0.0, 5.0),   # назад
            (0.0, 0.0)    # в исходную точку
        ]
        self.current_wp_idx = 0
        
        # Состояние
        self.current_position = None
        self.current_orientation = None
        self.scan_data = None
        self.drone_state = None
        self.path_points = []
        
        # Подписчики
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/mavros/state', State, self.state_callback)
        
        # Издатели
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.path_pub = rospy.Publisher('/drone_path', Path, queue_size=1)
        self.target_pub = rospy.Publisher('/current_target', Marker, queue_size=1)
        self.avoidance_pub = rospy.Publisher('/avoidance_vector', Marker, queue_size=1)
        
        # Сервисы
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/set_mode')
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        
        # Таймер для основного цикла
        self.timer = rospy.Timer(rospy.Duration(0.1), self.navigation_loop)
        
        rospy.loginfo("Простая демонстрация обхода препятствий в Gazebo запущена")
        
        # Инициализация дрона
        self.initialize_drone()
        
    def initialize_drone(self):
        """Инициализирует дрон: устанавливает режим GUIDED и армирует"""
        rospy.loginfo("Инициализация дрона...")
        
        # Ждем соединения с FCU
        rate = rospy.Rate(10)
        for _ in range(50):  # ждем до 5 секунд
            if self.drone_state and self.drone_state.connected:
                break
            rate.sleep()
        
        if not self.drone_state or not self.drone_state.connected:
            rospy.logwarn("Не удалось установить соединение с FCU")
            return
        
        # Устанавливаем режим GUIDED
        if not self.set_guided_mode():
            rospy.logerr("Не удалось установить режим GUIDED")
            return
        
        # Армируем дрон
        if not self.arm_drone():
            rospy.logerr("Не удалось армировать дрон")
            return
        
        # Взлетаем на заданную высоту
        self.takeoff()
        
    def set_guided_mode(self):
        """Устанавливает режим GUIDED"""
        if self.drone_state and self.drone_state.mode == "GUIDED":
            rospy.loginfo("Режим GUIDED уже установлен")
            return True
            
        rospy.loginfo("Установка режима GUIDED...")
        
        # Повторяем попытки до 5 раз
        for attempt in range(5):
            if self.set_mode_client(custom_mode="GUIDED").mode_sent:
                rospy.loginfo("Запрос на установку режима GUIDED отправлен")
                
                # Ждем подтверждения смены режима
                start_wait = rospy.Time.now()
                while (rospy.Time.now() - start_wait).to_sec() < 3.0:  # Ждем до 3 секунд
                    if self.drone_state and self.drone_state.mode == "GUIDED":
                        rospy.loginfo("Режим GUIDED установлен")
                        return True
                    rospy.sleep(0.1)
            
            rospy.logwarn("Попытка %d: Не удалось установить режим GUIDED", attempt+1)
            rospy.sleep(1.0)
        
        return False
    
    def arm_drone(self):
        """Армирует дрон"""
        if self.drone_state and self.drone_state.armed:
            rospy.loginfo("Дрон уже армирован")
            return True
            
        rospy.loginfo("Армирование дрона...")
        
        # Повторяем попытки до 5 раз
        for attempt in range(5):
            if self.arming_client(True).success:
                rospy.loginfo("Запрос на армирование отправлен")
                
                # Ждем подтверждения армирования
                start_wait = rospy.Time.now()
                while (rospy.Time.now() - start_wait).to_sec() < 3.0:  # Ждем до 3 секунд
                    if self.drone_state and self.drone_state.armed:
                        rospy.loginfo("Дрон успешно армирован")
                        return True
                    rospy.sleep(0.1)
            
            rospy.logwarn("Попытка %d: Не удалось армировать дрон", attempt+1)
            rospy.sleep(1.0)
        
        return False
    
    def takeoff(self):
        """Выполняет взлет на заданную высоту"""
        rospy.loginfo("Взлет на высоту %.1f метров...", self.target_altitude)
        
        # Отправляем команду взлета
        setpoint = PoseStamped()
        setpoint.header.frame_id = "map"
        setpoint.pose.position.z = self.target_altitude
        setpoint.pose.orientation.w = 1.0
        
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        
        while not rospy.is_shutdown() and (rospy.Time.now() - start_time).to_sec() < 10.0:
            setpoint.header.stamp = rospy.Time.now()
            self.setpoint_pub.publish(setpoint)
            rate.sleep()
            
            # Проверяем достижение высоты
            if self.current_position and abs(self.current_position.z - self.target_altitude) < 0.5:
                rospy.loginfo("Достигнута высота взлета")
                return True
        
        if self.current_position and abs(self.current_position.z - self.target_altitude) < 1.0:
            rospy.loginfo("Взлет завершен (не точно)")
            return True
        else:
            rospy.logwarn("Не удалось достичь высоты взлета")
            return False
    
    def scan_callback(self, msg):
        """Обработчик данных лидара"""
        self.scan_data = msg
        
    def pose_callback(self, msg):
        """Обработчик позиции дрона"""
        self.current_position = msg.pose.position
        self.current_orientation = msg.pose.orientation
        
        # Добавляем точку к пути
        if len(self.path_points) == 0 or math.sqrt((self.path_points[-1][0] - self.current_position.x)**2 + 
                                                 (self.path_points[-1][1] - self.current_position.y)**2) > 0.2:
            self.path_points.append((self.current_position.x, self.current_position.y, self.current_position.z))
            self.publish_path()
        
    def state_callback(self, msg):
        """Обработчик состояния дрона"""
        self.drone_state = msg
    
    def publish_path(self):
        """Публикует пройденный путь дрона"""
        if not self.path_points:
            return
            
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"
        
        for x, y, z in self.path_points:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose)
            
        self.path_pub.publish(path_msg)
    
    def publish_target_marker(self, target):
        """Публикует маркер целевой точки"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "target_points"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = target[0]
        marker.pose.position.y = target[1]
        marker.pose.position.z = self.target_altitude
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        
        self.target_pub.publish(marker)
    
    def publish_avoidance_vector(self, start, direction):
        """Публикует вектор обхода препятствий"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "avoidance"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Начальная точка
        start_point = Point()
        start_point.x = start[0]
        start_point.y = start[1]
        start_point.z = start[2]
        marker.points.append(start_point)
        
        # Конечная точка
        end_point = Point()
        end_point.x = start[0] + direction[0]
        end_point.y = start[1] + direction[1]
        end_point.z = start[2]
        marker.points.append(end_point)
        
        marker.scale.x = 0.1  # диаметр стрелки
        marker.scale.y = 0.2  # диаметр наконечника
        marker.scale.z = 0.2  # длина наконечника
        
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        self.avoidance_pub.publish(marker)
    
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
    
    def calculate_avoidance_direction(self):
        """Вычисляет направление обхода препятствий"""
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
            
        dx = wp[0] - self.current_position.x
        dy = wp[1] - self.current_position.y
        return math.sqrt(dx*dx + dy*dy)
    
    def navigation_loop(self, event):
        """Основной цикл навигации"""
        if not self.current_position or not self.drone_state or not self.drone_state.armed:
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
        dx = target[0] - self.current_position.x
        dy = target[1] - self.current_position.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Нормализуем вектор направления
        if distance > 0:
            dx /= distance
            dy /= distance
            
        # Проверяем наличие препятствий
        if self.is_obstacle_ahead():
            # Вычисляем вектор обхода
            avoid_x, avoid_y = self.calculate_avoidance_direction()
            
            # Комбинируем с направлением к цели (70% обход, 30% цель)
            dx = 0.3 * dx + 0.7 * avoid_x
            dy = 0.3 * dy + 0.7 * avoid_y
            
            # Нормализуем
            magnitude = math.sqrt(dx*dx + dy*dy)
            if magnitude > 0:
                dx /= magnitude
                dy /= magnitude
                
            rospy.loginfo("Обход препятствия: вектор (%.2f, %.2f)", dx, dy)
            
            # Публикуем вектор обхода
            if self.current_position:
                self.publish_avoidance_vector(
                    [self.current_position.x, self.current_position.y, self.current_position.z],
                    [dx, dy, 0]
                )
        
        # Создаем команду скорости
        cmd_vel = Twist()
        cmd_vel.linear.x = dx * self.max_linear_speed
        cmd_vel.linear.y = dy * self.max_linear_speed
        cmd_vel.linear.z = 0
        
        # Публикуем команду скорости
        self.cmd_vel_pub.publish(cmd_vel)
        
        # Публикуем целевую точку
        self.publish_target_marker(target)
    
    def run(self):
        """Запускает демонстрацию"""
        rospy.loginfo("Запуск демонстрации обхода препятствий...")
        rospy.spin()

if __name__ == '__main__':
    try:
        demo = SimpleGazeboDemo()
        demo.run()
    except rospy.ROSInterruptException:
        pass 