#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Path, Odometry
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import math
import time

class LocalPlanner:
    def __init__(self):
        rospy.init_node('local_planner')
        
        # Параметры планировщика
        self.obstacle_threshold = rospy.get_param('~obstacle_threshold', 0.8)  # метры
        self.safety_distance = rospy.get_param('~safety_distance', 0.5)       # метры
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 0.5)     # м/с
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 0.5)   # рад/с
        self.look_ahead_distance = rospy.get_param('~look_ahead_distance', 1.0)  # метры
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.2)  # метры
        self.use_vector_field = rospy.get_param('~use_vector_field', True)  # использовать векторное поле для обхода
        self.debug_visualization = rospy.get_param('~debug_visualization', True)  # визуализация для отладки
        
        # Состояние дрона
        self.current_pose = None
        self.current_path = None
        self.scan_data = None
        self.last_cmd = Twist()
        self.last_obstacle_time = 0
        self.obstacle_memory_time = 2.0  # время "памяти" об обнаруженном препятствии (сек)
        self.goal_reached = False
        
        # Публикаторы и подписчики
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.local_plan_pub = rospy.Publisher('/local_planner/current_plan', Path, queue_size=1)
        self.markers_pub = rospy.Publisher('/local_planner/debug_markers', MarkerArray, queue_size=1)
        
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/mavros/local_position/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/move_base/NavfnROS/plan', Path, self.path_callback)
        
        # Таймер для основного цикла планирования
        self.timer = rospy.Timer(rospy.Duration(0.1), self.planning_loop)
        
        rospy.loginfo("Локальный планировщик инициализирован")
    
    def scan_callback(self, scan_msg):
        self.scan_data = scan_msg
    
    def odom_callback(self, odom_msg):
        self.current_pose = odom_msg.pose.pose
    
    def path_callback(self, path_msg):
        if path_msg.poses:
            self.current_path = path_msg
            self.goal_reached = False
            rospy.loginfo("Получен новый глобальный план с %d точками", len(path_msg.poses))
    
    def get_target_point(self):
        """Находит следующую целевую точку из глобального пути с учетом look_ahead_distance"""
        if not self.current_path or not self.current_pose:
            return None
        
        min_dist = float('inf')
        closest_idx = 0
        
        # Находим ближайшую точку на пути
        for i, pose in enumerate(self.current_path.poses):
            dx = pose.pose.position.x - self.current_pose.position.x
            dy = pose.pose.position.y - self.current_pose.position.y
            dist = math.sqrt(dx*dx + dy*dy)
            
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # Проверяем, достигли ли мы конечной точки
        if closest_idx >= len(self.current_path.poses) - 3:
            last_point = self.current_path.poses[-1]
            dx = last_point.pose.position.x - self.current_pose.position.x
            dy = last_point.pose.position.y - self.current_pose.position.y
            dist_to_goal = math.sqrt(dx*dx + dy*dy)
            
            if dist_to_goal < self.goal_tolerance:
                if not self.goal_reached:
                    rospy.loginfo("Достигнута конечная точка пути!")
                    self.goal_reached = True
                return None
        
        # Выбираем точку впереди на расстоянии look_ahead_distance
        look_ahead_idx = closest_idx
        look_ahead_dist = 0
        
        while look_ahead_idx < len(self.current_path.poses) - 1 and look_ahead_dist < self.look_ahead_distance:
            curr_pose = self.current_path.poses[look_ahead_idx]
            next_pose = self.current_path.poses[look_ahead_idx + 1]
            
            dx = next_pose.pose.position.x - curr_pose.pose.position.x
            dy = next_pose.pose.position.y - curr_pose.pose.position.y
            segment_dist = math.sqrt(dx*dx + dy*dy)
            
            look_ahead_dist += segment_dist
            look_ahead_idx += 1
        
        return self.current_path.poses[look_ahead_idx]
    
    def is_path_blocked(self):
        """Проверяет, заблокирован ли путь препятствием"""
        if not self.scan_data:
            return False
        
        # Проверяем препятствия в секторе перед дроном
        ranges = np.array(self.scan_data.ranges)
        angle_min = self.scan_data.angle_min
        angle_max = self.scan_data.angle_max
        angle_increment = self.scan_data.angle_increment
        
        # Определяем сектор перед дроном (±30 градусов)
        front_sector = int(len(ranges) * 30 / (180 * (angle_max - angle_min) / math.pi))
        mid_idx = len(ranges) // 2
        front_ranges = ranges[mid_idx-front_sector:mid_idx+front_sector]
        
        # Заменяем inf и nan значения на максимальную дальность
        front_ranges = np.nan_to_num(front_ranges, nan=self.scan_data.range_max, posinf=self.scan_data.range_max)
        
        # Если есть препятствие ближе чем obstacle_threshold
        if np.any(front_ranges < self.obstacle_threshold) and np.any(front_ranges > 0.1):
            self.last_obstacle_time = time.time()
            return True
            
        # Проверяем "память" о препятствии
        if time.time() - self.last_obstacle_time < self.obstacle_memory_time:
            return True
            
        return False
    
    def calculate_avoidance_direction(self):
        """Вычисляет направление для избегания препятствий с использованием векторного поля"""
        if not self.scan_data:
            return 0.0
        
        ranges = np.array(self.scan_data.ranges)
        # Заменяем inf и nan значения на максимальную дальность
        max_range = self.scan_data.range_max
        ranges = np.nan_to_num(ranges, nan=max_range, posinf=max_range)
        
        # Получаем углы для каждого луча лидара
        angles = np.linspace(self.scan_data.angle_min, self.scan_data.angle_max, len(ranges))
        
        if self.use_vector_field:
            # Векторное поле для обхода препятствий
            # Каждое препятствие создает отталкивающий вектор, обратно пропорциональный расстоянию
            repulsive_x = 0
            repulsive_y = 0
            
            # Вычисляем отталкивающие векторы от препятствий
            for i, r in enumerate(ranges):
                if r < self.obstacle_threshold and r > 0.1:
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
                
                # Вычисляем угол отталкивающего вектора
                avoid_angle = math.atan2(repulsive_y, repulsive_x)
                
                # Визуализация для отладки
                if self.debug_visualization:
                    self.visualize_vector(repulsive_x, repulsive_y, "repulsive")
                
                return avoid_angle
            
        # Если векторное поле не используется или нет отталкивающих векторов,
        # используем простой алгоритм поиска максимального свободного пространства
        
        # Создаем веса в зависимости от угла (предпочитаем движение вперед)
        direction_weights = np.cos(angles) * 0.5 + 0.5  # от 0 до 1
        
        # Добавляем веса на основе расстояний до препятствий
        distance_weights = np.clip(ranges, 0, max_range) / max_range
        
        # Объединяем веса
        total_weights = direction_weights * distance_weights
        
        # Находим лучшее направление
        best_idx = np.argmax(total_weights)
        best_angle = angles[best_idx]
        
        return best_angle
    
    def visualize_vector(self, x, y, name, color=None):
        """Визуализирует вектор в RViz"""
        if not self.current_pose:
            return
            
        if color is None:
            if name == "repulsive":
                color = ColorRGBA(1.0, 0.0, 0.0, 0.8)  # красный
            elif name == "target":
                color = ColorRGBA(0.0, 1.0, 0.0, 0.8)  # зеленый
            else:
                color = ColorRGBA(0.0, 0.0, 1.0, 0.8)  # синий
        
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = name
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Начальная точка - текущая позиция дрона
        marker.points.append(Point(0, 0, 0))
        
        # Конечная точка - направление вектора
        scale = 2.0  # масштаб для визуализации
        marker.points.append(Point(x * scale, y * scale, 0))
        
        marker.scale.x = 0.1  # толщина стрелки
        marker.scale.y = 0.2  # ширина наконечника
        marker.scale.z = 0.2
        marker.color = color
        
        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        self.markers_pub.publish(marker_array)
    
    def planning_loop(self, event):
        """Основной цикл планирования"""
        if not self.current_pose or not self.scan_data:
            return
        
        cmd = Twist()
        
        # Проверяем, заблокирован ли путь
        path_blocked = self.is_path_blocked()
        
        if path_blocked:
            # Режим избегания препятствий
            rospy.logdebug("Препятствие обнаружено, выполняем маневр")
            avoid_angle = self.calculate_avoidance_direction()
            
            # Получаем текущий угол дрона
            orientation_q = self.current_pose.orientation
            _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
            
            # Вычисляем разницу углов для поворота
            angle_diff = avoid_angle - yaw
            # Нормализация угла в диапазоне [-pi, pi]
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            # П-регулятор для угловой скорости
            cmd.angular.z = min(max(1.0 * angle_diff, -self.max_angular_speed), self.max_angular_speed)
            
            # Если препятствие очень близко - замедляемся
            min_front_dist = min(np.array(self.scan_data.ranges)[len(self.scan_data.ranges)//2-10:len(self.scan_data.ranges)//2+10])
            if min_front_dist < self.safety_distance:
                cmd.linear.x = 0.0
            else:
                cmd.linear.x = self.max_linear_speed * 0.4  # Уменьшаем скорость при объезде
        else:
            # Режим следования по пути
            target = self.get_target_point()
            
            if target:
                # Вычисляем направление к целевой точке
                dx = target.pose.position.x - self.current_pose.position.x
                dy = target.pose.position.y - self.current_pose.position.y
                target_angle = math.atan2(dy, dx)
                
                # Визуализация целевой точки для отладки
                if self.debug_visualization:
                    self.visualize_vector(math.cos(target_angle), math.sin(target_angle), "target")
                
                # Текущий угол дрона
                orientation_q = self.current_pose.orientation
                _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
                
                # Разница углов
                angle_diff = target_angle - yaw
                # Нормализация угла в диапазоне [-pi, pi]
                while angle_diff > math.pi:
                    angle_diff -= 2 * math.pi
                while angle_diff < -math.pi:
                    angle_diff += 2 * math.pi
                
                # П-регулятор для угловой скорости с плавным управлением
                cmd.angular.z = min(max(0.8 * angle_diff, -self.max_angular_speed), self.max_angular_speed)
                
                # Линейная скорость зависит от угла до цели и расстояния до цели
                distance = math.sqrt(dx*dx + dy*dy)
                speed_factor = max(0, 1 - abs(angle_diff) / math.pi)
                
                # Уменьшаем скорость при приближении к цели
                if distance < self.look_ahead_distance:
                    speed_factor *= distance / self.look_ahead_distance
                    
                cmd.linear.x = self.max_linear_speed * speed_factor
            else:
                # Нет целевой точки или достигли цели - останавливаемся
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
        
        # Сглаживание команд для предотвращения резких движений
        alpha = 0.3  # коэффициент сглаживания (0-1)
        cmd.linear.x = alpha * cmd.linear.x + (1 - alpha) * self.last_cmd.linear.x
        cmd.angular.z = alpha * cmd.angular.z + (1 - alpha) * self.last_cmd.angular.z
        
        # Сохраняем текущую команду
        self.last_cmd = cmd
        
        # Публикуем команду управления
        self.cmd_vel_pub.publish(cmd)

if __name__ == '__main__':
    try:
        planner = LocalPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
