#!/usr/bin/env python3
import rospy
import numpy as np
import tf
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import math

class GlobalPlanner:
    def __init__(self):
        rospy.init_node('drone_global_planner')
        
        # Параметры
        self.plan_frequency = rospy.get_param('~plan_frequency', 1.0)  # Гц
        self.altitude = rospy.get_param('~altitude', 1.5)  # м
        self.use_markers = rospy.get_param('~use_markers', True)
        
        # Переменные состояния
        self.costmap = None
        self.current_pose = None
        self.goal_pose = None
        self.current_path = None
        
        # TF listener
        self.tf_listener = tf.TransformListener()
        
        # Подписки
        rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.costmap_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        
        # Публикаторы
        self.path_pub = rospy.Publisher('/drone_global_planner/path', Path, queue_size=1)
        self.markers_pub = rospy.Publisher('/drone_global_planner/path_markers', MarkerArray, queue_size=1)
        
        # Таймер для планирования
        rospy.Timer(rospy.Duration(1.0/self.plan_frequency), self.planning_loop)
        
        rospy.loginfo("Глобальный планировщик дрона инициализирован")
    
    def costmap_callback(self, costmap_msg):
        self.costmap = costmap_msg
        rospy.logdebug("Получена новая карта препятствий")
    
    def goal_callback(self, goal_msg):
        self.goal_pose = goal_msg
        rospy.loginfo("Получена новая целевая точка: %.2f, %.2f", 
                    goal_msg.pose.position.x, goal_msg.pose.position.y)
        
        # Сразу пересчитываем путь
        self.calculate_path()
    
    def get_current_pose(self):
        try:
            # Получаем текущую позицию дрона из TF
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            
            current_pose = PoseStamped()
            current_pose.header.frame_id = 'map'
            current_pose.header.stamp = rospy.Time.now()
            current_pose.pose.position.x = trans[0]
            current_pose.pose.position.y = trans[1]
            current_pose.pose.position.z = trans[2]
            current_pose.pose.orientation.x = rot[0]
            current_pose.pose.orientation.y = rot[1]
            current_pose.pose.orientation.z = rot[2]
            current_pose.pose.orientation.w = rot[3]
            
            self.current_pose = current_pose
            return True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Не удалось получить текущую позицию дрона")
            return False
    
    def calculate_path(self):
        """Расчет глобального пути с использованием A* алгоритма"""
        if not self.costmap or not self.goal_pose:
            return
        
        if not self.get_current_pose():
            return
        
        rospy.loginfo("Начинаем расчет глобального пути")
        
        # Получаем данные из карты препятствий
        width = self.costmap.info.width
        height = self.costmap.info.height
        resolution = self.costmap.info.resolution
        origin_x = self.costmap.info.origin.position.x
        origin_y = self.costmap.info.origin.position.y
        
        # Конвертируем текущую позицию и цель в индексы сетки
        start_x = int((self.current_pose.pose.position.x - origin_x) / resolution)
        start_y = int((self.current_pose.pose.position.y - origin_y) / resolution)
        goal_x = int((self.goal_pose.pose.position.x - origin_x) / resolution)
        goal_y = int((self.goal_pose.pose.position.y - origin_y) / resolution)
        
        # Проверяем границы
        start_x = max(0, min(start_x, width - 1))
        start_y = max(0, min(start_y, height - 1))
        goal_x = max(0, min(goal_x, width - 1))
        goal_y = max(0, min(goal_y, height - 1))
        
        # Реализация A* алгоритма
        # Для упрощения, здесь мы используем заглушку - прямую линию между точками
        # В реальном планировщике здесь должна быть полная реализация A*
        path_x = np.linspace(start_x, goal_x, 20)
        path_y = np.linspace(start_y, goal_y, 20)
        
        # Создаем сообщение Path
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = 'map'
        
        # Заполняем точки пути
        for i in range(len(path_x)):
            # Конвертируем обратно в мировые координаты
            world_x = path_x[i] * resolution + origin_x
            world_y = path_y[i] * resolution + origin_y
            
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = 'map'
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.position.z = self.altitude  # Установленная высота полета
            
            # Для ориентации - дрон смотрит в направлении движения
            if i < len(path_x) - 1:
                dx = path_x[i+1] - path_x[i]
                dy = path_y[i+1] - path_y[i]
                yaw = math.atan2(dy, dx)
                
                # Конвертируем yaw в кватернион
                quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
                pose.pose.orientation.x = quat[0]
                pose.pose.orientation.y = quat[1]
                pose.pose.orientation.z = quat[2]
                pose.pose.orientation.w = quat[3]
            else:
                # Последняя точка наследует ориентацию предыдущей
                pose.pose.orientation = path_msg.poses[-1].pose.orientation if path_msg.poses else self.goal_pose.pose.orientation
            
            path_msg.poses.append(pose)
        
        self.current_path = path_msg
        self.path_pub.publish(path_msg)
        
        # Визуализация пути с помощью маркеров
        if self.use_markers:
            self.visualize_path(path_msg)
        
        rospy.loginfo("Глобальный путь рассчитан, количество точек: %d", len(path_msg.poses))
    
    def visualize_path(self, path_msg):
        """Визуализирует путь с помощью маркеров в RViz"""
        if not path_msg.poses:
            return
        
        marker_array = MarkerArray()
        
        # Маркер для линии пути
        line_marker = Marker()
        line_marker.header.frame_id = 'map'
        line_marker.header.stamp = rospy.Time.now()
        line_marker.ns = 'path_line'
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.1  # толщина линии
        line_marker.color = ColorRGBA(0.0, 0.5, 1.0, 0.8)  # синий
        line_marker.pose.orientation.w = 1.0
        
        # Добавляем точки в линию
        for pose in path_msg.poses:
            p = Point()
            p.x = pose.pose.position.x
            p.y = pose.pose.position.y
            p.z = pose.pose.position.z
            line_marker.points.append(p)
        
        marker_array.markers.append(line_marker)
        
        # Маркеры для каждой точки пути
        for i, pose in enumerate(path_msg.poses):
            point_marker = Marker()
            point_marker.header.frame_id = 'map'
            point_marker.header.stamp = rospy.Time.now()
            point_marker.ns = 'path_points'
            point_marker.id = i + 1
            point_marker.type = Marker.SPHERE
            point_marker.action = Marker.ADD
            point_marker.pose = pose.pose
            point_marker.scale.x = 0.2
            point_marker.scale.y = 0.2
            point_marker.scale.z = 0.2
            
            # Первая и последняя точки другого цвета
            if i == 0:
                point_marker.color = ColorRGBA(0.0, 1.0, 0.0, 0.8)  # зеленый - старт
            elif i == len(path_msg.poses) - 1:
                point_marker.color = ColorRGBA(1.0, 0.0, 0.0, 0.8)  # красный - финиш
            else:
                point_marker.color = ColorRGBA(1.0, 1.0, 0.0, 0.6)  # желтый - промежуточные точки
            
            marker_array.markers.append(point_marker)
        
        self.markers_pub.publish(marker_array)
    
    def planning_loop(self, event):
        """Основной цикл планирования"""
        if self.current_pose and self.goal_pose and (not self.current_path or self.need_replan()):
            self.calculate_path()
    
    def need_replan(self):
        """Проверяет, нужно ли пересчитать путь"""
        # Здесь можно добавить логику, которая определяет, когда нужно пересчитать путь
        # Например, когда дрон отклонился от текущего пути на определенное расстояние
        return False  # Заглушка

if __name__ == '__main__':
    try:
        planner = GlobalPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 