#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

class CostmapDebugger:
    """
    Скрипт для отладки позиции на карте затрат
    """
    
    def __init__(self):
        rospy.init_node('costmap_debugger')
        
        # Параметры
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        
        # TF слушатель
        self.tf_listener = tf.TransformListener()
        
        # Подписчики
        self.global_costmap_sub = rospy.Subscriber('/move_base/global_costmap/costmap', 
                                                OccupancyGrid, self.global_costmap_callback)
        self.local_costmap_sub = rospy.Subscriber('/move_base/local_costmap/costmap', 
                                               OccupancyGrid, self.local_costmap_callback)
        
        # Издатели
        self.marker_pub = rospy.Publisher('/costmap_debug/markers', MarkerArray, queue_size=1)
        self.point_pub = rospy.Publisher('/costmap_debug/robot_position', PointStamped, queue_size=1)
        
        # Состояние
        self.global_costmap = None
        self.local_costmap = None
        
        rospy.loginfo("Отладчик карты затрат инициализирован")
        
    def global_costmap_callback(self, msg):
        """Обработчик сообщений с глобальной картой затрат"""
        self.global_costmap = msg
        rospy.loginfo("Получена глобальная карта затрат: размер %dx%d, разрешение %.3f", 
                     msg.info.width, msg.info.height, msg.info.resolution)
        rospy.loginfo("Начало глобальной карты затрат: x=%.2f, y=%.2f", 
                     msg.info.origin.position.x, msg.info.origin.position.y)
        
        # Проверяем позицию робота на карте
        self.check_robot_position()
        
    def local_costmap_callback(self, msg):
        """Обработчик сообщений с локальной картой затрат"""
        self.local_costmap = msg
        rospy.loginfo("Получена локальная карта затрат: размер %dx%d, разрешение %.3f", 
                     msg.info.width, msg.info.height, msg.info.resolution)
        rospy.loginfo("Начало локальной карты затрат: x=%.2f, y=%.2f", 
                     msg.info.origin.position.x, msg.info.origin.position.y)
        
    def get_robot_position(self):
        """Получение текущей позиции робота"""
        try:
            # Пытаемся получить позицию из TF
            trans, _ = self.tf_listener.lookupTransform(self.map_frame, self.base_frame, rospy.Time(0))
            return trans[0], trans[1], trans[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Не удалось получить позицию робота из TF")
            return None
            
    def is_position_on_costmap(self, x, y, costmap):
        """Проверка, находится ли позиция на карте затрат"""
        if costmap is None:
            return False
            
        info = costmap.info
        
        # Проверяем, находится ли точка в пределах карты
        map_x = (x - info.origin.position.x) / info.resolution
        map_y = (y - info.origin.position.y) / info.resolution
        
        if (map_x < 0 or map_x >= info.width or 
            map_y < 0 or map_y >= info.height):
            return False
            
        return True
        
    def check_robot_position(self):
        """Проверка позиции робота на карте затрат"""
        robot_pos = self.get_robot_position()
        if robot_pos is None:
            return
            
        x, y, z = robot_pos
        
        # Публикуем текущую позицию робота
        point = PointStamped()
        point.header.frame_id = self.map_frame
        point.header.stamp = rospy.Time.now()
        point.point.x = x
        point.point.y = y
        point.point.z = z
        self.point_pub.publish(point)
        
        # Проверяем, находится ли робот на глобальной карте затрат
        if self.global_costmap:
            on_global = self.is_position_on_costmap(x, y, self.global_costmap)
            rospy.loginfo("Позиция робота (%.2f, %.2f) %s в пределах глобальной карты затрат", 
                         x, y, "находится" if on_global else "НЕ находится")
            
            # Вычисляем индексы ячейки на карте
            info = self.global_costmap.info
            cell_x = int((x - info.origin.position.x) / info.resolution)
            cell_y = int((y - info.origin.position.y) / info.resolution)
            
            if on_global:
                rospy.loginfo("Индексы ячейки на глобальной карте: (%d, %d)", cell_x, cell_y)
                
                # Вычисляем границы карты
                min_x = info.origin.position.x
                min_y = info.origin.position.y
                max_x = min_x + info.width * info.resolution
                max_y = min_y + info.height * info.resolution
                
                rospy.loginfo("Границы глобальной карты: X=[%.2f, %.2f], Y=[%.2f, %.2f]", 
                             min_x, max_x, min_y, max_y)
            else:
                rospy.logwarn("Робот находится вне глобальной карты затрат!")
                
        # Проверяем, находится ли робот на локальной карте затрат
        if self.local_costmap:
            on_local = self.is_position_on_costmap(x, y, self.local_costmap)
            rospy.loginfo("Позиция робота (%.2f, %.2f) %s в пределах локальной карты затрат", 
                         x, y, "находится" if on_local else "НЕ находится")
            
        # Визуализируем границы карт
        self.visualize_costmap_bounds()
        
    def visualize_costmap_bounds(self):
        """Визуализация границ карт затрат"""
        markers = MarkerArray()
        marker_id = 0
        
        # Визуализация глобальной карты
        if self.global_costmap:
            info = self.global_costmap.info
            min_x = info.origin.position.x
            min_y = info.origin.position.y
            max_x = min_x + info.width * info.resolution
            max_y = min_y + info.height * info.resolution
            
            global_marker = Marker()
            global_marker.header.frame_id = self.map_frame
            global_marker.header.stamp = rospy.Time.now()
            global_marker.ns = "costmap_bounds"
            global_marker.id = marker_id
            marker_id += 1
            global_marker.type = Marker.LINE_STRIP
            global_marker.action = Marker.ADD
            global_marker.scale.x = 0.1  # Толщина линии
            global_marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)  # Зеленый
            global_marker.pose.orientation.w = 1.0
            
            # Добавляем точки для прямоугольника
            global_marker.points = []
            p1 = self.create_point(min_x, min_y, 0.1)
            p2 = self.create_point(max_x, min_y, 0.1)
            p3 = self.create_point(max_x, max_y, 0.1)
            p4 = self.create_point(min_x, max_y, 0.1)
            p5 = self.create_point(min_x, min_y, 0.1)  # Замыкаем контур
            
            global_marker.points.extend([p1, p2, p3, p4, p5])
            markers.markers.append(global_marker)
            
        # Визуализация локальной карты
        if self.local_costmap:
            info = self.local_costmap.info
            min_x = info.origin.position.x
            min_y = info.origin.position.y
            max_x = min_x + info.width * info.resolution
            max_y = min_y + info.height * info.resolution
            
            local_marker = Marker()
            local_marker.header.frame_id = self.map_frame
            local_marker.header.stamp = rospy.Time.now()
            local_marker.ns = "costmap_bounds"
            local_marker.id = marker_id
            marker_id += 1
            local_marker.type = Marker.LINE_STRIP
            local_marker.action = Marker.ADD
            local_marker.scale.x = 0.1  # Толщина линии
            local_marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)  # Красный
            local_marker.pose.orientation.w = 1.0
            
            # Добавляем точки для прямоугольника
            local_marker.points = []
            p1 = self.create_point(min_x, min_y, 0.2)
            p2 = self.create_point(max_x, min_y, 0.2)
            p3 = self.create_point(max_x, max_y, 0.2)
            p4 = self.create_point(min_x, max_y, 0.2)
            p5 = self.create_point(min_x, min_y, 0.2)  # Замыкаем контур
            
            local_marker.points.extend([p1, p2, p3, p4, p5])
            markers.markers.append(local_marker)
            
        # Публикуем маркеры
        if markers.markers:
            self.marker_pub.publish(markers)
            
    def create_point(self, x, y, z):
        """Создание точки для маркера"""
        from geometry_msgs.msg import Point
        p = Point()
        p.x = x
        p.y = y
        p.z = z
        return p
        
    def run(self):
        """Основная функция работы"""
        rate = rospy.Rate(1)  # 1 Гц
        
        while not rospy.is_shutdown():
            # Проверяем позицию робота на карте
            if self.global_costmap or self.local_costmap:
                self.check_robot_position()
                
            rate.sleep()

if __name__ == '__main__':
    try:
        debugger = CostmapDebugger()
        debugger.run()
    except rospy.ROSInterruptException:
        pass 