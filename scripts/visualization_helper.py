#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import ColorRGBA

class VisualizationHelper:
    """
    Класс для улучшения визуализации в RViz
    """
    def __init__(self):
        rospy.init_node('visualization_helper')
        
        # Параметры
        self.marker_size = rospy.get_param('~marker_size', 1.0)
        self.update_rate = rospy.get_param('~update_rate', 5.0)  # Гц
        
        # Издатели
        self.marker_pub = rospy.Publisher('/visualization_markers', MarkerArray, queue_size=10)
        self.grid_pub = rospy.Publisher('/visualization_grid', OccupancyGrid, queue_size=10)
        
        # Подписчики
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        
        # Переменные
        self.tf_listener = tf.TransformListener()
        self.map_data = None
        self.last_drone_pos = None
        
        # Таймер для обновления маркеров
        self.timer = rospy.Timer(rospy.Duration(1.0/self.update_rate), self.update_markers)
        
        rospy.loginfo("Визуализация инициализирована")
    
    def map_callback(self, msg):
        """Обработчик получения карты"""
        self.map_data = msg
        rospy.loginfo("Получена карта размером %dx%d", msg.info.width, msg.info.height)
        
        # Публикуем карту снова для RViz
        self.grid_pub.publish(msg)
    
    def update_markers(self, event):
        """Обновление маркеров для визуализации"""
        markers = MarkerArray()
        
        # Добавляем маркер центра карты
        center_marker = self.create_marker(
            id=0,
            type=Marker.SPHERE,
            position=(0, 0, 0),
            scale=(self.marker_size, self.marker_size, self.marker_size),
            color=(0, 1, 0, 1)  # Зеленый
        )
        markers.markers.append(center_marker)
        
        # Добавляем маркер осей координат
        axes_marker = self.create_marker(
            id=1,
            type=Marker.LINE_LIST,
            position=(0, 0, 0),
            scale=(0.1, 0, 0),  # Толщина линии
            color=(1, 1, 1, 1)  # Белый
        )
        
        # Ось X (красная)
        p1, p2 = Point(), Point()
        p1.x, p1.y, p1.z = 0, 0, 0
        p2.x, p2.y, p2.z = 2, 0, 0
        axes_marker.points.append(p1)
        axes_marker.points.append(p2)
        axes_marker.colors.append(ColorRGBA(1, 0, 0, 1))
        axes_marker.colors.append(ColorRGBA(1, 0, 0, 1))
        
        # Ось Y (зеленая)
        p1, p2 = Point(), Point()
        p1.x, p1.y, p1.z = 0, 0, 0
        p2.x, p2.y, p2.z = 0, 2, 0
        axes_marker.points.append(p1)
        axes_marker.points.append(p2)
        axes_marker.colors.append(ColorRGBA(0, 1, 0, 1))
        axes_marker.colors.append(ColorRGBA(0, 1, 0, 1))
        
        # Ось Z (синяя)
        p1, p2 = Point(), Point()
        p1.x, p1.y, p1.z = 0, 0, 0
        p2.x, p2.y, p2.z = 0, 0, 2
        axes_marker.points.append(p1)
        axes_marker.points.append(p2)
        axes_marker.colors.append(ColorRGBA(0, 0, 1, 1))
        axes_marker.colors.append(ColorRGBA(0, 0, 1, 1))
        
        markers.markers.append(axes_marker)
        
        # Добавляем маркер границ карты, если карта доступна
        if self.map_data:
            grid_marker = self.create_marker(
                id=2,
                type=Marker.LINE_STRIP,
                position=(0, 0, 0),
                scale=(0.1, 0, 0),  # Толщина линии
                color=(1, 1, 0, 1)  # Желтый
            )
            
            # Получаем размеры и положение карты
            resolution = self.map_data.info.resolution
            width = self.map_data.info.width * resolution
            height = self.map_data.info.height * resolution
            origin_x = self.map_data.info.origin.position.x
            origin_y = self.map_data.info.origin.position.y
            
            # Создаем прямоугольник границы карты
            p1 = Point(origin_x, origin_y, 0)
            p2 = Point(origin_x + width, origin_y, 0)
            p3 = Point(origin_x + width, origin_y + height, 0)
            p4 = Point(origin_x, origin_y + height, 0)
            
            grid_marker.points.append(p1)
            grid_marker.points.append(p2)
            grid_marker.points.append(p3)
            grid_marker.points.append(p4)
            grid_marker.points.append(p1)  # Замыкаем контур
            
            markers.markers.append(grid_marker)
        
        # Добавляем маркер текущей позиции дрона
        try:
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            self.last_drone_pos = trans
            
            drone_marker = self.create_marker(
                id=3,
                type=Marker.ARROW,
                position=trans,
                orientation=rot,
                scale=(0.5, 0.1, 0.1),  # Размеры стрелки
                color=(1, 0, 1, 1)  # Пурпурный
            )
            markers.markers.append(drone_marker)
            
            # Добавляем текстовую метку с координатами
            text_marker = self.create_marker(
                id=4,
                type=Marker.TEXT_VIEW_FACING,
                position=(trans[0], trans[1], trans[2] + 0.5),
                scale=(0.5, 0.5, 0.5),
                color=(1, 1, 1, 1)  # Белый
            )
            text_marker.text = "X: {:.2f}, Y: {:.2f}, Z: {:.2f}".format(trans[0], trans[1], trans[2])
            markers.markers.append(text_marker)
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn_throttle(5.0, "Не удалось получить трансформацию map -> base_link")
        
        # Публикуем все маркеры
        self.marker_pub.publish(markers)
    
    def create_marker(self, id, type, position, scale, color, orientation=None):
        """Создание маркера с заданными параметрами"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "visualization_helper"
        marker.id = id
        marker.type = type
        marker.action = Marker.ADD
        
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        
        if orientation:
            marker.pose.orientation.x = orientation[0]
            marker.pose.orientation.y = orientation[1]
            marker.pose.orientation.z = orientation[2]
            marker.pose.orientation.w = orientation[3]
        else:
            marker.pose.orientation.w = 1.0
        
        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]
        
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        
        marker.lifetime = rospy.Duration(1.0 / self.update_rate * 2)  # Маркер живет в 2 раза дольше периода обновления
        
        return marker
    
    def run(self):
        """Запуск узла"""
        rospy.spin()

if __name__ == "__main__":
    try:
        helper = VisualizationHelper()
        helper.run()
    except rospy.ROSInterruptException:
        pass 