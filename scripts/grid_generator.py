#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point

class GridGenerator:
    """
    Скрипт для генерации сетки в RViz для визуализации
    """
    
    def __init__(self):
        rospy.init_node('grid_generator')
        
        # Параметры
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.grid_size = rospy.get_param('~grid_size', 20)  # размер сетки в метрах
        self.cell_size = rospy.get_param('~cell_size', 1.0)  # размер ячейки в метрах
        
        # Издатель маркеров
        self.marker_pub = rospy.Publisher('/grid_markers', MarkerArray, queue_size=1)
        
        # Генерация и публикация сетки
        self.generate_grid()
        
        # Таймер для периодической публикации
        self.timer = rospy.Timer(rospy.Duration(1.0), self.publish_grid)
        
        rospy.loginfo("Генератор сетки запущен")
    
    def generate_grid(self):
        """
        Генерирует сетку для визуализации
        """
        self.marker_array = MarkerArray()
        
        # Создаем линии сетки
        marker_id = 0
        
        # Горизонтальные линии
        for i in range(-self.grid_size//2, self.grid_size//2 + 1):
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = rospy.Time.now()
            marker.ns = "grid_lines"
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            
            # Начальная и конечная точки линии
            start_point = Point()
            start_point.x = -self.grid_size / 2.0
            start_point.y = i * self.cell_size
            start_point.z = 0.0
            
            end_point = Point()
            end_point.x = self.grid_size / 2.0
            end_point.y = i * self.cell_size
            end_point.z = 0.0
            
            marker.points = [start_point, end_point]
            
            # Цвет и размер
            marker.scale.x = 0.05  # толщина линии
            marker.color = ColorRGBA(0.8, 0.8, 0.8, 0.5)  # светло-серый цвет
            
            # Добавляем маркер в массив
            self.marker_array.markers.append(marker)
        
        # Вертикальные линии
        for i in range(-self.grid_size//2, self.grid_size//2 + 1):
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = rospy.Time.now()
            marker.ns = "grid_lines"
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            
            # Начальная и конечная точки линии
            start_point = Point()
            start_point.x = i * self.cell_size
            start_point.y = -self.grid_size / 2.0
            start_point.z = 0.0
            
            end_point = Point()
            end_point.x = i * self.cell_size
            end_point.y = self.grid_size / 2.0
            end_point.z = 0.0
            
            marker.points = [start_point, end_point]
            
            # Цвет и размер
            marker.scale.x = 0.05  # толщина линии
            marker.color = ColorRGBA(0.8, 0.8, 0.8, 0.5)  # светло-серый цвет
            
            # Добавляем маркер в массив
            self.marker_array.markers.append(marker)
        
        # Добавляем оси координат
        # Ось X (красная)
        x_axis = Marker()
        x_axis.header.frame_id = self.map_frame
        x_axis.header.stamp = rospy.Time.now()
        x_axis.ns = "axes"
        x_axis.id = marker_id
        marker_id += 1
        x_axis.type = Marker.ARROW
        x_axis.action = Marker.ADD
        
        # Начальная и конечная точки
        x_axis.points.append(Point(0, 0, 0))
        x_axis.points.append(Point(1.0, 0, 0))
        
        # Цвет и размер
        x_axis.scale.x = 0.1  # толщина стрелки
        x_axis.scale.y = 0.2  # ширина наконечника
        x_axis.scale.z = 0.2  # высота наконечника
        x_axis.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)  # красный
        
        self.marker_array.markers.append(x_axis)
        
        # Ось Y (зеленая)
        y_axis = Marker()
        y_axis.header.frame_id = self.map_frame
        y_axis.header.stamp = rospy.Time.now()
        y_axis.ns = "axes"
        y_axis.id = marker_id
        marker_id += 1
        y_axis.type = Marker.ARROW
        y_axis.action = Marker.ADD
        
        # Начальная и конечная точки
        y_axis.points.append(Point(0, 0, 0))
        y_axis.points.append(Point(0, 1.0, 0))
        
        # Цвет и размер
        y_axis.scale.x = 0.1  # толщина стрелки
        y_axis.scale.y = 0.2  # ширина наконечника
        y_axis.scale.z = 0.2  # высота наконечника
        y_axis.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)  # зеленый
        
        self.marker_array.markers.append(y_axis)
        
        # Ось Z (синяя)
        z_axis = Marker()
        z_axis.header.frame_id = self.map_frame
        z_axis.header.stamp = rospy.Time.now()
        z_axis.ns = "axes"
        z_axis.id = marker_id
        marker_id += 1
        z_axis.type = Marker.ARROW
        z_axis.action = Marker.ADD
        
        # Начальная и конечная точки
        z_axis.points.append(Point(0, 0, 0))
        z_axis.points.append(Point(0, 0, 1.0))
        
        # Цвет и размер
        z_axis.scale.x = 0.1  # толщина стрелки
        z_axis.scale.y = 0.2  # ширина наконечника
        z_axis.scale.z = 0.2  # высота наконечника
        z_axis.color = ColorRGBA(0.0, 0.0, 1.0, 1.0)  # синий
        
        self.marker_array.markers.append(z_axis)
        
        # Добавляем метки координат
        for i in range(-self.grid_size//2, self.grid_size//2 + 1, 5):
            if i == 0:
                continue  # пропускаем начало координат
                
            # Метка по оси X
            x_label = Marker()
            x_label.header.frame_id = self.map_frame
            x_label.header.stamp = rospy.Time.now()
            x_label.ns = "labels"
            x_label.id = marker_id
            marker_id += 1
            x_label.type = Marker.TEXT_VIEW_FACING
            x_label.action = Marker.ADD
            x_label.pose.position.x = i * self.cell_size
            x_label.pose.position.y = -0.5
            x_label.pose.position.z = 0.2
            x_label.text = str(i)
            x_label.scale.z = 0.5  # размер текста
            x_label.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)  # белый
            
            self.marker_array.markers.append(x_label)
            
            # Метка по оси Y
            y_label = Marker()
            y_label.header.frame_id = self.map_frame
            y_label.header.stamp = rospy.Time.now()
            y_label.ns = "labels"
            y_label.id = marker_id
            marker_id += 1
            y_label.type = Marker.TEXT_VIEW_FACING
            y_label.action = Marker.ADD
            y_label.pose.position.x = -0.5
            y_label.pose.position.y = i * self.cell_size
            y_label.pose.position.z = 0.2
            y_label.text = str(i)
            y_label.scale.z = 0.5  # размер текста
            y_label.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)  # белый
            
            self.marker_array.markers.append(y_label)
    
    def publish_grid(self, event=None):
        """
        Публикует сетку
        """
        # Обновляем временные метки
        for marker in self.marker_array.markers:
            marker.header.stamp = rospy.Time.now()
        
        # Публикуем маркеры
        self.marker_pub.publish(self.marker_array)

if __name__ == '__main__':
    try:
        node = GridGenerator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 