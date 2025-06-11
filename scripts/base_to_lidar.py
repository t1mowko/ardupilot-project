#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf
import tf2_ros
import numpy as np
from geometry_msgs.msg import TransformStamped, Quaternion
from sensor_msgs.msg import LaserScan

class BaseToLidarTransform:
    """
    Скрипт для симуляции лидара и публикации трансформации между base_link и лидаром
    """
    
    def __init__(self):
        rospy.init_node('base_to_lidar_transform')
        
        # Параметры
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.lidar_frame = rospy.get_param('~lidar_frame', 'laser')
        self.publish_rate = rospy.get_param('~publish_rate', 50.0)
        
        # Параметры лидара
        self.scan_range_min = 0.1
        self.scan_range_max = 10.0
        self.scan_angle_min = -np.pi
        self.scan_angle_max = np.pi
        self.scan_angle_increment = np.pi / 180.0  # 1 градус
        self.scan_time = 0.1
        self.scan_time_increment = self.scan_time / 360.0
        
        # Смещение лидара относительно base_link
        self.lidar_x = 0.0
        self.lidar_y = 0.0
        self.lidar_z = 0.2  # 20 см над base_link
        
        # TF бродкастер
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Издатель лазерного скана
        self.scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=1)
        
        # Таймер для публикации трансформации и скана
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.publish_transform_and_scan)
        
        rospy.loginfo("Симуляция лидара запущена")
    
    def publish_transform_and_scan(self, event):
        """
        Публикует трансформацию между base_link и лидаром и симулирует лазерный скан
        """
        # Публикация трансформации
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = self.base_frame
        transform.child_frame_id = self.lidar_frame
        
        # Установка смещения
        transform.transform.translation.x = self.lidar_x
        transform.transform.translation.y = self.lidar_y
        transform.transform.translation.z = self.lidar_z
        
        # Установка ориентации (без поворота)
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        
        # Публикация трансформации
        self.tf_broadcaster.sendTransform(transform)
        
        # Публикация лазерного скана
        self.publish_scan()
    
    def publish_scan(self):
        """
        Публикует симулированный лазерный скан
        """
        current_time = rospy.Time.now()
        
        # Создание сообщения LaserScan
        scan = LaserScan()
        scan.header.stamp = current_time
        scan.header.frame_id = self.lidar_frame
        scan.angle_min = self.scan_angle_min
        scan.angle_max = self.scan_angle_max
        scan.angle_increment = self.scan_angle_increment
        scan.time_increment = self.scan_time_increment
        scan.range_min = self.scan_range_min
        scan.range_max = self.scan_range_max
        
        # Количество лучей
        num_readings = int((self.scan_angle_max - self.scan_angle_min) / self.scan_angle_increment)
        
        # Симуляция данных скана (пустое пространство вокруг)
        ranges = [self.scan_range_max for _ in range(num_readings)]
        
        # Добавление случайных препятствий для тестирования
        for i in range(5):  # 5 случайных препятствий
            obstacle_idx = np.random.randint(0, num_readings)
            obstacle_range = np.random.uniform(0.5, 5.0)
            
            # Установка препятствия и его окрестности
            width = np.random.randint(5, 20)  # ширина препятствия в лучах
            for j in range(-width//2, width//2):
                idx = (obstacle_idx + j) % num_readings
                if 0 <= idx < num_readings:
                    # Добавляем небольшую вариацию для реалистичности
                    variation = np.random.uniform(-0.1, 0.1)
                    ranges[idx] = obstacle_range + variation
        
        scan.ranges = ranges
        
        # Публикация скана
        self.scan_pub.publish(scan)

if __name__ == '__main__':
    try:
        node = BaseToLidarTransform()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 