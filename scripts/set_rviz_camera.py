#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import ColorRGBA
import tf
import math

class RVizCameraHelper:
    """
    Скрипт для помощи в установке камеры RViz в правильное положение
    """
    
    def __init__(self):
        rospy.init_node('set_rviz_camera')
        
        # Параметры
        self.rate = rospy.get_param('~rate', 1.0)  # Гц
        
        # Издатели
        self.marker_pub = rospy.Publisher('/camera_target', Marker, queue_size=1)
        
        # Переменные
        self.tf_listener = tf.TransformListener()
        
        rospy.loginfo("Скрипт установки камеры RViz запущен")
        
        # Публикуем маркер для центрирования камеры
        self.publish_camera_target_marker()
        
        # Вывести инструкции
        self.print_instructions()
        
    def publish_camera_target_marker(self):
        """Публикация маркера для центрирования камеры"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "camera_target"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Устанавливаем маркер в центре карты
        marker.pose.position = Point(0, 0, 0)
        marker.pose.orientation = Quaternion(0, 0, 0, 1)
        
        # Размер маркера
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        
        # Цвет маркера (синий полупрозрачный)
        marker.color = ColorRGBA(0.0, 0.0, 1.0, 0.5)
        
        # Время жизни маркера (постоянный)
        marker.lifetime = rospy.Duration(0)
        
        # Публикуем маркер
        self.marker_pub.publish(marker)
        rospy.loginfo("Маркер для центрирования камеры опубликован")
    
    def print_instructions(self):
        """Вывод инструкций для пользователя"""
        rospy.loginfo("="*50)
        rospy.loginfo("ИНСТРУКЦИИ ПО НАСТРОЙКЕ КАМЕРЫ RVIZ:")
        rospy.loginfo("1. В RViz нажмите кнопку 'Focus Camera' (или клавишу 'f')")
        rospy.loginfo("2. Затем щелкните на синем маркере в центре карты")
        rospy.loginfo("3. Используйте колесо мыши для настройки масштаба")
        rospy.loginfo("4. Удерживайте Shift + левую кнопку мыши для вращения камеры")
        rospy.loginfo("5. Удерживайте среднюю кнопку мыши для перемещения камеры")
        rospy.loginfo("="*50)
        rospy.loginfo("Рекомендуемые настройки:")
        rospy.loginfo("- Расстояние: 20-30 метров")
        rospy.loginfo("- Угол обзора: сверху под углом 45 градусов")
        rospy.loginfo("="*50)
    
    def run(self):
        """Запуск узла"""
        rate = rospy.Rate(self.rate)
        
        while not rospy.is_shutdown():
            # Обновляем маркер
            self.publish_camera_target_marker()
            rate.sleep()

if __name__ == "__main__":
    try:
        helper = RVizCameraHelper()
        helper.run()
    except rospy.ROSInterruptException:
        pass 