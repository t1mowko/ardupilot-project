#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import geometry_msgs.msg
from geographic_msgs.msg import GeoPointStamped

class GlobalOriginSetter:
    """
    Скрипт для установки глобального начала координат
    Решает проблему "PositionTargetGlobal failed because no origin"
    """
    
    def __init__(self):
        rospy.init_node('global_origin_setter')
        
        # Параметры
        self.latitude = rospy.get_param('~latitude', 55.751244)   # Москва по умолчанию
        self.longitude = rospy.get_param('~longitude', 37.618423)
        self.altitude = rospy.get_param('~altitude', 0.0)
        
        # Издатель для глобального начала координат
        self.origin_pub = rospy.Publisher('/mavros/global_position/set_gp_origin', GeoPointStamped, queue_size=10)
        
        rospy.loginfo("Инициализация установщика глобального начала координат")
        rospy.loginfo("Будет установлено начало координат: lat=%.6f, lon=%.6f, alt=%.1f", 
                     self.latitude, self.longitude, self.altitude)
        
    def set_global_origin(self):
        """Установка глобального начала координат"""
        origin = GeoPointStamped()
        origin.header.stamp = rospy.Time.now()
        origin.header.frame_id = "map"
        
        origin.position.latitude = self.latitude
        origin.position.longitude = self.longitude
        origin.position.altitude = self.altitude
        
        # Публикуем несколько раз для надежности
        rate = rospy.Rate(1)  # 1 Гц
        for i in range(5):
            self.origin_pub.publish(origin)
            rospy.loginfo("Опубликовано глобальное начало координат (%d/5)", i+1)
            rate.sleep()
            
        rospy.loginfo("Глобальное начало координат установлено")
        
    def run(self):
        """Основная функция работы"""
        # Даем время на инициализацию других узлов
        rospy.sleep(2.0)
        
        # Устанавливаем глобальное начало координат
        self.set_global_origin()

if __name__ == '__main__':
    try:
        setter = GlobalOriginSetter()
        setter.run()
    except rospy.ROSInterruptException:
        pass 