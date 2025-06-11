#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler
import sys
import time

def main():
    """Публикует начальную позу для AMCL"""
    rospy.init_node('fix_position', anonymous=True)
    
    # Получаем параметры
    map_center_x = rospy.get_param('~map_center_x', 50.0)
    map_center_y = rospy.get_param('~map_center_y', 50.0)
    
    # Создаем издателя для темы initialpose
    initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1, latch=True)
    
    # Ждем подключения подписчиков
    rospy.loginfo("Ожидание подключения подписчиков к /initialpose...")
    time.sleep(3.0)
    
    # Создаем сообщение с начальной позой
    initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = rospy.Time.now()
    
    # Устанавливаем позицию в центр карты
    initial_pose.pose.pose.position.x = map_center_x
    initial_pose.pose.pose.position.y = map_center_y
    initial_pose.pose.pose.position.z = 0.0
    
    # Устанавливаем ориентацию - смотрим вперед (по оси X)
    q = quaternion_from_euler(0, 0, 0)
    initial_pose.pose.pose.orientation.x = q[0]
    initial_pose.pose.pose.orientation.y = q[1]
    initial_pose.pose.pose.orientation.z = q[2]
    initial_pose.pose.pose.orientation.w = q[3]
    
    # Устанавливаем ковариацию
    initial_pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
    
    # Публикуем начальную позу несколько раз для надежности
    rospy.loginfo("Публикация начальной позы: x=%.2f, y=%.2f", map_center_x, map_center_y)
    
    for i in range(10):
        initial_pose_pub.publish(initial_pose)
        time.sleep(0.5)
    
    rospy.loginfo("Начальная поза установлена!")
    
    # Ждем некоторое время, чтобы позиция применилась
    time.sleep(2.0)
    
    return 0

if __name__ == '__main__':
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        sys.exit(1) 