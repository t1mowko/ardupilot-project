#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import tf
import geometry_msgs.msg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped

class TransformFixer:
    """
    Скрипт для исправления проблем с трансформациями в навигационной системе
    """
    
    def __init__(self):
        rospy.init_node('transform_fixer')
        
        # Параметры
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.publish_rate = rospy.get_param('~publish_rate', 50.0)  # Гц
        
        # Создаем broadcaster для публикации трансформаций
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        
        # Подписчики
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', geometry_msgs.msg.PoseStamped, 
                                         self.pose_callback)
        self.initial_pose_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, 
                                                self.initial_pose_callback)
        
        # Состояние
        self.latest_pose = None
        self.initial_pose_received = False
        self.map_to_odom_transform = None
        
        rospy.loginfo("Исправление трансформаций запущено")
        
        # Публикуем статические трансформации
        self.publish_static_transforms()
        
    def initial_pose_callback(self, msg):
        """Обработчик сообщений с начальной позицией"""
        rospy.loginfo("Получена начальная поза, обновляем трансформацию map->odom")
        
        # Создаем трансформацию map -> odom на основе начальной позы
        self.map_to_odom_transform = TransformStamped()
        self.map_to_odom_transform.header.stamp = rospy.Time.now()
        self.map_to_odom_transform.header.frame_id = self.map_frame
        self.map_to_odom_transform.child_frame_id = self.odom_frame
        
        # Копируем позицию и ориентацию из сообщения
        self.map_to_odom_transform.transform.translation.x = msg.pose.pose.position.x
        self.map_to_odom_transform.transform.translation.y = msg.pose.pose.position.y
        self.map_to_odom_transform.transform.translation.z = msg.pose.pose.position.z
        
        self.map_to_odom_transform.transform.rotation.x = msg.pose.pose.orientation.x
        self.map_to_odom_transform.transform.rotation.y = msg.pose.pose.orientation.y
        self.map_to_odom_transform.transform.rotation.z = msg.pose.pose.orientation.z
        self.map_to_odom_transform.transform.rotation.w = msg.pose.pose.orientation.w
        
        self.initial_pose_received = True
        
    def pose_callback(self, msg):
        """Обработчик сообщений с текущей позицией"""
        self.latest_pose = msg
        
    def publish_static_transforms(self):
        """Публикация статических трансформаций"""
        # Создаем статическую трансформацию base_link -> base_stabilized
        base_to_stabilized = TransformStamped()
        base_to_stabilized.header.stamp = rospy.Time.now()
        base_to_stabilized.header.frame_id = self.base_frame
        base_to_stabilized.child_frame_id = "base_stabilized"
        base_to_stabilized.transform.rotation.w = 1.0
        
        # Создаем статическую трансформацию base_stabilized -> base_footprint
        stabilized_to_footprint = TransformStamped()
        stabilized_to_footprint.header.stamp = rospy.Time.now()
        stabilized_to_footprint.header.frame_id = "base_stabilized"
        stabilized_to_footprint.child_frame_id = "base_footprint"
        stabilized_to_footprint.transform.rotation.w = 1.0
        
        # Публикуем статические трансформации
        self.static_broadcaster.sendTransform([base_to_stabilized, stabilized_to_footprint])
        rospy.loginfo("Статические трансформации опубликованы")
        
    def publish_transforms(self):
        """Публикация динамических трансформаций"""
        if self.map_to_odom_transform:
            # Обновляем временную метку
            self.map_to_odom_transform.header.stamp = rospy.Time.now()
            
            # Публикуем трансформацию map -> odom
            self.tf_broadcaster.sendTransform(self.map_to_odom_transform)
        
        if self.latest_pose:
            # Создаем трансформацию odom -> base_link на основе текущей позы
            odom_to_base = TransformStamped()
            odom_to_base.header.stamp = rospy.Time.now()
            odom_to_base.header.frame_id = self.odom_frame
            odom_to_base.child_frame_id = self.base_frame
            
            # Копируем позицию и ориентацию из сообщения с позой
            odom_to_base.transform.translation.x = self.latest_pose.pose.position.x
            odom_to_base.transform.translation.y = self.latest_pose.pose.position.y
            odom_to_base.transform.translation.z = self.latest_pose.pose.position.z
            
            odom_to_base.transform.rotation.x = self.latest_pose.pose.orientation.x
            odom_to_base.transform.rotation.y = self.latest_pose.pose.orientation.y
            odom_to_base.transform.rotation.z = self.latest_pose.pose.orientation.z
            odom_to_base.transform.rotation.w = self.latest_pose.pose.orientation.w
            
            # Публикуем трансформацию odom -> base_link
            self.tf_broadcaster.sendTransform(odom_to_base)
            
    def run(self):
        """Основной цикл работы"""
        rate = rospy.Rate(self.publish_rate)
        
        while not rospy.is_shutdown():
            self.publish_transforms()
            rate.sleep()

if __name__ == '__main__':
    try:
        fixer = TransformFixer()
        fixer.run()
    except rospy.ROSInterruptException:
        pass 