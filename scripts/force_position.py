#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf
import tf2_ros
import time
import sys
import actionlib
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def publish_initial_pose(x, y, theta=0.0):
    """Публикует начальную позу для AMCL"""
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1, latch=True)
    
    # Создаем сообщение
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = rospy.Time.now()
    
    # Устанавливаем позицию
    pose.pose.pose.position.x = x
    pose.pose.pose.position.y = y
    pose.pose.pose.position.z = 0.0
    
    # Устанавливаем ориентацию
    q = quaternion_from_euler(0, 0, theta)
    pose.pose.pose.orientation.x = q[0]
    pose.pose.pose.orientation.y = q[1]
    pose.pose.pose.orientation.z = q[2]
    pose.pose.pose.orientation.w = q[3]
    
    # Устанавливаем ковариацию (большие значения для большей неопределенности)
    pose.pose.covariance = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.5]
    
    # Публикуем несколько раз для надежности
    rospy.loginfo("Публикуем начальную позу: x=%.2f, y=%.2f, theta=%.2f", x, y, theta)
    for i in range(5):
        pub.publish(pose)
        time.sleep(0.5)

def clear_costmaps():
    """Очищает карты стоимости"""
    rospy.loginfo("Очищаем карты стоимости...")
    try:
        rospy.wait_for_service('/move_base/clear_costmaps', timeout=5.0)
        clear_costmaps_srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        clear_costmaps_srv()
        rospy.loginfo("Карты стоимости очищены")
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logerr("Ошибка при очистке карт стоимости: %s", str(e))

def send_goal(x, y, theta=0.0):
    """Отправляет цель для move_base"""
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    
    # Ждем, пока сервер запустится
    rospy.loginfo("Ожидание сервера move_base...")
    client.wait_for_server()
    
    # Создаем цель
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    
    # Устанавливаем позицию и ориентацию
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0.0
    
    q = quaternion_from_euler(0, 0, theta)
    goal.target_pose.pose.orientation.x = q[0]
    goal.target_pose.pose.orientation.y = q[1]
    goal.target_pose.pose.orientation.z = q[2]
    goal.target_pose.pose.orientation.w = q[3]
    
    # Отправляем цель
    rospy.loginfo("Отправляем цель: x=%.2f, y=%.2f, theta=%.2f", x, y, theta)
    client.send_goal(goal)
    
    # Ждем результат с таймаутом
    client.wait_for_result(rospy.Duration(5.0))

def check_tf_tree():
    """Проверяет дерево трансформаций"""
    listener = tf.TransformListener()
    try:
        rospy.loginfo("Проверяем трансформацию map -> base_link...")
        listener.waitForTransform("map", "base_link", rospy.Time(), rospy.Duration(5.0))
        (trans, rot) = listener.lookupTransform("map", "base_link", rospy.Time(0))
        rospy.loginfo("Текущая позиция робота: x=%.2f, y=%.2f", trans[0], trans[1])
        return True
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr("Ошибка при проверке трансформаций: %s", str(e))
        return False

def main():
    """Основная функция"""
    rospy.init_node('force_position', anonymous=True)
    
    # Получаем параметры
    map_center_x = rospy.get_param('~map_center_x', 50.0)
    map_center_y = rospy.get_param('~map_center_y', 50.0)
    
    # Ждем, чтобы система успела запуститься
    rospy.loginfo("Ожидание запуска системы...")
    time.sleep(5.0)
    
    # Публикуем начальную позу несколько раз
    for i in range(3):
        publish_initial_pose(map_center_x, map_center_y)
        time.sleep(1.0)
    
    # Очищаем карты стоимости
    clear_costmaps()
    
    # Проверяем трансформации
    if not check_tf_tree():
        rospy.logerr("Проблема с трансформациями, пытаемся исправить...")
        
        # Пытаемся опубликовать трансформацию напрямую
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_transformStamped = tf2_ros.TransformStamped()
        
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "map"
        static_transformStamped.child_frame_id = "odom"
        
        static_transformStamped.transform.translation.x = 0.0
        static_transformStamped.transform.translation.y = 0.0
        static_transformStamped.transform.translation.z = 0.0
        
        static_transformStamped.transform.rotation.x = 0.0
        static_transformStamped.transform.rotation.y = 0.0
        static_transformStamped.transform.rotation.z = 0.0
        static_transformStamped.transform.rotation.w = 1.0
        
        broadcaster.sendTransform(static_transformStamped)
        
        # Публикуем начальную позу еще раз
        time.sleep(1.0)
        publish_initial_pose(map_center_x, map_center_y)
    
    # Очищаем карты стоимости еще раз
    time.sleep(2.0)
    clear_costmaps()
    
    # Отправляем тестовую цель недалеко от текущей позиции
    time.sleep(2.0)
    send_goal(map_center_x + 1.0, map_center_y + 1.0)
    
    rospy.loginfo("Инициализация завершена!")
    
    # Продолжаем работать и периодически проверять/корректировать позицию
    rate = rospy.Rate(0.2)  # 0.2 Гц = каждые 5 секунд
    while not rospy.is_shutdown():
        if not check_tf_tree():
            rospy.logwarn("Проблема с трансформациями, пытаемся исправить...")
            publish_initial_pose(map_center_x, map_center_y)
            clear_costmaps()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass 