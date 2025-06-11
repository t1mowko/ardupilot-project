#!/usr/bin/env python3
import rospy
import sys
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler

def set_initial_pose():
    rospy.init_node('set_initial_pose', anonymous=True)
    
    # Получаем параметры
    map_center_x = rospy.get_param('~map_center_x', 50.0)
    map_center_y = rospy.get_param('~map_center_y', 50.0)
    initial_yaw = rospy.get_param('~initial_yaw', 0.0)
    
    # Создаем издателя для темы initialpose
    initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1, latch=True)
    
    # Даем время на инициализацию узлов
    rospy.sleep(2.0)
    
    # Создаем сообщение с начальной позой
    initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = rospy.Time.now()
    
    # Устанавливаем позицию в центр карты
    initial_pose.pose.pose.position.x = map_center_x
    initial_pose.pose.pose.position.y = map_center_y
    initial_pose.pose.pose.position.z = 0.0
    
    # Устанавливаем ориентацию
    q = quaternion_from_euler(0, 0, initial_yaw)
    initial_pose.pose.pose.orientation.x = q[0]
    initial_pose.pose.pose.orientation.y = q[1]
    initial_pose.pose.pose.orientation.z = q[2]
    initial_pose.pose.pose.orientation.w = q[3]
    
    # Устанавливаем ковариацию (диагональ единицы для упрощения)
    initial_pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
    
    # Публикуем начальную позу несколько раз для надежности
    rospy.loginfo("Устанавливаем начальную позу: x=%.2f, y=%.2f, yaw=%.2f", 
                 map_center_x, map_center_y, initial_yaw)
    
    for i in range(5):
        initial_pose_pub.publish(initial_pose)
        rospy.sleep(0.5)
    
    rospy.loginfo("Начальная поза установлена!")

if __name__ == '__main__':
    try:
        set_initial_pose()
    except rospy.ROSInterruptException:
        pass 