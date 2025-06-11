#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf
import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped

class CostmapPositionFixer:
    """
    Скрипт для исправления позиции робота на карте затрат
    """
    
    def __init__(self):
        rospy.init_node('costmap_position_fixer')
        
        # Параметры
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.use_map_center = rospy.get_param('~use_map_center', True)
        self.map_center_x = rospy.get_param('~map_center_x', 0.0)
        self.map_center_y = rospy.get_param('~map_center_y', 0.0)
        
        # TF слушатель и бродкастер
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener2 = tf2_ros.TransformListener(self.tf_buffer)
        
        # Подписчики
        self.global_costmap_sub = rospy.Subscriber('/move_base/global_costmap/costmap', 
                                                OccupancyGrid, self.global_costmap_callback)
        
        # Издатели
        self.initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
        
        # Состояние
        self.global_costmap = None
        self.initial_pose_set = False
        
        rospy.loginfo("Исправление позиции на карте затрат запущено")
        
    def global_costmap_callback(self, msg):
        """Обработчик сообщений с глобальной картой затрат"""
        self.global_costmap = msg
        rospy.loginfo("Получена глобальная карта затрат: размер %dx%d, разрешение %.3f", 
                     msg.info.width, msg.info.height, msg.info.resolution)
        rospy.loginfo("Начало глобальной карты затрат: x=%.2f, y=%.2f", 
                     msg.info.origin.position.x, msg.info.origin.position.y)
        
        # Вычисляем центр карты
        center_x = msg.info.origin.position.x + (msg.info.width * msg.info.resolution) / 2
        center_y = msg.info.origin.position.y + (msg.info.height * msg.info.resolution) / 2
        rospy.loginfo("Центр глобальной карты затрат: x=%.2f, y=%.2f", center_x, center_y)
        
        # Устанавливаем начальную позу, если еще не установлена
        if not self.initial_pose_set:
            self.set_initial_pose()
            
    def set_initial_pose(self):
        """Установка начальной позы робота"""
        if self.global_costmap is None:
            rospy.logwarn("Нет данных о глобальной карте затрат, не могу установить начальную позу")
            return
            
        # Создаем сообщение с начальной позой
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = rospy.Time.now()
        
        if self.use_map_center:
            # Используем центр карты или заданные координаты
            pose.pose.pose.position.x = self.map_center_x
            pose.pose.pose.position.y = self.map_center_y
        else:
            # Используем центр глобальной карты затрат
            pose.pose.pose.position.x = self.global_costmap.info.origin.position.x + (self.global_costmap.info.width * self.global_costmap.info.resolution) / 2
            pose.pose.pose.position.y = self.global_costmap.info.origin.position.y + (self.global_costmap.info.height * self.global_costmap.info.resolution) / 2
            
        pose.pose.pose.position.z = 0.0
        pose.pose.pose.orientation.w = 1.0  # Без вращения
        
        # Устанавливаем ковариацию (неопределенность позиции)
        pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
        
        # Публикуем начальную позу
        rospy.loginfo("Устанавливаем начальную позу: x=%.2f, y=%.2f", 
                     pose.pose.pose.position.x, pose.pose.pose.position.y)
        self.initial_pose_pub.publish(pose)
        
        # Также обновляем трансформацию map -> base_link напрямую
        self.update_transform(pose.pose.pose.position.x, pose.pose.pose.position.y)
        
        self.initial_pose_set = True
        
    def update_transform(self, x, y):
        """Обновление трансформации map -> base_link"""
        try:
            # Получаем текущую трансформацию odom -> base_link
            (trans, rot) = self.tf_listener.lookupTransform('odom', self.base_frame, rospy.Time(0))
            
            # Создаем новую трансформацию map -> odom
            self.tf_broadcaster.sendTransform(
                (x, y, 0),
                (0, 0, 0, 1),
                rospy.Time.now(),
                'odom',
                self.map_frame
            )
            
            rospy.loginfo("Трансформация map -> odom обновлена")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("Ошибка при обновлении трансформации: %s", str(e))
        
    def run(self):
        """Основная функция работы"""
        rate = rospy.Rate(1)  # 1 Гц
        
        # Ждем некоторое время для инициализации системы
        rospy.sleep(5)
        
        attempts = 0
        max_attempts = 10
        
        while not rospy.is_shutdown() and not self.initial_pose_set and attempts < max_attempts:
            if self.global_costmap:
                self.set_initial_pose()
            else:
                rospy.loginfo("Ожидаем данные о глобальной карте затрат...")
                
            attempts += 1
            rate.sleep()
            
        if not self.initial_pose_set:
            rospy.logwarn("Не удалось установить начальную позу после %d попыток", max_attempts)
        else:
            rospy.loginfo("Начальная поза успешно установлена")
            
        # Продолжаем работу, периодически обновляя позицию
        update_interval = 10  # Обновляем каждые 10 секунд
        last_update = rospy.Time.now()
        
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            if (now - last_update).to_sec() > update_interval and self.initial_pose_set:
                self.set_initial_pose()
                last_update = now
                
            rate.sleep()

if __name__ == '__main__':
    try:
        fixer = CostmapPositionFixer()
        fixer.run()
    except rospy.ROSInterruptException:
        pass 