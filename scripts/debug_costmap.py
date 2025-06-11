#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf
import time
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from std_srvs.srv import Empty
from tf.transformations import quaternion_from_euler
from rosgraph_msgs.msg import Log
from visualization_msgs.msg import Marker
import re

class CostmapDebugger:
    def __init__(self):
        rospy.init_node('debug_costmap', anonymous=True)
        
        # Подписываемся на глобальную карту стоимости
        self.global_costmap_sub = rospy.Subscriber('/move_base/global_costmap/costmap', 
                                                 OccupancyGrid, 
                                                 self.global_costmap_callback)
        
        # Подписываемся на логи для отслеживания ошибок
        self.rosout_sub = rospy.Subscriber('/rosout', 
                                         Log, 
                                         self.rosout_callback)
        
        # Создаем издателя для начальной позы
        self.init_pose_pub = rospy.Publisher('/initialpose', 
                                           PoseWithCovarianceStamped, 
                                           queue_size=1, 
                                           latch=True)
        
        # Создаем издателя для позиции в MAVROS
        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_position/local',
                                          PoseStamped,
                                          queue_size=10)
        
        # Создаем издателя для маркера центра карты
        self.marker_pub = rospy.Publisher('/center_marker',
                                        Marker,
                                        queue_size=1,
                                        latch=True)
        
        # Параметры
        self.map_center_x = rospy.get_param('~map_center_x', 0.0)
        self.map_center_y = rospy.get_param('~map_center_y', 0.0)
        
        # Флаги
        self.received_costmap = False
        self.costmap_info = None
        self.error_detected = False
        self.last_error_time = 0
        self.error_cooldown = 5.0  # Секунды между исправлениями ошибок
        
        # TF слушатель
        self.tf_listener = tf.TransformListener()
        
        # Таймер для периодических действий
        rospy.Timer(rospy.Duration(1.0), self.periodic_check)  # Увеличена частота проверок
        
        # Таймер для публикации setpoint
        rospy.Timer(rospy.Duration(0.1), self.publish_setpoint)
        
        # Таймер для публикации маркера центра
        rospy.Timer(rospy.Duration(1.0), self.publish_center_marker)
        
        rospy.loginfo("Инициализация отладчика карты стоимости завершена")
    
    def global_costmap_callback(self, msg):
        """Обработчик сообщения глобальной карты стоимости"""
        self.costmap_info = msg.info
        
        if not self.received_costmap:
            self.received_costmap = True
            
            rospy.loginfo("Получена глобальная карта стоимости:")
            rospy.loginfo("  Разрешение: %.3f м/пиксель", msg.info.resolution)
            rospy.loginfo("  Ширина: %d пикселей (%.2f м)", msg.info.width, msg.info.width * msg.info.resolution)
            rospy.loginfo("  Высота: %d пикселей (%.2f м)", msg.info.height, msg.info.height * msg.info.resolution)
            rospy.loginfo("  Начало координат: (%.2f, %.2f)", msg.info.origin.position.x, msg.info.origin.position.y)
            
            # Вычисляем координаты центра карты
            center_x = msg.info.origin.position.x + (msg.info.width * msg.info.resolution) / 2.0
            center_y = msg.info.origin.position.y + (msg.info.height * msg.info.resolution) / 2.0
            rospy.loginfo("  Центр карты: (%.2f, %.2f)", center_x, center_y)
            
            # Устанавливаем позицию в центр карты
            self.map_center_x = 0.0  # Принудительно устанавливаем в (0,0)
            self.map_center_y = 0.0
            
            # Публикуем начальную позу
            self.publish_initial_pose(self.map_center_x, self.map_center_y)
            
            # Очищаем карты стоимости
            self.clear_costmaps()
        
        # Проверяем, находится ли начальная поза внутри карты
        pose_x = self.map_center_x
        pose_y = self.map_center_y
        
        min_x = msg.info.origin.position.x
        min_y = msg.info.origin.position.y
        max_x = min_x + (msg.info.width * msg.info.resolution)
        max_y = min_y + (msg.info.height * msg.info.resolution)
        
        # Добавляем отступ от края карты
        margin = 1.0  # метры
        safe_min_x = min_x + margin
        safe_min_y = min_y + margin
        safe_max_x = max_x - margin
        safe_max_y = max_y - margin
        
        if safe_min_x <= pose_x <= safe_max_x and safe_min_y <= pose_y <= safe_max_y:
            if self.error_detected:
                rospy.loginfo("  Начальная поза (%.2f, %.2f) НАХОДИТСЯ внутри карты - проблема решена", pose_x, pose_y)
                self.error_detected = False
        else:
            rospy.logerr("  Начальная поза (%.2f, %.2f) НАХОДИТСЯ ВНЕ карты или слишком близко к краю!", pose_x, pose_y)
            rospy.logerr("  Границы карты: (%.2f, %.2f) - (%.2f, %.2f)", min_x, min_y, max_x, max_y)
            
            # Корректируем позицию, чтобы она была внутри карты с безопасным отступом
            corrected_x = max(safe_min_x, min(safe_max_x, pose_x))
            corrected_y = max(safe_min_y, min(safe_max_y, pose_y))
            
            # Если карта очень маленькая или начало координат смещено, используем центр карты
            if safe_max_x - safe_min_x < 2*margin or safe_max_y - safe_min_y < 2*margin:
                corrected_x = center_x
                corrected_y = center_y
                rospy.logwarn("  Карта слишком маленькая, используем её центр: (%.2f, %.2f)", center_x, center_y)
            
            rospy.loginfo("  Корректируем позицию на (%.2f, %.2f)", corrected_x, corrected_y)
            self.map_center_x = corrected_x
            self.map_center_y = corrected_y
            
            # Публикуем скорректированную позицию
            self.publish_initial_pose(corrected_x, corrected_y)
            
            # Очищаем карты стоимости
            self.clear_costmaps()
            self.error_detected = True
    
    def rosout_callback(self, msg):
        """Обработчик сообщений rosout для отслеживания ошибок"""
        # Проверяем наличие сообщений об ошибках в логах
        message = msg.msg
        if "robot's start position is off the global costmap" in message:
            current_time = time.time()
            if current_time - self.last_error_time > self.error_cooldown:
                rospy.logerr("Обнаружена ошибка: робот вне карты стоимости. Исправляем...")
                self.error_detected = True
                self.last_error_time = current_time
                
                # Очищаем карты стоимости
                self.clear_costmaps()
                
                # Если у нас есть информация о карте стоимости
                if self.costmap_info:
                    # Вычисляем центр карты
                    center_x = self.costmap_info.origin.position.x + (self.costmap_info.width * self.costmap_info.resolution) / 2.0
                    center_y = self.costmap_info.origin.position.y + (self.costmap_info.height * self.costmap_info.resolution) / 2.0
                    
                    # Используем центр карты как безопасную позицию
                    self.map_center_x = center_x
                    self.map_center_y = center_y
                    
                    rospy.loginfo("Переустанавливаем позицию в центр карты: (%.2f, %.2f)", center_x, center_y)
                
                # Публикуем начальную позу
                self.publish_initial_pose(self.map_center_x, self.map_center_y)
    
    def publish_initial_pose(self, x, y, theta=0.0):
        """Публикует начальную позу для AMCL"""
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
        
        # Устанавливаем очень маленькую ковариацию для большей уверенности
        pose.pose.covariance = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.01]
        
        # Публикуем несколько раз для надежности
        rospy.loginfo("Публикуем начальную позу: x=%.2f, y=%.2f, theta=%.2f", x, y, theta)
        for i in range(10):  # Увеличено количество публикаций
            self.init_pose_pub.publish(pose)
            time.sleep(0.1)
    
    def publish_setpoint(self, event):
        """Публикует setpoint для MAVROS"""
        if not self.received_costmap:
            return
            
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = rospy.Time.now()
        
        # Устанавливаем позицию
        pose.pose.position.x = self.map_center_x
        pose.pose.position.y = self.map_center_y
        pose.pose.position.z = 2.0  # Высота 2 метра
        
        # Устанавливаем ориентацию
        q = quaternion_from_euler(0, 0, 0)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        
        # Публикуем setpoint
        self.setpoint_pub.publish(pose)
    
    def publish_center_marker(self, event):
        """Публикует маркер в центре карты для визуализации"""
        if not self.received_costmap:
            return
        
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "center_marker"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Позиция маркера - центр карты
        marker.pose.position.x = 0.0  # Центр карты в (0,0)
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.5  # Немного над землей
        
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # Размер маркера
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        
        # Цвет маркера (зеленый)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.5  # Полупрозрачный
        
        # Время жизни маркера
        marker.lifetime = rospy.Duration(2.0)  # 2 секунды
        
        # Публикуем маркер
        self.marker_pub.publish(marker)
    
    def check_tf_tree(self):
        """Проверяет дерево трансформаций"""
        try:
            self.tf_listener.waitForTransform("map", "base_link", rospy.Time(), rospy.Duration(1.0))
            (trans, rot) = self.tf_listener.lookupTransform("map", "base_link", rospy.Time(0))
            rospy.loginfo("Текущая позиция робота в TF: x=%.2f, y=%.2f", trans[0], trans[1])
            
            # Проверяем, совпадает ли позиция с ожидаемой
            if abs(trans[0] - self.map_center_x) > 0.1 or abs(trans[1] - self.map_center_y) > 0.1:
                rospy.logwarn("Позиция в TF отличается от ожидаемой!")
                
                # Публикуем правильную позицию
                self.publish_initial_pose(self.map_center_x, self.map_center_y)
                
                # Очищаем карты стоимости
                self.clear_costmaps()
                
                return False
            
            return True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Ошибка при проверке трансформаций: %s", str(e))
            return False
    
    def clear_costmaps(self):
        """Очищает карты стоимости"""
        try:
            rospy.wait_for_service('/move_base/clear_costmaps', timeout=1.0)
            clear_costmaps_srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            clear_costmaps_srv()
            rospy.loginfo("Карты стоимости очищены")
            
            # Дополнительно ждем, чтобы карты успели обновиться
            time.sleep(0.5)
            
            return True
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr("Ошибка при очистке карт стоимости: %s", str(e))
            return False
    
    def periodic_check(self, event):
        """Периодическая проверка и корректировка"""
        if not self.received_costmap:
            rospy.logwarn_throttle(5.0, "Еще не получена карта стоимости")
            return
        
        # Проверяем трансформации
        tf_ok = self.check_tf_tree()
        
        # Если трансформации в порядке, проверяем наличие ошибок
        if tf_ok and not self.error_detected:
            return
        
        # Публикуем начальную позу для надежности
        self.publish_initial_pose(self.map_center_x, self.map_center_y)
        
        # Очищаем карты стоимости
        self.clear_costmaps()

if __name__ == '__main__':
    try:
        debugger = CostmapDebugger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 