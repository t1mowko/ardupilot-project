#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid

class GoalFixer:
    """
    Скрипт для исправления целей, которые находятся вне карты
    Решает проблему "The goal sent to the navfn planner is off the global costmap"
    """
    
    def __init__(self):
        rospy.init_node('goal_fixer')
        
        # Параметры
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.safety_margin = rospy.get_param('~safety_margin', 1.0)  # Отступ от края карты в метрах
        
        # Состояние
        self.map_data = None
        self.map_info = None
        self.map_origin = None
        self.map_resolution = None
        self.map_width = None
        self.map_height = None
        
        # Подписчики и издатели
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        self.fixed_goal_pub = rospy.Publisher('/move_base_simple/fixed_goal', PoseStamped, queue_size=10)
        
        # Перенаправление
        self.relay_sub = rospy.Subscriber('/move_base_simple/fixed_goal', PoseStamped, self.relay_callback)
        self.relay_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        
        rospy.loginfo("Инициализация исправления целей")
        
    def map_callback(self, msg):
        """Обработчик сообщений с картой"""
        self.map_data = msg.data
        self.map_info = msg.info
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.map_resolution = msg.info.resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        
        rospy.loginfo("Получена карта: размер %dx%d, разрешение %.3f м/пиксель", 
                     self.map_width, self.map_height, self.map_resolution)
        rospy.loginfo("Начало карты: x=%.2f, y=%.2f", self.map_origin[0], self.map_origin[1])
        
    def goal_callback(self, msg):
        """Обработчик сообщений с целью"""
        if self.map_info is None:
            rospy.logwarn("Карта еще не получена, не могу проверить цель")
            return
            
        # Получаем координаты цели
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        goal_z = msg.pose.position.z
        
        # Проверяем, находится ли цель в пределах карты
        map_max_x = self.map_origin[0] + self.map_width * self.map_resolution
        map_max_y = self.map_origin[1] + self.map_height * self.map_resolution
        
        # Добавляем отступ от края карты
        min_x = self.map_origin[0] + self.safety_margin
        min_y = self.map_origin[1] + self.safety_margin
        max_x = map_max_x - self.safety_margin
        max_y = map_max_y - self.safety_margin
        
        # Проверяем и корректируем координаты
        fixed_x = np.clip(goal_x, min_x, max_x)
        fixed_y = np.clip(goal_y, min_y, max_y)
        
        if fixed_x != goal_x or fixed_y != goal_y:
            rospy.logwarn("Цель (%.2f, %.2f) находится вне карты, исправлено на (%.2f, %.2f)", 
                         goal_x, goal_y, fixed_x, fixed_y)
            
            # Создаем исправленную цель
            fixed_goal = PoseStamped()
            fixed_goal.header = msg.header
            fixed_goal.pose.position.x = fixed_x
            fixed_goal.pose.position.y = fixed_y
            fixed_goal.pose.position.z = goal_z
            fixed_goal.pose.orientation = msg.pose.orientation
            
            # Публикуем исправленную цель
            self.fixed_goal_pub.publish(fixed_goal)
        else:
            # Цель в пределах карты, просто перенаправляем
            self.fixed_goal_pub.publish(msg)
            
    def relay_callback(self, msg):
        """Перенаправление исправленной цели обратно в move_base"""
        # Добавляем небольшую задержку, чтобы избежать циклов
        rospy.sleep(0.1)
        self.relay_pub.publish(msg)
        
    def run(self):
        """Основная функция работы"""
        rospy.spin()

if __name__ == '__main__':
    try:
        fixer = GoalFixer()
        fixer.run()
    except rospy.ROSInterruptException:
        pass 