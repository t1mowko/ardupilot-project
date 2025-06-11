#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped

class DroneArmTester:
    """
    Скрипт для тестирования армирования и взлета дрона
    """
    
    def __init__(self):
        rospy.init_node('drone_arm_tester')
        
        # Параметры
        self.takeoff_altitude = rospy.get_param('~takeoff_altitude', 2.0)
        
        # Подписчики
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_callback)
        
        # Издатели
        self.local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        
        # Сервисы
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        
        # Состояние
        self.current_state = State()
        
        rospy.loginfo("Тестирование армирования дрона инициализировано")
        
    def state_callback(self, msg):
        """Обработчик состояния MAVROS"""
        self.current_state = msg
        
    def wait_for_connection(self):
        """Ожидание подключения к FCU"""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not self.current_state.connected:
            rospy.loginfo("Ожидание подключения к FCU...")
            rate.sleep()
            
        if self.current_state.connected:
            rospy.loginfo("Подключение к FCU установлено")
            return True
        else:
            rospy.logerr("Не удалось подключиться к FCU")
            return False
            
    def arm_drone(self):
        """Армирование дрона"""
        rospy.loginfo("Попытка переключения в режим GUIDED...")
        
        # Отправляем несколько setpoint сообщений перед переключением в GUIDED
        pose = PoseStamped()
        pose.pose.position.z = self.takeoff_altitude
        pose.pose.orientation.w = 1.0
        
        rate = rospy.Rate(20)
        for i in range(100):
            if rospy.is_shutdown():
                return False
            self.local_pos_pub.publish(pose)
            rate.sleep()
            
        # Переключаемся в режим GUIDED
        set_mode_response = self.set_mode_client(custom_mode="GUIDED")
        if not set_mode_response.mode_sent:
            rospy.logwarn("Не удалось переключиться в режим GUIDED")
            return False
            
        rospy.loginfo("Режим GUIDED установлен, попытка армирования...")
        
        # Армируем дрон
        arm_response = self.arming_client(True)
        if not arm_response.success:
            rospy.logwarn("Армирование не удалось")
            return False
            
        rospy.loginfo("Дрон успешно армирован")
        return True
        
    def takeoff(self):
        """Взлет дрона"""
        rospy.loginfo("Начинаем взлет на высоту %.1f м", self.takeoff_altitude)
        
        # Создаем сообщение с целевой позицией для взлета
        pose = PoseStamped()
        pose.pose.position.z = self.takeoff_altitude
        pose.pose.orientation.w = 1.0
        
        # Отправляем команду взлета
        rate = rospy.Rate(10)
        start_time = time.time()
        duration = 10.0  # Время взлета в секундах
        
        while not rospy.is_shutdown() and time.time() - start_time < duration:
            self.local_pos_pub.publish(pose)
            rate.sleep()
            
        rospy.loginfo("Взлет завершен")
        
    def run(self):
        """Основная функция работы"""
        if not self.wait_for_connection():
            return
            
        rospy.loginfo("Начинаем тестирование армирования...")
        
        if self.arm_drone():
            rospy.loginfo("Армирование успешно, начинаем взлет...")
            self.takeoff()
            rospy.loginfo("Тестирование завершено")
        else:
            rospy.logerr("Не удалось армировать дрон")
            
        rospy.spin()

if __name__ == '__main__':
    try:
        tester = DroneArmTester()
        tester.run()
    except rospy.ROSInterruptException:
        pass 