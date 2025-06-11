#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import Header

class DroneController:
    """
    Скрипт для ручного армирования и взлета дрона
    """
    
    def __init__(self):
        rospy.init_node('drone_controller')
        
        # Параметры
        self.takeoff_altitude = rospy.get_param('~takeoff_altitude', 2.0)  # Высота взлета в метрах
        
        # Подписчики
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_callback)
        
        # Издатели
        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        
        # Сервисы
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        
        # Состояние
        self.current_state = State()
        
        rospy.loginfo("Контроллер дрона инициализирован")
        
    def state_callback(self, msg):
        """Обработчик состояния MAVROS"""
        self.current_state = msg
        
    def wait_for_connection(self):
        """Ожидание подключения к FCU"""
        rospy.loginfo("Ожидание подключения к FCU...")
        
        rate = rospy.Rate(10)  # 10 Гц
        while not rospy.is_shutdown() and not self.current_state.connected:
            rate.sleep()
            
        if self.current_state.connected:
            rospy.loginfo("Подключение к FCU установлено")
            return True
        else:
            rospy.logerr("Не удалось подключиться к FCU")
            return False
            
    def arm(self):
        """Армирование дрона"""
        rospy.loginfo("Попытка армирования дрона...")
        
        # Отправляем несколько setpoint сообщений перед переключением в OFFBOARD
        setpoint = PoseStamped()
        setpoint.header.frame_id = "map"
        setpoint.pose.position.x = 0.0  # Центр карты
        setpoint.pose.position.y = 0.0
        setpoint.pose.position.z = self.takeoff_altitude
        setpoint.pose.orientation.w = 1.0
        
        # Отправляем несколько сообщений, чтобы FCU начал их принимать
        rate = rospy.Rate(20)  # 20 Гц
        for i in range(100):
            if rospy.is_shutdown():
                return False
            setpoint.header.stamp = rospy.Time.now()
            self.setpoint_pub.publish(setpoint)
            rate.sleep()
            
        # Переключаемся в режим GUIDED (для ArduPilot)
        rospy.loginfo("Переключение в режим GUIDED...")
        response = self.set_mode_client(custom_mode="GUIDED")
        if not response.mode_sent:
            rospy.logwarn("Не удалось переключиться в режим GUIDED!")
            return False
            
        # Армируем дрон
        rospy.loginfo("Армирование дрона...")
        if not self.arming_client(True).success:
            rospy.logwarn("Армирование не удалось!")
            return False
            
        rospy.loginfo("Дрон успешно армирован")
        return True
        
    def takeoff(self):
        """Взлет дрона"""
        rospy.loginfo("Начинаем взлет на высоту %.1f м", self.takeoff_altitude)
        
        # Создаем сообщение с целевой позицией для взлета
        setpoint = PoseStamped()
        setpoint.header.frame_id = "map"
        setpoint.pose.position.x = 0.0  # Центр карты
        setpoint.pose.position.y = 0.0
        setpoint.pose.position.z = self.takeoff_altitude
        setpoint.pose.orientation.w = 1.0
        
        # Отправляем команду взлета
        rate = rospy.Rate(10)  # 10 Гц
        start_time = time.time()
        duration = 30.0  # Максимальное время взлета в секундах
        
        while not rospy.is_shutdown() and time.time() - start_time < duration:
            setpoint.header.stamp = rospy.Time.now()
            self.setpoint_pub.publish(setpoint)
            rate.sleep()
            
        rospy.loginfo("Взлет завершен")
        return True
        
    def run(self):
        """Основная функция работы"""
        if not self.wait_for_connection():
            return
            
        rospy.loginfo("Нажмите Enter для армирования дрона...")
        input()
        
        if self.arm():
            rospy.loginfo("Нажмите Enter для взлета на %.1f м...", self.takeoff_altitude)
            input()
            self.takeoff()
            
            rospy.loginfo("Дрон готов к навигации")
            rospy.spin()

if __name__ == '__main__':
    try:
        controller = DroneController()
        controller.run()
    except rospy.ROSInterruptException:
        pass 