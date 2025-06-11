#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import mavros
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from geometry_msgs.msg import PoseStamped
import time

class DroneController:
    def __init__(self):
        rospy.init_node('arm_takeoff', anonymous=True)
        
        # Параметры
        self.takeoff_height = rospy.get_param('~takeoff_height', 2.0)
        self.arm_timeout = rospy.get_param('~arm_timeout', 60.0)
        
        # Состояние дрона
        self.state = State()
        self.extended_state = ExtendedState()
        self.local_pos = PoseStamped()
        self.current_altitude = 0.0
        
        # Подписываемся на топики
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_callback)
        self.extended_state_sub = rospy.Subscriber('/mavros/extended_state', ExtendedState, self.extended_state_callback)
        self.local_pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_pos_callback)
        
        # Издатели
        self.local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        
        # Сервисы
        rospy.loginfo("Ожидание сервисов MAVROS...")
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/set_mode')
        rospy.wait_for_service('/mavros/cmd/takeoff')
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.takeoff_client = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        
        # Инициализация позиции
        self.local_pos.pose.position.x = 0
        self.local_pos.pose.position.y = 0
        self.local_pos.pose.position.z = self.takeoff_height
        self.local_pos.pose.orientation.w = 1.0
        
        # Частота обновления
        self.rate = rospy.Rate(20.0)
        
        rospy.loginfo("Инициализация контроллера дрона завершена")
    
    def state_callback(self, msg):
        """Обработчик состояния дрона"""
        self.state = msg
        
    def extended_state_callback(self, msg):
        """Обработчик расширенного состояния дрона"""
        self.extended_state = msg
        
    def local_pos_callback(self, msg):
        """Обработчик локальной позиции"""
        self.current_altitude = msg.pose.position.z
    
    def set_guided_mode(self):
        """Переключение в режим GUIDED с повторными попытками"""
        rospy.loginfo("Переключение в режим GUIDED...")
        
        # Повторяем попытки до 5 раз
        for attempt in range(5):
            if self.state.mode == "GUIDED":
                rospy.loginfo("Режим GUIDED уже установлен")
                return True
                
            if self.set_mode_client(custom_mode="GUIDED").mode_sent:
                rospy.loginfo("Запрос на установку режима GUIDED отправлен")
                
                # Ждем подтверждения смены режима
                start_wait = time.time()
                while time.time() - start_wait < 3.0:  # Ждем до 3 секунд
                    if self.state.mode == "GUIDED":
                        rospy.loginfo("Режим GUIDED установлен")
                        return True
                    self.rate.sleep()
            
            rospy.logwarn("Попытка %d: Не удалось установить режим GUIDED", attempt+1)
            self.rate.sleep()
        
        rospy.logerr("Не удалось установить режим GUIDED после нескольких попыток")
        return False
    
    def arm_drone(self):
        """Армирование дрона с повторными попытками"""
        rospy.loginfo("Армирование дрона...")
        
        # Если уже армирован, ничего не делаем
        if self.state.armed:
            rospy.loginfo("Дрон уже армирован")
            return True
            
        start_time = time.time()
        while not self.state.armed and time.time() - start_time < self.arm_timeout:
            # Публикуем setpoint перед армированием
            for i in range(10):
                self.local_pos_pub.publish(self.local_pos)
                self.rate.sleep()
                
            # Отправляем команду армирования
            arm_result = self.arming_client(True)
            if arm_result.success:
                rospy.loginfo("Команда армирования принята")
                
                # Ждем подтверждения армирования
                wait_start = time.time()
                while time.time() - wait_start < 5.0:  # Ждем до 5 секунд
                    if self.state.armed:
                        rospy.loginfo("Дрон успешно армирован!")
                        return True
                    self.local_pos_pub.publish(self.local_pos)
                    self.rate.sleep()
            else:
                rospy.logwarn("Не удалось армировать дрон, повторная попытка... (%s)", arm_result)
            
            # Продолжаем публиковать setpoint
            for i in range(5):
                self.local_pos_pub.publish(self.local_pos)
                self.rate.sleep()
        
        if self.state.armed:
            rospy.loginfo("Дрон успешно армирован!")
            return True
        else:
            rospy.logerr("Превышено время ожидания армирования")
            return False
    
    def takeoff_direct(self):
        """Прямая команда взлета через сервис takeoff"""
        rospy.loginfo("Отправка прямой команды взлета на высоту %.1f метров...", self.takeoff_height)
        
        # Отправляем команду взлета
        takeoff_result = self.takeoff_client(min_pitch=0, yaw=0, latitude=0, longitude=0, altitude=self.takeoff_height)
        
        if takeoff_result.success:
            rospy.loginfo("Команда взлета принята")
            
            # Ждем достижения высоты
            start_wait = time.time()
            while time.time() - start_wait < 30.0:  # Ждем до 30 секунд
                # Проверяем высоту
                if self.current_altitude >= self.takeoff_height * 0.8:  # 80% от целевой высоты
                    rospy.loginfo("Взлет успешен! Текущая высота: %.2f м", self.current_altitude)
                    return True
                
                # Продолжаем публиковать setpoint
                self.local_pos_pub.publish(self.local_pos)
                self.rate.sleep()
                
                # Каждые 5 секунд выводим текущую высоту
                if int(time.time() - start_wait) % 5 == 0:
                    rospy.loginfo("Ожидание взлета... Текущая высота: %.2f м", self.current_altitude)
        else:
            rospy.logerr("Команда взлета отклонена: %s", takeoff_result)
            return False
        
        rospy.logwarn("Взлет не удался в течение таймаута")
        return False
    
    def arm_and_takeoff(self):
        """Армирует дрон и выполняет взлет"""
        # Сначала публикуем несколько сообщений с целевой позицией
        rospy.loginfo("Инициализация setpoint...")
        for i in range(50):
            self.local_pos_pub.publish(self.local_pos)
            self.rate.sleep()
        
        # Переключаемся в режим GUIDED
        if not self.set_guided_mode():
            return False
        
        # Армируем дрон
        if not self.arm_drone():
            return False
        
        # Пробуем прямую команду взлета
        if self.takeoff_direct():
            return True
            
        # Если прямая команда не сработала, используем setpoint
        rospy.loginfo("Прямая команда взлета не сработала, используем setpoint...")
        start_time = time.time()
        while time.time() - start_time < 30.0:  # Ждем до 30 секунд
            self.local_pos_pub.publish(self.local_pos)
            
            # Проверяем высоту
            if self.current_altitude >= self.takeoff_height * 0.8:  # 80% от целевой высоты
                rospy.loginfo("Взлет успешен! Текущая высота: %.2f м", self.current_altitude)
                return True
                
            # Каждые 5 секунд выводим текущую высоту
            if int(time.time() - start_time) % 5 == 0:
                rospy.loginfo("Ожидание взлета через setpoint... Текущая высота: %.2f м", self.current_altitude)
                
            self.rate.sleep()
        
        rospy.logerr("Не удалось выполнить взлет")
        return False
    
    def run(self):
        """Основной цикл работы"""
        # Ждем соединения с FCU
        rospy.loginfo("Ожидание подключения к FCU...")
        while not rospy.is_shutdown() and not self.state.connected:
            self.rate.sleep()
        
        rospy.loginfo("Подключение к FCU установлено")
        
        # Армируем и взлетаем
        if not self.arm_and_takeoff():
            rospy.logerr("Не удалось выполнить взлет")
            return
        
        # Продолжаем публиковать позицию
        rospy.loginfo("Удерживаем позицию...")
        while not rospy.is_shutdown():
            self.local_pos_pub.publish(self.local_pos)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = DroneController()
        controller.run()
    except rospy.ROSInterruptException:
        pass 