#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import mavros
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from geometry_msgs.msg import PoseStamped, TwistStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix
import time
import math

class MavrosDirectControl:
    """
    Класс для прямого управления дроном через MAVROS без использования move_base
    """
    def __init__(self):
        rospy.init_node('mavros_direct_control')
        
        # Параметры
        self.takeoff_height = rospy.get_param('~takeoff_height', 2.0)
        self.arm_timeout = rospy.get_param('~arm_timeout', 60.0)
        self.command_rate = rospy.get_param('~command_rate', 20.0)  # Гц
        
        # Состояние дрона
        self.state = None
        self.extended_state = None
        self.local_position = None
        self.global_position = None
        
        # Флаги состояния
        self.armed = False
        self.guided_mode = False
        self.position_received = False
        self.takeoff_complete = False
        
        # Подписчики
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_callback)
        self.extended_state_sub = rospy.Subscriber('/mavros/extended_state', ExtendedState, self.extended_state_callback)
        self.local_pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_position_callback)
        self.global_pos_sub = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.global_position_callback)
        
        # Издатели
        self.local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        self.initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        
        # Сервисы
        rospy.loginfo("Ожидание сервисов MAVROS...")
        try:
            rospy.wait_for_service('/mavros/cmd/arming', timeout=10.0)
            rospy.wait_for_service('/mavros/set_mode', timeout=10.0)
            rospy.wait_for_service('/mavros/cmd/takeoff', timeout=10.0)
            
            self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            self.takeoff_client = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
            
            rospy.loginfo("Все сервисы MAVROS доступны")
        except rospy.ROSException as e:
            rospy.logerr("Ошибка при ожидании сервисов MAVROS: %s", str(e))
            rospy.logerr("Продолжаем работу, но некоторые функции могут быть недоступны")
        
        # Целевая позиция
        self.target_position = PoseStamped()
        self.target_position.pose.position.z = self.takeoff_height
        self.target_position.pose.orientation.w = 1.0
        
        # Частота обновления
        self.rate = rospy.Rate(self.command_rate)
        
        # Инициализация
        self.send_initial_setpoints()
        
        rospy.loginfo("Инициализация контроллера прямого управления завершена")
    
    def send_initial_setpoints(self):
        """Отправляет начальные setpoint для инициализации OFFBOARD режима"""
        rospy.loginfo("Отправка начальных setpoint...")
        
        # Отправляем несколько сообщений с нулевой позицией
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = "base_link"
        initial_pose.pose.position.z = 0.0
        initial_pose.pose.orientation.w = 1.0
        
        # Публикуем 100 сообщений с частотой 20 Гц
        for i in range(100):
            initial_pose.header.stamp = rospy.Time.now()
            self.local_pos_pub.publish(initial_pose)
            self.rate.sleep()
    
    def state_callback(self, msg):
        """Обработчик состояния дрона"""
        self.state = msg
        self.armed = msg.armed
        self.guided_mode = (msg.mode == "GUIDED" or msg.mode == "OFFBOARD")
    
    def extended_state_callback(self, msg):
        """Обработчик расширенного состояния дрона"""
        self.extended_state = msg
    
    def local_position_callback(self, msg):
        """Обработчик локальной позиции"""
        self.local_position = msg
        self.position_received = True
        
        # Проверяем завершение взлета
        if not self.takeoff_complete and self.local_position.pose.position.z >= self.takeoff_height * 0.8:
            self.takeoff_complete = True
            rospy.loginfo("Взлет завершен! Высота: %.2f м", self.local_position.pose.position.z)
    
    def global_position_callback(self, msg):
        """Обработчик глобальной позиции"""
        self.global_position = msg
    
    def set_guided_mode(self):
        """Переключение в режим GUIDED с повторными попытками"""
        if self.guided_mode:
            rospy.loginfo("Режим GUIDED уже установлен")
            return True
            
        rospy.loginfo("Переключение в режим GUIDED...")
        
        # Повторяем попытки до 5 раз
        for attempt in range(5):
            if self.set_mode_client(custom_mode="GUIDED").mode_sent:
                rospy.loginfo("Запрос на установку режима GUIDED отправлен")
                
                # Ждем подтверждения смены режима
                start_wait = time.time()
                while time.time() - start_wait < 3.0:  # Ждем до 3 секунд
                    if self.guided_mode:
                        rospy.loginfo("Режим GUIDED установлен")
                        return True
                    self.rate.sleep()
            
            rospy.logwarn("Попытка %d: Не удалось установить режим GUIDED", attempt+1)
            self.rate.sleep()
        
        rospy.logerr("Не удалось установить режим GUIDED после нескольких попыток")
        return False
    
    def arm_drone(self):
        """Армирование дрона с повторными попытками"""
        if self.armed:
            rospy.loginfo("Дрон уже армирован")
            return True
            
        rospy.loginfo("Армирование дрона...")
        
        start_time = time.time()
        while not self.armed and time.time() - start_time < self.arm_timeout:
            # Публикуем setpoint перед армированием
            for i in range(5):
                self.target_position.header.stamp = rospy.Time.now()
                self.local_pos_pub.publish(self.target_position)
                self.rate.sleep()
                
            # Отправляем команду армирования
            arm_result = self.arming_client(True)
            if arm_result.success:
                rospy.loginfo("Команда армирования принята")
                
                # Ждем подтверждения армирования
                wait_start = time.time()
                while time.time() - wait_start < 5.0:  # Ждем до 5 секунд
                    if self.armed:
                        rospy.loginfo("Дрон успешно армирован!")
                        return True
                    
                    # Продолжаем публиковать setpoint
                    self.target_position.header.stamp = rospy.Time.now()
                    self.local_pos_pub.publish(self.target_position)
                    self.rate.sleep()
            else:
                rospy.logwarn("Не удалось армировать дрон, повторная попытка... (%s)", arm_result)
            
            # Продолжаем публиковать setpoint
            for i in range(5):
                self.target_position.header.stamp = rospy.Time.now()
                self.local_pos_pub.publish(self.target_position)
                self.rate.sleep()
        
        if self.armed:
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
                if self.position_received and self.local_position.pose.position.z >= self.takeoff_height * 0.8:
                    rospy.loginfo("Взлет успешен! Текущая высота: %.2f м", self.local_position.pose.position.z)
                    self.takeoff_complete = True
                    return True
                
                # Продолжаем публиковать setpoint
                self.target_position.header.stamp = rospy.Time.now()
                self.local_pos_pub.publish(self.target_position)
                self.rate.sleep()
                
                # Каждые 5 секунд выводим текущую высоту
                if int(time.time() - start_wait) % 5 == 0 and self.position_received:
                    rospy.loginfo("Ожидание взлета... Текущая высота: %.2f м", self.local_position.pose.position.z)
        else:
            rospy.logerr("Команда взлета отклонена: %s", takeoff_result)
            return False
        
        rospy.logwarn("Взлет не удался в течение таймаута")
        return False
    
    def takeoff_setpoint(self):
        """Взлет с использованием setpoint"""
        if not self.position_received:
            rospy.logwarn("Нет данных о позиции дрона, ожидание...")
            start_wait = time.time()
            while not self.position_received and time.time() - start_wait < 10.0:
                self.rate.sleep()
                
            if not self.position_received:
                rospy.logerr("Не удалось получить данные о позиции дрона")
                return False
        
        # Устанавливаем целевую позицию для взлета
        self.target_position.header.frame_id = "base_link"
        self.target_position.pose.position.x = 0.0
        self.target_position.pose.position.y = 0.0
        self.target_position.pose.position.z = self.takeoff_height
        self.target_position.pose.orientation.w = 1.0
        
        rospy.loginfo("Взлет с использованием setpoint на высоту %.1f м...", self.takeoff_height)
        
        # Публикуем целевую позицию
        start_time = time.time()
        while not self.takeoff_complete and time.time() - start_time < 60.0:  # Ждем до 60 секунд
            self.target_position.header.stamp = rospy.Time.now()
            self.local_pos_pub.publish(self.target_position)
            
            # Каждые 5 секунд выводим текущую высоту
            if int(time.time() - start_time) % 5 == 0 and self.position_received:
                rospy.loginfo("Взлет через setpoint... Высота: %.2f м", self.local_position.pose.position.z)
                
            self.rate.sleep()
            
            # Проверяем достижение высоты
            if self.position_received and self.local_position.pose.position.z >= self.takeoff_height * 0.8:
                rospy.loginfo("Взлет через setpoint успешен! Текущая высота: %.2f м", self.local_position.pose.position.z)
                self.takeoff_complete = True
                return True
        
        if self.takeoff_complete:
            return True
        else:
            rospy.logerr("Не удалось выполнить взлет через setpoint")
            return False
    
    def move_to_position(self, x, y, z):
        """Перемещение в заданную позицию"""
        if not self.takeoff_complete:
            rospy.logwarn("Сначала выполните взлет")
            return False
            
        rospy.loginfo("Перемещение в позицию (%.2f, %.2f, %.2f)...", x, y, z)
        
        # Устанавливаем целевую позицию
        self.target_position.header.frame_id = "map"  # Используем глобальную систему координат
        self.target_position.pose.position.x = x
        self.target_position.pose.position.y = y
        self.target_position.pose.position.z = z
        
        # Публикуем целевую позицию
        start_time = time.time()
        while time.time() - start_time < 30.0:  # Ждем до 30 секунд
            self.target_position.header.stamp = rospy.Time.now()
            self.local_pos_pub.publish(self.target_position)
            
            # Вычисляем расстояние до целевой точки
            if self.position_received:
                dx = self.local_position.pose.position.x - x
                dy = self.local_position.pose.position.y - y
                dz = self.local_position.pose.position.z - z
                distance = math.sqrt(dx*dx + dy*dy + dz*dz)
                
                # Если достигли цели, выходим
                if distance < 0.5:  # 0.5 метра - точность достижения
                    rospy.loginfo("Достигнута позиция (%.2f, %.2f, %.2f)", x, y, z)
                    return True
                
                # Каждые 5 секунд выводим текущую позицию и расстояние
                if int(time.time() - start_time) % 5 == 0:
                    rospy.loginfo("Текущая позиция: (%.2f, %.2f, %.2f), расстояние: %.2f м", 
                                self.local_position.pose.position.x,
                                self.local_position.pose.position.y,
                                self.local_position.pose.position.z,
                                distance)
            
            self.rate.sleep()
        
        rospy.logwarn("Истек таймаут достижения позиции")
        return False
    
    def land(self):
        """Посадка дрона"""
        rospy.loginfo("Выполняем посадку...")
        
        # Переключаемся в режим LAND
        if self.set_mode_client(custom_mode="LAND").mode_sent:
            rospy.loginfo("Команда посадки отправлена")
            
            # Ждем завершения посадки
            start_time = time.time()
            while time.time() - start_time < 60.0:  # Ждем до 60 секунд
                if not self.armed:
                    rospy.loginfo("Посадка успешно завершена")
                    return True
                    
                # Каждые 5 секунд выводим текущую высоту
                if int(time.time() - start_time) % 5 == 0 and self.position_received:
                    rospy.loginfo("Выполняется посадка... Высота: %.2f м", self.local_position.pose.position.z)
                    
                self.rate.sleep()
            
            rospy.logwarn("Истек таймаут посадки")
            return False
        else:
            rospy.logerr("Не удалось отправить команду посадки")
            return False
    
    def run(self):
        """Основной цикл управления"""
        rospy.loginfo("Запуск основного цикла управления...")
        
        # Переключаемся в режим GUIDED и армируем дрон
        if not self.set_guided_mode():
            rospy.logerr("Не удалось установить режим GUIDED, отмена операции")
            return
            
        if not self.arm_drone():
            rospy.logerr("Не удалось армировать дрон, отмена операции")
            return
            
        rospy.loginfo("Дрон переведен в режим GUIDED и армирован")
        
        # Публикуем начальную позу для инициализации TF
        self.publish_initial_pose()
        
        # Пытаемся взлететь
        if not self.takeoff_direct():
            rospy.loginfo("Прямой взлет не удался, пробуем взлететь через setpoint")
            if not self.takeoff_setpoint():
                rospy.logerr("Не удалось выполнить взлет")
                return
        
        # Основной цикл управления
        while not rospy.is_shutdown():
            # Публикуем текущую позицию для обновления TF
            if self.position_received:
                self.publish_initial_pose()
            
            # Проверяем состояние дрона
            if not self.armed:
                rospy.logwarn("Дрон разармирован, завершение работы")
                break
            
            self.rate.sleep()
    
    def publish_initial_pose(self):
        """Публикация текущей позиции в топик initialpose для обновления TF"""
        if not self.position_received:
            return
            
        # Создаем сообщение PoseWithCovarianceStamped
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.stamp = rospy.Time.now()
        initial_pose.header.frame_id = "map"
        
        # Заполняем позицию из текущей позиции дрона
        initial_pose.pose.pose.position.x = self.local_position.pose.position.x
        initial_pose.pose.pose.position.y = self.local_position.pose.position.y
        initial_pose.pose.pose.position.z = self.local_position.pose.position.z
        initial_pose.pose.pose.orientation = self.local_position.pose.orientation
        
        # Устанавливаем ковариацию (диагональная матрица с небольшими значениями)
        for i in range(36):
            if i % 7 == 0:  # Диагональные элементы
                initial_pose.pose.covariance[i] = 0.01
            else:
                initial_pose.pose.covariance[i] = 0.0
        
        # Публикуем сообщение
        self.initial_pose_pub.publish(initial_pose)
        
        # Выводим текущую позицию (не слишком часто)
        if int(rospy.Time.now().to_sec()) % 5 == 0:
            rospy.loginfo("Текущая позиция робота в TF: x=%.2f, y=%.2f",
                        self.local_position.pose.position.x,
                        self.local_position.pose.position.y)

if __name__ == '__main__':
    try:
        controller = MavrosDirectControl()
        controller.run()
    except rospy.ROSInterruptException:
        pass 