#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
import math
import threading
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Header

class MultiDroneController:
    """
    Контроллер для управления несколькими дронами
    """
    
    def __init__(self):
        rospy.init_node('multi_drone_controller')
        
        # Параметры
        self.num_drones = rospy.get_param('~num_drones', 5)
        self.takeoff_altitude = rospy.get_param('~takeoff_altitude', 2.0)
        self.arm_timeout = rospy.get_param('~arm_timeout', 60.0)
        self.command_rate = rospy.get_param('~command_rate', 20.0)
        
        # Начальные и конечные точки
        self.start_positions = []
        self.end_positions = []
        
        # Формируем начальные позиции в виде линии
        for i in range(self.num_drones):
            self.start_positions.append(Point(0.0, i * 2.0, self.takeoff_altitude))
            self.end_positions.append(Point(10.0, i * 2.0, self.takeoff_altitude))
        
        # Создаем структуры для каждого дрона
        self.drones = []
        for i in range(self.num_drones):
            drone_id = i + 1
            drone = {
                'id': drone_id,
                'namespace': f'/drone{drone_id}',
                'state': State(),
                'state_sub': None,
                'local_pos_pub': None,
                'arming_client': None,
                'set_mode_client': None,
                'armed': False,
                'guided': False,
                'current_position': Point(0, 0, 0),
                'target_position': self.start_positions[i],
                'end_position': self.end_positions[i],
                'status': 'initializing'  # initializing, arming, taking_off, moving, completed
            }
            self.drones.append(drone)
        
        # Настраиваем подписчиков, издателей и сервисы для каждого дрона
        for drone in self.drones:
            ns = drone['namespace']
            drone['state_sub'] = rospy.Subscriber(
                f'{ns}/mavros/state', 
                State, 
                lambda msg, d=drone: self.state_callback(msg, d)
            )
            drone['local_pos_pub'] = rospy.Publisher(
                f'{ns}/mavros/setpoint_position/local',
                PoseStamped,
                queue_size=10
            )
            drone['arming_client'] = rospy.ServiceProxy(
                f'{ns}/mavros/cmd/arming',
                CommandBool
            )
            drone['set_mode_client'] = rospy.ServiceProxy(
                f'{ns}/mavros/set_mode',
                SetMode
            )
        
        # Частота обновления
        self.rate = rospy.Rate(self.command_rate)
        
        rospy.loginfo(f"Контроллер для {self.num_drones} дронов инициализирован")
    
    def state_callback(self, msg, drone):
        """Обработчик состояния дрона"""
        drone['state'] = msg
        drone['armed'] = msg.armed
        drone['guided'] = (msg.mode == "GUIDED")
    
    def set_guided_mode(self, drone):
        """Переключение в режим GUIDED для конкретного дрона"""
        if drone['guided']:
            return True
            
        rospy.loginfo(f"Дрон {drone['id']}: Переключение в режим GUIDED...")
        
        # Повторяем попытки до 5 раз
        for attempt in range(5):
            if drone['state'].mode == "GUIDED":
                rospy.loginfo(f"Дрон {drone['id']}: Режим GUIDED уже установлен")
                return True
                
            if drone['set_mode_client'](custom_mode="GUIDED").mode_sent:
                rospy.loginfo(f"Дрон {drone['id']}: Запрос на установку режима GUIDED отправлен")
                
                # Ждем подтверждения смены режима
                start_wait = time.time()
                while time.time() - start_wait < 3.0:  # Ждем до 3 секунд
                    if drone['state'].mode == "GUIDED":
                        rospy.loginfo(f"Дрон {drone['id']}: Режим GUIDED установлен")
                        drone['guided'] = True
                        return True
                    self.rate.sleep()
            
            rospy.logwarn(f"Дрон {drone['id']}: Попытка {attempt+1}: Не удалось установить режим GUIDED")
            self.rate.sleep()
        
        rospy.logerr(f"Дрон {drone['id']}: Не удалось установить режим GUIDED после нескольких попыток")
        return False
    
    def arm_drone(self, drone):
        """Армирование конкретного дрона"""
        if drone['armed']:
            rospy.loginfo(f"Дрон {drone['id']}: Уже армирован")
            return True
            
        rospy.loginfo(f"Дрон {drone['id']}: Армирование...")
        
        # Создаем сообщение с целевой позицией
        setpoint = PoseStamped()
        setpoint.header.frame_id = "map"
        setpoint.pose.position = drone['target_position']
        setpoint.pose.orientation.w = 1.0
        
        start_time = time.time()
        while not drone['armed'] and time.time() - start_time < self.arm_timeout:
            # Публикуем setpoint перед армированием
            for i in range(10):
                setpoint.header.stamp = rospy.Time.now()
                drone['local_pos_pub'].publish(setpoint)
                self.rate.sleep()
                
            # Отправляем команду армирования
            arm_result = drone['arming_client'](True)
            if arm_result.success:
                rospy.loginfo(f"Дрон {drone['id']}: Команда армирования принята")
                
                # Ждем подтверждения армирования
                wait_start = time.time()
                while time.time() - wait_start < 5.0:  # Ждем до 5 секунд
                    if drone['armed']:
                        rospy.loginfo(f"Дрон {drone['id']}: Успешно армирован!")
                        return True
                    setpoint.header.stamp = rospy.Time.now()
                    drone['local_pos_pub'].publish(setpoint)
                    self.rate.sleep()
            else:
                rospy.logwarn(f"Дрон {drone['id']}: Не удалось армировать, повторная попытка...")
            
            # Продолжаем публиковать setpoint
            for i in range(5):
                setpoint.header.stamp = rospy.Time.now()
                drone['local_pos_pub'].publish(setpoint)
                self.rate.sleep()
                
        if not drone['armed']:
            rospy.logerr(f"Дрон {drone['id']}: Не удалось армировать после нескольких попыток")
            return False
            
        return True
    
    def takeoff(self, drone):
        """Взлет конкретного дрона"""
        rospy.loginfo(f"Дрон {drone['id']}: Взлет на высоту {self.takeoff_altitude} м...")
        
        # Создаем сообщение с целевой позицией для взлета
        setpoint = PoseStamped()
        setpoint.header.frame_id = "map"
        setpoint.pose.position = drone['target_position']
        setpoint.pose.orientation.w = 1.0
        
        # Отправляем команду взлета
        start_time = time.time()
        duration = 10.0  # Время взлета в секундах
        
        while not rospy.is_shutdown() and time.time() - start_time < duration:
            setpoint.header.stamp = rospy.Time.now()
            drone['local_pos_pub'].publish(setpoint)
            self.rate.sleep()
            
        rospy.loginfo(f"Дрон {drone['id']}: Взлет завершен")
        return True
    
    def move_to_target(self, drone):
        """Перемещение дрона к целевой точке"""
        rospy.loginfo(f"Дрон {drone['id']}: Перемещение к целевой точке ({drone['end_position'].x}, {drone['end_position'].y}, {drone['end_position'].z})...")
        
        # Создаем сообщение с целевой позицией
        setpoint = PoseStamped()
        setpoint.header.frame_id = "map"
        setpoint.pose.position = drone['end_position']
        setpoint.pose.orientation.w = 1.0
        
        # Отправляем команду перемещения
        start_time = time.time()
        duration = 30.0  # Время на перемещение в секундах
        
        while not rospy.is_shutdown() and time.time() - start_time < duration:
            setpoint.header.stamp = rospy.Time.now()
            drone['local_pos_pub'].publish(setpoint)
            self.rate.sleep()
            
        rospy.loginfo(f"Дрон {drone['id']}: Достиг целевой точки")
        return True
    
    def process_drone(self, drone):
        """Обработка одного дрона в отдельном потоке"""
        rospy.loginfo(f"Запуск обработки дрона {drone['id']}...")
        
        # Переключаемся в режим GUIDED
        if not self.set_guided_mode(drone):
            rospy.logerr(f"Дрон {drone['id']}: Не удалось установить режим GUIDED, отмена операции")
            return
        
        drone['status'] = 'arming'
        
        # Армируем дрон
        if not self.arm_drone(drone):
            rospy.logerr(f"Дрон {drone['id']}: Не удалось армировать, отмена операции")
            return
            
        drone['status'] = 'taking_off'
        
        # Взлетаем
        if not self.takeoff(drone):
            rospy.logerr(f"Дрон {drone['id']}: Не удалось взлететь, отмена операции")
            return
            
        drone['status'] = 'moving'
        
        # Перемещаемся к целевой точке
        if not self.move_to_target(drone):
            rospy.logerr(f"Дрон {drone['id']}: Не удалось переместиться к целевой точке")
            return
            
        drone['status'] = 'completed'
        rospy.loginfo(f"Дрон {drone['id']}: Успешно завершил миссию")
    
    def run(self):
        """Запуск управления всеми дронами"""
        rospy.loginfo("Запуск управления дронами...")
        
        # Создаем потоки для каждого дрона
        threads = []
        for drone in self.drones:
            thread = threading.Thread(target=self.process_drone, args=(drone,))
            threads.append(thread)
            thread.start()
            # Небольшая задержка между запусками дронов
            time.sleep(2.0)
        
        # Ждем завершения всех потоков
        for thread in threads:
            thread.join()
            
        rospy.loginfo("Все дроны завершили выполнение миссии")

if __name__ == '__main__':
    try:
        controller = MultiDroneController()
        controller.run()
    except rospy.ROSInterruptException:
        pass 