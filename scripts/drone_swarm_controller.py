#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
import math
import numpy as np
from threading import Thread, Lock
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

class DroneSwarmController:
    """
    Контроллер для управления роем дронов
    """
    
    def __init__(self):
        rospy.init_node('drone_swarm_controller')
        
        # Параметры
        self.num_drones = rospy.get_param('~num_drones', 5)
        self.takeoff_altitude = rospy.get_param('~takeoff_altitude', 2.0)
        self.arm_timeout = rospy.get_param('~arm_timeout', 60.0)
        self.command_rate = rospy.get_param('~command_rate', 20.0)
        self.formation_distance = rospy.get_param('~formation_distance', 2.0)
        
        # Точки маршрута
        self.start_point = Point(0.0, 0.0, self.takeoff_altitude)
        self.end_point = Point(10.0, 0.0, self.takeoff_altitude)
        
        # Создаем структуры для каждого дрона
        self.drones = []
        self.mutex = Lock()
        
        # Формации роя
        self.formations = {
            'v_formation': self.generate_v_formation,
            'line_formation': self.generate_line_formation,
            'circle_formation': self.generate_circle_formation
        }
        
        # Текущая формация
        self.current_formation = 'v_formation'
        
        # Маркеры для визуализации
        self.marker_pub = rospy.Publisher('/drone_swarm/markers', MarkerArray, queue_size=10)
        self.markers = MarkerArray()
        
        # Инициализация дронов
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
                'position': Point(0, 0, 0),
                'target_position': Point(0, 0, 0),
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
            drone['local_pos_sub'] = rospy.Subscriber(
                f'{ns}/mavros/local_position/pose',
                PoseStamped,
                lambda msg, d=drone: self.local_pos_callback(msg, d)
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
        
        rospy.loginfo(f"Контроллер роя из {self.num_drones} дронов инициализирован")
    
    def state_callback(self, msg, drone):
        """Обработчик состояния дрона"""
        with self.mutex:
            drone['state'] = msg
            drone['armed'] = msg.armed
            drone['guided'] = (msg.mode == "GUIDED")
    
    def local_pos_callback(self, msg, drone):
        """Обработчик локальной позиции дрона"""
        with self.mutex:
            drone['position'].x = msg.pose.position.x
            drone['position'].y = msg.pose.position.y
            drone['position'].z = msg.pose.position.z
    
    def generate_v_formation(self, leader_pos):
        """Генерирует V-образную формацию относительно позиции лидера"""
        positions = []
        
        # Лидер в вершине V-формации
        positions.append(Point(leader_pos.x, leader_pos.y, leader_pos.z))
        
        # Вычисляем направление движения
        direction = Point(
            self.end_point.x - self.start_point.x,
            self.end_point.y - self.start_point.y,
            0
        )
        
        # Нормализуем вектор направления
        length = math.sqrt(direction.x**2 + direction.y**2)
        if length > 0:
            direction.x /= length
            direction.y /= length
        
        # Вычисляем перпендикулярный вектор
        perp = Point(-direction.y, direction.x, 0)
        
        # Добавляем дронов в V-формации
        for i in range(1, self.num_drones):
            if i % 2 == 1:  # Нечетные - слева
                side = -1
                idx = (i + 1) // 2
            else:  # Четные - справа
                side = 1
                idx = i // 2
                
            pos = Point(
                leader_pos.x - direction.x * idx * self.formation_distance,
                leader_pos.y - direction.y * idx * self.formation_distance + side * perp.y * self.formation_distance,
                leader_pos.z
            )
            positions.append(pos)
        
        return positions
    
    def generate_line_formation(self, leader_pos):
        """Генерирует линейную формацию относительно позиции лидера"""
        positions = []
        
        # Вычисляем направление движения
        direction = Point(
            self.end_point.x - self.start_point.x,
            self.end_point.y - self.start_point.y,
            0
        )
        
        # Нормализуем вектор направления
        length = math.sqrt(direction.x**2 + direction.y**2)
        if length > 0:
            direction.x /= length
            direction.y /= length
        
        # Вычисляем перпендикулярный вектор
        perp = Point(-direction.y, direction.x, 0)
        
        # Добавляем дронов в линейной формации (перпендикулярно движению)
        for i in range(self.num_drones):
            offset = (i - (self.num_drones - 1) / 2) * self.formation_distance
            pos = Point(
                leader_pos.x + perp.x * offset,
                leader_pos.y + perp.y * offset,
                leader_pos.z
            )
            positions.append(pos)
        
        return positions
    
    def generate_circle_formation(self, leader_pos):
        """Генерирует круговую формацию относительно позиции лидера"""
        positions = []
        
        # Лидер в центре
        positions.append(Point(leader_pos.x, leader_pos.y, leader_pos.z))
        
        # Добавляем дронов по кругу
        if self.num_drones > 1:
            angle_step = 2 * math.pi / (self.num_drones - 1)
            for i in range(1, self.num_drones):
                angle = i * angle_step
                pos = Point(
                    leader_pos.x + self.formation_distance * math.cos(angle),
                    leader_pos.y + self.formation_distance * math.sin(angle),
                    leader_pos.z
                )
                positions.append(pos)
        
        return positions
    
    def update_formation_positions(self, leader_pos):
        """Обновляет позиции дронов в соответствии с текущей формацией"""
        formation_func = self.formations.get(self.current_formation, self.generate_v_formation)
        return formation_func(leader_pos)
    
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
                    with self.mutex:
                        if drone['guided']:
                            rospy.loginfo(f"Дрон {drone['id']}: Режим GUIDED установлен")
                            return True
                    self.rate.sleep()
            
            rospy.logwarn(f"Дрон {drone['id']}: Попытка {attempt+1}: Не удалось установить режим GUIDED")
            self.rate.sleep()
        
        rospy.logerr(f"Дрон {drone['id']}: Не удалось установить режим GUIDED после нескольких попыток")
        return False
    
    def arm_drone(self, drone):
        """Армирование конкретного дрона"""
        with self.mutex:
            if drone['armed']:
                rospy.loginfo(f"Дрон {drone['id']}: Уже армирован")
                return True
            
        rospy.loginfo(f"Дрон {drone['id']}: Армирование...")
        
        # Создаем сообщение с целевой позицией
        setpoint = PoseStamped()
        setpoint.header.frame_id = "map"
        with self.mutex:
            setpoint.pose.position = drone['position']
        setpoint.pose.position.z = self.takeoff_altitude
        setpoint.pose.orientation.w = 1.0
        
        start_time = time.time()
        while time.time() - start_time < self.arm_timeout:
            with self.mutex:
                if drone['armed']:
                    rospy.loginfo(f"Дрон {drone['id']}: Успешно армирован!")
                    return True
            
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
                    with self.mutex:
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
                
        rospy.logerr(f"Дрон {drone['id']}: Не удалось армировать после нескольких попыток")
        return False
    
    def takeoff(self, drone):
        """Взлет конкретного дрона"""
        rospy.loginfo(f"Дрон {drone['id']}: Взлет на высоту {self.takeoff_altitude} м...")
        
        # Создаем сообщение с целевой позицией для взлета
        setpoint = PoseStamped()
        setpoint.header.frame_id = "map"
        with self.mutex:
            setpoint.pose.position.x = drone['position'].x
            setpoint.pose.position.y = drone['position'].y
        setpoint.pose.position.z = self.takeoff_altitude
        setpoint.pose.orientation.w = 1.0
        
        # Отправляем команду взлета
        start_time = time.time()
        duration = 15.0  # Время взлета в секундах
        
        while not rospy.is_shutdown() and time.time() - start_time < duration:
            setpoint.header.stamp = rospy.Time.now()
            drone['local_pos_pub'].publish(setpoint)
            
            # Проверяем достижение высоты
            with self.mutex:
                if abs(drone['position'].z - self.takeoff_altitude) < 0.5:
                    rospy.loginfo(f"Дрон {drone['id']}: Достиг высоты взлета")
                    return True
                    
            self.rate.sleep()
            
        with self.mutex:
            if abs(drone['position'].z - self.takeoff_altitude) < 1.0:
                rospy.loginfo(f"Дрон {drone['id']}: Взлет завершен (не точно)")
                return True
            else:
                rospy.logwarn(f"Дрон {drone['id']}: Не удалось достичь высоты взлета")
                return False
    
    def move_swarm_to_position(self, target_position, formation):
        """Перемещение роя дронов к целевой точке в заданной формации"""
        rospy.loginfo(f"Перемещение роя в формации {formation} к точке ({target_position.x}, {target_position.y}, {target_position.z})...")
        
        self.current_formation = formation
        
        # Вычисляем позиции для каждого дрона в формации
        formation_positions = self.update_formation_positions(target_position)
        
        # Обновляем целевые позиции дронов
        for i, drone in enumerate(self.drones):
            if i < len(formation_positions):
                with self.mutex:
                    drone['target_position'] = formation_positions[i]
        
        # Публикуем маркеры для визуализации целевых позиций
        self.publish_target_markers(formation_positions)
        
        # Отправляем команды перемещения
        start_time = time.time()
        timeout = 60.0  # Таймаут в секундах
        
        while not rospy.is_shutdown() and time.time() - start_time < timeout:
            # Проверяем, достигли ли все дроны своих позиций
            all_reached = True
            
            for i, drone in enumerate(self.drones):
                if i >= len(formation_positions):
                    continue
                    
                # Создаем сообщение с целевой позицией
                setpoint = PoseStamped()
                setpoint.header.frame_id = "map"
                setpoint.header.stamp = rospy.Time.now()
                
                with self.mutex:
                    target_pos = drone['target_position']
                    current_pos = drone['position']
                    
                    # Проверяем, достиг ли дрон целевой позиции
                    dx = abs(current_pos.x - target_pos.x)
                    dy = abs(current_pos.y - target_pos.y)
                    dz = abs(current_pos.z - target_pos.z)
                    
                    if dx > 0.5 or dy > 0.5 or dz > 0.5:
                        all_reached = False
                
                setpoint.pose.position = target_pos
                setpoint.pose.orientation.w = 1.0
                
                # Отправляем команду
                drone['local_pos_pub'].publish(setpoint)
            
            if all_reached:
                rospy.loginfo("Все дроны достигли целевых позиций в формации")
                return True
                
            self.rate.sleep()
        
        rospy.logwarn("Не все дроны достигли целевых позиций за отведенное время")
        return False
    
    def publish_target_markers(self, positions):
        """Публикация маркеров целевых позиций для визуализации"""
        markers = MarkerArray()
        
        for i, pos in enumerate(positions):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "target_positions"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = pos
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.5
            marker.lifetime = rospy.Duration(0)
            
            markers.markers.append(marker)
        
        self.marker_pub.publish(markers)
    
    def process_drone(self, drone):
        """Обработка одного дрона в отдельном потоке"""
        rospy.loginfo(f"Запуск обработки дрона {drone['id']}...")
        
        # Переключаемся в режим GUIDED
        if not self.set_guided_mode(drone):
            rospy.logerr(f"Дрон {drone['id']}: Не удалось установить режим GUIDED, отмена операции")
            return
        
        with self.mutex:
            drone['status'] = 'arming'
        
        # Армируем дрон
        if not self.arm_drone(drone):
            rospy.logerr(f"Дрон {drone['id']}: Не удалось армировать, отмена операции")
            return
            
        with self.mutex:
            drone['status'] = 'taking_off'
        
        # Взлетаем
        if not self.takeoff(drone):
            rospy.logerr(f"Дрон {drone['id']}: Не удалось взлететь, отмена операции")
            return
            
        with self.mutex:
            drone['status'] = 'ready'
            
        rospy.loginfo(f"Дрон {drone['id']}: Готов к полету в рое")
    
    def run_swarm_mission(self):
        """Выполнение миссии роя дронов"""
        rospy.loginfo("Запуск миссии роя дронов...")
        
        # Инициализация и взлет всех дронов
        threads = []
        for drone in self.drones:
            thread = Thread(target=self.process_drone, args=(drone,))
            threads.append(thread)
            thread.start()
            time.sleep(2.0)  # Небольшая задержка между запусками дронов
        
        # Ждем, пока все дроны будут готовы
        for thread in threads:
            thread.join()
        
        # Проверяем, все ли дроны готовы
        all_ready = True
        for drone in self.drones:
            with self.mutex:
                if drone['status'] != 'ready':
                    all_ready = False
                    break
        
        if not all_ready:
            rospy.logerr("Не все дроны готовы к выполнению миссии, отмена")
            return
            
        rospy.loginfo("Все дроны готовы, начинаем выполнение миссии")
        
        # Шаг 1: Формируем V-образную формацию в начальной точке
        mid_point = Point(
            (self.start_point.x + self.end_point.x) / 2,
            (self.start_point.y + self.end_point.y) / 2,
            self.takeoff_altitude
        )
        
        # Перемещаемся в начальную точку в V-формации
        if not self.move_swarm_to_position(self.start_point, 'v_formation'):
            rospy.logerr("Не удалось сформировать начальную формацию")
            return
            
        rospy.loginfo("Рой построен в V-формацию в начальной точке")
        rospy.sleep(5.0)  # Пауза для демонстрации
        
        # Шаг 2: Перестраиваемся в линейную формацию в средней точке
        if not self.move_swarm_to_position(mid_point, 'line_formation'):
            rospy.logerr("Не удалось перестроиться в линейную формацию")
            return
            
        rospy.loginfo("Рой перестроен в линейную формацию в средней точке")
        rospy.sleep(5.0)  # Пауза для демонстрации
        
        # Шаг 3: Перестраиваемся в круговую формацию в конечной точке
        if not self.move_swarm_to_position(self.end_point, 'circle_formation'):
            rospy.logerr("Не удалось перестроиться в круговую формацию")
            return
            
        rospy.loginfo("Рой перестроен в круговую формацию в конечной точке")
        rospy.sleep(5.0)  # Пауза для демонстрации
        
        # Шаг 4: Возвращаемся в V-формацию для завершения
        if not self.move_swarm_to_position(self.end_point, 'v_formation'):
            rospy.logerr("Не удалось вернуться в V-формацию")
            return
            
        rospy.loginfo("Миссия роя успешно завершена!")

if __name__ == '__main__':
    try:
        controller = DroneSwarmController()
        controller.run_swarm_mission()
    except rospy.ROSInterruptException:
        pass 