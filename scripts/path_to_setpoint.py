#!/usr/bin/env python3
import rospy
import math
import time
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker

class PathFollower:
    def __init__(self):
        rospy.init_node('path_to_setpoint')

        # Параметры
        self.wp_radius = rospy.get_param('~waypoint_radius', 0.5)  # Радиус достижения точки пути
        self.altitude = rospy.get_param('~altitude', 1.5)  # Высота полета
        self.use_custom_planner = rospy.get_param('~use_custom_planner', True)  # Использовать ли наш планировщик
        self.takeoff_height = rospy.get_param('~takeoff_height', 2.0)  # Высота взлета
        self.wait_for_position = rospy.get_param('~wait_for_position', True)  # Ждать ли определения позиции
        self.position_timeout = rospy.get_param('~position_timeout', 30.0)  # Таймаут ожидания позиции в секундах
        
        # Состояние
        self.current_path = None
        self.current_wp_index = 0
        self.drone_state = None
        self.current_position = None
        self.path_complete = False
        self.armed = False
        self.guided_mode = False
        self.takeoff_complete = False  # Флаг завершения взлета
        self.takeoff_start_time = None  # Время начала взлета
        self.takeoff_timeout = rospy.Duration(10.0)  # Таймаут взлета (10 сек)
        self.takeoff_min_alt = 1.0  # Минимальная высота для определения взлета
        self.global_costmap_info = None  # Информация о глобальной карте стоимости
        
        # Паблишеры
        self.setpoint_pub = rospy.Publisher(
            '/mavros/setpoint_position/local',
            PoseStamped, queue_size=1)
            
        # Публикатор для маркера текущей цели
        self.target_marker_pub = rospy.Publisher(
            '/path_target_marker',
            Marker, queue_size=1)
        
        # Подписчики
        # Поддерживаем как move_base так и наш собственный планировщик
        if self.use_custom_planner:
            rospy.Subscriber('/drone_global_planner/path', Path, self.path_cb)
        else:
            rospy.Subscriber('/move_base/NavfnROS/plan', Path, self.path_cb)
            
        rospy.Subscriber('/mavros/state', State, self.state_cb)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.costmap_cb)
        
        # Сервисы арминга и смены режима
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/set_mode')
        self.arm_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        
        # Ждем определения позиции, если это требуется
        if self.wait_for_position:
            self.wait_for_position_estimate()
        
        # Таймер для публикации setpoint
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_cb)
        
        # Задержка перед первым армингом
        rospy.loginfo("Инициализация PathFollower, ожидание соединения с MAVROS...")
        rate = rospy.Rate(10)
        for _ in range(50):
            # Публикуем «нулевую» позу, чтобы OFFBOARD/GUIDED принял связь
            self.publish_hover_setpoint()
            rate.sleep()
            
        # Переводим в GUIDED (или OFFBOARD) и армим
        self.set_guided_mode()
        
        # Запускаем взлет
        self.takeoff()
        
        rospy.loginfo("PathFollower готов и выполняет взлет")
    
    def costmap_cb(self, msg):
        """Обработчик сообщений глобальной карты стоимости"""
        self.global_costmap_info = msg.info
        
        # При первом получении карты стоимости выводим информацию
        if not hasattr(self, 'costmap_received') or not self.costmap_received:
            self.costmap_received = True
            rospy.loginfo("Получена глобальная карта стоимости:")
            rospy.loginfo("  Размер: %dx%d пикселей", msg.info.width, msg.info.height)
            rospy.loginfo("  Разрешение: %.3f м/пиксель", msg.info.resolution)
            rospy.loginfo("  Начало координат: (%.2f, %.2f)", 
                         msg.info.origin.position.x, msg.info.origin.position.y)
    
    def wait_for_position_estimate(self):
        """Ожидает получения оценки позиции дрона"""
        rospy.loginfo("Ожидание определения позиции дрона...")
        
        start_time = rospy.Time.now()
        rate = rospy.Rate(2)  # 2 Гц
        
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            
            if self.current_position is not None:
                rospy.loginfo("Позиция дрона определена! Начинаем работу.")
                return True
                
            if elapsed > self.position_timeout:
                rospy.logwarn("Истек таймаут ожидания позиции (%.1f сек). Продолжаем без позиции.", 
                             self.position_timeout)
                return False
                
            rospy.loginfo_throttle(5.0, "Ожидание определения позиции... (%.1f сек из %.1f)", 
                                  elapsed, self.position_timeout)
            rate.sleep()
        
        return False
    
    def state_cb(self, msg):
        """Обработка состояния дрона от MAVROS"""
        self.drone_state = msg
        self.armed = msg.armed
        self.guided_mode = (msg.mode == "GUIDED" or msg.mode == "OFFBOARD")
    
    def pose_cb(self, msg):
        """Обработка текущей позиции дрона"""
        self.current_position = msg
        
        # Проверяем достижение высоты взлета
        if not self.takeoff_complete and self.current_position and self.takeoff_start_time:
            current_alt = self.current_position.pose.position.z
            elapsed = rospy.Time.now() - self.takeoff_start_time
            
            # Если достигли минимальной высоты или истек таймаут
            if current_alt >= self.takeoff_min_alt or elapsed > self.takeoff_timeout:
                self.takeoff_complete = True
                rospy.loginfo("Взлет завершен! Высота: %.2f м", current_alt)
                
        # Проверяем, находится ли дрон в пределах карты стоимости
        if self.global_costmap_info and self.current_position:
            self.check_position_in_costmap()
    
    def check_position_in_costmap(self):
        """Проверяет, находится ли текущая позиция дрона в пределах глобальной карты стоимости"""
        if not self.global_costmap_info or not self.current_position:
            return
            
        # Получаем границы карты
        min_x = self.global_costmap_info.origin.position.x
        min_y = self.global_costmap_info.origin.position.y
        max_x = min_x + (self.global_costmap_info.width * self.global_costmap_info.resolution)
        max_y = min_y + (self.global_costmap_info.height * self.global_costmap_info.resolution)
        
        # Текущая позиция дрона
        drone_x = self.current_position.pose.position.x
        drone_y = self.current_position.pose.position.y
        
        # Добавляем отступ от края карты для безопасности
        margin = 2.0  # метры
        safe_min_x = min_x + margin
        safe_min_y = min_y + margin
        safe_max_x = max_x - margin
        safe_max_y = max_y - margin
        
        # Проверяем, находится ли дрон в безопасной зоне карты
        if drone_x < safe_min_x or drone_x > safe_max_x or drone_y < safe_min_y or drone_y > safe_max_y:
            rospy.logwarn_throttle(5.0, "ВНИМАНИЕ: Дрон находится слишком близко к краю карты или за её пределами!")
            rospy.logwarn_throttle(5.0, "Позиция дрона: (%.2f, %.2f), границы карты: (%.2f, %.2f) - (%.2f, %.2f)",
                                 drone_x, drone_y, min_x, min_y, max_x, max_y)
            
            # Если дрон полностью за пределами карты, возвращаем его в центр
            if drone_x < min_x or drone_x > max_x or drone_y < min_y or drone_y > max_y:
                rospy.logerr_throttle(5.0, "КРИТИЧЕСКАЯ ОШИБКА: Дрон за пределами карты! Возвращаем в центр.")
                
                # Вычисляем центр карты
                center_x = min_x + (self.global_costmap_info.width * self.global_costmap_info.resolution) / 2.0
                center_y = min_y + (self.global_costmap_info.height * self.global_costmap_info.resolution) / 2.0
                
                # Создаем новый путь к центру карты
                self.create_path_to_center(center_x, center_y)
    
    def create_path_to_center(self, center_x, center_y):
        """Создает простой путь от текущей позиции дрона к центру карты"""
        if not self.current_position:
            return
            
        # Создаем новый путь
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = rospy.Time.now()
        
        # Добавляем текущую позицию как начальную точку
        start_pose = PoseStamped()
        start_pose.header = path.header
        start_pose.pose.position.x = self.current_position.pose.position.x
        start_pose.pose.position.y = self.current_position.pose.position.y
        start_pose.pose.position.z = self.current_position.pose.position.z
        start_pose.pose.orientation = self.current_position.pose.orientation
        path.poses.append(start_pose)
        
        # Добавляем целевую точку в центре карты
        target_pose = PoseStamped()
        target_pose.header = path.header
        target_pose.pose.position.x = center_x
        target_pose.pose.position.y = center_y
        target_pose.pose.position.z = self.current_position.pose.position.z
        target_pose.pose.orientation = self.current_position.pose.orientation
        path.poses.append(target_pose)
        
        # Устанавливаем новый путь
        self.current_path = path
        self.current_wp_index = 0
        self.path_complete = False
        
        rospy.loginfo("Создан новый путь к центру карты: (%.2f, %.2f)", center_x, center_y)
    
    def path_cb(self, msg: Path):
        """Обработка нового пути"""
        if not msg.poses:
            rospy.logwarn("Получен пустой план пути")
            return
            
        # Если получен новый путь - начинаем следовать с первой точки
        self.current_path = msg
        self.current_wp_index = 0
        self.path_complete = False
        
        rospy.loginfo("Получен новый план пути с %d точками", len(msg.poses))
    
    def set_guided_mode(self):
        """Переводит дрон в режим GUIDED и армит его"""
        if not self.armed:
            try:
                self.mode_srv(0, 'GUIDED')
                rospy.sleep(0.5)  # Ждем применения режима
                self.arm_srv(True)
                rospy.loginfo("Дрон переведен в режим GUIDED и армирован")
            except rospy.ServiceException as e:
                rospy.logerr("Ошибка при армировании: %s", str(e))
    
    def takeoff(self):
        """Запускает процедуру взлета"""
        # Сбрасываем флаг завершения взлета
        self.takeoff_complete = False
        # Записываем время начала взлета
        self.takeoff_start_time = rospy.Time.now()
        
        rospy.loginfo("Начинаем взлет на высоту %.2f м", self.takeoff_height)
        
        # Отправляем команду на взлет в цикле (первая команда может быть проигнорирована)
        rate = rospy.Rate(5)  # 5 Гц
        for _ in range(10):  # Отправляем команду 10 раз
            takeoff_setpoint = PoseStamped()
            takeoff_setpoint.header.stamp = rospy.Time.now()
            takeoff_setpoint.header.frame_id = 'map'
            
            # Используем текущую позицию по X и Y, если она известна
            if self.current_position:
                takeoff_setpoint.pose.position.x = self.current_position.pose.position.x
                takeoff_setpoint.pose.position.y = self.current_position.pose.position.y
            else:
                takeoff_setpoint.pose.position.x = 0
                takeoff_setpoint.pose.position.y = 0
            
            takeoff_setpoint.pose.position.z = self.takeoff_height
            takeoff_setpoint.pose.orientation.w = 1.0
            
            self.setpoint_pub.publish(takeoff_setpoint)
            rate.sleep()
    
    def timer_cb(self, event):
        """Основной цикл следования по пути"""
        # Проверяем, что мы имеем все необходимые данные
        if not self.drone_state:
            return
            
        # Если не армированы или не в GUIDED, пытаемся исправить
        if not self.armed or not self.guided_mode:
            self.set_guided_mode()
            return
            
        # Если взлет еще не завершен, продолжаем взлет
        if not self.takeoff_complete:
            self.continue_takeoff()
            return
            
        # Если нет текущего пути или путь завершен, просто висим на месте
        if not self.current_path or self.path_complete:
            self.publish_hover_setpoint()
            return
            
        # Получаем текущую целевую точку
        target = self.current_path.poses[self.current_wp_index]
        
        # Публикуем маркер целевой точки
        self.publish_target_marker(target)
        
        # Проверяем, достигли ли мы текущей точки (если у нас есть позиция)
        if self.current_position:
            dist_to_wp = self.distance_to_waypoint(target)
            
            if dist_to_wp < self.wp_radius:
                # Переходим к следующей точке
                self.current_wp_index += 1
                rospy.loginfo("Достигнута точка %d, расстояние: %.2f м", 
                            self.current_wp_index, dist_to_wp)
                
                # Проверяем, не закончился ли путь
                if self.current_wp_index >= len(self.current_path.poses):
                    rospy.loginfo("Путь завершен! Поддерживаем последнюю позицию.")
                    self.path_complete = True
                    return
        
        # Публикуем текущую целевую точку
        self.publish_setpoint(target)
    
    def publish_target_marker(self, target):
        """Публикует маркер текущей целевой точки"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "path_target"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Позиция маркера - текущая целевая точка
        marker.pose.position.x = target.pose.position.x
        marker.pose.position.y = target.pose.position.y
        marker.pose.position.z = target.pose.position.z
        
        marker.pose.orientation.w = 1.0
        
        # Размер маркера
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        
        # Цвет маркера (синий)
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.7
        
        # Время жизни маркера
        marker.lifetime = rospy.Duration(0.5)
        
        # Публикуем маркер
        self.target_marker_pub.publish(marker)
    
    def continue_takeoff(self):
        """Продолжает отправлять команды взлета"""
        takeoff_setpoint = PoseStamped()
        takeoff_setpoint.header.stamp = rospy.Time.now()
        takeoff_setpoint.header.frame_id = 'map'
        
        # Используем текущую позицию по X и Y
        if self.current_position:
            takeoff_setpoint.pose.position.x = self.current_position.pose.position.x
            takeoff_setpoint.pose.position.y = self.current_position.pose.position.y
        else:
            takeoff_setpoint.pose.position.x = 0
            takeoff_setpoint.pose.position.y = 0
        
        takeoff_setpoint.pose.position.z = self.takeoff_height
        takeoff_setpoint.pose.orientation.w = 1.0
        
        self.setpoint_pub.publish(takeoff_setpoint)
        
        # Проверяем истечение таймаута взлета
        if self.takeoff_start_time and (rospy.Time.now() - self.takeoff_start_time) > self.takeoff_timeout:
            rospy.logwarn("Истек таймаут взлета! Принудительно считаем взлет завершенным.")
            self.takeoff_complete = True
    
    def distance_to_waypoint(self, waypoint):
        """Вычисляет расстояние до заданной точки"""
        if not self.current_position:
            return float('inf')  # Бесконечность, если нет текущей позиции
            
        dx = waypoint.pose.position.x - self.current_position.pose.position.x
        dy = waypoint.pose.position.y - self.current_position.pose.position.y
        dz = waypoint.pose.position.z - self.current_position.pose.position.z
        
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    def publish_setpoint(self, target):
        """Публикует целевую точку для MAVROS"""
        setpoint = PoseStamped()
        setpoint.header.stamp = rospy.Time.now()
        setpoint.header.frame_id = 'map'
        
        # Копируем позицию из целевой точки
        setpoint.pose.position.x = target.pose.position.x
        setpoint.pose.position.y = target.pose.position.y
        
        # Используем заданную высоту полета
        if target.pose.position.z > 0:
            setpoint.pose.position.z = target.pose.position.z
        else:
            setpoint.pose.position.z = self.altitude
        
        # Копируем ориентацию из целевой точки
        setpoint.pose.orientation = target.pose.orientation
        
        # Публикуем setpoint
        self.setpoint_pub.publish(setpoint)
    
    def publish_hover_setpoint(self):
        """Публикует команду зависания на месте"""
        hover_setpoint = PoseStamped()
        hover_setpoint.header.stamp = rospy.Time.now()
        hover_setpoint.header.frame_id = 'map'
        
        # Используем текущую позицию, если она известна
        if self.current_position:
            hover_setpoint.pose.position.x = self.current_position.pose.position.x
            hover_setpoint.pose.position.y = self.current_position.pose.position.y
            hover_setpoint.pose.position.z = self.current_position.pose.position.z
            hover_setpoint.pose.orientation = self.current_position.pose.orientation
        else:
            # Иначе используем центр карты
            hover_setpoint.pose.position.x = 0
            hover_setpoint.pose.position.y = 0
            hover_setpoint.pose.position.z = self.altitude
            hover_setpoint.pose.orientation.w = 1.0
        
        # Публикуем setpoint
        self.setpoint_pub.publish(hover_setpoint)

if __name__ == '__main__':
    try:
        follower = PathFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
