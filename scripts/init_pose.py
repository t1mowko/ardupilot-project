#!/usr/bin/env python3
import rospy
import time
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import Header

class DroneInitializer:
    """
    Скрипт для инициализации дрона и установки его начальной позиции
    """
    
    def __init__(self):
        rospy.init_node('drone_initializer')
        
        # Параметры
        self.arm_timeout = rospy.get_param('~arm_timeout', 60.0)  # Таймаут на армирование в секундах
        self.takeoff_altitude = rospy.get_param('~takeoff_altitude', 2.0)  # Высота взлета в метрах
        self.map_center_x = rospy.get_param('~map_center_x', 50.0)  # Центр карты X
        self.map_center_y = rospy.get_param('~map_center_y', 50.0)  # Центр карты Y
        
        # Публикаторы
        self.initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1, latch=True)
        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        
        # Подписчики
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_callback)
        
        # Сервисы
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        
        # Состояние
        self.current_state = State()
        self.current_state.connected = False
        self.current_state.armed = False
        self.current_state.mode = ""
        
        rospy.loginfo("Инициализатор дрона запущен")
        
    def state_callback(self, msg):
        """Обработчик состояния MAVROS"""
        self.current_state = msg
        
    def set_initial_pose(self):
        """Установка начальной позы дрона на карте"""
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        
        # Устанавливаем позицию в центр карты
        pose.pose.pose.position.x = self.map_center_x
        pose.pose.pose.position.y = self.map_center_y
        pose.pose.pose.position.z = 0.0
        
        # Ориентация - смотрим вперед (по оси X)
        pose.pose.pose.orientation.x = 0.0
        pose.pose.pose.orientation.y = 0.0
        pose.pose.pose.orientation.z = 0.0
        pose.pose.pose.orientation.w = 1.0
        
        # Ковариация - уверенность в позиции
        pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
        
        # Публикуем начальную позу
        self.initial_pose_pub.publish(pose)
        rospy.loginfo("Установлена начальная поза: x=%.1f, y=%.1f", self.map_center_x, self.map_center_y)
        
    def wait_for_connection(self, timeout=30):
        """Ожидание подключения к FCU"""
        start_time = time.time()
        rate = rospy.Rate(10)  # 10 Гц
        
        rospy.loginfo("Ожидание подключения к FCU...")
        while not rospy.is_shutdown() and not self.current_state.connected:
            if time.time() - start_time > timeout:
                rospy.logwarn("Таймаут подключения к FCU!")
                return False
            rate.sleep()
            
        rospy.loginfo("Подключение к FCU установлено")
        return True
    
    def arm_and_takeoff(self):
        """Армирование и взлет дрона"""
        if not self.wait_for_connection():
            return False
            
        # Отправляем несколько setpoint сообщений перед переключением в OFFBOARD
        setpoint = PoseStamped()
        setpoint.header.frame_id = "map"
        setpoint.pose.position.x = self.map_center_x
        setpoint.pose.position.y = self.map_center_y
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
        if not self.set_mode_client(custom_mode="GUIDED").success:
            rospy.logwarn("Не удалось переключиться в режим GUIDED!")
            return False
            
        # Армируем дрон
        rospy.loginfo("Армирование дрона...")
        start_time = time.time()
        while not self.current_state.armed and not rospy.is_shutdown():
            if time.time() - start_time > self.arm_timeout:
                rospy.logwarn("Таймаут армирования!")
                return False
                
            if not self.arming_client(True).success:
                rospy.logwarn("Армирование не удалось, повторная попытка...")
            
            rate.sleep()
            
        rospy.loginfo("Дрон армирован")
        
        # Взлет
        rospy.loginfo("Взлет на высоту %.1f м...", self.takeoff_altitude)
        start_time = time.time()
        while not rospy.is_shutdown() and time.time() - start_time < 30.0:
            setpoint.header.stamp = rospy.Time.now()
            self.setpoint_pub.publish(setpoint)
            rate.sleep()
            
        rospy.loginfo("Взлет завершен")
        return True
        
    def run(self):
        """Основная функция работы"""
        # Устанавливаем начальную позу
        self.set_initial_pose()
        
        # Ждем немного, чтобы позиция обработалась
        rospy.sleep(2.0)
        
        # Армируем и взлетаем
        if self.arm_and_takeoff():
            rospy.loginfo("Инициализация дрона успешно завершена")
        else:
            rospy.logerr("Не удалось инициализировать дрон!")

if __name__ == '__main__':
    try:
        initializer = DroneInitializer()
        initializer.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 