#!/usr/bin/env python3
import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, TransformStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

class CoordinateSync:
    """
    Класс для синхронизации координат между различными системами координат
    и улучшения навигации дрона.
    """
    def __init__(self):
        rospy.init_node('coordinate_sync')
        
        # Параметры
        self.use_mavros = rospy.get_param('~use_mavros', True)
        self.use_odom = rospy.get_param('~use_odom', True)
        self.publish_tf = rospy.get_param('~publish_tf', True)
        self.sync_rate = rospy.get_param('~sync_rate', 30.0)  # Увеличена частота до 30 Гц
        self.force_map_origin = rospy.get_param('~force_map_origin', False)
        
        # Подписчики
        if self.use_mavros:
            rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.mavros_pose_callback)
        if self.use_odom:
            rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # Издатели
        self.initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        
        # TF
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Переменные
        self.last_mavros_pose = None
        self.last_odom = None
        
        # Таймер для синхронизации
        self.timer = rospy.Timer(rospy.Duration(1.0/self.sync_rate), self.sync_callback)
        
        rospy.loginfo("Координатный синхронизатор инициализирован")
    
    def mavros_pose_callback(self, msg):
        """Обработчик позиции MAVROS"""
        self.last_mavros_pose = msg
    
    def odom_callback(self, msg):
        """Обработчик одометрии"""
        self.last_odom = msg
    
    def sync_callback(self, event):
        """Периодическая синхронизация координат"""
        if self.publish_tf:
            self.publish_transforms()
        
        # Публикуем начальную позу для локализации
        self.publish_initial_pose()
    
    def publish_transforms(self):
        """Публикация трансформаций TF"""
        try:
            # Если используем MAVROS, публикуем трансформацию map -> base_link
            if self.use_mavros and self.last_mavros_pose:
                transform = TransformStamped()
                transform.header.stamp = rospy.Time.now()
                transform.header.frame_id = "map"
                transform.child_frame_id = "base_link"
                
                # Копируем позицию из MAVROS
                transform.transform.translation.x = self.last_mavros_pose.pose.position.x
                transform.transform.translation.y = self.last_mavros_pose.pose.position.y
                transform.transform.translation.z = self.last_mavros_pose.pose.position.z
                
                # Копируем ориентацию из MAVROS
                transform.transform.rotation = self.last_mavros_pose.pose.orientation
                
                # Публикуем трансформацию
                self.tf_broadcaster.sendTransform(transform)
                
            # Если используем одометрию, публикуем трансформацию odom -> base_link
            elif self.use_odom and self.last_odom:
                transform = TransformStamped()
                transform.header.stamp = rospy.Time.now()
                transform.header.frame_id = "odom"
                transform.child_frame_id = "base_link"
                
                # Копируем позицию из одометрии
                transform.transform.translation.x = self.last_odom.pose.pose.position.x
                transform.transform.translation.y = self.last_odom.pose.pose.position.y
                transform.transform.translation.z = self.last_odom.pose.pose.position.z
                
                # Копируем ориентацию из одометрии
                transform.transform.rotation = self.last_odom.pose.pose.orientation
                
                # Публикуем трансформацию
                self.tf_broadcaster.sendTransform(transform)
                
        except Exception as e:
            rospy.logwarn("Ошибка при публикации трансформаций: %s", str(e))
    
    def publish_initial_pose(self):
        """Публикация начальной позы для локализации"""
        try:
            # Получаем текущую позицию робота из TF
            transform = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time(0))
            
            # Создаем сообщение начальной позы
            initial_pose = PoseWithCovarianceStamped()
            initial_pose.header.stamp = rospy.Time.now()
            initial_pose.header.frame_id = "map"
            
            # Заполняем позицию
            initial_pose.pose.pose.position.x = transform.transform.translation.x
            initial_pose.pose.pose.position.y = transform.transform.translation.y
            initial_pose.pose.pose.position.z = transform.transform.translation.z
            
            # Заполняем ориентацию
            initial_pose.pose.pose.orientation = transform.transform.rotation
            
            # Заполняем ковариацию (диагональная матрица с небольшими значениями)
            initial_pose.pose.covariance = [0.01, 0, 0, 0, 0, 0,
                                          0, 0.01, 0, 0, 0, 0,
                                          0, 0, 0.01, 0, 0, 0,
                                          0, 0, 0, 0.01, 0, 0,
                                          0, 0, 0, 0, 0.01, 0,
                                          0, 0, 0, 0, 0, 0.01]
            
            # Публикуем начальную позу
            self.initial_pose_pub.publish(initial_pose)
            
            # Выводим информацию о текущей позиции
            roll, pitch, yaw = euler_from_quaternion([
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ])
            
            rospy.loginfo_throttle(1.0, "Текущая позиция робота в TF: x=%.2f, y=%.2f",
                                 transform.transform.translation.x, transform.transform.translation.y)
            rospy.loginfo_throttle(1.0, "Публикуем начальную позу: x=%.2f, y=%.2f, theta=%.2f",
                                 initial_pose.pose.pose.position.x, initial_pose.pose.pose.position.y,
                                 yaw)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            if self.force_map_origin:
                # Если не удалось получить трансформацию, используем начало координат
                initial_pose = PoseWithCovarianceStamped()
                initial_pose.header.stamp = rospy.Time.now()
                initial_pose.header.frame_id = "map"
                
                # Позиция в начале координат
                initial_pose.pose.pose.position.x = 0.0
                initial_pose.pose.pose.position.y = 0.0
                initial_pose.pose.pose.position.z = 0.0
                
                # Ориентация - нулевой угол
                q = quaternion_from_euler(0, 0, 0)
                initial_pose.pose.pose.orientation.x = q[0]
                initial_pose.pose.pose.orientation.y = q[1]
                initial_pose.pose.pose.orientation.z = q[2]
                initial_pose.pose.pose.orientation.w = q[3]
                
                # Заполняем ковариацию
                initial_pose.pose.covariance = [0.01, 0, 0, 0, 0, 0,
                                              0, 0.01, 0, 0, 0, 0,
                                              0, 0, 0.01, 0, 0, 0,
                                              0, 0, 0, 0.01, 0, 0,
                                              0, 0, 0, 0, 0.01, 0,
                                              0, 0, 0, 0, 0, 0.01]
                
                # Публикуем начальную позу
                self.initial_pose_pub.publish(initial_pose)
                
                rospy.loginfo_throttle(5.0, "Используем начало координат для начальной позы")
            else:
                rospy.logwarn_throttle(5.0, "Не удалось получить трансформацию map -> base_link: %s", str(e))
    
    def run(self):
        """Запуск узла"""
        rospy.spin()

if __name__ == "__main__":
    try:
        sync = CoordinateSync()
        sync.run()
    except rospy.ROSInterruptException:
        pass 