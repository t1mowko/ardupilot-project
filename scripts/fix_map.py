#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler

class MapFixer:
    """
    Скрипт для проверки и исправления карты
    """
    
    def __init__(self):
        rospy.init_node('map_fixer')
        
        # Параметры
        self.use_fixed_origin = rospy.get_param('~use_fixed_origin', True)
        self.origin_x = rospy.get_param('~origin_x', -100.0)  # Начало координат по X
        self.origin_y = rospy.get_param('~origin_y', -100.0)  # Начало координат по Y
        self.map_size = rospy.get_param('~map_size', 200.0)  # Размер карты в метрах
        
        # Подписываемся на карту
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        
        # Публикуем исправленную карту
        self.map_pub = rospy.Publisher('/fixed_map', OccupancyGrid, queue_size=1, latch=True)
        
        # Публикатор для начальной позиции
        self.initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1, latch=True)
        
        # Флаг, указывающий, была ли карта исправлена
        self.map_fixed = False
        
        # Запрашиваем карту напрямую через сервис
        rospy.loginfo("Ожидание сервиса map_server/static_map...")
        try:
            rospy.wait_for_service('static_map', timeout=10.0)
            get_map = rospy.ServiceProxy('static_map', GetMap)
            response = get_map()
            self.map_callback(response.map)
        except rospy.ROSException as e:
            rospy.logwarn("Не удалось получить карту через сервис: %s", str(e))
            rospy.loginfo("Ожидание карты через топик...")
        
        rospy.loginfo("MapFixer инициализирован")
        
    def map_callback(self, map_msg):
        """Обработка полученной карты"""
        if self.map_fixed:
            return
        
        rospy.loginfo("Получена карта размером %dx%d, разрешение %.3f м/пиксель",
                     map_msg.info.width, map_msg.info.height, map_msg.info.resolution)
        rospy.loginfo("Начало координат карты: (%.2f, %.2f, %.2f)",
                     map_msg.info.origin.position.x,
                     map_msg.info.origin.position.y,
                     map_msg.info.origin.position.z)
        
        # Преобразуем данные карты в массив numpy
        data = np.array(map_msg.data, dtype=np.int8).reshape(map_msg.info.height, map_msg.info.width)
        
        # Анализируем карту
        unknown_cells = np.sum(data == -1)
        occupied_cells = np.sum(data > 0)
        free_cells = np.sum(data == 0)
        total_cells = map_msg.info.width * map_msg.info.height
        
        rospy.loginfo("Статистика карты:")
        rospy.loginfo("  - Неизвестные клетки: %d (%.1f%%)", unknown_cells, 100.0 * unknown_cells / total_cells)
        rospy.loginfo("  - Занятые клетки: %d (%.1f%%)", occupied_cells, 100.0 * occupied_cells / total_cells)
        rospy.loginfo("  - Свободные клетки: %d (%.1f%%)", free_cells, 100.0 * free_cells / total_cells)
        
        # Создаем новую карту с оптимальным размером
        resolution = map_msg.info.resolution
        # Вычисляем необходимое количество пикселей для карты размером 200x200 метров
        required_pixels = int(self.map_size / resolution)
        
        # Устанавливаем фиксированный размер карты 4000x4000 пикселей (200м при разрешении 0.05)
        new_width = required_pixels
        new_height = required_pixels
        
        rospy.loginfo("Создаем новую карту размером %dx%d пикселей", new_width, new_height)
        
        # Вычисляем смещение для центрирования старой карты
        x_offset = (new_width - map_msg.info.width) // 2
        y_offset = (new_height - map_msg.info.height) // 2
        
        # Создаем новый массив данных
        new_data = np.zeros((new_height, new_width), dtype=np.int8)
        
        # Устанавливаем все ячейки как свободные (0)
        new_data.fill(0)
        
        # Копируем старую карту в новую
        if map_msg.info.width > 0 and map_msg.info.height > 0:
            y_start = max(0, y_offset)
            y_end = min(new_height, y_offset + map_msg.info.height)
            x_start = max(0, x_offset)
            x_end = min(new_width, x_offset + map_msg.info.width)
            
            src_y_start = max(0, -y_offset)
            src_y_end = src_y_start + (y_end - y_start)
            src_x_start = max(0, -x_offset)
            src_x_end = src_x_start + (x_end - x_start)
            
            if x_end > x_start and y_end > y_start:
                new_data[y_start:y_end, x_start:x_end] = data[src_y_start:src_y_end, src_x_start:src_x_end]
        
        # Создаем сообщение с новой картой
        new_map = OccupancyGrid()
        new_map.header = map_msg.header
        new_map.info = map_msg.info
        new_map.info.width = new_width
        new_map.info.height = new_height
        
        # Устанавливаем новое начало координат
        if self.use_fixed_origin:
            new_map.info.origin.position.x = self.origin_x
            new_map.info.origin.position.y = self.origin_y
        else:
            # Обновляем начало координат с учетом смещения
            new_map.info.origin.position.x = map_msg.info.origin.position.x - x_offset * resolution
            new_map.info.origin.position.y = map_msg.info.origin.position.y - y_offset * resolution
        
        # Преобразуем массив обратно в список
        new_map.data = new_data.flatten().tolist()
        
        # Публикуем новую карту
        self.map_pub.publish(new_map)
        
        rospy.loginfo("Опубликована исправленная карта размером %dx%d", new_width, new_height)
        rospy.loginfo("Новое начало координат: (%.2f, %.2f, %.2f)",
                     new_map.info.origin.position.x,
                     new_map.info.origin.position.y,
                     new_map.info.origin.position.z)
        
        # Публикуем начальную позу в центре карты
        self.publish_initial_pose()
        
        self.map_fixed = True
    
    def publish_initial_pose(self):
        """Публикация начальной позы дрона в центре карты"""
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        
        # Центр карты - точка (0,0)
        pose.pose.pose.position.x = 0.0
        pose.pose.pose.position.y = 0.0
        pose.pose.pose.position.z = 0.0
        
        # Ориентация - смотрим вперед (по оси X)
        q = quaternion_from_euler(0, 0, 0)
        pose.pose.pose.orientation.x = q[0]
        pose.pose.pose.orientation.y = q[1]
        pose.pose.pose.orientation.z = q[2]
        pose.pose.pose.orientation.w = q[3]
        
        # Ковариация - уверенность в позиции
        pose.pose.covariance = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.01]
        
        # Публикуем начальную позу
        self.initial_pose_pub.publish(pose)
        rospy.loginfo("Установлена начальная поза в центре карты: x=0.0, y=0.0")

if __name__ == '__main__':
    try:
        map_fixer = MapFixer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 