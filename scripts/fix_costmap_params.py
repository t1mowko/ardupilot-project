#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import dynamic_reconfigure.client
import time
import std_srvs.srv

def main():
    """Корректирует параметры карты стоимости во время выполнения"""
    rospy.init_node('fix_costmap_params', anonymous=True)
    
    # Ждем, чтобы система успела запуститься
    rospy.loginfo("Ожидание запуска move_base...")
    time.sleep(10.0)
    
    try:
        # Создаем клиенты для динамической реконфигурации
        global_client = dynamic_reconfigure.client.Client('/move_base/global_costmap')
        local_client = dynamic_reconfigure.client.Client('/move_base/local_costmap')
        
        # Корректируем глобальную карту стоимости
        global_params = {
            'width': 100.0,
            'height': 100.0,
            'origin_x': 0.0,
            'origin_y': 0.0,
            'transform_tolerance': 1.0,
            'update_frequency': 5.0,
            'publish_frequency': 2.0
        }
        
        # Корректируем локальную карту стоимости
        local_params = {
            'width': 8.0,
            'height': 8.0,
            'origin_x': -4.0,  # Центрируем относительно робота
            'origin_y': -4.0,  # Центрируем относительно робота
            'transform_tolerance': 0.5,
            'update_frequency': 10.0,
            'publish_frequency': 5.0
        }
        
        # Применяем параметры
        rospy.loginfo("Применяем параметры глобальной карты стоимости...")
        global_client.update_configuration(global_params)
        
        rospy.loginfo("Применяем параметры локальной карты стоимости...")
        local_client.update_configuration(local_params)
        
        rospy.loginfo("Параметры карты стоимости успешно обновлены!")
        
        # Очищаем карты стоимости
        rospy.loginfo("Очищаем карты стоимости...")
        clear_client = rospy.ServiceProxy('/move_base/clear_costmaps', std_srvs.srv.Empty)
        clear_client()
        
        rospy.loginfo("Карты стоимости очищены!")
        
    except Exception as e:
        rospy.logerr("Ошибка при обновлении параметров карты стоимости: %s", str(e))
    
    return 0

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass 