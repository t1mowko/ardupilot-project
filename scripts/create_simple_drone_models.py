#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import shutil
import re

def create_drone_model(source_model_path, drone_id, base_port=9002):
    """
    Создает модель дрона с уникальным ID и портами
    
    Args:
        source_model_path: Путь к исходной модели дрона
        drone_id: Идентификатор дрона (1-5)
        base_port: Базовый порт для FDM и IMU
    """
    # Создаем имя для новой модели
    model_name = f"iris_drone{drone_id}"
    
    # Определяем пути
    models_dir = os.path.dirname(source_model_path)
    target_model_path = os.path.join(models_dir, model_name)
    
    # Удаляем модель, если она уже существует
    if os.path.exists(target_model_path):
        print(f"Удаление существующей модели {model_name}...")
        shutil.rmtree(target_model_path)
    
    # Создаем директорию для новой модели
    os.makedirs(target_model_path, exist_ok=True)
    
    # Копируем все файлы из исходной модели
    for item in os.listdir(source_model_path):
        source_item = os.path.join(source_model_path, item)
        target_item = os.path.join(target_model_path, item)
        
        if os.path.isdir(source_item):
            shutil.copytree(source_item, target_item)
        else:
            shutil.copy2(source_item, target_item)
    
    # Модифицируем model.sdf - меняем порты FDM и IMU
    sdf_path = os.path.join(target_model_path, "model.sdf")
    if os.path.exists(sdf_path):
        # Вычисляем порты для этого дрона
        fdm_port = base_port + (drone_id - 1) * 10
        imu_port = fdm_port + 1
        
        modify_sdf_ports(sdf_path, fdm_port, imu_port, drone_id)
    
    # Модифицируем model.config - меняем имя модели
    config_path = os.path.join(target_model_path, "model.config")
    if os.path.exists(config_path):
        modify_config_file(config_path, drone_id)
    
    print(f"Создана модель дрона {drone_id} с портами FDM={fdm_port}, IMU={imu_port}")
    return target_model_path

def modify_sdf_ports(sdf_path, fdm_port, imu_port, drone_id):
    """
    Модифицирует порты в SDF файле
    
    Args:
        sdf_path: Путь к SDF файлу
        fdm_port: Порт для FDM
        imu_port: Порт для IMU
        drone_id: ID дрона
    """
    try:
        with open(sdf_path, 'r') as file:
            content = file.read()
        
        # Меняем имя модели
        content = re.sub(r'<model name="[^"]*"', f'<model name="iris_drone{drone_id}"', content)
        
        # Меняем порт FDM
        content = re.sub(r'<fdm_port>[0-9]+</fdm_port>', f'<fdm_port>{fdm_port}</fdm_port>', content)
        
        # Меняем порт IMU
        content = re.sub(r'<imu_port>[0-9]+</imu_port>', f'<imu_port>{imu_port}</imu_port>', content)
        
        with open(sdf_path, 'w') as file:
            file.write(content)
        
        print(f"Порты в SDF файле изменены: FDM={fdm_port}, IMU={imu_port}")
        
    except Exception as e:
        print(f"Ошибка при модификации SDF файла: {e}")

def modify_config_file(config_path, drone_id):
    """
    Модифицирует файл конфигурации модели
    
    Args:
        config_path: Путь к файлу конфигурации
        drone_id: Идентификатор дрона
    """
    try:
        with open(config_path, 'r') as file:
            content = file.read()
        
        # Заменяем теги <n> на <name> если они есть
        content = content.replace('<n>', '<name>')
        content = content.replace('</n>', '</name>')
        
        # Меняем имя модели
        content = re.sub(r'<name>[^<]*</name>', f'<name>Iris Drone {drone_id}</name>', content)
        
        with open(config_path, 'w') as file:
            file.write(content)
        
        print(f"Файл конфигурации модифицирован для дрона {drone_id}")
        
    except Exception as e:
        print(f"Ошибка при модификации файла конфигурации: {e}")

def main():
    if len(sys.argv) < 2:
        print("Использование: python create_simple_drone_models.py <путь_к_модели_iris_with_ardupilot>")
        print("Например: python create_simple_drone_models.py /home/timowko/ardupilot_gazebo/models/iris_with_ardupilot")
        return
    
    source_model_path = sys.argv[1]
    if not os.path.exists(source_model_path):
        print(f"Ошибка: Путь {source_model_path} не существует")
        return
    
    # Создаем модели для 5 дронов с разными портами
    base_port = 9002  # Стандартный порт FDM для первого дрона
    models = []
    for drone_id in range(1, 6):
        model_path = create_drone_model(source_model_path, drone_id, base_port)
        models.append(model_path)
    
    print("\nСозданы следующие модели дронов:")
    for i, model_path in enumerate(models):
        fdm_port = base_port + i * 10
        imu_port = fdm_port + 1
        print(f"Дрон {i+1}: {os.path.basename(model_path)} (FDM: {fdm_port}, IMU: {imu_port})")
    
    print("\nИнструкция по использованию:")
    print("1. Запустите Gazebo")
    print("2. Добавьте модели дронов в симуляцию")
    print("3. Для каждого дрона запустите SITL с соответствующими параметрами:")
    
    for i in range(5):
        drone_id = i + 1
        fdm_port = base_port + i * 10
        mavlink_port = 14550 + i
        print(f"   Дрон {drone_id}: sim_vehicle.py -v ArduCopter -f gazebo-iris --sysid {drone_id} --instance {i} --out=udp:127.0.0.1:{mavlink_port} --console")

if __name__ == "__main__":
    main() 