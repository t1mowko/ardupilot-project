#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import shutil
import re

def create_unique_drone_model(source_model_path, drone_id):
    """
    Создает уникальную модель дрона с указанным ID на основе базовой модели
    
    Args:
        source_model_path: Путь к исходной модели дрона
        drone_id: Идентификатор дрона (1-5)
    
    Returns:
        Путь к созданной модели
    """
    # Создаем имя для новой модели
    model_name = f"iris_with_ardupilot_drone{drone_id}"
    
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
    
    # Модифицируем model.sdf
    sdf_path = os.path.join(target_model_path, "model.sdf")
    if os.path.exists(sdf_path):
        modify_sdf_file(sdf_path, drone_id)
    
    # Модифицируем model.config
    config_path = os.path.join(target_model_path, "model.config")
    if os.path.exists(config_path):
        modify_config_file(config_path, drone_id)
    
    print(f"Создана уникальная модель дрона: {model_name}")
    return target_model_path

def modify_sdf_file(sdf_path, drone_id):
    """
    Модифицирует SDF файл, чтобы сделать модель уникальной
    
    Args:
        sdf_path: Путь к SDF файлу
        drone_id: Идентификатор дрона
    """
    try:
        # Читаем содержимое файла
        with open(sdf_path, 'r') as file:
            content = file.read()
        
        # Исправляем XML-декларацию
        content = '<?xml version="1.0"?>\n<sdf version="1.6">'
        
        # Меняем имя модели
        content = re.sub(r'<model name="[^"]*">', f'<model name="iris_with_ardupilot_drone{drone_id}">', content)
        
        # Добавляем суффикс с ID дрона ко всем именам линков и джойнтов
        content = re.sub(r'<link name="([^"]*)"', f'<link name="\\1_drone{drone_id}"', content)
        content = re.sub(r'<joint name="([^"]*)"', f'<joint name="\\1_drone{drone_id}"', content)
        
        # Обновляем ссылки на родительский и дочерний линки
        content = re.sub(r'<parent>([^<]*)</parent>', f'<parent>\\1_drone{drone_id}</parent>', content)
        content = re.sub(r'<child>([^<]*)</child>', f'<child>\\1_drone{drone_id}</child>', content)
        
        # Сохраняем изменения
        with open(sdf_path, 'w') as file:
            file.write(content)
        
        print(f"SDF файл модифицирован для дрона {drone_id}")
        
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
        # Читаем содержимое файла
        with open(config_path, 'r') as file:
            content = file.read()
        
        # Исправляем XML-декларацию и сохраняем остальное содержимое
        if '<?xml' in content:
            content = '<?xml version="1.0"?>\n' + content.split('?>', 1)[1]
        
        # Заменяем неправильные теги <n> на <name>
        content = content.replace('<n>', '<name>')
        content = content.replace('</n>', '</name>')
        
        # Изменяем имя модели
        content = re.sub(r'<name>[^<]*</name>', f'<name>Iris with ArduPilot (Дрон {drone_id})</name>', content)
        
        # Сохраняем изменения
        with open(config_path, 'w') as file:
            file.write(content)
        
        print(f"Файл конфигурации модифицирован для дрона {drone_id}")
        
    except Exception as e:
        print(f"Ошибка при модификации файла конфигурации: {e}")

def main():
    if len(sys.argv) < 2:
        print("Использование: python create_unique_drone_models.py <путь_к_модели_iris_with_ardupilot>")
        print("Например: python create_unique_drone_models.py /home/timowko/ardupilot_gazebo/models/iris_with_ardupilot")
        return
    
    source_model_path = sys.argv[1]
    if not os.path.exists(source_model_path):
        print(f"Ошибка: Путь {source_model_path} не существует")
        return
    
    # Создаем уникальные модели для 5 дронов
    models = []
    for drone_id in range(1, 6):
        model_path = create_unique_drone_model(source_model_path, drone_id)
        models.append(model_path)
    
    print("\nСозданы следующие модели дронов:")
    for i, model_path in enumerate(models):
        print(f"Дрон {i+1}: {os.path.basename(model_path)}")
    
    print("\nИнструкция по использованию:")
    print("1. Запустите Gazebo")
    print("2. Добавьте каждую модель дрона в симуляцию через Insert -> <имя_модели>")
    print("3. Запустите SITL для каждого дрона с соответствующим ID")

if __name__ == "__main__":
    main() 