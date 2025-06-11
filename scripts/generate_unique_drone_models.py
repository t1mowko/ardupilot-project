#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import shutil
import xml.etree.ElementTree as ET

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
    
    # Проверяем, существует ли уже такая модель
    if os.path.exists(target_model_path):
        print(f"Модель {model_name} уже существует, пропускаем...")
        return target_model_path
    
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
        # Парсим SDF файл
        tree = ET.parse(sdf_path)
        root = tree.getroot()
        
        # Изменяем имя модели
        model = root.find("model")
        if model is not None:
            model_name = f"iris_with_ardupilot_drone{drone_id}"
            model.set("name", model_name)
        
        # Изменяем имена всех линков, добавляя суффикс с ID дрона
        for link in root.findall(".//link"):
            original_name = link.get("name")
            if original_name:
                new_name = f"{original_name}_drone{drone_id}"
                link.set("name", new_name)
        
        # Изменяем имена всех джойнтов и их ссылки на линки
        for joint in root.findall(".//joint"):
            original_name = joint.get("name")
            if original_name:
                new_name = f"{original_name}_drone{drone_id}"
                joint.set("name", new_name)
            
            # Обновляем ссылки на родительский и дочерний линки
            parent = joint.find("parent")
            if parent is not None and parent.text:
                parent.text = f"{parent.text}_drone{drone_id}"
            
            child = joint.find("child")
            if child is not None and child.text:
                child.text = f"{child.text}_drone{drone_id}"
        
        # Сохраняем изменения
        tree.write(sdf_path, encoding="utf-8", xml_declaration=True)
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
        # Парсим XML файл конфигурации
        tree = ET.parse(config_path)
        root = tree.getroot()
        
        # Изменяем имя модели
        name = root.find("name")
        if name is not None:
            name.text = f"Iris with ArduPilot (Drone {drone_id})"
        
        # Сохраняем изменения
        tree.write(config_path, encoding="utf-8", xml_declaration=True)
        print(f"Файл конфигурации модифицирован для дрона {drone_id}")
        
    except Exception as e:
        print(f"Ошибка при модификации файла конфигурации: {e}")

def main():
    if len(sys.argv) < 2:
        print("Использование: python generate_unique_drone_models.py <путь_к_модели_iris_with_ardupilot>")
        print("Например: python generate_unique_drone_models.py /home/timowko/ardupilot_gazebo/models/iris_with_ardupilot")
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