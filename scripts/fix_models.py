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
        
        # Сохраняем XML-декларацию и начало файла
        if '<?xml' in content and '<sdf' in content:
            header = content.split('<model', 1)[0]
            rest = '<model' + content.split('<model', 1)[1]
        else:
            header = '<?xml version="1.0"?>\n<sdf version="1.6">\n'
            rest = content
        
        # Меняем имя модели
        rest = re.sub(r'<model name="[^"]*"', f'<model name="iris_with_ardupilot_drone{drone_id}"', rest)
        
        # Добавляем суффикс с ID дрона ко всем именам линков и джойнтов
        rest = re.sub(r'<link name="([^"]*)"', f'<link name="\\1_drone{drone_id}"', rest)
        rest = re.sub(r'<joint name="([^"]*)"', f'<joint name="\\1_drone{drone_id}"', rest)
        
        # Обновляем ссылки на родительский и дочерний линки
        rest = re.sub(r'<parent>([^<]*)</parent>', f'<parent>\\1_drone{drone_id}</parent>', rest)
        rest = re.sub(r'<child>([^<]*)</child>', f'<child>\\1_drone{drone_id}</child>', rest)
        
        # Сохраняем изменения
        with open(sdf_path, 'w') as file:
            file.write(header + rest)
        
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
        # Создаем новый файл конфигурации с нуля
        config_content = f"""<?xml version="1.0"?>
<model>
  <name>Iris with ArduPilot (Дрон {drone_id})</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>

  <author>
    <name>Fadri Furrer</name>
    <email>fadri.furrer@mavt.ethz.ch</email>
  </author>
  <author>
    <name>Michael Burri</name>
  </author>
  <author>
    <name>Mina Kamel</name>
  </author>
  <author>
    <name>Janosch Nikolic</name>
  </author>
  <author>
    <name>Markus Achtelik</name>
  </author>

  <maintainer email="hsu@osrfoundation.org">john hsu</maintainer>

  <description>
    starting with iris_with_standoffs
    add LiftDragPlugin
    add ArduCopterPlugin
    attach gimbal_small_2d model with GimbalSmall2dPlugin
  </description>
  <depend>
    <model>
      <uri>model://gimbal_small_2d</uri>
      <version>1.0</version>
    </model>
    <model>
      <uri>model://iris_with_standoffs</uri>
      <version>1.0</version>
    </model>
  </depend>
</model>
"""
        
        # Сохраняем новый файл конфигурации
        with open(config_path, 'w') as file:
            file.write(config_content)
        
        print(f"Файл конфигурации создан для дрона {drone_id}")
        
    except Exception as e:
        print(f"Ошибка при модификации файла конфигурации: {e}")

def main():
    if len(sys.argv) < 2:
        print("Использование: python fix_models.py <путь_к_модели_iris_with_ardupilot>")
        print("Например: python fix_models.py /home/timowko/ardupilot_gazebo/models/iris_with_ardupilot")
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