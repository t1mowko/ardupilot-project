#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import glob

def fix_config_file(config_path):
    """
    Исправляет файл конфигурации модели
    
    Args:
        config_path: Путь к файлу конфигурации
    """
    try:
        # Читаем содержимое файла
        with open(config_path, 'r') as file:
            content = file.read()
        
        # Заменяем неправильные теги <n> на <name>
        content = content.replace('<n>', '<name>')
        content = content.replace('</n>', '</name>')
        
        # Сохраняем изменения
        with open(config_path, 'w') as file:
            file.write(content)
        
        print(f"Файл конфигурации исправлен: {config_path}")
        
    except Exception as e:
        print(f"Ошибка при исправлении файла конфигурации: {e}")

def main():
    if len(sys.argv) < 2:
        print("Использование: python fix_config.py <путь_к_директории_с_моделями>")
        print("Например: python fix_config.py /home/timowko/ardupilot_gazebo/models/iris_with_ardupilot_drone*")
        return
    
    pattern = sys.argv[1]
    model_dirs = glob.glob(pattern)
    
    if not model_dirs:
        print(f"Не найдено моделей по шаблону: {pattern}")
        return
    
    for model_dir in model_dirs:
        config_path = os.path.join(model_dir, "model.config")
        if os.path.exists(config_path):
            fix_config_file(config_path)
    
    print("\nГотово! Все файлы конфигурации исправлены.")

if __name__ == "__main__":
    main() 