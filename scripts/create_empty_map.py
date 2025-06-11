#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import numpy as np
from PIL import Image

# Размер карты в пикселях (2000x2000 пикселей)
width, height = 2000, 2000

# Создаем пустую карту (белый цвет = свободное пространство)
# 255 = белый (свободное пространство), 0 = черный (препятствие)
map_data = np.ones((height, width), dtype=np.uint8) * 255

# Создаем тонкую рамку по периметру карты
border_width = 5
map_data[0:border_width, :] = 127  # Верхняя граница
map_data[-border_width:, :] = 127  # Нижняя граница
map_data[:, 0:border_width] = 127  # Левая граница
map_data[:, -border_width:] = 127  # Правая граница

# Путь для сохранения
script_dir = os.path.dirname(os.path.abspath(__file__))
maps_dir = os.path.join(os.path.dirname(script_dir), 'maps')

# Создаем директорию maps, если она не существует
os.makedirs(maps_dir, exist_ok=True)

# Сохраняем карту в формате PGM
map_path = os.path.join(maps_dir, 'empty_map.pgm')
img = Image.fromarray(map_data)
img.save(map_path)

print(f"Пустая карта создана и сохранена в {map_path}") 