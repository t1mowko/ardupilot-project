#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import glob
import sys

def fix_line_endings(directory):
    """
    Исправляет окончания строк в файлах Python с DOS (CRLF) на UNIX (LF)
    """
    print(f"Исправление окончаний строк в директории: {directory}")
    
    # Находим все Python файлы
    python_files = glob.glob(f"{directory}/**/*.py", recursive=True)
    
    count = 0
    for file_path in python_files:
        try:
            # Читаем содержимое файла
            with open(file_path, 'rb') as f:
                content = f.read()
            
            # Проверяем, содержит ли файл CRLF окончания строк
            if b'\r\n' in content:
                # Заменяем CRLF на LF
                content = content.replace(b'\r\n', b'\n')
                
                # Записываем обратно в файл
                with open(file_path, 'wb') as f:
                    f.write(content)
                
                print(f"Исправлен файл: {file_path}")
                count += 1
        except Exception as e:
            print(f"Ошибка при обработке файла {file_path}: {e}")
    
    print(f"Всего исправлено файлов: {count}")

if __name__ == "__main__":
    # Если указан аргумент командной строки, используем его как директорию
    if len(sys.argv) > 1:
        directory = sys.argv[1]
    else:
        # Иначе используем текущую директорию скрипта
        directory = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    
    fix_line_endings(directory) 