# Запуск роя дронов с уникальными моделями

Этот документ содержит инструкции по запуску роя из 5 дронов с уникальными моделями для предотвращения конфликтов в Gazebo.

## Проблема с одинаковыми моделями

При использовании одной и той же модели дрона (`iris_with_ardupilot`) для нескольких экземпляров в Gazebo возникают проблемы:
- Сообщения об ошибках "link X down"
- Конфликты между дронами из-за одинаковых идентификаторов в SDF файлах
- Проблемы с подключением SITL к конкретному дрону

## Решение: Создание уникальных моделей

Для решения этой проблемы мы создадим уникальные модели для каждого дрона, изменив идентификаторы в SDF файлах.

### Шаг 1: Генерация уникальных моделей

```bash
# Сделайте скрипт исполняемым
chmod +x scripts/generate_unique_drone_models.py

# Запустите скрипт, указав путь к оригинальной модели
python3 scripts/generate_unique_drone_models.py /home/timowko/ardupilot_gazebo/models/iris_with_ardupilot
```

Скрипт создаст 5 уникальных моделей дронов:
- `iris_with_ardupilot_drone1`
- `iris_with_ardupilot_drone2`
- `iris_with_ardupilot_drone3`
- `iris_with_ardupilot_drone4`
- `iris_with_ardupilot_drone5`

### Шаг 2: Запуск Gazebo

```bash
roslaunch gazebo_ros empty_world.launch
```

### Шаг 3: Добавление уникальных моделей в Gazebo

В Gazebo:
1. Нажмите Insert -> (найдите каждую из созданных моделей)
2. Добавьте каждую модель дрона в симуляцию:
   - `iris_with_ardupilot_drone1`
   - `iris_with_ardupilot_drone2`
   - `iris_with_ardupilot_drone3`
   - `iris_with_ardupilot_drone4`
   - `iris_with_ardupilot_drone5`
3. Разместите дроны в форме сетки с расстоянием примерно 2-3 метра между ними

### Шаг 4: Запуск SITL для каждого дрона

В отдельных терминалах запустите:

```bash
# Дрон 1
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --sysid 1 --instance 0 --out=udp:127.0.0.1:14561 --out=udp:127.0.0.1:14550 --console
```

```bash
# Дрон 2
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --sysid 2 --instance 1 --out=udp:127.0.0.1:14562 --out=udp:127.0.0.1:14550 --console
```

```bash
# Дрон 3
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --sysid 3 --instance 2 --out=udp:127.0.0.1:14563 --out=udp:127.0.0.1:14550 --console
```

```bash
# Дрон 4
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --sysid 4 --instance 3 --out=udp:127.0.0.1:14564 --out=udp:127.0.0.1:14550 --console
```

```bash
# Дрон 5
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --sysid 5 --instance 4 --out=udp:127.0.0.1:14565 --out=udp:127.0.0.1:14550 --console
```

Дождитесь полной инициализации каждого экземпляра ArduCopter перед запуском следующего.

### Шаг 5: Запуск контроллера роя дронов

```bash
roslaunch basic_nav drone_swarm.launch
```

## Автоматизированный запуск

Для автоматизации процесса можно использовать скрипт:

```bash
# Сделайте скрипт исполняемым
chmod +x scripts/start_drone_swarm_unique.sh

# Запустите скрипт
./scripts/start_drone_swarm_unique.sh
```

## Устранение неполадок

1. **Ошибка "link X down"**:
   - Убедитесь, что вы используете уникальные модели для каждого дрона
   - Проверьте, что SDF файлы каждой модели имеют уникальные идентификаторы

2. **Дрон не реагирует на команды SITL**:
   - Проверьте, что каждый экземпляр SITL имеет уникальный `--sysid` и `--instance`
   - Убедитесь, что порты UDP не конфликтуют

3. **Проблемы с подключением через QGC**:
   - Проверьте, что все дроны отправляют данные на порт 14550 (`--out=udp:127.0.0.1:14550`)
   - Убедитесь, что у каждого дрона уникальный системный ID (SYSID) 