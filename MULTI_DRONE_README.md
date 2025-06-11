# Запуск нескольких дронов

Этот документ содержит инструкции по запуску демонстрации с 5 дронами, которые взлетают и летят из точки А в точку Б.

## Подготовка

1. Убедитесь, что все скрипты имеют права на выполнение:
   ```bash
   chmod +x scripts/multi_drone_controller.py scripts/start_multi_drone_simulation.sh scripts/simple_multi_drone_demo.py
   ```

## Вариант 1: Полная демонстрация с несколькими экземплярами SITL

### Шаг 1: Запустите Gazebo
```bash
roslaunch gazebo_ros empty_world.launch
```

### Шаг 2: Добавьте модели дронов в Gazebo
Добавьте 5 моделей дронов из `/home/timowko/ardupilot_gazebo/models/iris_with_ardupilot` в Gazebo.
Разместите их на некотором расстоянии друг от друга, чтобы избежать столкновений.

### Шаг 3: Запустите экземпляры SITL для каждого дрона
В отдельных терминалах запустите:

```bash
# Дрон 1
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --sysid 1 --instance 0 --out=udp:127.0.0.1:14551 --console
```

```bash
# Дрон 2
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --sysid 2 --instance 1 --out=udp:127.0.0.1:14552 --console
```

```bash
# Дрон 3
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --sysid 3 --instance 2 --out=udp:127.0.0.1:14553 --console
```

```bash
# Дрон 4
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --sysid 4 --instance 3 --out=udp:127.0.0.1:14554 --console
```

```bash
# Дрон 5
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --sysid 5 --instance 4 --out=udp:127.0.0.1:14555 --console
```

Дождитесь полной инициализации каждого экземпляра ArduCopter перед запуском следующего.

### Шаг 4: Запустите навигационный стек для нескольких дронов
```bash
roslaunch basic_nav multi_drone.launch
```

## Вариант 2: Упрощенная демонстрация с одним экземпляром SITL

Этот вариант проще в настройке, но менее реалистичен, так как использует один экземпляр SITL для эмуляции нескольких дронов.

### Шаг 1: Запустите Gazebo
```bash
roslaunch gazebo_ros empty_world.launch
```

### Шаг 2: Добавьте модель дрона в Gazebo
Добавьте модель дрона из `/home/timowko/ardupilot_gazebo/models/iris_with_ardupilot` в Gazebo.

### Шаг 3: Запустите SITL
```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --console
```

### Шаг 4: Запустите упрощенную демонстрацию
```bash
roslaunch basic_nav simple_multi_drone_demo.launch
```

## Наблюдение за полетом

После запуска навигационного стека:

1. В RViz вы увидите визуализацию всех дронов
2. Дроны автоматически армируются, взлетают и летят из точки А в точку Б
3. Наблюдайте за логами в терминале для отслеживания прогресса

## Настройка

Вы можете изменить количество дронов и высоту взлета, отредактировав параметры в launch-файле:
```bash
roslaunch basic_nav multi_drone.launch num_drones:=3 takeoff_altitude:=3.0
```

Или для упрощенной демонстрации:
```bash
roslaunch basic_nav simple_multi_drone_demo.launch num_drones:=3 takeoff_altitude:=3.0
```

## Устранение неполадок

1. Если дрон не армируется, проверьте логи SITL и убедитесь, что он правильно инициализирован
2. Если дрон не взлетает, проверьте, что он получает корректные команды через MAVROS
3. Если возникают проблемы с трансформациями, проверьте вывод `tf_monitor` и `rqt_tf_tree` 