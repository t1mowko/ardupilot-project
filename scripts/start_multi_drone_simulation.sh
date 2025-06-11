#!/bin/bash
# Скрипт для запуска симуляции нескольких дронов в WSL

echo "Запуск симуляции для нескольких дронов в WSL..."

# Определяем команду для запуска нового WSL терминала
# Для Windows 10 это может быть через powershell или cmd
TERM_CMD="cmd.exe /c start wsl.exe -d Ubuntu-20.04"

# 1. Запускаем Gazebo в отдельном терминале
$TERM_CMD bash -c "cd /home/timowko/catkin_ws && source devel/setup.bash && roslaunch gazebo_ros empty_world.launch"

echo "Запущен Gazebo. Теперь вручную добавьте модели дронов из: /home/timowko/ardupilot_gazebo/models/iris_with_ardupilot"
echo "После добавления моделей дронов в Gazebo, нажмите Enter для продолжения..."
read

# 2. Запускаем несколько экземпляров SITL в отдельных терминалах
# Дрон 1
$TERM_CMD bash -c "cd ~/ardupilot/ArduCopter && sim_vehicle.py -v ArduCopter -f gazebo-iris --sysid 1 --instance 0 --out=udp:127.0.0.1:14551 --console"

echo "Запущен SITL для дрона 1. Дождитесь полной инициализации ArduCopter, затем нажмите Enter для запуска следующего дрона..."
read

# Дрон 2
$TERM_CMD bash -c "cd ~/ardupilot/ArduCopter && sim_vehicle.py -v ArduCopter -f gazebo-iris --sysid 2 --instance 1 --out=udp:127.0.0.1:14552 --console"

echo "Запущен SITL для дрона 2. Дождитесь полной инициализации ArduCopter, затем нажмите Enter для запуска следующего дрона..."
read

# Дрон 3
$TERM_CMD bash -c "cd ~/ardupilot/ArduCopter && sim_vehicle.py -v ArduCopter -f gazebo-iris --sysid 3 --instance 2 --out=udp:127.0.0.1:14553 --console"

echo "Запущен SITL для дрона 3. Дождитесь полной инициализации ArduCopter, затем нажмите Enter для запуска следующего дрона..."
read

# Дрон 4
$TERM_CMD bash -c "cd ~/ardupilot/ArduCopter && sim_vehicle.py -v ArduCopter -f gazebo-iris --sysid 4 --instance 3 --out=udp:127.0.0.1:14554 --console"

echo "Запущен SITL для дрона 4. Дождитесь полной инициализации ArduCopter, затем нажмите Enter для запуска следующего дрона..."
read

# Дрон 5
$TERM_CMD bash -c "cd ~/ardupilot/ArduCopter && sim_vehicle.py -v ArduCopter -f gazebo-iris --sysid 5 --instance 4 --out=udp:127.0.0.1:14555 --console"

echo "Запущен SITL для дрона 5. Дождитесь полной инициализации ArduCopter, затем нажмите Enter для запуска навигационного стека..."
read

# 3. Запускаем навигационный стек для нескольких дронов
$TERM_CMD bash -c "cd /home/timowko/catkin_ws && source devel/setup.bash && roslaunch basic_nav multi_drone.launch"

echo "Запущен навигационный стек для нескольких дронов."
echo ""
echo "Инструкция по использованию:"
echo "1. Дождитесь инициализации всех дронов"
echo "2. Наблюдайте за перемещением дронов из точки А в точку Б в RViz"
echo "3. Проверяйте логи выполнения в терминале с multi_drone.launch" 