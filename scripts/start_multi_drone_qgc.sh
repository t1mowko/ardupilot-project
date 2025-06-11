#!/bin/bash
# Скрипт для запуска симуляции нескольких дронов в WSL с подключением к QGC

echo "Запуск симуляции для нескольких дронов с подключением к QGC..."

# Определяем команду для запуска нового WSL терминала
# Для Windows 10 это может быть через powershell или cmd
TERM_CMD="cmd.exe /c start wsl.exe -d Ubuntu-20.04"

# 1. Запускаем Gazebo в отдельном терминале
$TERM_CMD bash -c "cd /home/timowko/catkin_ws && source devel/setup.bash && roslaunch gazebo_ros empty_world.launch"

echo "Запущен Gazebo. Теперь вручную добавьте модели дронов из: /home/timowko/ardupilot_gazebo/models/iris_with_ardupilot"
echo "После добавления моделей дронов в Gazebo, нажмите Enter для продолжения..."
read

# 2. Запускаем несколько экземпляров SITL в отдельных терминалах
# Дрон 1 (SYSID 1, порт 14551, UDP выход 14561)
$TERM_CMD bash -c "cd ~/ardupilot/ArduCopter && sim_vehicle.py -v ArduCopter -f gazebo-iris --sysid 1 --instance 0 --out=udp:127.0.0.1:14561 --out=udp:127.0.0.1:14550 --console"

echo "Запущен SITL для дрона 1. Дождитесь полной инициализации ArduCopter, затем нажмите Enter для запуска следующего дрона..."
read

# Дрон 2 (SYSID 2, порт 14552, UDP выход 14562)
$TERM_CMD bash -c "cd ~/ardupilot/ArduCopter && sim_vehicle.py -v ArduCopter -f gazebo-iris --sysid 2 --instance 1 --out=udp:127.0.0.1:14562 --out=udp:127.0.0.1:14550 --console"

echo "Запущен SITL для дрона 2. Дождитесь полной инициализации ArduCopter, затем нажмите Enter для запуска следующего дрона..."
read

# Дрон 3 (SYSID 3, порт 14553, UDP выход 14563)
$TERM_CMD bash -c "cd ~/ardupilot/ArduCopter && sim_vehicle.py -v ArduCopter -f gazebo-iris --sysid 3 --instance 2 --out=udp:127.0.0.1:14563 --out=udp:127.0.0.1:14550 --console"

echo "Запущен SITL для дрона 3. Дождитесь полной инициализации ArduCopter, затем нажмите Enter для запуска следующего дрона..."
read

# Дрон 4 (SYSID 4, порт 14554, UDP выход 14564)
$TERM_CMD bash -c "cd ~/ardupilot/ArduCopter && sim_vehicle.py -v ArduCopter -f gazebo-iris --sysid 4 --instance 3 --out=udp:127.0.0.1:14564 --out=udp:127.0.0.1:14550 --console"

echo "Запущен SITL для дрона 4. Дождитесь полной инициализации ArduCopter, затем нажмите Enter для запуска следующего дрона..."
read

# Дрон 5 (SYSID 5, порт 14555, UDP выход 14565)
$TERM_CMD bash -c "cd ~/ardupilot/ArduCopter && sim_vehicle.py -v ArduCopter -f gazebo-iris --sysid 5 --instance 4 --out=udp:127.0.0.1:14565 --out=udp:127.0.0.1:14550 --console"

echo "Запущен SITL для дрона 5. Дождитесь полной инициализации ArduCopter, затем нажмите Enter для запуска MAVROS..."
read

# 3. Запускаем MAVROS для всех дронов
$TERM_CMD bash -c "cd /home/timowko/catkin_ws && source devel/setup.bash && roslaunch basic_nav multi_drone_qgc.launch"

echo "Запущен MAVROS для всех дронов."
echo ""
echo "Инструкция по использованию QGC:"
echo "1. Запустите QGC на вашем компьютере"
echo "2. В QGC настройте подключение по UDP к порту 14550"
echo "3. QGC должен автоматически обнаружить все 5 дронов (проверьте Vehicle список)"
echo "4. Для каждого дрона вы можете:"
echo "   - Армировать и взлетать"
echo "   - Устанавливать точки миссии"
echo "   - Переключаться между дронами через Vehicle список"
echo ""
echo "Примечание: Каждый дрон имеет свой системный ID (SYSID) от 1 до 5" 