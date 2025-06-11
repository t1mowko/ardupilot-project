#!/bin/bash
# Скрипт для запуска симуляции роя дронов с уникальными моделями в WSL

echo "Запуск симуляции роя дронов с уникальными моделями в WSL..."

# Определяем команду для запуска нового WSL терминала
# Для Windows 10 это может быть через powershell или cmd
TERM_CMD="cmd.exe /c start wsl.exe -d Ubuntu-20.04"

# 0. Генерация уникальных моделей дронов
echo "Генерация уникальных моделей дронов..."
python3 scripts/generate_unique_drone_models.py /home/timowko/ardupilot_gazebo/models/iris_with_ardupilot

echo "Уникальные модели дронов созданы. Нажмите Enter для запуска Gazebo..."
read

# 1. Запускаем Gazebo в отдельном терминале
$TERM_CMD bash -c "cd /home/timowko/catkin_ws && source devel/setup.bash && roslaunch gazebo_ros empty_world.launch"

echo "Запущен Gazebo. Теперь вручную добавьте уникальные модели дронов:"
echo "- iris_with_ardupilot_drone1"
echo "- iris_with_ardupilot_drone2"
echo "- iris_with_ardupilot_drone3"
echo "- iris_with_ardupilot_drone4"
echo "- iris_with_ardupilot_drone5"
echo ""
echo "Разместите дроны в форме сетки с расстоянием примерно 2-3 метра между ними"
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

echo "Запущен SITL для дрона 5. Дождитесь полной инициализации ArduCopter, затем нажмите Enter для запуска контроллера роя..."
read

# 3. Запускаем контроллер роя дронов
$TERM_CMD bash -c "cd /home/timowko/catkin_ws && source devel/setup.bash && roslaunch basic_nav drone_swarm.launch"

echo "Запущен контроллер роя дронов."
echo ""
echo "Инструкция по наблюдению за роем:"
echo "1. Наблюдайте за перемещением дронов в Gazebo"
echo "2. Дроны автоматически взлетят и начнут выполнять миссию роя"
echo "3. Рой будет перемещаться в различных формациях:"
echo "   - V-образная формация (как стая птиц)"
echo "   - Линейная формация (перпендикулярно направлению движения)"
echo "   - Круговая формация (вокруг центрального дрона)"
echo ""
echo "4. Вы также можете подключиться к дронам через QGC:"
echo "   - Запустите QGC на вашем компьютере"
echo "   - В QGC настройте подключение по UDP к порту 14550"
echo "   - Вы сможете наблюдать за состоянием дронов и их параметрами"
echo ""
echo "Примечание: Благодаря использованию уникальных моделей дронов, проблема 'link X down' должна быть решена." 