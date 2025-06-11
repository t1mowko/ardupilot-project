#!/bin/bash
# Скрипт для запуска тестирования навигационного стека дрона в WSL

echo "Запуск симуляции для планировщика пути дрона в WSL..."

# Определяем команду для запуска нового WSL терминала
# Для Windows 10 это может быть через powershell или cmd
TERM_CMD="cmd.exe /c start wsl.exe -d Ubuntu-20.04"

# 1. Запускаем Gazebo в отдельном терминале
$TERM_CMD bash -c "cd /home/timowko/catkin_ws && source devel/setup.bash && roslaunch gazebo_ros empty_world.launch"

echo "Запущен Gazebo. Теперь вручную добавьте модель дрона из: /home/timowko/ardupilot_gazebo/models/iris_with_ardupilot"
echo "После добавления модели дрона в Gazebo, нажмите Enter для продолжения..."
read

# 2. Запускаем SITL в отдельном терминале
$TERM_CMD bash -c "cd ~/ardupilot/ArduCopter && sim_vehicle.py -v ArduCopter -f gazebo-iris --sysid 1 --instance 0 --out=udp:127.0.0.1:14550 --console"

echo "Запущен SITL. Дождитесь полной инициализации ArduCopter, затем нажмите Enter для запуска навигационного стека..."
read

# 3. Запускаем навигационный стек
$TERM_CMD bash -c "cd /home/timowko/catkin_ws && source devel/setup.bash && roslaunch basic_nav basic_nav.launch"

echo "Запущен навигационный стек. Используйте RViz для установки 2D Nav Goal."
echo ""
echo "Инструкция по использованию:"
echo "1. В RViz нажмите кнопку '2D Nav Goal'"
echo "2. Щелкните на карте, чтобы установить целевую точку"
echo "3. Проверяйте логи выполнения в терминале с basic_nav.launch" 