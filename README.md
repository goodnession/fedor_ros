# Создание пакета
Предварительно создав рабочую директорию и папку **/src**, создаём в ней пакет
```
ros2 pkg create --build-type ament_python fedor_control
```
# Запуск сервера
Перед запуском пакета необходимо запустить вебсервер
```
python3 src/fedod_control/fedor_gui/main.py
```
# Запуск пакета
1. Собираем пакет
```
colcon build
```
2. Инициализируем установочные файлы
```
. install/setup.bash
```
3. Запускаем пакет с помощью **launch**
```
ros2 launch ./src/fedor_control/fedor_control/launch/fedor_launch.py
```