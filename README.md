# module-3.15   
ROS пакет для взаимодействия с ArUco марке.

## Описание

---
**ROS-пакет:**
* ПО: ROS 2 Galactic Geochelone  
* Язык программирования: Python  
* Публикуемые сообщения: положения маркера  

**Функционал:** 
* Распознавание ArUco-маркеров
* Оценка положения ArUco-маркеров относительно платформы

## Установка ROS 2 Galactic (Ubuntu Focal):  

----
### Пред настройки  
    1. sudo apt install software-properties-common  
    2. sudo add-apt-repository universe  
    3. sudo apt update  
    4. sudo apt install curl  
    5. ## добавить ключ GPG ROS 2 с помощью apt

    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg  
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

### Установка нужных пакетов  
**1. Пакет для ROS**

    sudo apt update  
    sudo apt install -y \  
      python3-pip \  
      python3-pytest-cov \  
      ros-dev-tools  
**2. Пакеты для Python** [необязательно] 
 
    python3 -m pip install -U \
       flake8-blind-except \
       flake8-builtins \
       flake8-class-newline \
       flake8-comprehensions \
       flake8-deprecated \
       flake8-docstrings \
       flake8-import-order \
       flake8-quotes \
       pytest-repeat \
       pytest-rerunfailures \
       pytest \
       setuptools

### Установка ROS 2 Galactic
**1. Скачивание кода**

     mkdir -p *~/ros2_galactic/src*
     cd ~/ros2_galactic
     vcs import --input https://raw.githubusercontent.com/ros2/ros2/galactic/ros2.repos src
     sudo apt upgrade
     sudo rosdep init
     rosdep update
     rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-5.3.1 urdfdom_headers"
**2. Билдинг кода в рабочий директории** [примерно: 3 часа в зависимоти от тех. характеристик]

    cd ~/ros2_galactic/
    colcon build --symlink-install     

**3. Устонавка скрипта в оболочку bash**  

    ~/ros2_galactic/install/local_setup.bash    

## Создание ROS пакета
    mkdir -p ~/ros2_ws/src  
    cd ~/ros2_ws/src  
    ros2 pkg create --build-type ament_python py_pubsub  
В директории *ros2_ws/src/py_pubsub/py_pubsu* находится проект на языке python. 

### Настройки пакета 

В **package.xml** нужно изменить  

    <description>Examples of minimal publisher/subscriber using rclpy</description>
    <maintainer email="you@email.com">Your Name</maintainer>
    <license>Apache License 2.0</license>
В конце файла нужно добавить необходимые зависимости для узлов 

Пример :

    <exec_depend>rclpy</exec_depend>
    <exec_depend>std_msgs</exec_depend>
    <exec_depend>sensor_msgs</exec_depend>

В **setup.py** нужно повторить основную информацию

    maintainer='YourName',
    maintainer_email='you@email.com',
    description='Examples of minimal publisher/subscriber using rclpy',
    license='Apache License 2.0', 

И следующую конструкцию: 

    entry_points={
        'console_scripts': [
            'detector = py_pubsub.DetectionNode:main',
            'image_invader = py_pubsub.PublisherNode:main',
            ],
    },


### Сборка 

    cd 
    source ~/ros2_galactic/install/local_setup.bash
    cd ~/ros2_ws
    . install/setup.bash
    rosdep install -i --from-path src --rosdistro foxy -y
    colcon build --packages-select py_pubsub

### Запуск в новом терминале (разных)

    ros2 run py_pubsub detector
    ros2 run py_pubsub image_invader


## Cтруктура проекта 

| Название директории | Описание                        |
|---------------------|---------------------------------|
| ros_ws              | Рабочая директория              |
| ├── build           | Файлы компиляции                |
| ├── install         | Установщик                      |
| ├── log             | Логи                            |
| └── src             | Директория проекта              |
  
    
В директории src/py_pubsub/py_pubsub находится весь исходных код приложения

1. Файл define.py - константы и настраиваемые параметры  
2. Файл utils.py - функции для детектирования и взаимодействия с маркерами  
3. Файл DetectionNode.py - код для запуска детектирующий ноды 
4. Файл PublisherNode.py - код для запуска захватывающий изображение с камеры ноды

  PublisherNode  
            |  
            |  
       (Image)  
            |  
            |  
[sensors_msgs/image]  
            |  
            |  
       (Image)  
            |  
            |  
 DetectionNode  
            |  
            |  
Сохраняет файл с изображением  
[TODO: отправить координаты маркера в topic]  
