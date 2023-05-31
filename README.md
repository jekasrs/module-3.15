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
    ros2 pkg create --build-type ament_python fiducial_marker_pose  
В директории *ros2_ws/src/fiducial_marker_pose/fiducial_marker_pose* находится проект на языке python. 

### Настройки пакета 

В **package.xml** нужно изменить  

      <name>fiducial_marker_pose</name>
      <version>1.0.0</version>
      <description>Package for reading video stream and publishing information about detected AruCo markers</description>
      <author email="jekasjeny1012@gmail.com">Evgenii Smirnov</author>
      <maintainer email="voltage.machine@yandex.ru">Polytech Voltage Machine</maintainer>
В конце файла нужно добавить необходимые зависимости для узлов 

Пример :

    <depend>rclpy</depend>
    <depend>sensor_msgs</depend>
    <depend>python3-opencv</depend>
    <depend>yaml</depend>
    <exec_depend>ros2launch</exec_depend>

В **setup.py** нужно повторить основную информацию

    author='Evgenii Smirnov',
    author_email='jekasjeny1012@gmail.com',
    maintainer='Polytech Voltage Machine',
    maintainer_email='voltage.machine@yandex.ru',
    license='Apache License'

И следующую конструкцию: 

    entry_points={
        'console_scripts': [
            'marker_estimator = fiducial_marker_pose.MarkerEstimator:main',
            'camera_publisher = fiducial_marker_pose.CameraPublisher:main',
        ],
    },


### Сборка 

    cd 
    source ~/ros2_galactic/install/local_setup.bash
    cd ~/ros2_ws
    rosdep install -i --from-path src --rosdistro foxy -y
    colcon build --packages-select fiducial_marker_pose

### Запуск основного функционального узла, определяющий маркер: 
    . install/setup.bash
    ros2 run fiducial_marker_pose marker_estimator
    
### Запуск отправляющиего изображения узла (опциональный, нужен только для отладки): 
    . install/setup.bash
    ros2 run fiducial_marker_pose camera_publisher


## Cтруктура проекта 

| Название директории      | Описание                                       |
|--------------------------|------------------------------------------------|
| ros2_ws                  | Рабочая директория                             |
| ├── .calibration         | Скрипт для калибрвоки камеры                   |
| ├── .generator           | Скрипт для создания изображения аруко маркеров |
| ├── config               | Глобальные параметры в YAML-файл               |                                       |
| ├── fiducial_marker_pose | Директория проекта (приложение)                |
| ├── launch               | launch-файл для запуска узла                   |
| ├── test                 | Cкрипты для тестирования                       |
| ├── msg                  | Опредления структуры сообщения                 |
| ├── package.xml          | Описание свойств пакета                        |
| └── setup.py             | Конфигурация проека                            |
  
### Функциональная схема пакета

  PublisherNode          узел, отправляющий изображение  
            |  
            |  
       (Image)  
            |  
            |  
   [image_raw]          topic 1   
            |  
            |  
       (Image)  
            |  
            |  
 DetectionNode           узел, определяющий положение маркер  
            |  
            |  
      (Marker)  
            |  
            |  
   [marker_raw]          topic 2   
