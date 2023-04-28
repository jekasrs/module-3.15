# module-3.15   
ROS пакет для взаимодействия с ArUco марке.

##Описание

---
**ROS-пакет:**
* ПО: ROS 2 Galactic Geochelone  
* Язык программирования: C++, Python  
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
