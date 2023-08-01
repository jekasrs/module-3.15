# Обнаружение и оценка положения маркеров на изображении с камеры (напр. ArUco)

Пакет предназначен для получения изображения, распознавания ArUco маркеров и публикации стандартных сообщений Robot Operating System (ROS) в виде обнаруженных маркеров. В основе лежит библиотека OpenCV — библиотека алгоритмов компьютерного зрения, обработки изображений и численных алгоритмов общего назначения с открытым кодом. 

В пакете можно использовать маркеры из словаря с внутренней бинарной квадратной матрицей размером 4x4.  
Возможные словари:
- cv2.aruco.DICT_4X4_50 - словарь из 50 маркеров
- cv2.aruco.DICT_4X4_100 - словарь из 100 маркеров
- cv2.aruco.DICT_4X4_250 - словарь из 250 маркеров
- cv2.aruco.DICT_4X4_1000 - словарь из 1000 маркеров

Существующие пакеты (_не проверены_):
* [clover/aruco_pose](https://github.com/CopterExpress/clover)
* [aruco_ros](https://github.com/pal-robotics/aruco_ros/tree/humble-devel)
* [ros_aruco_opencv](https://github.com/fictionlab/ros_aruco_opencv/tree/humble)
* [fiducials](https://github.com/UbiquityRobotics/fiducials/tree/foxy-devel)

### Генерация ArUco маркеров
Для создания ArUco маркеров можно воспользоваться скриптом в пакете [.demo](fiducial_marker_pose/.demo). Чтобы сгенерировать маркеры, нужно заполнить [aruco_marker_generate.yaml](fiducial_marker_pose/.demo/aruco_marker_generate.yaml) файл в этой же директории или создать свой на основе примера.  
  
**1. Параметры файла конфигурации**  
Необходимо определить следующие параметры:  
   - _aruco_type_: тип маркеров ArUco, который может быть выбран из предопределенных словарей.  
   - _marker_ids_: список идентификаторов маркеров, которые необходимо сгенерировать.  
   - _image_size_: размер выходного изображения маркера в пикселях.
   - _marker_border_: ширина черной границы маркера в битах.  
   - _dir_path_: путь, по которому будут сохранены сгенерированные маркеры.  

**2. Запуск скрипта**  
Запустить генерацию маркеров можно одним из следующих образов:  
1. Без передачи аргументов командной строки. Используется файл конфигурации [aruco_marker_generate.yaml](fiducial_marker_pose/.demo/aruco_marker_generate.yaml) по умолчанию.
```bash
    python aruco_marker_generate.py
```

2. С передачей параметра --config-dir и пути к файлу конфигурации.  
```bash
    python aruco_marker_generate.py --config-dir path/to/config.yaml
```
При выполнении скрипта нужно обязательно указывать аргументы командной строки соответствующим образом. В противном случае будет выведено сообщение "Unknown option: {option}", указывающее на неизвестную опцию.

### О детектировании
Обнаружение ArUco маркеров на изображении состоит из двух шагов:   

1. _Обнаружение кандидатов на маркеры_:  
Анализируется изображение для поиска квадратных фигур, которые могут быть потенциальными маркерами. Алгоритма обнаружения контуров.  

2. _Оценка каждого кандидата на маркер_:  
Анализируется внутренняя кодификация каждого потенциального маркера. Распознавание битов маркера (поиск черных и белых квадратов).  

Для обнаружения маркеров испольузется класс [ArucoDetector](https://docs.opencv.org/4.x/d2/d1a/classcv_1_1aruco_1_1ArucoDetector.html). При создании экземпляра, можно установить множество параметров с помощью структуры [DetectorParametrs](https://docs.opencv.org/4.x/d1/dcd/structcv_1_1aruco_1_1DetectorParameters.html).  

Все параметры установлены по умолчанию, например:
- _cornerRefinementMaxIterations_ = 30: максимальное количество итераций для уточнения углов
- _cornerRefinementMinAccuracy_ = 0.1: минимальная точность для уточнения углов
- _markerBorderBits_ = 1: количество битов на границе маркера
- _minMarkerSizeRate_ = 0.03: минимальный размер маркера
- ... 
- Всего 36 различных параметров. 

После выполнения этих шагов возвращается список обнаруженных маркеров, каждый из которых содержит **положение его четырех углов** на изображении в их первоначальном порядке и **идентификатор маркера**.

3. Кватернион  

**Кватернионы** - это набор четырех чисел, которые представляют ориентацию объекта в трехмерном пространстве.  

**Вектор трансляции** - представляет собой трехкомпонентный вектор, содержащий информацию о смещении объекта по осям координат в трехмерном пространстве. Он определяет расстояние и направление перемещения объекта относительно камеры.
   
**Вектор ротации** - представляет собой трехкомпонентный вектор, содержащий информацию о повороте объекта в трехмерном пространстве. Он определяет ось вращения и угол поворота вокруг этой оси.

С помощью найденных четырех угловых точек маркера, мы можем определить положение и ориентацию объекта относительно камеры:
- Вектор трансляции и ротации можно найти с помощью функцию openCV [_solvePNP_](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#ga549c2075fac14829ff4a58bc931c033d)
    - Всего доступны _9 различных методов_ для нахождения векторов трансляции и ротации.   
    - SOLVEPNP_ITERATIVE: уточнение позы с использованием нелинейной схемы минимизации Левенберга-Маркардта. Метод по умолчанию. Не оптимальное решение. 
В этом случае функция находит такую позу, которая минимизирует ошибку репроекции, то есть сумму квадратов расстояний между наблюдаемыми проекциями и спроецированными.
    - SOLVEPNP_IPPE_SQUARE: метод основан на статье Тоби Коллинза и Адриана Бартоли. «Бесконечно малая оценка положения на плоскости» [[52]](https://link.springer.com/article/10.1007/s11263-014-0725-5). Создателями OpenCV метод подписан: "подходит для оценки положения маркера". Метод SOLVEPNP_IPPE_SQUARE использует аналитическое выражение для вычисления гомографии объекта-изображения. Это позволяет ему быть значительно более быстрым, поскольку поза решается полностью **аналитически**. Фактически, этот метод может быть примерно в **50-80 раз быстрее**, чем SOLVEPNP_ITERATIVE. В данной [статье](https://link.springer.com/article/10.1007/s11263-014-0725-5) есть описание этого аналитеческого метода. **На данный момент этот алгоритм используется в пакете.**  

- Затем строится кватернион с помощью [библиотеки scipy](https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.as_quat.html)  
   - Вычисляется матрица поворота для вектора ротации  
   - Строится кватерниор через библиотечну функцию _as_quat_  

Вектор трансляции и кватернион образуют сообщение [Marker](pose_interfaces/msg/Marker.msg) - для определения положения и ориентацию объекта. 

#### 3. Тесты  
Для тестирования пакета (измерение расстояние от камеры до маркера) можно воспользоваться скриптом в пакете [.demo](fiducial_marker_pose/.demo). Чтобы получить гафики, нужно заполнить [aruco_marker_test.yaml](fiducial_marker_pose/.demo/aruco_marker_test.yaml) файл в этой же директории или создать свой на основе примера.  
  
**1. Параметры файла конфигурации**  
Необходимо определить следующие параметры:  
   - _input_files_:
       - _path_: пути к входным файлам, содержащим данные.
       - _marker_size_: размеры маркеров, связанные с каждым входным файлом.
   - _marker_id_: идентификатор маркера для фильтрации данных. В алгориме будет обработана только информация с указанным marker_id.  

Скрипт считывает все _пути_ к входным файлам и соответствующие им _размеры_ _маркеров_ из YAML-файла. Параметр _marker_id_ используется для фильтрации данных, чтобы обрабатывать только данные с указанным идентификатором маркера. Параметр _test_name_ используется для предоставления описательного заголовка для графика.  

**2. Запуск скрипта**  
Запустить тестирование можно одним из следующих образов:  
1. Без передачи аргументов командной строки. Используется файл конфигурации [aruco_marker_test.yaml](fiducial_marker_pose/.demo/aruco_marker_test.yaml) по умолчанию.
```bash
    python aruco_marker_test.py
```

2. С передачей параметра --config-dir и пути к файлу конфигурации.  
```bash
    python aruco_marker_test.py --config-dir path/to/config.yaml
```

В папке [test](fiducial_marker_pose/.demo/test) есть примеры полученных сообщений работы пакета. На них проведены тесты и представлены следующие результаты: 

| Тест 1 |  Тест 2 |
| ------------- |:------------------:|
| ![ ](/.images/1_5.jpg) | ![ ](/.images/2.jpg) |   

На первом графике расположены результаты трех маркеров (длина маркера: 5 см, 10 см, 15 см) на дистанции 1.5 метра.
На втором графике расположены результаты трех маркеров (длина маркера: 5 см, 10 см, 15 см) на дистанции 2.0 метра.     

Лучше всего показал себя маркер 15x15 cм. Стабильнее всего определялось расстояние, разброс значений не большой. Также все значения находятся ближе всего к истинному значению, медиана находится почти около целового значения.    

### Взаимодействие с ROS
1. Загрузим пакет fiducial_marker_pose в пользовательское рабочее пространство ros2_ws и соберем его  
```bash
    source /opt/ros/galactic/setup.bash
    cd ~/ros2_ws/src
    git clone https://github.com/Polytech-VM-Team/fiducial_marker_pose
    cd ..
    rosdep install -i --from-path src --rosdistro galactic -y
    colcon build
```
2. Проверим, что сообщения [Marker](pose_interfaces/msg/Marker.msg) и [MarkerArray](pose_interfaces/msg/MarkerArray.msg) доступны к ипользованию
```bash
    ros2 interfaces show pose_interfaces/msg/Marker  
    ros2 interfaces show pose_interfaces/msg/MarkerArray  
```
3. Запустим рабочее пространство ROS 2 и узел fiducial_marker_pose  
```bash 
    . install/setup.bash
    ros2 launch fiducial_marker_pose fiducial_marker_pose.launch.py
```
rqt граф выглядит следующим образом:  
![ ](/.images/graph.png)

4. Узел получает данные и публикует сообщения в специальные темы, приведенные ниже. Проверим содержимое опубликованных сообщений в теме /camera/marker_pose:  
   - /camera/image_raw - [sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)
   - /camera/marker_pose - [sensor_msgs/MarkerArray](pose_interfaces/msg/MarkerArray.msg)
   - /camera/camera_info - [sensor_msgs/CameraInfo](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html)  
```bash
    source /opt/ros/galactic/setup.bash
    ros2 topic echo /camera/marker_pose  
```
  
5. Если установить в [fiducial_marker_pose.yaml](fiducial_marker_pose/config/fiducial_marker_pose.yaml) параметр **use_compressed**=True, то пакет будет получать сжатые изображения из темы:  
   - /camera/image_compressed - [sensor_msgs/CompressedImage](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CompressedImage.html)

    
  
6. Если установить в [fiducial_marker_pose.yaml](fiducial_marker_pose/config/fiducial_marker_pose.yaml) параметр **debug**=True, то пакет будет публиковать изображения с отрисованными векторами обнаруженного маркера и отрисовывать маркеры в [rviz](http://wiki.ros.org/rviz):  
   - /camera/image_debug - [sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)  
   - /camera/image_rviz - [visualization_msgs/MarkerArray](http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/MarkerArray.html)  

Чтобы запустить приложение RVIZ, в отдельном терминале вводим:  
```bash
    source /opt/ros/galactic/setup.bash
    rviz2 
```
Для отображения объектов нужно установить значение 'linked_camera' во всех frame_id. Пример, как выгялдит визуализация в RVIZ.  
![ ](/.images/rviz.png)
  
### Полезные источники  
[ROS Message](https://wiki.ros.org/sensor_msgs)  
[OpenCV documentation for ArUco](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)  
[Infinitesimal Plane-Based Pose Estimation](https://link.springer.com/article/10.1007/s11263-014-0725-5)
