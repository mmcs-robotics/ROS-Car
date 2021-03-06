# ROS-car

Здесь находятся материалы с семинара по ROS, проходящего на мехмате ЮФУ в первой половине 2020 года.

## Куда это клонировать?

`cd <catkin_workspace>/src`

## Что необходимо в первую очередь?

Установленная `ROS`, [`ArduinoIDE`](https://www.arduino.cc/en/Main/Software), `rosserial`.

### Установка нужных частей `rosserial`
```bash
sudo apt-get install ros-melodic-rosserial-arduino
sudo apt-get install ros-melodic-rosserial
sudo apt-get install ros-melodic-rosserial-server
```

### Донастройка `ArduinoIDE`

Для работы требуется [подключить поддержку `ESP` в среде разработки](https://habr.com/ru/post/371853/) и установить rosserial_arduino library:

`<sketchbook>` - папка со штуками для ардуино, указана в настройках `ArduinoIDE`, по умолчанию `~/Arduino`

```bash
cd <sketchbook>/libraries
rosrun rosserial_arduino make_libraries.py .
```

### Если проблемы с правами на COM порт

В Ubuntu COM-порты защищены групповой политикой и доступны пользователю root и группе `dialout`. Добавим себя в эту группу (чтобы добавление в группу заработало, необходимо перелогиниться текущим пользователем).
```
sudo usermod -a -G dialout <USERNAME>
```

### Если используем виртуалку

#### Донастраиваем [VirtualBox](https://www.virtualbox.org/)
Если при запуске ошибка с hyper visor, а для него уже всё установлено, в том числе в BIOS, то необходимо выполнить в консоли Windows от имени администратора `bcdedit /set hypervisorlaunchtype off` и перезагрузить компьютер. (Если вдруг вы работаете с Docker, то для его работы нужно будет выставлять параметр в `auto`. К сожалению, на Windows нельзя одновременно работать с Docker и VirtualBox).

Для работы с Arduino нужно пробросить USB в виртуалку. Для этого необходим [Oracle VM VirtualBox Extension Pack](https://www.virtualbox.org/wiki/Downloads).
После его установки можно будет включить контроллеры USB 2.0 или 3.0. в `Настройки (машины)/USB/Включить контроллер USB`.

## Что тут есть?

### `src/ros_car_topic.ino`

Скетч для заливки на `ESP8266 12-E` через `ArduinoIDE`.

Скетч подключается к Wi-Fi и `rosserver` (который надо поднять командой `roslaunch rosserial_server socket.launch`), для чего надо будет заполнить соответствующие переменные. Также там есть нода, подписывающаяся на топик `drive`.

### `scripts/drive_keyboard.py`

Нода для управления машинкой стрелками клавиатуры путём отправки сообщений в топик.

Для запуска требует **root** прав и установленной библиотеки `keyboard`(ставится через `pip`).

```bash
sudo su
source devel/setup.bash
rosrun ros_car drive_keyboard.py
```
### `scripts/ros_car_android.cpp`

Нода для управления машинкой наклонами телефона.

```bash
rosrun ros_car ros_car_node
```

для работы необходимо скачать [приложение](https://play.google.com/store/apps/details?id=org.ros.android.sensors_driver&hl=ru), и [подключить](https://wiki.ros.org/android_sensors_driver/Tutorials/Connecting%20to%20a%20ROS%20Master) его к ros