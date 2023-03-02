# ALEX_serial_cpp
git config --global user.email "you@example.com"
git config --global user.name "Your Name"
  

Команды для запуска нод:


Отправить сообщение на микроконтроолер: 
ros2 run serial_communicate_cpp to_leonardo


Получить сообщение от микроконтроллера: 
ros2 run serial_communicate_cpp from_leonardo




Ноды используют custom_msg: 

int16 lw_spd: 0

int16 rw_spd: 0

int16 body_angle: 0

int16 hand_angle: 0
