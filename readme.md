## Roarm manipulator control and real time torque

how to use the roarm m2s manipulator go the https://github.com/waveshareteam/roarm_ws_em0

to turn on the motor drivers of manipulator
ros2 run roarm_driver roarm_driver
turn the rviz digital twin of manipulator
ros2 launch roarm_moveit_cmd command_control.launch.py
to make real time data of servo motor of manipulator such as position
ros2 run roarm_moveit_cmd movepointcmd

voltage_date_test, roarm_driver and voltage_subsriber should to located into the roarm_ws_em0/src/roarm_main/roarm_driver
you should to add the them to path into the setup.py

this code will show real time graph by matplotlib of torque from servo motors. This is smart servo motor ST3215 Bus Servo Interface, from them can be get the data of tempreture, torque, position, voltage, currect and so on.

ros2 run roarm_driver voltage_data_test 
ros2 run roarm_driver voltage_subscriber
to go manipulator to particular coordinates by moveit and inverse kinemitics
ros2 run send_goalpoint move_point

program realization 

youtube video link
https://www.youtube.com/watch?v=mPZ9lYyAxys