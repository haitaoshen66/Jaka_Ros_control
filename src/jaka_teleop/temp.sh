colcon build --packages-select hello_moveit
ros2 launch jaka_minicobo_moveit_config demo.launch.py use_rviz_sim:=true

cd /home/pinnmax/jaka_ros2_ws/src/jaka_ros2 &&
source install/setup.bash
ros2 run hello_moveit hello_moveit

ros2 service call /jaka_rviz_driver/get_ik jaka_msgs/srv/GetIK "{
ref_joint: [0, 1.57, -1.57, 1.57, 1.57, 0],
cartesian_pose: [0, 0, 0, -0.28, -0.2, 0.4]
}"

ros2 service call /jaka_rviz_driver/get_ik jaka_msgs/srv/GetIK "{
ref_joint: [0, 1.57, -1.57, 1.57, 1.57, 0],
cartesian_pose: [90, 10, 20, -0.28, -0.2, 0.4]
}"

ros2 service call /jaka_rviz_driver/get_ik jaka_msgs/srv/GetIK "{
ref_joint: [0, 1.57, -1.57, 1.57, 1.57, 0],
cartesian_pose: [0, 0, 0, -0.28, -0.2, 0.5]
}"

ros2 service call /jaka_driver/servo_move_enable jaka_msgs/srv/ServoMoveEnable "{
  enable: true
}"

colcon build --packages-select jaka_driver
ros2 service call /jaka_driver/servo_j jaka_msgs/srv/ServoMove "{
  pose: [0, -1.57, 1.57, 1.57, 1.57, 0]
}"
ros2 run jaka_driver servoj_demo

ros2 service call /jaka_rviz_driver/get_ik jaka_msgs/srv/GetIK "{
ref_joint: [0, 1.57, -1.57, 1.57, 1.57, 0],
cartesian_pose: [90, 10, 20, -0.28, -0.2, 0.4]
}"
real (-200, -500, 280)
rviz->real
x->-zcolcon build --packages-select 
y->x
z->-y

rviz->real
(90,0,0)->(-90,90,90)
(90,10,20)->(-150,70,30)
rx ->-rz
ry ->rx-90
rz ->-ry-90

ros2 run jaka_rviz_client client
ros2 run jaka_rviz_client jaka_servoj
ros2 launch jaka_driver robot_start.launch.py ip:=192.168.10.90
#抓取目标
ros2 service call /jaka_driver/joint_move jaka_msgs/srv/Move "{
  pose: [0, 0, -1.57, -1.57, 1.57, 0.78],
  has_ref: false,
  ref_joint: [0],
  mvvelo: 0.5,
  mvacc: 0.5,
  mvtime: 0.0,
  mvradii: 0.0,
  coord_mode: 0,
  index: 0
}"
# 低位抓取
ros2 service call /jaka_driver/joint_move jaka_msgs/srv/Move "{
  pose: [0, -1.57, 0, -1.57, 0.78, 0.78],
  has_ref: false,
  ref_joint: [0],
  mvvelo: 0.5,
  mvacc: 0.5,
  mvtime: 0.0,
  mvradii: 0.0,
  coord_mode: 0,
  index: 0
}"
# 开门
ros2 service call /jaka_driver/joint_move jaka_msgs/srv/Move "{
  pose: [1.87,-1.4,-0.023,-1.35,-1.33,0.85],
  has_ref: false,
  ref_joint: [0],
  mvvelo: 0.5,
  mvacc: 0.5,
  mvtime: 0.0,
  mvradii: 0.0,sht@sht-ASUS-TUF-Gaming-A15-FA507RC:~/DIJA$ 


  coord_mode: 0,
  index: 0
}"
# 中位
ros2 service call /jaka_driver/joint_move jaka_msgs/srv/Move "{
  pose: [1.50,-1,-1.0,0,-0.8,-0.785],
  has_ref: false,
  ref_joint: [0],
  mvvelo: 0.5,
  mvacc: 0.5,
  mvtime: 0.0,
  mvradii: 0.0,
  coord_mode: 0,
  index: 0
}"
# open gripper
ros2 service call /jaka_driver/gripper_init jaka_msgs/srv/ServoMoveEnable "{
  enable: true
}"
# close gripper
ros2 service call /jaka_driver/gripper_control jaka_msgs/srv/ServoMoveEnable "{
  enable: false
}"
ros2 launch jaka_teleop teleop_launch.py

# 夹药3,4层
ros2 service call /jaka_driver/joint_move jaka_msgs/srv/Move "{
  pose: [1.5,-0.763,-1.176,-0.159,-1,-0.5],
  has_ref: false,
  ref_joint: [0],
  mvvelo: 0.5,
  mvacc: 0.5,
  mvtime: 0.0,
  mvradii: 0.0,
  coord_mode: 0,
  index: 0
}"

# 夹药2层
ros2 service call /jaka_driver/joint_move jaka_msgs/srv/Move "{
  pose: [0.0,0.0,0.0,0.0,0.0,0.0],
  has_ref: false,
  ref_joint: [0],
  mvvelo: 0.5,
  mvacc: 0.5,
  mvtime: 0.0,
  mvradii: 0.0,
  coord_mode: 0,
  index: 0
}"
# 酒店清洁low
[0.52,-1.36,-0.61,-1.6,0.52,2.44]
# 酒店清洁扔垃圾
[1.76,-0.1,-1.93,-0.45,-1.3,1.34]
# 扯袋子
[1.65,-1.24,-1.52,-1.1,-0.9,1.34]

ls /dev/video*

ros2 launch jaka_driver robot_start.launch.py ip:=192.168.10.90
ros2 service call /jaka_driver/linear_move jaka_msgs/srv/Move "{
  pose: [353.1,-3.2,428.8,0.257,-0.231,0.657],
  mvvelo: 5.0,
  mvacc: 5.0,
  has_ref: false,
  ref_joint: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  mvtime: 0.0,
  mvradii: 0.0,
  coord_mode: 0,
  index: 0
}"

ros2 service call /jaka_driver/linear_move jaka_msgs/srv/Move "{
  pose: [253.1,-3.2,428.8,0.0, 0.0, -0.785],
  mvvelo: 50.0,
  mvacc: 50.0,
  has_ref: false,
  ref_joint: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  mvtime: 0.0,
  mvradii: 0.0,
  coord_mode: 0,
  index: 0
}"

ros2 service call /jaka_driver/linear_move jaka_msgs/srv/Move "{
  pose: [-200,-227.0,250.0,0.0,0.0,-0.785],
  mvvelo: 50.0,
  mvacc: 50.0,
  has_ref: false,
  ref_joint: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  mvtime: 0.0,
  mvradii: 0.0,
  coord_mode: 0,
  index: 0
}"