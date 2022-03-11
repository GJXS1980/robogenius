# 功能包介绍
人工智能仿真平台的功能包都放置在名为workstation的文件夹中,文件夹含有以下功能包
```bash
- agv_cobot                (cobot 7轴机械臂描述文件功能包)
- agv_vehicle             （AGV 移动底盘描述文件功能包）
- all_world               （世界描述文件功能包）
- cobot_moveit_config     （AGV 移动底盘与cobot 机械臂的moveit配置描述文件功能包）
- gazebo-pkgs             （gazebo夹抓修复插件功能包）
- general-message-pkgs    （gazebo夹抓修复插件附带消息功能包）
- motion_control          （基于python调用move_group控制机械臂功能包）
- object_detection_2d     （基于HSV颜色识别与抓取功能包）
- realsense2_description  （realsense2的描述文件功能包）
- realsense_plugin        （realsense插件功能包）
- scara_ws                （带scara机械臂工作台描述文件功能包）
- turtlebot3              （turtlebot3相关功能包）
```

### agv_cobot 功能包
agv_cobot目录下包含如下所示的文件结构，由config、launch、meshes、textures、urdf文件夹构成

```bash
.
├── config   （rviz配置文件）
├── launch   （rviz启动文件、gazebo启动文件）
├── meshes   （机器人关节模型）
├── textures （机器人纹理文件）
└── urdf     （机器人描述文件）
```
需要注意的是，在urdf文件夹下除cobot机械臂模型外，还包含realsense的描述文件。在agv_cobot.urdf.xacro文件中除了描述了机械臂各关节坐标关系外还包含realsense相机，通过launch文件夹可以启动对于仿真环境的机械臂仿真。

### agv_vehicle 功能包
agv_vehicle目录下包含如下所示的文件结构，由config、launch、meshes、textures、urdf文件夹构成

```bash
.
├── config   （rviz配置文件）
├── launch   （rviz启动文件、gazebo启动文件）
├── meshes   （机器人关节模型）
├── textures （机器人纹理文件）
└── urdf     （机器人描述文件）
```
需要注意的是，在urdf文件夹下包含有两类描述文件，agv_vehicle.urdf.xacro为移动底盘的描述文件，agv_arm.urdf.xacro为移动底盘搭载机械臂的描述文件，后续进行moveit的配置也是基于该描述文件进行。

### all_world 功能包
all_world目录下包含如下所示的文件结构，由launch、models与worlds文件夹构成
```bash
.
├── launch                    （场景启动文件）
├── models                    （场景模型描述文件）
│   ├── all_world_base        （围栏）
│   │   └── meshes            （物体模型）
│   ├── M10_sheet_metal       （钣金）
│   │   └── meshes            （物体模型）
│   ├── M10_sheet_metal_base  （钣金物料盘）
│   │   └── meshes            （物体模型）
│   ├── material_desk         （物料摆放桌）
│   │   └── meshes            （物体模型）
│   └── top_material          （置物架）
│       └── meshes
└── worlds                    （世界描述文件）
```
在models文件夹包含有相关场景模型的sdf 描述文件及模型文件，使用该功能包中launch文件夹的gazebo_spawn.launch可以生成目标场景，将其保存在worlds中用于后续开发。

### cobot_moveit_config 功能包
cobot_moveit_config目录下包含如下所示的文件结构，由config、launch文件夹构成
```bash
.
├── config      （相关配置文件）
└── launch      （相关启动文件）
```
需要注意的是，config中保存中移动平台搭载机械臂的机械臂配置文件，srdf为moveit配置文件，ros_controllers.yaml为机械臂控制器配置文件，通过launch文件可以启动仿真环境与rviz交互式运动规划控制。

### motion_control 功能包
motion_control目录下包含如下所示的文件结构，由include、scripts和src文件夹构成
```bash
.
├── include               （相关头文件）
│   └── motion_control    
├── scripts               （基于python调用move_group完成机械臂控制脚本）
└── src                   （基于c++调用move_group完成机械臂控制脚本）
```

### object_detection_2d 功能包
object_detection_2d目录下包含如下所示的文件结构，由msg、src文件夹构成
```bash
.
├── msg
└── src    （c++脚本代码）
```

### realsense2_description 功能包
realsense2_description目录下包含如下所示的文件结构，由meshes文件夹构成
```bash
.
└── meshes   （realsense模型文件）
```
realsense的描述文件在agv_cobot中，此处只包含realsense的模型文件。

### realsense_plugin 功能包
realsense_plugin目录下包含如下所示的文件结构，由include、src文件夹构成
```bash
.
├── include                         （realsense插件头文件）
│   └── realsense_gazebo_plugin
└── src                             （realsense插件代码）
```
由于gazebo的插件中不包含realsense的插件依赖，故需要添加此功能包以支持realsense在gazebo中进行仿真。

### scara_ws 功能包
scara_ws目录下包含如下所示的文件结构，由config、launch、meshes、textures和urdf文件夹构成
```bash
.
├── config       （相关配置文件）
├── launch       （rviz启动文件、gazebo启动文件）
├── meshes       （工作台关节模型文件）
├── textures     （模型纹理文件）
└── urdf         （模型描述文件）
```

### turtlebot3 功能包
turtlebot3目录下包含如下所示的文件结构，主要涉及turtlebot3的描述文件、启动文件、建图与导航文件、键盘控制以及相关的仿真启动文件。
```bash
.
├── turtlebot3
├── turtlebot3_bringup
├── turtlebot3_description
├── turtlebot3_example
├── turtlebot3_gazebo_plugin
├── turtlebot3_msgs
├── turtlebot3_navigation
├── turtlebot3_simulations
├── turtlebot3_slam
└── turtlebot3_teleop
```
该功能包为turtlebot3功能包，在原有的基础上，补充了AGV的slam与导航功能。

# 人工智能仿真平台操作手册
下面将从几个方面展开如何基于workstation文件夹搭建人工智能仿真平台；

### 世界场景生成
进入all_world功能包下models文件夹，里面每个文件夹包含场景模型的描述文件及模型文件。</br>
由于场景模型描述文件的指向的路径是绝对路径，故为作者的绝对路径，后续使用需要对模型sdf描述文件中的绝对路径进行替换。打开其中一个模型文件夹打开model.sdf文件，可以看到文件中的uri标签为绝对路径描述</br>
```xml
#原路径
<uri>/home/pickle/hpro_ws/src/workstation/all_world/models/all_world_base/meshes/all_world_base_link.STL</uri>

#修改格式，替换用户名与工作空间名字为您设置的相应名字
<uri>/home/username/xxxx_ws/src/workstation/all_world/models/all_world_base/meshes/all_world_base_link.STL</uri>
```

针对models文件夹中的每个模型文件的sdf文件中的路径进行替换后，启动终端输入如下命令
```bash
roslaunch all_world gazebo_spawn.launch
```
键入命令后将启动gazebo，gazebo启动完成后可以看到界面中显示的场景模型</br>
点击左上角files标签，选择save world as ，选择worlds文件夹，保存世界文件为workstation.world对原有文件进行替换。

### 仿真环境启动与SLAM建图导航（turtlebot3 版本）
##### turtlebot3 建图
首先在.bahrc添加如下代码
```bash
export TURTLEBOT3_MODEL=${TB3_MODEL}
export TURTLEBOT3_MODEL=waffle_pi
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/turtlebot/src/turtlebot3/turtlebot3_gazebo_plugin/build
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot/src/turtlebot3/turtlebot3_gazebo_plugin/models
```
打开终端在终端中键入如下命令更新终端环境
```bash
source ~/.bashrc
```
启动仿真世界并加载turtlebot3机器人模型
```bash
roslaunch all_world world_gazebo_turtlebot.launch
```
启动SLAM建图算法
```bash
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```
启动远程控制
```bash
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
保存地图
```bash
rosrun map_server map_saver -f ~/map
```

##### turtlebot3 导航
启动仿真环境并加载turtlebot3机器人模型
```bash
roslaunch all_world world_gazebo_turtlebot.launch
```
启动导航
```bash
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
```

### 仿真环境启动与SLAM建图导航（AGV 移动机械臂版本）
创建场景模型，并保存到<code>~/hg_ws/src/workstation/all_world/worlds/workstation_(工号).world</code>

根据<code>~/hg_ws/src/workstation/all_world/launch/workstation_demo.launch</code>路径下的workstation_demo.launch作为模板，按照workstation_(工号).launch命名格式进行命名

修改workstation_(工号).launch文件中的参数，调用workstation_(工号).world模型

修改<code>～/hg_ws/src/workstation/cobot_moveit_config/launch/demo_gazebo.launch</code>文件中的demo_gazebo.launch参数，参数如下：
```bash
  <!-- launch the gazebo simulator and spawn the robot -->
  <include file="$(find all_world)/launch/workstation_demo.launch" >
    <arg name="paused" value="$(arg paused)"/>
    <arg name="gazebo_gui" value="$(arg gazebo_gui)"/>
    <arg name="urdf_path" value="$(arg urdf_path)"/>
  </include>
```
把workstation_demo.launch改为workstation_(工号).launch。

##### AGV建图
启动仿真世界并加载AGV移动平台机器人模型
```bash
roslaunch cobot_moveit_config demo_gazebo.launch
```
启动SLAM建图算法
```bash
roslaunch turtlebot3_slam agv_slam.launch
```
启动远程控制
```bash
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
保存地图
```bash
rosrun map_server map_saver -f ~/map
```

##### AGV导航
启动仿真环境并加载AGV移动平台机器人模型
```bash
roslaunch cobot_moveit_config demo_gazebo.launch
```
启动导航
```bash
roslaunch turtlebot3_navigation agv_navigation.launch map_file:=$HOME/map.yaml
```

### cobot 7轴机械臂规划
启动仿真环境并加载rviz motion planning，此时通过rviz交互式可以实现对机械臂的规划与控制
```bash
roslaunch cobot_moveit_config demo_gazebo.launch
```

通过python脚本控制机械臂运动
```bash
rosrun motion_control cobot_motion.py
```
需要注意的是，需要将cobot_motion.py文件的属性设置为可执行程序

##### 视觉识别与抓取
启动仿真世界加载场景与机器人模型
```bash
roslaunch cobot_moveit_config demo_gazebo.launch
```
启动视觉识别脚本
```bash
rosrun opencv_object_tracking object_filter_orientation1 
```

启动识别抓取脚本
```bash
rosrun opencv_object_tracking get_target 
```

### GUI 界面介绍
首先进行GUI界面相关配置的修改
```bash
进入如下工作目录
/home/user_name/xxxxx_ws/src/workstation/hg_gui/src
```
打开main_window.cpp文件</br>
Ctrl+F搜索 home，可以看到如下一类的代码段，将其中的用户名与工作空间改为您自己的对应目录
```cpp
void MainWindow::on_pushButton_remote_clicked()
{
    bash_remote = new QProcess;
    QString remote_cmd = "source /home/pickle/hpro_ws/devel/setup.bash\nroslaunch turtlebot3_teleop turtlebot3_teleop_key.launch\n";
    bash_remote->start("bash");
    bash_remote->write(remote_cmd.toLocal8Bit());
    on_button_connect_clicked(true);
}
```
启动GUI界面
```bash
roscore
rosrun hg_gui hg_gui
```

### 功能包tree
```bash
.
├── turtlebot3
├── turtlebot3_bringup
│   ├── camera_info
│   ├── launch
│   │   └── includes
│   ├── scripts
│   └── src
├── turtlebot3_description
│   ├── meshes
│   │   ├── bases
│   │   ├── sensors
│   │   └── wheels
│   ├── rviz
│   └── urdf
├── turtlebot3_example
│   ├── action
│   ├── launch
│   ├── nodes
│   ├── rviz
│   └── src
│       └── turtlebot3_example
├── turtlebot3_gazebo_plugin
│   ├── build
│   │   └── CMakeFiles
│   │       ├── 3.5.1
│   │       │   ├── CompilerIdC
│   │       │   └── CompilerIdCXX
│   │       ├── image_listener.dir
│   │       │   └── src
│   │       ├── lidar_listener.dir
│   │       │   └── src
│   │       └── turtlebot3.dir
│   │           └── src
│   ├── models
│   │   ├── turtlebot3_burger
│   │   │   └── meshes
│   │   ├── turtlebot3_house
│   │   ├── turtlebot3_waffle
│   │   │   └── meshes
│   │   ├── turtlebot3_waffle_pi
│   │   │   └── meshes
│   │   └── turtlebot3_world
│   │       └── meshes
│   ├── src
│   └── worlds
├── turtlebot3_msgs
│   └── msg
├── turtlebot3_navigation
│   ├── launch
│   ├── maps
│   ├── param
│   └── rviz
├── turtlebot3_simulations
│   ├── turtlebot3_fake
│   │   ├── include
│   │   │   └── turtlebot3_fake
│   │   ├── launch
│   │   ├── rviz
│   │   └── src
│   ├── turtlebot3_gazebo
│   │   ├── include
│   │   │   └── turtlebot3_gazebo
│   │   ├── launch
│   │   ├── models
│   │   │   ├── turtlebot3_autorace
│   │   │   │   ├── ground_picture
│   │   │   │   ├── traffic_bar_down
│   │   │   │   │   └── materials
│   │   │   │   │       ├── scripts
│   │   │   │   │       └── textures
│   │   │   │   ├── traffic_bar_up
│   │   │   │   │   └── materials
│   │   │   │   │       ├── scripts
│   │   │   │   │       └── textures
│   │   │   │   ├── traffic_light_green
│   │   │   │   ├── traffic_light_red
│   │   │   │   ├── traffic_light_yellow
│   │   │   │   ├── traffic_parking
│   │   │   │   │   └── materials
│   │   │   │   │       ├── scripts
│   │   │   │   │       └── textures
│   │   │   │   ├── traffic_stop
│   │   │   │   │   └── materials
│   │   │   │   │       ├── scripts
│   │   │   │   │       └── textures
│   │   │   │   └── traffic_tunnel
│   │   │   │       └── materials
│   │   │   │           ├── scripts
│   │   │   │           └── textures
│   │   │   ├── turtlebot3_burger
│   │   │   │   └── meshes
│   │   │   ├── turtlebot3_house
│   │   │   ├── turtlebot3_plaza
│   │   │   │   └── goal_box
│   │   │   ├── turtlebot3_square
│   │   │   │   └── goal_box
│   │   │   ├── turtlebot3_waffle
│   │   │   │   └── meshes
│   │   │   ├── turtlebot3_waffle_pi
│   │   │   │   └── meshes
│   │   │   └── turtlebot3_world
│   │   │       └── meshes
│   │   ├── rviz
│   │   ├── src
│   │   └── worlds
│   └── turtlebot3_simulations
├── turtlebot3_slam
│   ├── bag
│   ├── config
│   ├── include
│   │   └── turtlebot3_slam
│   ├── launch
│   ├── rviz
│   └── src
└── turtlebot3_teleop
    ├── launch
    ├── nodes
    └── src
        └── turtlebot3_teleop

```