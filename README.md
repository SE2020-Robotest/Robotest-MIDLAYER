# Robotest-MIDLAYER
## 机器人端相关代码

### 运行流程:
#### 编译:
下载源文件之后，在Robotest-MIDLAYER/robot目录下用catkin_make编译。  
在该目录下运行:
```
source devel/setup.bash
```
#### 测试:
```
roscore  
roslaunch turtlebot_gazebo turtlebot_world.launch  
roslaunch robot_port test.launch #测试
```
#### 运行:
```
roscore  
roslaunch turtlebot_bringup minimal.launch  
sudo chmod a+rw /dev/ttyUSB0  
roslaunch turtlebot_navigation gmapping_demo.launch #和上一条在同一终端中执行。  
roslaunch robot_port robot_port.launch #正式运行  
rosrun map_server map_saver -f ~/map #保存地图
```

### 自定义的ros消息:
Robotest-MIDLAYER/robot/src/robot_port/msg:
>point_2d:  
>    float64 x  
>    float64 y

>path:  
>    point_2d[] p

>path_ori:  
>    float64 start_time  
>    float64 end_time  
>    point_2d[] p

>posi:  
>    float64 stamp  
>    float64 x  
>    float64 y  
>    float64 vx  
>    float64 vy  
>    float64 angle

>voice_cmd:  
>    float64 stamp  
>    string cmd

>map_object:  
>    bool type 	# False means CUBE, True means CYLINDER  
>    float64 x  
>    float64 y  
>    float64 w  
>    float64 h

>vmap:  
>    float32 w  
>    float32 h  
>    map_object[] obj

>enum_type:  
>    int16 type

>response:  
>    float64 stamp  
>    string node  
>    string discription  
>    bool response

>stop:  
>    bool stop

### 相关topics:
* virtual_map: (从控制端)收到初始化的地图配置信息后发布到这个topic
* posi: 要发送位置等信息到控制端和AR端，请发布到这个topic
* path_ori: 发送识别得到的路径，communicater会将之发送至控制端，同时Navi节点会开始依此执行导航任务
* path: 发送修正后的实际路径到控制端，请发布到这个topic
* voice_cmd: 发送语音识别得到的指令，请发布到这个topic
* xfspeech: 迅飞语音识别，识别结果会发布到这个topic
* drive_cmd: 接受键盘开车的指令
* dst: 识别得到的目标点，传给navi节点
* response_to_ctrl: 往控制端回传基本状态: OK = 0; ERROR = 1; FINISHED = 2
* log_msg: 要记录到日志中的输出发布到该topic，将记录到total_log文件中，并且将发送到控制端

### 节点:
Robotest-MIDLAYER/robot/src/robot_port/scripts:
* communicater: 负责与控制端和AR端通讯的节点。
* navi_node: 负责自动寻路，修正路径，导航，驾驶机器人。
* posi_publisher: 负责发布位置信息给控制端。
* drive: 负责处理简单的移动指令，比如旋转/前进/后退（键盘开车）。
* mark_vmap: 负责标记虚拟地图对应坐标系。标记时，先保证机器人移动至原点，然后机器人前方朝向x周正方向，然后单独运行该节点。所有涉及坐标的部分都依赖该节点标记的坐标系。
* fake_cmd: 通过命令行输入，发布一些命令。用于测试:
>旋转/spin: 开始旋转  
>停止旋转/stop spinning  
>comm test  
>start exp: 强制将状态设置为initializing  
>run exp: 强制将状态设置为running  
>stop exp: 强制将状态设置为sleeping  
>load map: 加载预置的一张地图   
>move_dst test: 发送一个预置的目的地到navi节点，让机器人移动过去  
>move_path test: 发送一段预置的路径到navi节点，让机器人沿路径移动过去。最好先保证机器人位于地图的右上角  

>move to: 回车后会要求输入一个x, y坐标，然后将其作为目的地发送至navi节点  
>status: 返回当前robot和exp的状态。
* test: 测试模块。请勿直接运行本节点，需要其他节点运行才能进行测试。测试时请调用test.launch文件。
* voice_pub: 将xfspeech中的语音结果简单处理，并识别其中的指令，将有效指令发布到voice_cmd话题中。

### 其他相关python文件:
* enum_list: 统一管理程序中的一些重要参数、常数，包括语音识别的指令。
* grid_graoh: 柵格图。用于navi节点中的加载地图（柵格化）以及寻路。
* log: 记录日志。包括将日志打印到屏幕，发布到log_msg话题，以及记录到文件中。
* my_pq: 优先级队列。在Dijkstra寻路中需要用到。
* status: 统一管理实验的所有状态，以及相关状态的转移。
* trans: 获取位姿速度信息；实现坐标、向量等在世界坐标系、机器人中心坐标系、虚拟地图坐标系间的转换。

### 参数:
* robot_status: 机器人的状态，有4种: 
>sleeping: 目前没有实验。  
>initializing: 正在启动一次实验。此时主要完成加载地图的操作。  
>running: 正在进行实验。这是最主要的状态。其更细化的状态为exp_status

* exp_status: 实验中的状态
>waiting: 等待指令。  
>recogniting: 指令识别中（手势识别）。此状态与moving状态必须严格分离，因为要杜绝手势识别和移动同时发生的情况。  
>moving: 执行导航等任务，正在移动中。此状态和上一个状态之间不能直接转换，只能先转换到wating状态。

* connected: 指示是否与控制端建立连接
* TEST_MODE: True代表测试模式，将不会实际想外发送消息。False代表不是测试模式。
* base_frame: 定位时作为依据的tf帧，备选项为/odom, /map。即依据里程计定位还是gmapping定位。
* user_posi: 预设的用户位置。由于时间有限，我们预设好用户的坐标。如果有能力，之后可以修改为自动识别，具体接口只需修改navi_node中get_posi方法即可。

### launch文件:
* robot_port.launch: 实际运行时请launch本文件
* test.launch: 测试robot_port包时请launch本文件

### 未来:
该项目目前可能无法实现手势路径导航，但是导航模块中已经实现相关功能，未来如有实现该功能的计划只需将识别得到的路径发送到path_ori话题即可。  
注意，坐标必须是相对于虚拟地图坐标系的，如果得到的坐标是相对于机器人的坐标，可以用trans模块中的方法进行坐标转换。
