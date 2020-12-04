# Robotest-MIDLAYER
## 机器人端相关代码

### 自定义的ros消息:
Robotest-MIDLAYER/robot/src/robot_port/msg:
>point_2d:  
>    float64 x  
>    float64 y

>path:  
>    point_2d[] p

>path_ori:  
>    int32 start_time  
>    int32 end_time  
>    point_2d[] p

>posi:  
>    int32 stamp  
>    float64 x  
>    float64 y  
>    float64 vx  
>    float64 vy  
>    float64 angle

>voice_cmd:  
>    int32 stamp  
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

### 相关topics:
* virtual_map: (从控制端)收到初始化的地图配置信息后发布到这个topic
* posi: 要发送位置等信息到控制端和AR端，请发布到这个topic
* path_ori: 发送识别得到的路径到控制端，请发布到这个topic
* path: 发送修正后的实际路径到控制端，请发布到这个topic
* voice_cmd: 发送语音识别得到的指令，请发布到这个topic
* drive_cmd: 接受键盘开车的指令
* dst: 识别得到的目标点，传给navi节点
* response_to_ctrl: 往控制端回传基本状态: OK = 0; ERROR = 1; FINISHED = 2;

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
>move test: 发送一个预置的目的地到navi节点，让机器人移动过去  
>move to: 回车后会要求输入一个x, y坐标，然后将其作为目的地发送至navi节点
>status: 返回当前robot和exp的状态。

### 参数:
#### robot_status: 机器人的状态，有4种: 
>sleeping: 目前没有实验。  
>initializing: 正在启动一次实验。此时主要完成加载地图的操作。  
>running: 正在进行实验。这是最主要的状态。其更细化的状态为exp_status

#### exp_status: 实验中的状态
>waiting: 等待指令。  
>recogniting: 指令识别中（手势识别）。此状态与moving状态必须严格分离，因为要杜绝手势识别和移动同时发生的情况。  
>moving: 执行导航等任务，正在移动中。此状态和上一个状态之间不能直接转换，只能先转换到wating状态。

#### robot_radius: 机器人的半径（单位为cm）
#### grid_size: 将地图柵格化处理时格子的尺寸（单位为cm）

