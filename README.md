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
* send_rb_posi: 要发送位置等信息到控制端和AR端，请发布到这个topic
* send_rb_path_ori: 发送识别得到的路径到控制端，请发布到这个topic
* send_rb_path: 发送修正后的实际路径到控制端，请发布到这个topic
* send_rb_voice_cmd: 发送语音识别得到的指令，请发布到这个topic

### 节点：
Robotest-MIDLAYER/robot/src/robot_port/scripts:
* communicater.py:负责与控制端和AR端通讯的节点。

