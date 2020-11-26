# python grpc

## 编译proto文件

### 配置

+ ~~首先下载protoc编译器: [github地址]([Release Protocol Buffers v3.14.0 · protocolbuffers/protobuf (github.com)](https://github.com/protocolbuffers/protobuf/releases/tag/v3.14.0))~~

+ ~~将解压目录下的bin目录加入到环境变量path~~
+ 安装grpc的依赖包

```bash
pip install grpcio
pip install protobuf
pip install grpcio-tools
```

### 编译命令

```bash
python -m grpc_tools.protoc --python_out=. --grpc_python_out=. -I. msg.proto
```

### 编译结果

在相应目录下生成两个文件:

+ 数据定义文件`msg_pb2.py`
+  GRPC服务文件`msg_pb2_grpc.py`

## 机器人端接收数据的GRPC Server

`robot_server.py`

机器人可接收的消息有:

+ 控制端的命令消息: Connect, Start, Stop
+ AR端的语音消息: 流式传输
+ 控制端的配置地图

请实现以下函数对应消息的响应, 其中request变量即为从其它端传递过来的数据, 函数中也给出了相应数据结构的操作方法示例.

+ `ConfigMap`

+ `ControlCommand`
+ `SendVoiceFile`

机器人接收的语音为str类型的二进制字符串

语音消息的接收与发送可能会存在问题. 但其中的框架不会发生变化.

在使用时请开辟一子线程运行该服务.

启动服务的函数为`server()`

接收语音数据由于是流式传输, 所以`bytestr`只是一部分二进制. 将其全部加在一起才是完整的. protobuf传输的是bytes类型的消息, 而根据其官方文档中所说python对应的数据结构为str, 但目前并不知晓如何将该str转为二进制数据写入语音文件.

## 机器人发送数据

`robot_client.py`

在使用前请修改ip地址 默认端口8888, 若修改端口号请提前告知其它端

`sendRBPosition`发送机器人位置数据

`sendRBPath`发送机器人路径识别数据

`sendVoiceResult`发送语音识别结果

`sendRobotFinishedMsg`发送停止实验后机器人回归到原点的finished信息

具体使用方法请参见相应函数的注释