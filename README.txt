ros2 run moveit_setup_assistant moveit_setup_assistant
使用moveit_setup_assistant配置arm和exchanger

启动方法：
arm和gazebo仿真：启动arm_moveit_conig里面的launch的gazebo.launch.py，
然后启动moveit.launch.py。一定要遵守启动顺序。
其他都是默认原来的工程文件

参考资料：

https://blog.csdn.net/joyopirate/article/details/129424607
在ROS2中，通过MoveIt2控制Gazebo中的自定义机械手

【Moveit!+Gazebo仿真报错：
Unable to identify any set of controllers that can actuate the specified joints 】
https://blog.csdn.net/weixin_44836543/article/details/132523970?sharetype=blog&shareId=132523970&sharerefer=APP&sharesource=2403_88972552&sharefrom=link
注意：仿真的时候必须检查1：urdf文件可以在moveit规划（这个是moveit_setup_assistant生成的）
2：gazebo能显示机械臂，3,使用rqt检查节点有没有联系，否则会出现上面的问题