# This project is a example of gazebo joint with moveit2 (version:ROS2 humble)

>arm（机械臂）和exchanger（兑矿口）是两个不同的机械臂模型,由于exchanger（兑矿口）控制器没有做好，该部分无法被控制，只能先固定
（urdf和meshes文件已隐藏）
### 以下是使用gazebo和rviz2实例，可以通过rviz2控制gazebo模型的运动
![image](doc/gazebo.png)
![image](doc/rviz2.png)

### 可以在exchanger（兑矿口）模型中修改矿口的灯条颜色
方式：把灯条当做一个固定的link，修改它的gazebo属性即可
![image](doc/example.png)
![image](doc/blue_light.png)
