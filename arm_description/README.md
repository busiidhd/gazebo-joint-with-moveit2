# 24赛季国赛工程机械臂结构描述文件

`main` 分支为24赛季国赛工程

## 可视化

```bash
# 提前进入机械臂描述文件urdf的工作环境
# 如 机械臂描述文件urdf 在 ~/Desktop/arm_ws 路径编译
# source ~/Desktop/arm_ws/install/setup.bash 

ros2 launch arm_description display.launch.py 
```
## 统一规定

SolidWorks 装配为机械限位至不可移动，以便可以在实物上复现图纸上的装配，用作确定电机零点。

关节从 0 开始依次命名为 `joint0`，`joint1`，...，`joint5`，连杆同理`link0`，`link1`，...`link5`

相机坐标系命名为 `camera_link`，末端执行器命名为 `end_link`

若适配新机械臂，请指定描述文件包名为 `arm_description`，文件夹名可为 `xxx_arm_description` 以作区分，且新机械臂创建新分支上传。

基底坐标系（base_link）与末端执行器坐标系（end_link）方向指定为 z 轴朝上，x 轴朝前，y 轴操作，目的是与 PnP 3D 坐标系（world）方向一致

## SolidWorks 导出 URDF

[solidworks_urdf_exporter](https://github.com/ros/solidworks_urdf_exporter/releases)

个人使用v1.6.1 (SolidWorks 2021) + SolidWorks 2023

第一次导出的时候会出现Axis参数全为0的情况，Axis参数表示绕那个轴旋转或者移动，Axis参数全为0必定出错。

可以后期手动填写，但建议进行第二次导出，第二次导出Axis参数就会自动写上了。

同时，第二次导出也可以进行修正自动生成的坐标系，因为自动生成的坐标系不一定准确。特别是 `base_link` 要 z 轴朝上

导出的描述文件包是ros1的，只需要把 `meshes`，`urdf`，`textures` 复制出来，粘贴到新建的ros2包即可。新机械臂可以参考此包进行修改为ros2。

[在SolidWorks里修改模型并以URDF导出](https://zhuanlan.zhihu.com/p/616711291)

[Solidworks导出URDF模型](https://blog.csdn.net/lixushi/article/details/122471782)

[机械手Solidworks建模及其URDF文件导出](https://blog.csdn.net/joyopirate/article/details/129743734#t2)

[sw2urdf导出的urdf文件中的惯性参数（inertial）错误的问题](https://blog.csdn.net/joyopirate/article/details/131102160)

## Note

一般而言，`<robot name>`写成机械臂名称，如`arm`，那么描述文件包的命名就为 `arm_description`

`urdf` 文件的 `<robot name>` 不要写成与包名一致，不然 moveit 会导入不了 `SRDF`，出现以下报错

```bash
[rviz2-3] [ERROR] [1730362425.255181812] [rviz]: Could not find parameter robot_description_semantic and did not receive robot_description_semantic via std_msgs::msg::String subscription within 10.000000 seconds.
[rviz2-3] Error:   Could not parse the SRDF XML File. Error=XML_ERROR_EMPTY_DOCUMENT ErrorID=13 (0xd) Line number=0
[rviz2-3]          at line 732 in ./src/model.cpp
[rviz2-3] [ERROR] [1730362425.262467569] [moveit_rdf_loader.rdf_loader]: Unable to parse SRDF
[rviz2-3] [ERROR] [1730362425.285115586] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Robot model not loaded
```

`urdf` 文件的 `limit 中 velocity`，要加上小数点 `.000001`，不然会导致 moveit config 中的 `joint_limits.yaml` 的 `max_velocity` 生成为整数型，也会出现上述错误。

```xml
    <limit
      lower="0"
      upper="1.605702"
      effort="100.0"
      velocity="1.000001" />
```