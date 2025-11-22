# lazy_ros2_tools
ros2 quick scripts for create pkg, colcon build and install. etc.

It can make you easier to operate ros2 commands. 

Put these shell(.sh) files to your workspace directories(e.g. ros_ws)

1. when you want to "ros2 pkg create", juse use 

```
zhuguangsheng@zhuguangsheng-ThinkPad-L14-Gen-5:~/Documents/ros2/ros2_ws$ ./lazy_pkg_create.sh 
==========================================
      ROS2 包创建助手
==========================================
您要保证在工作目录例如ros_ws,我将要进入src目录
请输入包名: wowpy

请选择构建类型:
1) ament_python (Python)
2) ament_cmake (C++)
请输入选择 (1 或 2): 1

默认依赖: rclpy
请输入额外的依赖项（用空格分隔，直接回车跳过）: 

请输入节点名 [默认: wowpy]: wownode

请输入维护者姓名 [默认: ROS Developer]: 
请输入维护者邮箱 [默认: rosdev@example.com]: 

请输入包描述 [默认: no description]: 

==========================================
           创建参数汇总
==========================================
包名: wowpy
构建类型: ament_python
依赖项: rclpy
节点名: wownode
维护者: ROS Developer <rosdev@example.com>
描述: no description
许可证: Apache-2.0
==========================================
是否创建包？(y/n): y

正在创建 ROS2 包...
==========================================
going to create a new package
package name: wowpy
destination directory: /home/zhuguangsheng/Documents/ros2/ros2_ws/src
package format: 3
version: 0.0.0
description: no description
maintainer: ['ROS Developer <rosdev@example.com>']
licenses: ['Apache-2.0']
build type: ament_python
dependencies: ['rclpy']
node_name: wownode
creating folder ./wowpy
creating ./wowpy/package.xml
creating source folder
creating folder ./wowpy/wowpy
creating ./wowpy/setup.py
creating ./wowpy/setup.cfg
creating folder ./wowpy/resource
creating ./wowpy/resource/wowpy
creating ./wowpy/wowpy/__init__.py
creating folder ./wowpy/test
creating ./wowpy/test/test_copyright.py
creating ./wowpy/test/test_flake8.py
creating ./wowpy/test/test_pep257.py
creating ./wowpy/wowpy/wownode.py

==========================================
✅ 包创建成功！
==========================================
包位置: /home/zhuguangsheng/Documents/ros2/ros2_ws/src/wowpy

下一步操作:
1. cd wowpy
2. 编辑节点代码
3. colcon build --packages-select wowpy
4. ros2 run wowpy wownode

结束
```


2. when you want to "ros2 colcon build", juse use  

```
zhuguangsheng@zhuguangsheng-ThinkPad-L14-Gen-5:~/Documents/ros2/ros2_ws$ ./lazy_colcon_build_quick.sh 
未指定目录参数，正在自动寻找src目录...
找到src目录: /home/zhuguangsheng/Documents/ros2/ros2_ws/src
找到以下子目录:
==================
 1) learnurdf
 2) wowcpp
 3) my_first_pkg
 4) wowpy
 5) my_py_pack
 6) slamware_ros2_sdk_linux-x86_64-gcc9
 0) 编译所有包 (all)
==================
请选择要编译的包编号 (0-6): 4
开始编译包: wowpy
Starting >>> wowpy   
Finished <<< wowpy [0.58s]          

Summary: 1 package finished [0.72s]
编译成功完成!

```

3. when you wnat to use source install/setup.sh, just use
```
zhuguangsheng@zhuguangsheng-ThinkPad-L14-Gen-5:~/Documents/ros2/ros2_ws$ ./lazy_install_quick.sh 
calling source install/setup.bash
done.

```

Best wishes for you.
祝你提高效率早点下班

