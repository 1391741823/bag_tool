#  将文件夹数据（图片）转换成bag包py代码（工作空间已经帮你创建好了）

##  在workspaces/src文件下（编译部分）：
catkin_make

source devel/setup.bash

##  编译成功后
###  1、终端1运行：
roscore

###  2、终端2运行：
rosrun create_bag node

### 代码说明：
1、代码编译：catkin_make

2、第一个是运行编译的包名字叫create_bag 后面是调用相关的ros包

####  相关说明
1、已修改相关Cmakelists.txt与torosbag.cpp文件需要修改的位置
2、torosbag.cpp文件更新
