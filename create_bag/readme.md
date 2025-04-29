#  将文件夹数据（图片）转换成bag包py代码（工作空间已经帮你创建好了）
##  运行：
###  在workspaces/src文件下：
catkin_make

###  编译成功后运行：
catkin_create_pkg create_bag std_msg rospy roscpp

### 代码说明：
1、代码编译：catkin_make

2、第一个是运行编译的包名字叫create_bag 后面是调用相关的ros包

####  相关说明
1、已修改相关Cmakelists.txt与torosbag.cpp文件需要修改的位置
