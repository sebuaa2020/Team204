自定义消息使用方法:

使用其他包内自定义消息

1. 在CMakefile.txt中的find_package中添加自定义消息所在的包名.即wpr_msgs

2. 在package.xml中添加:

   <build_depend>wpr_msgs</build_depend>
   <run_depend>wpr_msgs</run_depend>

详情参加[博客](https://blog.csdn.net/wuguangbin1230/article/details/78567416).