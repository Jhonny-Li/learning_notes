tf wiki链接：http://wiki.ros.org/tf#static_transform_publisher

命令行工具：
view_frames: visualizes the full tree of coordinate transforms.

tf_monitor: monitors transforms between frames.

tf_echo: prints specified transform to screen

roswtf: with the tfwtf plugin, helps you track down problems with tf.

static_transform_publisher is a command line tool for sending static transforms.

You may also wish to use the tf_remap node, which is a utility node for remapping coordinate transforms.

tf_monitor：将有关当前坐标变换树的信息输出到控制台。 例如：
$ rosrun tf tf_monitor
RESULTS: for all Frames

Frames:
Frame: /base_footprint published by /robot_pose_ekf Average Delay: 0.0469324 Max Delay: 0.0501503
Frame: /base_laser_link published by /robot_state_publisher Average Delay: 0.00891066 Max Delay: 0.009591
Frame: /base_link published by /robot_state_publisher Average Delay: 0.00891147 Max Delay: 0.009592
0.00891431 Max Delay: 0.009595

... editing for the sake of brevity ...

Broadcasters:
Node: /realtime_loop 94.7371 Hz, Average Delay: 0.000599916 Max Delay: 0.001337
Node: /robot_pose_ekf 30.8259 Hz, Average Delay: 0.0469324 Max Delay: 0.0501503
Node: /robot_state_publisher 25.8099 Hz, Average Delay: 0.0089224 Max Delay: 0.00960276



tf_monitor <source_frame> <target_target>监视特定的转换。 例如，监视从/ base_footprint到/ odom的转换：

$ rosrun tf tf_monitor /base_footprint /odom
RESULTS: for /base_footprint to /odom
Chain currently is: /base_footprint -> /odom
Net delay     avg = 0.00371811: max = 0.012472

Frames:
Frame: /base_footprint published by /robot_pose_ekf Average Delay: 0.0465218 Max Delay: 0.051754
Frame: /odom published by /realtime_loop Average Delay: 0.00062444 Max Delay: 0.001553

Broadcasters:
Node: /realtime_loop 95.3222 Hz, Average Delay: 0.00062444 Max Delay: 0.001553
Node: /robot_pose_ekf 30.9654 Hz, Average Delay: 0.0465218 Max Delay: 0.051754
Node: /robot_state_publisher 25.9839 Hz, Average Delay: 0.00903061 Max Delay: 0.00939562



tf_echo
tf_echo <source_frame> <target_frame>。打印有关source_frame和target_frame之间的特定转换的信息。 例如，回显/ map和/ odom之间的转换：

$ rosrun tf tf_echo /map /odom
At time 1263248513.809
- Translation: [2.398, 6.783, 0.000]
- Rotation: in Quaternion [0.000, 0.000, -0.707, 0.707]
in RPY [0.000, -0.000, -1.570]



static_transform_publisher
static_transform_publisher x y z yaw pitch roll frame_id child_frame_id period_in_ms。使用x/y/z偏移（米）和偏航/俯仰/滚转弧度将静态坐标变换发布到tf。（偏航是围绕Z旋转，俯仰是围绕Y旋转，横摇是围绕X旋转 弧度制）。周期（以毫秒为单位）指定发送转换的频率。100毫秒（10赫兹）是一个很好的数值。

static_transform_publisher x y z qx qy qz qw frame_id child_frame_id  period_in_ms。使用以米和四元数表示的x / y / z偏移量将静态坐标转换发布到tf。 周期（以毫秒为单位）指定发送转换的频率。 100ms（10hz）是一个不错的选择。
static_transform_publisher被设计为手动使用的命令行工具，也可以在roslaunch文件中用于设置静态转换。例如：


   1 <launch>
   2 <node pkg="tf" type="static_transform_publisher" name="link1_broadcaster" args="1 0 0 0 0 0 1 link1_parent link1 100" />
   3 </launch>



view_frames。view_frames是一个图形调试工具，可创建当前转换树的PDF图形。

例：$ rosrun tf view_frames
您可能希望在完成后查看图形，因此在Ubuntu系统上的典型用法是：
$ rosrun tf view_frames
$ evince frames.pdf

因此，添加.bashrc的有用快捷方式是：
alias tf='cd /var/tmp && rosrun tf view_frames && evince frames.pdf &'
注意：另请参见rqt_tf_tree，它允许对帧进行动态自检。

roswtf plugin ：roswtf 插件
roswtf tf带有roswtf插件，该插件在您运行roswtf时自动运行。 该插件将分析您当前的tf配置，并尝试查找常见问题。 要运行，只需正常调用roswtf：

$ roswtf



