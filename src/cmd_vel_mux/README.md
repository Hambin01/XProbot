<!--
 * @Author: hambin
 * @Date: 2023-03-10 13:40:12
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2023-04-04 13:38:02
 * @FilePath: can_to_joy/README.md
 * @Description: 
-->
# __cmd_vel_mux  package 说明__
## 1.概要
> 完成多个`geometry_msgs/Twist`消息类型的速度融合和优先级控制；   
> 完成控制器`sensor_msgs::Joy`消息类型到`geometry_msgs/Twist`消息类型的转换；
## 2. 配置说明
> 配置参数见`params.yaml`文件：
```
cmd_nodes:
  - topic_name:  "can1"                     #速度话题名
    timeout:     3                          #释放时间间隔
    priority:    1                          #优先级
    short_desc:  "controllers cmd_vel "     #描述信息

  - topic_name:  "homing_cmd_vel"
    timeout:     3
    priority:    10
    short_desc:  "Navigation stack controller"

  - topic_name:  "/dev/input/js0"
    timeout:     3
    priority:    4
    short_desc:  "F710 joy"

  - topic_name:  "keyboard_cmd_vel"
    timeout:     3
    priority:    0
    short_desc:  "keyboard"

  - topic_name:  "filter_cmd_vel"
    timeout:     3
    priority:    2
    short_desc:  "filter"

  - topic_name:  "web"
    timeout:     3
    priority:    20
    short_desc:  "web"

publisher:       "cmd_vel_rectified"


joy_types:
  - joy_frame: "/dev/input/js0"
    max_linear_speed: 0.5
    max_angular_speed: 0.005
  - joy_frame: "/can1"
    max_linear_speed: 0.5
    max_angular_speed: 0.005
  - joy_frame: "/web"
    max_linear_speed: 0.5
    max_angular_speed: 0.005

```
`cmd_nodes`：需要加入的速度topic, `priority`值越小优先级越高，高优先级可抢断低优先级，最好不要配置同样等级的优先级，对应的优先级停止控制超过`timeout`时间才会释放控制权，`publisher`是输出的速度`toipc`，不可与`cmd_nodes`中的`topic_name`重名；   

`joy_types`：需要转换的`sensor_msgs::Joy`消息类型，`frame`是topic的frame,将转换成名称为frame的`geometry_msgs/Twist`类型topic；   
`max_linear_speed`:最大线速度   
`angular_speed`:最大角速度

## 2.joy按键功能说明
> __F710__（X模式）:
>```
>LT+左摇杆：底盘前进后退(axes[2]+axes[1])
>LT+右摇杆：转弯(axes[2]+axes[3])
>RT+A+左摇杆：机械臂1动作(axes[5]+button[0]+axes[1])
>RT+B+左摇杆：机械臂2动作(axes[5]+button[1]+axes[1])
>RT+X+左摇杆：机械臂3动作(axes[5]+button[2]+axes[1])
>RT+Y+左摇杆：机械臂4动作(axes[5]+button[3]+axes[1])
>RT+LB+左摇杆：机械臂5动作(axes[5]+button[4]+axes[1])
>RT+RB+左摇杆：机械臂6动作(axes[5]+button[5]+axes[1])
>BACK：急停
>```   
> __can遥控器__(joy键值同上):
>```
>底盘+左摇杆+备用1：底盘前进后退
>底盘+右摇杆+备用1：转弯
>机械臂+1+左摇杆+备用1：机械臂1动作
>机械臂+2+左摇杆+备用1：机械臂2动作
>机械臂+3+左摇杆+备用1：机械臂3动作
>机械臂+4+左摇杆+备用1：机械臂4动作
>机械臂+5+左摇杆+备用1：机械臂5动作
>机械臂+6+左摇杆+备用1：机械臂6动作
>BACK：急停
>``` 

## 3. 运行
>roslaunch  cmd_vel_mux   cmd_vel_mux.launch