<!--
 * @Author: hambin
 * @Date: 2023-03-10 13:40:12
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2023-03-21 16:44:58
 * @FilePath: can_to_joy/README.md
 * @Description: 
-->
# __pose_init  package 说明__
## 1.概要
> 完成机器人位姿的保存和初始化
## 2. 配置说明
> 配置参数见`pose_init.launch`文件：
```
<launch>

    <rosparam file="$(find pose_init)/config/params.yaml" command="load" />
    <node pkg="pose_init" type="pose_init_node" name="pose_init_node" output="screen" respawn="true">
        <param name="pose_file" value="$(find pose_init)/config/params.yaml"/>
        <param name="save_period" value="1.0"/>
    </node>

</launch>
```
`pose_file`：位姿文件路径；
`save_period`: 位姿保存周期；



