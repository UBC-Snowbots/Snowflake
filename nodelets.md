
# List all declared nodelets (remember to source devel/setup.sh for self-written nodelets)
rosrun nodelet declared_nodelets

# List all nodelet xml declaration files (using pluginlib)
rospack plugins --attrib=plugin nodelet

# Running instance of nodelet manager:

rosrun nodelet nodelet manager __name:=nodelet_manager

# Running a nodelet
rosrun nodelet nodelet load package_name/registered_nodelet_name name_of_nodelet_manager __name:=nodelet1 remapped_topic/abc:=remapped_to _rosparam_name:=param_value

# Roslaunch implementation:

```
<launch>
  <node pkg="nodelet" type="nodelet" name="standalone_nodelet"  args="manager"/>

  <node pkg="nodelet" type="nodelet" name="Plus"
        args="load nodelet_tutorial_math/Plus standalone_nodelet">
    <remap from="/Plus/out" to="remapped_output"/>
  </node>
  <rosparam param="Plus2" file="$(find nodelet_tutorial_math)/plus_default.yaml"/>
  <node pkg="nodelet" type="nodelet" name="Plus2" args="load nodelet_tutorial_math/Plus standalone_nodelet">
    <rosparam file="$(find nodelet_tutorial_math)/plus_default.yaml"/>
  </node>
  <node pkg="nodelet" type="nodelet" name="Plus3" args="standalone nodelet_tutorial_math/Plus">
    <param name="value" type="double" value="2.5"/>
    <remap from="Plus3/in" to="Plus2/out"/>
  </node>
</launch>
```

# to show nodelets running

rosnode list



