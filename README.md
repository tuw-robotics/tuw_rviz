# tuw_rviz
package with rviz plugins to support tuw_msgs

## tuw_pose_rviz_plugins
This plugin is a copy of the common rviz pose plugin and can be used as a template for new plugins.


## run
```
ros2 run tuw_graph_server graph_server_node --ros-args --params-file ./ws02/src/tuw_graph/tuw_graph_server/config/graph_server/cave.yaml
ros2 lifecycle set /graph_server configure; sleep 1; ros2 lifecycle set /graph_server activate
```
