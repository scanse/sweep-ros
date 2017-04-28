# sweep-ros

Scanse Sweep ROS Driver and Node

This node is currently publishing a pointcloud2 msg. Use sweep2scan.launch for conversion to laserscan msg.

## Compatibility:
Currently, sweep-ros is only compatible with sweep firmware v1.1 or greater.

You can check the firmware version installed on your sweep device by using a serial terminal (see [manual](https://s3.amazonaws.com/scanse/Sweep_user_manual.pdf)) or more easily using the sweep visualizer (see [instructions](https://support.scanse.io/hc/en-us/articles/224557908-Upgrading-Firmware)).