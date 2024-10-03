# SpaceMouse ROS2 Publisher

This project implements a ROS2 node that publishes SpaceMouse input as ROS2 messages. It uses the `pyspacemouse` library to interface with the SpaceMouse device and publishes the input as `geometry_msgs/Twist` messages.

## Prerequisites

- ROS2 (tested with Humble, but should work with other distributions)
- Python 3
- pyspacemouse library

## Node Details

- Node name: `spacemouse_publisher`
- Published topic: `/spacemouse_cmd`
- Message type: `geometry_msgs/Twist`

The node publishes the SpaceMouse input as follows:
- Linear motion (x, y, z) is published to the `linear` field of the Twist message
- Angular motion (roll, pitch, yaw) is published to the `angular` field of the Twist message

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contributing

Contributions to this project are welcome. Please feel free to submit a Pull Request.

## Issues

If you find any bugs or have feature requests, please file them in the [issue tracker](https://github.com/LumenYoung/spacemouse_ros2/issues).
