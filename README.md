# ROS2 Components

This repository demonstrates the usage of ROS2 components. 

## Usage

```bash
usage: ros2 component [-h] Call `ros2 component <command> -h` for more detailed usage. ...

Various component related sub-commands

optional arguments:
  -h, --help            show this help message and exit

Commands:
  list        Output a list of running containers and components
  load        Load a component into a container node
  standalone  Run a component into its own standalone container node
  types       Output a list of components registered in the ament index
  unload      Unload a component from a container node

  Call `ros2 component <command> -h` for more detailed usage.
```

Listing the available components.
```bash
ros2 component types
```

Starting up the component manager.
```bash
ros2 run rclcpp_components component_container
```

Loading component talker to the component manager.
```bash
ros2 component load /ComponentManager ros2-components component::component_talker
```

Listing all loaded component
```bash
ros2 component list
```

Unloading component from component manager
```bash
ros2 component unload /ComponentManager 1
```

Running tmux script
```bash
ros2 run ros2-component start_lifecycle_demo.bash -w /your/path/to/workspace
```


## Reference
 - [ROSCON_Video](https://vimeo.com/378916125)
 - [foxy_tutorial](https://docs.ros.org/en/foxy/Tutorials/Composition.html)
 - [article](https://medium.com/@nullbyte.in/ros2-from-the-ground-up-part-4-building-robust-robotics-systems-with-ros2-composition-209ed023d8e5)
