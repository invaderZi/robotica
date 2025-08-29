# Guia de Execução - Prática 1 ROS 2

##

### 🔹 PARTE A - Publisher/Subscriber

```bash
cd PARTE_A
colcon build --packages-select cpp_pubsub

# Terminal 1:
source install/setup.bash
ros2 run cpp_pubsub talker

# Terminal 2:
source install/setup.bash
ros2 run cpp_pubsub listener
```

### 🔹 PARTE B - Service/Client

```bash
cd PARTE_B
colcon build --packages-select cpp_srvcli

# Terminal 1:
source install/setup.bash
ros2 run cpp_srvcli server

# Terminal 2:
source install/setup.bash
ros2 run cpp_srvcli client 2 3
```

### 🔹 PARTE C - Interfaces/Parâmetros/Plugins

```bash
cd PARTE_C

# C.1 - Interfaces
colcon build --packages-up-to more_interfaces

# Terminal 1:
source install/setup.bash
ros2 run more_interfaces publish_address_book

# Terminal 2:
source install/setup.bash
ros2 topic echo /address_book

# C.2 - Parâmetros
colcon build --packages-select cpp_parameters

# Terminal 1:

source install/setup.bash
ros2 run cpp_parameters minimal_param_node

# Terminal 2

source install/setup.bash
ros2 param set /minimal_param_node my_parameter MUDOU

# C.3 - Plugins
colcon build --packages-select polygon_base polygon_plugins
source install/setup.bash
ros2 run polygon_base area_node
```

### 🔹 PARTE D - Actions

```bash
cd PARTE_D

# D.1 - Interface
colcon build --packages-select action_tutorials_interfaces
source install/setup.bash
ros2 interface show action_tutorials_interfaces/action/Fibonacci

# D.2 - Implementação
colcon build --packages-select action_tutorials_cpp

# Terminal 1:

source install/setup.bash
ros2 run action_tutorials_cpp fibonacci_action_server

# Terminal 2:
source install/setup.bash
ros2 run action_tutorials_cpp fibonacci_action_client
```

### 🔹 PARTE E - Launch Files

```bash
cd PARTE_E

# E.1 - Turtlesim

# Terminal 1:

ros2 launch launch/turtlesim_mimic_launch.py

# Terminal 2:

ros2 topic pub -r 1 /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}"

# Terminal 3:
rqt_graph



# E.2 - Launch Integrado
colcon build
source install/setup.bash
ros2 launch cpp_launch_example my_script_launch.py
```

### 🔹 PARTE F - Event Handlers

```bash
cd PARTE_F
colcon build

# F.1 - Substituições
# Terminal 1:

source install/setup.bash
ros2 launch launch_tutorial example_main.launch.py
# Terminal 2:

source install/setup.bash
ros2 launch launch_tutorial example_substitutions.launch.py --show-args

ros2 launch launch_tutorial example_substitutions.launch.py turtlesim_ns:='turtlesim3' use_provided_red:='True' new_background_r:=200


# F.2 - Eventos
ros2 launch launch_tutorial example_event_handlers.launch.py turtlesim_ns:='turtlesim3' use_provided_red:='True' new_background_r:=200
```

## Importante

- Execute `source install/setup.bash` após cada build
