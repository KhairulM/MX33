# Installing Libs
## Librealsense

> #### For Ubuntu 24.04
> As of 22 May 2025 we used v2.56.3. In the librealsense directory do `git checkout v2.56.03` first

Follow this instruction https://dev.intelrealsense.com/docs/compiling-librealsense-for-linux-ubuntu-guide. Be sure to build the **release** version on target machine.

## Libzmq

> #### Message Encryption
> The message encryption used here is [CurveZMQ](http://curvezmq.org/). To build ZMQ with encryption enabled you need to install [libsodium](https://doc.libsodium.org/installation) and do `cmake -DWITH_LIBSODIUM=ON -DENABLE_CURVE=ON ..`

```bash
cd lib/libzmq
mkdir build && cd build
cmake ..
make -j$(($(nproc)-1))
sudo make install
```

## Cppzmq

```bash
cd lib/cppzmq
mkdir build && cd build
cmake ..
make -j$(($(nproc)-1))
sudo make install
```

## MsgPack
No need to build, its a header only library. Just be sure to clone the right branch (**cpp_master**)

## Libpcl

```bash
sudo apt install libpcl-dev
```

## Octomap
```bash
cd lib/octomap
mkdir build && cd build
cmake ..
make -j$(($(nproc)-1))
sudo make install
```

# Building the Project
This repository contains implementations for:
- **Broker**: Message broker for pub/sub and service registry
- **Map Server**: Combines pointclouds from multiple robots into a global map
- **Robot**: Client that registers with map server and publishes pointcloud data
- **Demo programs**: Examples for camera publisher/subscriber and service client/server

To build all components:
```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

This will create the following executables in the `build/` directory:
- `broker` - Message broker
- `map_server` - Global map server
- `robot` - Robot client
- `camera_publisher`, `camera_subscriber` - Camera demo programs
- `demo_service_server`, `demo_service_client` - Service demo programs

# Running the System

## 1. Start the Broker
The broker handles message routing and service discovery. Run it first:

```bash
./build/broker [frontend_port] [backend_port] [service_lookup_port] [service_add_port]
```

**Default ports:**
- Frontend (publishers): 5555
- Backend (subscribers): 5556
- Service lookup: 5557
- Service add: 5558

**Example:**
```bash
./build/broker
# Or with custom ports:
./build/broker 5555 5556 5557 5558
```

## 2. Start the Map Server
The map server receives pointcloud data from robots and combines them into a global map:

```bash
./build/map_server [broker_address] [transformation_file] [broker_public_key]
```

**Parameters:**
- `broker_address`: Broker address (default: `tcp://localhost:5555`)
- `transformation_file`: File containing robot-to-global transformations (default: `global_to_local_tf.txt`)
- `broker_public_key`: Path to broker's public key for encryption (optional)

**Example:**
```bash
./build/map_server tcp://localhost:5555 global_to_local_tf.txt
```

**Transformation File Format (`global_to_local_tf.txt`):**
Each line represents the local-to-global transformation for a robot:
```
robot_id x y z qx qy qz qw
```
Where `(x, y, z)` is translation and `(qx, qy, qz, qw)` is rotation quaternion.

**Example:**
```
robot_1 0.0 0.0 0.0 0.0 0.0 0.0 1.0
robot_2 5.0 0.0 0.0 0.0 0.0 0.0 1.0
robot_3 0.0 5.0 0.0 0.0 0.0 0.707107 0.707107
```

If the file doesn't exist, identity transforms are used for all robots.

## 3. Start Robot Client(s)
Each robot registers with the map server and publishes pointcloud data:

```bash
./build/robot [robot_id] [broker_address] [broker_public_key]
```

**Parameters:**
- `robot_id`: Unique identifier for the robot (default: `robot_1`)
- `broker_address`: Broker address (default: `tcp://localhost:5555`)
- `broker_public_key`: Path to broker's public key for encryption (optional)

**Examples:**
```bash
# Terminal 1: Robot 1
./build/robot robot_1 tcp://localhost:5555

# Terminal 2: Robot 2
./build/robot robot_2 tcp://localhost:5555

# Terminal 3: Robot 3
./build/robot robot_3 tcp://localhost:5555
```

## Complete Workflow Example

```bash
# Terminal 1: Start broker
cd /path/to/MX33
./build/broker

# Terminal 2: Start map server
./build/map_server tcp://localhost:5555 global_to_local_tf.txt

# Terminal 3: Start first robot
./build/robot robot_1 tcp://localhost:5555

# Terminal 4: Start second robot
./build/robot robot_2 tcp://localhost:5555

# Terminal 5: Start third robot (optional)
./build/robot robot_3 tcp://localhost:5555
```

The map server will:
1. Register each robot as it connects
2. Receive pointcloud data from all robots
3. Transform each robot's pointcloud to the global frame
4. Combine all pointclouds into a unified global map
5. Update the global map continuously at 5 Hz

## Message Encryption (Optional)

To enable message encryption with CurveZMQ:
1. Build libzmq with libsodium support (see installation instructions above)
2. Generate key pairs for broker and clients
3. Pass the broker's public key path when starting map server and robots

**Note:** The current robot implementation generates dummy pointcloud data for testing. In production, integrate with actual sensors (e.g., Intel RealSense cameras).