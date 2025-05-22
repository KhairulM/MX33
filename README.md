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

# Building the Example
In this repository there is an example implementation on how to use the **Publisher**, **Subscriber**, and **Broker** class. To build these demos, follow these steps
```bash
# edit broker.cpp, camera_publisher.cpp, camera_subscriber.cpp if you don't want to enable message encryption
mkdir build && cd build
cmake ..
make
```