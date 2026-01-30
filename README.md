# TAUV Drivers Structure

### Folder Layout

* **tauv_drivers/** (Root)
    * Top-level ROS 2 package directory. Contains `package.xml` and `CMakeLists.txt`.
* **tauv_drivers/** (Inner Folder)
    * The Python module. This allows scripts to share code. must be named this cuz python is a goofy goober
    * **__init__.py**: Required empty file that tells Python this folder is an importable package.
* **scripts/**
    * Python executables and ROS 2 nodes. These are the files called by `ros2 run`.
* **src/**
    * C++ source files (.cpp).
* **include/tauv_drivers/**
    * C++ header files (.hpp).
* **launch/***
    * Files used to start multiple nodes at once.
* **OLD/**
    * Old drivers that may seem useful later but i dont want to deel with them rn
---