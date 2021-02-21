# ROS2Plugin

### Out of Tree Compilation

Out of tree compilation is now the recommended procedure to compile SOFA plugins in the Mimesis team.

It avoid dependency conflicts between your plugin, SOFA and other plugins compiled in-tree with SOFA.

An out-of-tree plugin has its own build folder independent from SOFA's.

```
cd $SOFA_BUILD_DIRECTORY
mkdir ROS2plugin && cd ROS2plugin
cmake $SOFA_WORK_DIRECTORY/ROS2plugin
make -j
make install
```

Finally, our new plugin must be added into SOFA's search path.
We can do it using a single symbolic link creation:
```
ln -s $SOFA_BUILD_DIRECTORY/ROS2plugin/install $SOFA_BUILD_DIRECTORY/sofa/install/plugins/ROS2plugin
```

#### Troubleshoot

If your SOFA installation is not yet in your library path, you can either export it:
```
export SOFA_ROOT=$SOFA_BUILD_DIRECTORY/sofa/install
export PATH=$PATH:$SOFA_ROOT/bin
```

Or inform its path in the CMake call

```
cmake $SOFA_WORK_DIRECTORY/ROS2plugin -DCMAKE_PREFIX_PATH=$SOFA_BUILD_DIRECTORY/sofa/install
```

This last step only needs to be performed once, as long as the installation paths remain the same


#### Workspace Organization

The usual SOFA project workspace is structured as it follows:

```
.
├── src                             ~ [alias SOFA_WORK_DIRECTORY]
│   ├── sofa
│   ├── sofaconfig
│   │
│   │   ... (other plugins)
│   │
│   └── ros2plugin
├── build                           ~ [alias SOFA_BUILD_DIRECTORY]
│   ├── sofa
│   │
│   │   ... (other out-of-tree plugins)
│   │
│   └── ros2plugin
```