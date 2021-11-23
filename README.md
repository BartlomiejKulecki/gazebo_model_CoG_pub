# Gazebo model plugin - center of gravity publisher

To build run:  
```
cd gazebo_model_cog_pub
mkdir build && cd build
cmake ..
make
```

In order to use put line:
```
<plugin name="model_cog" filename="libmodel_cog.so"/>
```
in <model> section in your gazebo model sdf file.