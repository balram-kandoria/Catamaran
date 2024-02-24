# Autonomous Catamaran Project

## Catamaran

The hull shape references the [Pauline](https://drive.google.com/file/d/0B32-8ZWvIt6URWtMSzY5eFM0LTg/view?resourcekey=0-qiQcJYNRlNALtBKUkP0fuw), a centerboard sloop sailing yacht. The plans for the Pauline were found on [Ship Modell](http://www.shipmodell.com/index_files/0PLAN2B.html#5S)

## ASV Wave Simulation

See [ASV Wave Simulation](https://github.com/srmainwaring/asv_wave_sim) for more information about wave mechanics. 

### Install Martitime dependencies
```bash
sudo apt-get update
sudo apt-get install libcgal-dev libfftw3-dev
```
### Pull Maritime Module
```bash
git submodule update --init
```
### Compile package (Must be in /catamaran directory)
```bash
colcon build --symlink-install --merge-install --cmake-args \
-DCMAKE_BUILD_TYPE=RelWithDebInfo \
-DBUILD_TESTING=ON \
-DCMAKE_CXX_STANDARD=17
```

### Source the workspace
```bash
source ./install/setup.bash
```

### Optional GUI Plugin 
```bash
cd path/catmaran/src/asv_wave_sim/gz-waves/src/gui/plugins/waves_control 
mkdir build && cd build
cmake .. && make
```

### Set Environmental Variables 
```bash
# Recommend updating .bashrc

# for future use - to support multiple Gazebo versions
export GZ_VERSION=garden

# not usually required as should default to localhost address
export GZ_IP=127.0.0.1

# ensure the model and world files are found
export GZ_SIM_RESOURCE_PATH=\
$GZ_SIM_RESOURCE_PATH:\
$HOME/catmaran/src/asv_wave_sim/gz-waves-models/models:\
$HOME/catmaran/src/asv_wave_sim/gz-waves-models/world_models:\
$HOME/catmaran/src/asv_wave_sim/gz-waves-models/worlds

# ensure the system plugins are found
export GZ_SIM_SYSTEM_PLUGIN_PATH=\
$GZ_SIM_SYSTEM_PLUGIN_PATH:\
$HOME/catmaran/install/lib

# ensure the gui plugin is found
export GZ_GUI_PLUGIN_PATH=\
$GZ_GUI_PLUGIN_PATH:\
$HOME/catmaran/src/asv_wave_sim/gz-waves/src/gui/plugins/waves_control/build
```

### Start Maritime simulation
```bash
LIBGL_ALWAYS_SOFTWARE=1 gz sim waves.sdf
```