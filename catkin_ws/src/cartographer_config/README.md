# Cartographer for Racecar

For the most up to date instructions, please refer to the [official google cartographer documentation](https://google-cartographer-ros.readthedocs.io/en/latest/).

If cartographer is not already installed, [install cartographer](https://google-cartographer-ros.readthedocs.io/en/latest/):

    # Install ninja
    sudo apt-get install ninja-build python-wstool python-rosdep
    
    # Make a workspace for cartographer
    mkdir ~/cartographer_ws
    cd ~/cartographer_ws
    wstool init src
    
    # Fetch cartographer_ros
    wstool merge -t src https://raw.githubusercontent.com/googlecartographer/cartographer_ros/master/cartographer_ros.rosinstall
    wstool update -t src
    
    # Install proto3
    src/cartographer/scripts/install_proto3.sh
    
    # Install deb dependencies
    sudo apt-get update
    sudo rosdep init
    rosdep update
    rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
    
    # Build and install.
    catkin_make_isolated --install --use-ninja
    source install_isolated/setup.bash

Then add this to your ~/.bashrc

    source ~/cartographer_ws/install_isolated/setup.bash

Clone this repo into your `racecar_ws` (not your `cartographer_ws`!) and `catkin_make`

    cd ~/racecar_ws/src
    git clone https://github.com/mit-rss/cartographer_config.git
    cd ~/racecar_ws
    catkin_make
    source devel/setup.bash

To run cartographer on the car, simply run:

    roslaunch cartographer_config cartographer.launch

To run cartographer in the simulator first change the `broadcast_transform` parameter in the simulator's `params.yaml` file to `false:

    broadcast_transform: false

This prevents a clash between the ground truth transform published by the simulator and the estimated transform published by cartographer. Then run cartographer like this:

    roslaunch cartographer_config cartographer_simulated.launch
