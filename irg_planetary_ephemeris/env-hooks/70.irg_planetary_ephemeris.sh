export GAZEBO_MODEL_PATH="$(colcon list | grep irg_planetary_ephemeris | cut -f 2)/models:${GAZEBO_MODEL_PATH}"
