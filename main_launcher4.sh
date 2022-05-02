#!/bin/bash

NUMID_DRONE=4
UAV_MASS=1.5

export AEROSTACK_PROJECT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

gnome-terminal  \
`#---------------------------------------------------------------------------------------------` \
`# Pixhawk Interface                                                                           ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Pixhawk Interface" --command "bash -c \"
roslaunch pixhawk_interface pixhawk_interface.launch \
--wait namespace:=drone$NUMID_DRONE \
 acro_mode:=true \
 simulation_mode:=true \
 fcu_url:=udp://:14543@localhost:14583 \
 tgt_system:=$NUMID_DRONE \
 tgt_component:=1;
exec bash\"" \
`#---------------------------------------------------------------------------------------------` \
`# Basic Behaviors                                                                             ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Basic Behaviors" --command "bash -c \"
roslaunch basic_quadrotor_behaviors basic_quadrotor_behaviors.launch --wait \
  namespace:=drone$NUMID_DRONE;
exec bash\"" \
`#---------------------------------------------------------------------------------------------` \
`# Quadrotor Motion With DF Control                                                            ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Quadrotor Motion With DF Control" --command "bash -c \"
roslaunch quadrotor_motion_with_df_control quadrotor_motion_with_df_control.launch --wait \
    namespace:=drone$NUMID_DRONE \
    uav_mass:=$UAV_MASS;
exec bash\"" \
`#---------------------------------------------------------------------------------------------` \
`# Throttle Controller                                                                         ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Throttle Controller" --command "bash -c \"
roslaunch thrust2throttle_controller thrust2throttle_controller.launch --wait \
  namespace:=drone$NUMID_DRONE \
  uav_mass:=$UAV_MASS;
exec bash\"" \
`#---------------------------------------------------------------------------------------------` \
`# Flightmare                                                                                  ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Flightmare_px4" --command "bash -c \"
roslaunch flightmare_px4 flightmare_sitl_gazebo_4.launch --wait \
  quad_name:=drone$NUMID_DRONE;
exec bash\""  &

sleep 5
