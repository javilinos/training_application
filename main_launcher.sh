#!/bin/bash

NUMID_DRONE=1
UAV_MASS=1.5

export AEROSTACK_PROJECT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

gnome-terminal  \
`#---------------------------------------------------------------------------------------------` \
`# Mission                                                                         		 ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "python_based_mission_interpreter_process" --command "bash -c \"
roslaunch python_based_mission_interpreter_process python_based_mission_interpreter_process.launch --wait \
  drone_id_namespace:=drone$NUMID_DRONE \
  drone_id_int:=$NUMID_DRONE \
  my_stack_directory:=${AEROSTACK_PROJECT} \
  mission:=training.py \
  mission_configuration_folder:=${AEROSTACK_PROJECT}/configs/mission;
exec bash\"" \
`#---------------------------------------------------------------------------------------------` \
`# Pixhawk Interface                                                                           ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Pixhawk Interface" --command "bash -c \"
roslaunch pixhawk_interface pixhawk_interface.launch \
--wait namespace:=drone$NUMID_DRONE \
 acro_mode:=true \
 simulation_mode:=true \
 fcu_url:=udp://:14540@localhost:14580 \
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
`# localize with gazebo                                                                        ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "localization"  --command "bash -c \"
rosrun localize_with_gazebo localize_with_gazebo.py --wait ;
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
roslaunch flightmare_px4 flightmare_sitl_gazebo.launch --wait \
  quad_name:=drone$NUMID_DRONE;
exec bash\""  &

sleep 5
