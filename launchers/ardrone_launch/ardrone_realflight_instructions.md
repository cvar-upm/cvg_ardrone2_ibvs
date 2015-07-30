screen -AmdS ardrone -t terminal bash
screen -S    ardrone -X screen -t roscore              ${IBVS_STACK}/launchers/ardrone_launch/sh_files/start_roscore.sh


screen -S    ardrone -X screen -t Ardrone_Autonomy     ${IBVS_STACK}/launchers/ardrone_launch/sh_files/run_ardrone_autonomy.sh
screen -S    ardrone -X screen -t Driver_Parrot        ${IBVS_STACK}/launchers/ardrone_launch/sh_files/run_driver_parrot.sh

screen -S    ardrone -X screen -t Opentld_node	       ${IBVS_STACK}/launchers/ardrone_launch/sh_files/run_opentld_node.sh
screen -S    ardrone -X screen -t IBVS_Controller      ${IBVS_STACK}/launchers/ardrone_launch/sh_files/run_IBVS_controller.sh
screen -S    ardrone -X screen -t drone_ekf            ${IBVS_STACK}/launchers/ardrone_launch/sh_files/run_ekf_odometry.sh
screen -S    ardrone -X screen -t drone_logger         ${IBVS_STACK}/launchers/ardrone_launch/sh_files/run_droneLogger_node.sh

screen -S    ardrone -X screen -t drone_interface      ${IBVS_STACK}/launchers/ardrone_launch/sh_files/run_interface_node.sh
