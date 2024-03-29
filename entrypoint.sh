#!/bin/bash

set -x

LOCAL_IP=`hostname  -I | cut -f2 -d' '`

CLARE=/home/dgorissen/clare
LOGS=$CLARE/logs

mkdir -p $LOGS
rm -rf $LOGS/*

run_mode=$1

echo
echo "--------------------------------------------"
echo "* Current directory: " $(pwd)
echo "* Local IP detected as ${LOCAL_IP}"
echo "* Logs directory set to ${LOGS}"
echo "* Run mode argument passed:" $run_mode
echo "--------------------------------------------"
echo

# Setup environment
echo "* Setting up environment"
set +x
source ~/openvino/bin/setupvars.sh
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/install/setup.bash
source ${CLARE}/ros/devel/setup.bash
set -x

echo "* Starting roscore"

export ROS_LOG_DIR=${LOGS}/ros
# Start ROS master
export ROS_HOSTNAME=${LOCAL_IP}
export ROS_MASTER_URI=http://${LOCAL_IP}:11311
roscore -v --master-logger-level=info 2>&1 | tee $LOGS/roscore.txt &
echo $! > $LOGS/roscore.pid

# ID of the current run, generated when roscore is started
while [ -z `rosparam get /run_id` ]
do
    echo "Waiting for roscore to be detected..."
    sleep 2
done

RUN_ID=`rosparam get /run_id`
echo "Roscore running, runid is " ${RUN_ID}

# Log directory
ROSOUT_FILE=$(roslaunch-logs)/rosout.log

if [[ "$run_mode" = "minimal" ]]; then
    echo
    echo "====== Running minimal mode ======"
    echo
    # Keep container running
    echo "* Tailing rosout file ${ROSOUT_FILE}"
    tail -f ${ROSOUT_FILE}
fi

if [[ "$run_mode" = "ui" ]]; then
    echo
    echo "====== Running with web UI ======"
    echo

    # Start web backend
    echo "* Starting web backend"
    /home/dgorissen/.pyenv/shims/python3 $CLARE/interface/server/app.py 2>&1 | tee $LOGS/backend.txt &
    echo $! > $LOGS/backend.pid
    sleep 2

    # Start web frontend
    echo "* Starting web frontend"
    #https://stackoverflow.com/questions/69665222/node-js-17-0-1-gatsby-error-digital-envelope-routinesunsupported-err-os
    export NODE_OPTIONS=--openssl-legacy-provider
    npm run serve --prefix $CLARE/interface/client > $LOGS/frontend.txt 2>&1 &
    echo $! > $LOGS/frontend.pid
    sleep 2
fi

# Start ROS nodes
echo "* Starting ROS nodes"
roslaunch --wait ${CLARE}/ros/launch/clare.launch

# Keep container running
echo "* Tailing rosout file ${ROSOUT_FILE}"
tail -f ${ROSOUT_FILE}
