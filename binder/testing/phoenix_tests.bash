#!/usr/bin/env bash

export $(grep -v '^#' binder/testing/.env | xargs -d '\n')

make docker-login

docker stop environment-presseter || true && docker rm environment-presseter || true
docker stop rosbridge || true && docker rm rosbridge || true
docker stop phoenix1-binder-dev || true && docker rm phoenix1-binder-dev || true
docker stop phoenix-perception-binder-dev|| true && docker rm phoenix-perception-binder-dev || true
docker stop local-websocket || true && docker rm local-websocket || true
docker stop jest-integration-tests || true && docker rm jest-integration-tests || true

docker run --privileged -h environment-presseter \
  --detach \
  --name environment-presseter \
  --env COLORFGBG \
  --env EMAIL \
  --env GIT_AUTHOR_EMAIL \
  --env GIT_AUTHOR_NAME \
  --env GIT_COMMITTER_EMAIL \
  --env GIT_COMMITTER_NAME \
  --env SSH_AUTH_SOCK \
  --env TERM \
  --env USER=${USER} \
  --env GROUP=${USER} \
  --env USER_ID=`id -u ${USER}` \
  --env GROUP_ID=`getent group ${USER} | awk -F: '{printf $$3}'` \
  -v /dev:/dev \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ${PWD}:/home/${USER} \
  --env DISPLAY \
  --env SIMULATION=true \
  --env DEVELOPING=false \
  --env ROS_MASTER_URI=http://localhost:11311 \
  --env ROS_IP=localhost \
  --env ROS_HOSTNAME=localhost \
  --env NVIDIA_VISIBLE_DEVICES=all \
  --gpus all \
  --env NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display \
  --env LD_LIBRARY_PATH=/usr/local/nvidia/lib64 \
  --env VIDEO_GROUP_ID=`getent group video | awk -F: '{printf $$3}'` \
  -v /dev/dri:/dev/dri \
  -v /home/${USER}/.ssh:/home/${USER}/.ssh \
  -v /run/user/`id -u ${USER}`/keyring/ssh:/run/user/`id -u ${USER}`/keyring/ssh \
  --cap-add=SYS_PTRACE \
  --net=host \
  --privileged \
  --add-host environment-presseter:127.0.0.1 \
  ${IMAGE_REGISTRY}/environment-presseter:environment-presseter-prod-${ENVIRONMENT_PRESSETER_VERSION} bash -c "cd /opt/environment-presseter/share/environment_bringup/scripts/ && ./run_bringup.bash"

docker run --privileged -h rosbridge \
  --detach \
  --name rosbridge \
  --env COLORFGBG \
  --env EMAIL \
  --env GIT_AUTHOR_EMAIL \
  --env GIT_AUTHOR_NAME \
  --env GIT_COMMITTER_EMAIL \
  --env GIT_COMMITTER_NAME \
  --env SSH_AUTH_SOCK \
  --env TERM \
  --env USER=${USER} \
  --env GROUP=${USER} \
  --env USER_ID=`id -u ${USER}` \
  --env GROUP_ID=`getent group ${USER} | awk -F: '{printf $$3}'` \
  -v /dev:/dev \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ${PWD}:/home/${USER} \
  --env DISPLAY \
  --env SIMULATION=true \
  --env DEVELOPING=false \
  --env ROS_MASTER_URI=http://localhost:11311 \
  --env ROS_IP=localhost \
  --env ROS_HOSTNAME=localhost \
  --env NVIDIA_VISIBLE_DEVICES=all \
  --gpus all \
  --env NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display \
  --env LD_LIBRARY_PATH=/usr/local/nvidia/lib64 \
  --env VIDEO_GROUP_ID=`getent group video | awk -F: '{printf $$3}'` \
  -v /dev/dri:/dev/dri \
  -v /home/${USER}/.ssh:/home/${USER}/.ssh \
  -v /run/user/`id -u ${USER}`/keyring/ssh:/run/user/`id -u ${USER}`/keyring/ssh \
  --cap-add=SYS_PTRACE \
  --net=host \
  --privileged \
  --add-host environment-presseter:127.0.0.1 \
  ${IMAGE_REGISTRY}/ais-rosbridge:ais-rosbridge-prod-${ROSBRIDGE_VERSION} bash -c "/wait && source /opt/ais-rosbridge/setup.bash && roslaunch --wait rosbridge_server rosbridge_websocket.launch"

docker run --privileged -h phoenix1-binder-dev \
  --detach \
  --name phoenix1-binder-dev \
  --env COLORFGBG \
  --env EMAIL \
  --env GIT_AUTHOR_EMAIL \
  --env GIT_AUTHOR_NAME \
  --env GIT_COMMITTER_EMAIL \
  --env GIT_COMMITTER_NAME \
  --env SSH_AUTH_SOCK \
  --env TERM \
  --env USER=${USER} \
  --env GROUP=${USER} \
  --env USER_ID=`id -u ${USER}` \
  --env GROUP_ID=`getent group ${USER} | awk -F: '{printf $$3}'` \
  -v /dev:/dev \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ${PWD}:/home/${USER} \
  --env DISPLAY \
  --env SIMULATION=true \
  --env DEVELOPING=false \
  --env ROS_MASTER_URI=http://localhost:11311 \
  --env ROS_IP=localhost \
  --env ROS_HOSTNAME=localhost \
  --env LAUNCH_GAZEBO=false \
  --env NVIDIA_VISIBLE_DEVICES=all \
  --gpus all \
  --env NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display \
  --env LD_LIBRARY_PATH=/usr/local/nvidia/lib64 \
  --env VIDEO_GROUP_ID=`getent group video | awk -F: '{printf $$3}'` \
  -v /dev/dri:/dev/dri \
  -v /home/${USER}/.ssh:/home/${USER}/.ssh \
  -v /run/user/`id -u ${USER}`/keyring/ssh:/run/user/`id -u ${USER}`/keyring/ssh \
  --cap-add=SYS_PTRACE \
  --net=host \
  --privileged \
  --add-host phoenix1-binder-prod:127.0.0.1 \
  ${IMAGE_REGISTRY}/binders:phoenix1-dev-${TAG} bash -c 'source /opt/phoenix1/setup.bash && roslaunch phoenix1_bringup bringup.launch'

docker run --privileged -h phoenix-perception-binder-dev \
  --detach \
  --name phoenix-perception-binder-dev \
  --env COLORFGBG \
  --env EMAIL \
  --env GIT_AUTHOR_EMAIL \
  --env GIT_AUTHOR_NAME \
  --env GIT_COMMITTER_EMAIL \
  --env GIT_COMMITTER_NAME \
  --env SSH_AUTH_SOCK \
  --env TERM \
  --env USER=${USER} \
  --env GROUP=${USER} \
  --env USER_ID=`id -u ${USER}` \
  --env GROUP_ID=`getent group ${USER} | awk -F: '{printf $$3}'` \
  -v /dev:/dev \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ${PWD}:/home/${USER} \
  --env DISPLAY \
  --env SIMULATION=true \
  --env DEVELOPING=false \
  --env FIELD_NAME=SIM \
  --env ROS_MASTER_URI=http://localhost:11311 \
  --env ROS_IP=localhost \
  --env ROS_HOSTNAME=localhost \
  --env NVIDIA_VISIBLE_DEVICES=all \
  --gpus all \
  --env NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display \
  --env LD_LIBRARY_PATH=/usr/local/nvidia/lib64 \
  --env VIDEO_GROUP_ID=`getent group video | awk -F: '{printf $$3}'` \
  -v /dev/dri:/dev/dri \
  -v /home/${USER}/.ssh:/home/${USER}/.ssh \
  -v /run/user/`id -u ${USER}`/keyring/ssh:/run/user/`id -u ${USER}`/keyring/ssh \
  --cap-add=SYS_PTRACE \
  --net=host \
  --privileged \
  --add-host phoenix-perception-binder-prod:127.0.0.1 \
  ${IMAGE_REGISTRY}/binders:phoenix1-perception-prod-${PHOENIX_PERCEPTION_VERSION} bash -c 'source /opt/phoenix1-perception/setup.bash && roslaunch phoenix_detection bringup.launch && tail -F anything'

docker run --privileged -h local-websocket \
  --detach \
  --name local-websocket \
  --net=host \
  --env NODE_ENV=LOCAL \
  -v /etc/timezone:/etc/timezone:ro \
  -v /etc/localtime:/etc/localtime:ro \
  -v ${HOME}/.ais/logs:${HOME}/.ais/logs \
  ${IMAGE_REGISTRY}/ais-local-websocket:${LOCAL_WEBSOCKET_VERSION}

sleep 1m

docker run --name jest-integration-tests \
  --net=host \
  --volume /etc/timezone:/etc/timezone:ro \
  --volume /etc/localtime:/etc/localtime:ro \
  --volume /dev/dri:/dev/dri \
  "aisadmin/ais-integration-tests:${INTERFACE_TESTING}" \
  npm run phoenix-unit-tests

docker stop environment-presseter || true && docker rm environment-presseter || true
docker stop rosbridge || true && docker rm rosbridge || true
docker stop phoenix1-binder-dev || true && docker rm phoenix1-binder-dev || true
docker stop phoenix-perception-binder-dev|| true && docker rm phoenix-perception-binder-dev || true
docker stop local-websocket || true && docker rm local-websocket || true
docker stop jest-integration-tests || true && docker rm jest-integration-tests || true
