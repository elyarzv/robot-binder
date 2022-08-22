.DEFAULT_GOAL := help
SHELL=/bin/bash

TAG?=local
LATEST?=
FLEET?=${PROJECT}
DOCKERHUB_REGISTRY=aisadmin/${IMAGE_REPO_NAME}
AWS_ACCOUNT_ID=062427299064
AWS_DEFAULT_REGION=ca-central-1
AWS_ECR_REGISTRY=${AWS_ACCOUNT_ID}.dkr.ecr.${AWS_DEFAULT_REGION}.amazonaws.com/${IMAGE_REPO_NAME}

# Image names
PUSH_BASE?=true

# Subbase
SUBBASE_IMAGE_TAG?=${PROJECT}-subbase-${TAG}${LATEST}
SUBBASE_IMAGE_NAME?=${IMAGE_REPO_NAME}:${SUBBASE_IMAGE_TAG}
SUBBASE_IMAGE_NAME_AWS_ECR?=${AWS_ECR_REGISTRY}:${SUBBASE_IMAGE_TAG}
SUBBASE_IMAGE_NAME_DOCKERHUB?=${DOCKERHUB_REGISTRY}:${SUBBASE_IMAGE_TAG}

# Base
BASE_IMAGE_TAG?=${PROJECT}-base-${TAG}${LATEST}
BASE_IMAGE_NAME?=${IMAGE_REPO_NAME}:${BASE_IMAGE_TAG}
BASE_IMAGE_NAME_AWS_ECR?=${AWS_ECR_REGISTRY}:${BASE_IMAGE_TAG}
BASE_IMAGE_NAME_DOCKERHUB?=${DOCKERHUB_REGISTRY}:${BASE_IMAGE_TAG}

# Development
DEV_IMAGE_TAG?=${PROJECT}-dev-${TAG}${LATEST}
DEV_IMAGE_NAME?=${IMAGE_REPO_NAME}:${DEV_IMAGE_TAG}
DEV_IMAGE_NAME_AWS_ECR?=${AWS_ECR_REGISTRY}:${DEV_IMAGE_TAG}
DEV_IMAGE_NAME_DOCKERHUB?=${DOCKERHUB_REGISTRY}:${DEV_IMAGE_TAG}

# Production
PROD_IMAGE_TAG?=${PROJECT}-prod-${TAG}${LATEST}
PROD_IMAGE_NAME?=${IMAGE_REPO_NAME}:${PROD_IMAGE_TAG}
PROD_IMAGE_NAME_AWS_ECR?=${AWS_ECR_REGISTRY}:${PROD_IMAGE_TAG}
PROD_IMAGE_NAME_DOCKERHUB?=${DOCKERHUB_REGISTRY}:${PROD_IMAGE_TAG}

# Git submodules
src-update-submodules: ## Update submodules to match their version
	git submodule sync --recursive
	git submodule update --init --recursive

# ROS

.PHONY: build
WS_SRC?=.
build: ## Build the project in debug mode
	make build-robot-workspace
	make build-simulation-workspace

.PHONY: build-robot-workspace
build-robot-workspace: ## Build the robot workspace in debug mode
	/bin/bash -c ' \
		. /opt/ros/${ROS_DISTRO}/setup.bash && \
		(. /opt/underlay_ws/setup.bash || :) && \
		cd robot_ws && \
		colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && \
		. install/setup.bash'

.PHONY: build-simulation-workspace
build-simulation-workspace: ## Build the robot workspace in debug mode
	/bin/bash -c ' \
		. /opt/ros/${ROS_DISTRO}/setup.bash && \
		. /opt/underlay_ws/setup.bash && \
		. ${HOME}/robot_ws/install/setup.bash && \
		cd simulation_ws && \
		colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && \
		. install/setup.bash'

.PHONY: build-rs
build-rs: build-robot-workspace build-simulation-workspace

build-release: ## Build the project in release mode
	/bin/bash -c ' \
		. /opt/ros/${ROS_DISTRO}/setup.bash && \

		cd robot_ws && \
		colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --merge-install --install-base /opt/${PROJECT}'

build-sim-release: ## Build the project in release mode
	/bin/bash -c ' \
		. /opt/ros/${ROS_DISTRO}/setup.bash && \
		cd simulation_ws && \
		colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --merge-install --install-base /opt/${PROJECT}_sim'

.PHONY: clean-robot-ws
clean-robot-ws: ## Clean install, log and build dirs from the workspace
		@echo "##########################"
		@echo "Cleaning up robot_ws build artifacts"
		cd robot_ws && \
		rm -rf build install log

.PHONY: clean-simulation-ws
clean-simulation-ws: ## Clean install, log and build dirs from the workspace
		@echo "##########################"
		@echo "Cleaning up simulation_ws build artifacts"
		cd simulation_ws && \
		rm -rf build install log

.PHONY: clean-rs
## Clean install, log and build dirs from the workspace
clean-rs: clean-robot-ws clean-simulation-ws

.PHONY: clean
clean: ## Clean install, log and build dirs from the workspace
		@echo "##########################"
		@echo "Cleaning up  ${PROJECT_DIR}"
		rm -rf build install log
		@echo "Cleaning up  ${PROJECT_DIR} submodules"

.PHONY: stop-simulation
stop-simulation: ## Stop simulation
	@echo Stoping simulation, killing some processes
	pkill ros
	pkill tmux
	pkill gzserver
	pkill gzclient

# Autoware

.PHONY: build-autoware
build-autoware: ## Build the project in debug mode
	/bin/bash -c ' \
		. /opt/ros/${ROS_DISTRO}/setup.bash && \
		. ${AUTOWARE_DIR}/setup.bash && \
		cd ${WS_SRC} && \
		colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug && \
		. install/setup.bash'

.PHONY: build-robot-workspace-autoware
build-robot-workspace-autoware: ## Build the robot workspace in debug mode
	/bin/bash -c ' \
		. /opt/ros/${ROS_DISTRO}/setup.bash && \
		. ${AUTOWARE_DIR}/setup.bash && \
		cd robot_ws && \
		colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug && \
		. install/setup.bash'

.PHONY: build-simulation-workspace-autoware
build-simulation-workspace-autoware: ## Build the robot workspace in debug mode
	/bin/bash -c ' \
		. /opt/ros/${ROS_DISTRO}/setup.bash && \
		. ${AUTOWARE_DIR}/setup.bash && \
		cd simulation_ws && \
		colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug && \
		. install/setup.bash'

.PHONY: build-rs-autoware
build-rs-autoware: build-robot-workspace-autoware build-simulation-workspace-autoware

build-release-autoware: ## Build the project in release mode
	/bin/bash -c ' \
		. /opt/ros/${ROS_DISTRO}/setup.bash && \
		. ${AUTOWARE_DIR}/setup.bash && \
		cd robot_ws && \
		colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --install-base /opt/${PROJECT}'

build-sim-release-autoware: ## Build the project in release mode
	/bin/bash -c ' \
		. /opt/ros/${ROS_DISTRO}/setup.bash && \
		. ${AUTOWARE_DIR}/setup.bash && \
		cd simulation_ws && \
		colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --install-base /opt/${PROJECT}_sim'

.PHONY: build-tests
build-tests: ## Build the project tests
	make build-robot-tests
	make build-simulation-tests

.PHONY: build-robot-tests
build-robot-tests: ## Build the project in debug mode
	/bin/bash -c ' \
	. /opt/ros/${ROS_DISTRO}/setup.bash && \
	. /opt/underlay_ws/setup.bash && \
	. ${HOME}/robot_ws/install/setup.bash && \
	cd ${HOME}/robot_ws && \
	colcon test --merge-install --executor sequential --event-handlers console_direct+ && \
	colcon test-result --all --verbose'
.PHONY: build-simulation-tests

build-simulation-tests: ## Build the project in debug mode
	/bin/bash -c ' \
	. /opt/ros/${ROS_DISTRO}/setup.bash && \
	. /opt/underlay_ws/setup.bash && \
	. ${HOME}/robot_ws/install/setup.bash && \
	. ${HOME}/simulation_ws/install/setup.bash && \
	cd ${HOME}/simulation_ws && \
	colcon test --merge-install && \
	colcon test-result --all --verbose'

.PHONY: build-tests-results
build-tests-results: ## Build the project tests
	make build-robot-tests-results
	make build-simulation-tests-results

.PHONY: build-robot-tests-results
build-robot-tests-results:
	/bin/bash -c ' \
	. /opt/ros/${ROS_DISTRO}/setup.bash && \
	. /opt/underlay_ws/setup.bash && \
	. ${HOME}/robot_ws/install/setup.bash && \
	cd ${HOME}/robot_ws && \
	colcon test-result --verbose'

.PHONY: built-simulation-tests-results
build-simulation-tests-results:
	/bin/bash -c ' \
	. /opt/ros/${ROS_DISTRO}/setup.bash && \
	. /opt/underlay_ws/setup.bash && \
	. ${HOME}/robot_ws/install/setup.bash && \
	. ${HOME}/simulation_ws/install/setup.bash && \
	cd ${HOME}/simulation_ws && \
	colcon test-result --verbose'

# Docker

.upd-params: ## Set docker image names based on environment variables
# They are now defined as global environment variables on line 12

docker-login:
	aws ecr get-login-password --region ${AWS_DEFAULT_REGION} | docker login --username AWS --password-stdin ${AWS_ACCOUNT_ID}.dkr.ecr.${AWS_DEFAULT_REGION}.amazonaws.com

dockerhub-login:
	aws secretsmanager get-secret-value --region ${AWS_DEFAULT_REGION} --secret-id DOCKERHUB_DEPLOYMENT | jq --raw-output '.SecretString' | jq -r .\"DOCKER_PASSWORD\" | docker login --username aisdeployment --password-stdin

.build-docker-subbase: ## Build the docker subbase image
	DOCKER_BUILDKIT=1 docker build -f binder/Dockerfile.subbase \
	-t ${SUBBASE_IMAGE_NAME} \
	-t ${SUBBASE_IMAGE_NAME_AWS_ECR} \
	--build-arg AUTOWARE_VER=${AUTOWARE_VER} \
	--build-arg ROS_DISTRO=${ROS_DISTRO} \
	--build-arg PROJECT=${PROJECT} \
	--build-arg TAG=${TAG} \
	.

.build-docker-base: ## Build the docker base image
	DOCKER_BUILDKIT=1 docker build -f binder/Dockerfile.base \
	-t ${BASE_IMAGE_NAME} \
	-t ${BASE_IMAGE_NAME_AWS_ECR} \
	--build-arg OSRF_BASE_IMAGE=${OSRF_BASE_IMAGE} \
	--build-arg AUTOWARE_VER=${AUTOWARE_VER} \
	--build-arg ROS_DISTRO=${ROS_DISTRO} \
	--build-arg PROJECT=${PROJECT} \
	--build-arg IMAGE_MODE=${IMAGE_MODE} \
	.

.build-docker-dev: ## Build the docker dev image
	DOCKER_BUILDKIT=1 docker build -f binder/Dockerfile.dev \
	-t ${DEV_IMAGE_NAME} \
	-t ${DEV_IMAGE_NAME_AWS_ECR} \
	--build-arg BASE_IMAGE=${BASE_IMAGE_NAME} \
	--build-arg PROJECT=${PROJECT} \
	.

.build-docker-dev-build-tests:
	DOCKER_BUILDKIT=1 docker build -f binder/Dockerfile.dev \
	--build-arg BASE_IMAGE=${BASE_IMAGE_NAME} \
	--build-arg PROJECT=${PROJECT} \
	--target=build-unit-tests \
	.

.build-docker-dev-export-tests:
	DOCKER_BUILDKIT=1 docker build -f binder/Dockerfile.dev \
	--build-arg BASE_IMAGE=${BASE_IMAGE_NAME} \
	--build-arg PROJECT=${PROJECT} \
	--target=export-unit-tests --output type=local,dest=reports \
	.

.build-docker-prod: ## Build the docker prod image
	DOCKER_BUILDKIT=1 docker build -f binder/Dockerfile.prod \
	-t ${PROD_IMAGE_NAME} \
	-t ${PROD_IMAGE_NAME_AWS_ECR} \
	--build-arg BASE_IMAGE=${BASE_IMAGE_NAME} \
	--build-arg DEV_IMAGE=${DEV_IMAGE_NAME} \
	--build-arg PROJECT=${PROJECT} \
	.

docker-build-subbase: .upd-params .build-docker-subbase	## build subbase image
docker-build-base: .upd-params .build-docker-base	## build base image
docker-build-dev: .upd-params .build-docker-dev  ## build dev image
docker-build-dev-build-tests: .upd-params .build-docker-dev-build-tests ## build dev image
docker-build-dev-export-tests: .upd-params .build-docker-dev-export-tests ## build dev image
docker-build-prod: .upd-params .build-docker-prod  ## build prod cpu image

docker-pull-all: 
	docker-pull-base 
	docker-pull-dev
	docker-pull-prod

docker-test-prod: .upd-params ## Run simple test on the local image and check for the required ros nodes and topics
	echo "Running tests"
	docker run --rm -v ${PWD}/tests/:/tmp/tests/ --entrypoint bash  ${PROD_IMAGE_NAME} -c 'cd /tmp/tests && bash run_tests.bash'
	echo "Tests finished successfuly"
	@sleep 4
	docker ps -a

ifeq ($(PUSH_BASE), true)
docker-push-subbase: docker-login dockerhub-login .upd-params
	docker push ${SUBBASE_IMAGE_NAME_AWS_ECR}
	docker push ${SUBBASE_IMAGE_NAME_DOCKERHUB}
else
docker-push-subbase:
endif

ifeq ($(PUSH_BASE), true)
docker-push-base: docker-login dockerhub-login .upd-params
	docker tag ${BASE_IMAGE_NAME_AWS_ECR} ${BASE_IMAGE_NAME_DOCKERHUB}
	docker push ${BASE_IMAGE_NAME_DOCKERHUB}
	docker push ${BASE_IMAGE_NAME_AWS_ECR}
else
docker-push-base:
endif

docker-push-dev: docker-login dockerhub-login .upd-params
	docker tag ${DEV_IMAGE_NAME_AWS_ECR} ${DEV_IMAGE_NAME_DOCKERHUB}
	docker push ${DEV_IMAGE_NAME_DOCKERHUB}
	docker push ${DEV_IMAGE_NAME_AWS_ECR}

docker-push-prod: docker-login dockerhub-login .upd-params
	docker tag ${PROD_IMAGE_NAME_AWS_ECR} ${PROD_IMAGE_NAME_DOCKERHUB}
	docker push ${PROD_IMAGE_NAME_DOCKERHUB}
	docker push ${PROD_IMAGE_NAME_AWS_ECR}
	
docker-pull-base: docker-login .upd-params
	docker pull ${BASE_IMAGE_NAME_AWS_ECR}
	docker tag ${BASE_IMAGE_NAME_AWS_ECR} ${BASE_IMAGE_NAME}

docker-pull-dev: docker-login .upd-params
	docker pull ${DEV_IMAGE_NAME_AWS_ECR}
	docker tag ${DEV_IMAGE_NAME_AWS_ECR} ${DEV_IMAGE_NAME}

docker-pull-prod: docker-login .upd-params
	docker pull ${PROD_IMAGE_NAME_AWS_ECR}
	docker tag ${PROD_IMAGE_NAME_AWS_ECR} ${PROD_IMAGE_NAME}

docker-pull-jupyter: docker-login .upd-params
	docker pull ${AWS_ECR_REGISTRY}:jupyter-ros-latest
	docker tag ${AWS_ECR_REGISTRY}:jupyter-ros-latest jupyter-ros

docker-prune:
	docker system prune --all --force --volumes

docker-remove-no-tags: ## Remove Docker images with no tags
	docker rmi $$(docker images -f dangling=true -q)

docker-remove-containers:
	docker rm $$(docker ps -a -q)

# Dependencies
rosdep-simulate: ## Create the script for installing the required dependencies at: binder/base/rosdep-install.sh
	docker run -v $${PWD}:/root ${OSRF_BASE_IMAGE} /bin/bash -c ' \
		apt-get update && apt-get install curl python3-pip -y && \
		rm /etc/ros/rosdep/sources.list.d/20-default.list && \
	    rosdep init && \
		rosdep update --rosdistro ${ROS_DISTRO} && \
		. /opt/ros/${ROS_DISTRO}/setup.bash && \
		cd /root && \
		rosdep install --default-yes --from-paths robot_ws --ignore-src --rosdistro ${ROS_DISTRO} --reinstall --simulate | sed -e "s/  sudo -H //" | tee /root/binder/base/rosdep-install.sh'
	sudo chown ${USER}:${USER} binder/base/rosdep-install.sh
	chmod +x binder/base/rosdep-install.sh

rosdep-simulate-underlay: ## Create the script for installing the required dependencies at: binder/base/rosdep-install.sh
	docker run -v $${PWD}:/root ${OSRF_BASE_IMAGE} /bin/bash -c ' \
		apt-get update && apt-get install curl python3-pip -y && \
		rm /etc/ros/rosdep/sources.list.d/20-default.list && \
	    rosdep init && \
		rosdep update --rosdistro ${ROS_DISTRO} && \
		. /opt/ros/${ROS_DISTRO}/setup.bash && \
		cd /root && \
		rosdep install --default-yes --from-paths binder/underlay_ws --ignore-src --rosdistro ${ROS_DISTRO} --reinstall --simulate | sed -e "s/  sudo -H //" | tee /root/binder/base/rosdep-install-underlay.sh'
	sudo chown ${USER}:${USER} binder/base/rosdep-install-underlay.sh
	chmod +x binder/base/rosdep-install-underlay.sh

# ROS builds
ros-dep-install: ## Install ros dependencies for the target $ROS_DISTRO
	/bin/bash -c ' \
		. /opt/ros/${ROS_DISTRO}/setup.bash && \
		apt-get update && \
		cd /opt/ros/${ROS_DISTRO}/ && \
		sudo rosdep fix-permissions && \
		rosdep update --rosdistro ${ROS_DISTRO} && \
		rosdep install --default-yes --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} -r && \
		rm -rf /var/lib/apt/lists/* /tmp/apt-packages'

rosdep-install-user: ## Install ros dependencies for the target $ROS_DISTRO as $USER
	/bin/bash -c ' \
		. /opt/ros/${ROS_DISTRO}/setup.bash && \
		sudo apt-get update && \
		sudo rosdep fix-permissions && \
		rosdep update --rosdistro ${ROS_DISTRO} && \
		rosdep install --default-yes --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} && \
		sudo rm -rf /var/lib/apt/lists/* /tmp/apt-packages'

rosnode-kill:
	rosnode kill -a; killall -9 rosmaster; killall -9 roscore

docker-stop-base: .upd-params
	docker stop ${PROJECT}-binder-base || true && docker rm ${PROJECT}-binder-base || true

docker-stop-dev: .upd-params
	docker stop ${PROJECT}-binder-dev|| true && docker rm ${PROJECT}-binder-dev || true

docker-stop-prod: .upd-params
	docker stop ${PROJECT}-binder-prod || true && docker rm ${PROJECT}-binder-prod || true

docker-start-base: .upd-params docker-stop-base
	docker run \
	-h ${PROJECT}-binder-base \
	--detach \
	--name ${PROJECT}-binder-base \
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
	-v $${PWD}:${HOME} \
	--env DISPLAY \
	--env NVIDIA_VISIBLE_DEVICES=all \
	--gpus all \
	--env NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display \
	--env LD_LIBRARY_PATH=/usr/local/nvidia/lib64 \
	--env VIDEO_GROUP_ID=`getent group video | awk -F: '{printf $$3}'` \
	-v /dev/dri:/dev/dri \
	-v ${HOME}/.ssh:${HOME}/.ssh \
	-v /run/user/`id -u ${USER}`/keyring/ssh:/run/user/`id -u ${USER}`/keyring/ssh \
	--cap-add=SYS_PTRACE \
	--net=host \
	--privileged \
	--add-host ${PROJECT}-binder-base:127.0.0.1 \
	${BASE_IMAGE_NAME}
	xhost +local:'${PROJECT}-binder-base'

docker-start-base-cpu: .upd-params docker-stop-base
	docker run \
	-h ${PROJECT}-binder-base \
	--detach \
	--name ${PROJECT}-binder-base \
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
	-v $${PWD}:${HOME} \
	--env DISPLAY \
	--env VIDEO_GROUP_ID=`getent group video | awk -F: '{printf $$3}'` \
	-v /dev/dri:/dev/dri \
	-v ${HOME}/.ssh:${HOME}/.ssh \
	-v /run/user/`id -u ${USER}`/keyring/ssh:/run/user/`id -u ${USER}`/keyring/ssh \
	--cap-add=SYS_PTRACE \
	--net=host \
	--privileged \
	--add-host ${PROJECT}-binder-base:127.0.0.1 \
	${BASE_IMAGE_NAME}
	xhost +local:'${PROJECT}-binder-base'

docker-start-dev: .upd-params docker-stop-dev
	docker run \
	-h ${PROJECT}-binder-dev \
	--detach \
	--name ${PROJECT}-binder-dev \
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
	--env-file ./binder/${FLEET}-fleet-management/${ROBOT_NAME}.env \
	--env-file ./binder/simulation.env \
	-v /dev:/dev \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	-v $${PWD}:${HOME} \
	--env DISPLAY \
	--env NVIDIA_VISIBLE_DEVICES=all \
	--gpus all \
	--env NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display \
	--env LD_LIBRARY_PATH=/usr/local/nvidia/lib64 \
	--env VIDEO_GROUP_ID=`getent group video | awk -F: '{printf $$3}'` \
	--env SIMULATION=true \
	--env DEVELOPING=true \
	-v /dev/dri:/dev/dri \
	-v ${HOME}/.ssh:${HOME}/.ssh \
	-v ${HOME}/.ais:${HOME}/.ais \
	-v /run/user/`id -u ${USER}`/keyring/ssh:/run/user/`id -u ${USER}`/keyring/ssh \
	--cap-add=SYS_PTRACE \
	--net=host \
	--privileged \
	--add-host ${PROJECT}-binder-dev:127.0.0.1 \
	${DEV_IMAGE_NAME}
	xhost +local:'${PROJECT}-binder-dev'

docker-start-robot-cpu: .upd-params docker-stop-dev
	docker run \
	-h ${PROJECT}-binder-dev \
	--detach \
	--name ${PROJECT}-binder-dev \
	--env COLORFGBG \
	--env EMAIL \
	--env GIT_AUTHOR_EMAIL \
	--env GIT_AUTHOR_NAME \
	--env GIT_COMMITTER_EMAIL \
	--env GIT_COMMITTER_NAME \
	--env SSH_AUTH_SOCK \
	--env TERM \
	--env-file ./binder/${FLEET}-fleet-management/${ROBOT_NAME}.env \
	--env SIMULATION=false \
	--env DEVELOPING=true \
	--env ROS_MASTER_URI=http://bigtop-base:11311 \
	--env ROS_IP=bigtop-base \
	--env ROS_HOSTNAME=bigtop-base \
	-v ${HOME}/.ais/:${HOME}/.ais:rw \
	-v /dev:/dev \
	-v $${PWD}:${HOME} \
	-v /dev/dri:/dev/dri \
	-v ${HOME}/.ssh:${HOME}/.ssh \
	-v /run/user/`id -u ${USER}`/keyring/ssh:/run/user/`id -u ${USER}`/keyring/ssh \
	--cap-add=SYS_PTRACE \
	--net=host \
	--privileged \
	--add-host ${PROJECT}-binder-dev:127.0.0.1 \
	${DEV_IMAGE_NAME}

docker-start-robot-gpu: .upd-params docker-stop-dev
	docker run \
	-h ${PROJECT}-binder-dev \
	--detach \
	--name ${PROJECT}-binder-dev \
	--env COLORFGBG \
	--env EMAIL \
	--env GIT_AUTHOR_EMAIL \
	--env GIT_AUTHOR_NAME \
	--env GIT_COMMITTER_EMAIL \
	--env GIT_COMMITTER_NAME \
	--env SSH_AUTH_SOCK \
	--env TERM \
	--env-file ./binder/${FLEET}-fleet-management/${ROBOT_NAME}.env \
	--env SIMULATION=false \
	--env DEVELOPING=true \
	--env ROS_MASTER_URI=http://bigtop-base:11311 \
	--env ROS_IP=uv-vision-module \
	--env ROS_HOSTNAME=uv-vision-module \
	--env NVIDIA_VISIBLE_DEVICES=all \
	--gpus all \
	--env NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display \
	--env LD_LIBRARY_PATH=/usr/local/nvidia/lib64 \
	--runtime=nvidia \
	-v ${HOME}/.ais/:${HOME}/.ais:rw \
	-v /dev:/dev \
	-v /etc/hosts:/etc/hosts \
	-v $${PWD}:${HOME} \
	-v /dev/dri:/dev/dri \
	-v ${HOME}/.ssh:${HOME}/.ssh \
	-v /run/user/`id -u ${USER}`/keyring/ssh:/run/user/`id -u ${USER}`/keyring/ssh \
	--cap-add=SYS_PTRACE \
	--net=host \
	--privileged \
	--add-host ${PROJECT}-binder-dev:127.0.0.1 \
	${DEV_IMAGE_NAME}
	

docker-start-dev-cpu: .upd-params docker-stop-dev
	docker run \
	-h ${PROJECT}-binder-dev \
	--detach \
	--name ${PROJECT}-binder-dev \
	--env COLORFGBG \
	--env TERM \
	-v /dev/shm:/dev/shm \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	--env SIMULATION=false \
	--env DEVELOPING=true \
	--env DISPLAY \
	--env VIDEO_GROUP_ID=`getent group video | awk -F: '{printf $$3}'` \
	--volume /etc/timezone:/etc/timezone:ro \
	--volume /etc/localtime:/etc/localtime:ro \
	-v /dev/dri:/dev/dri \
	--cap-add=SYS_PTRACE \
	--net=host \
	--privileged \
	--add-host ${PROJECT}-binder-dev:127.0.0.1 \
	--volume $${PWD}/binder/.ais:/root/.ais \
	aisadmin/binders:phoenix1-dev-develop

docker-start-root-gpu: .upd-params docker-stop-dev
	docker run \
	-h ${PROJECT}-binder-dev \
	--detach \
	--name ${PROJECT}-binder-dev \
	--env COLORFGBG \
	--env TERM \
	-v /dev/shm:/dev/shm \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	--env DISPLAY \
	--env VIDEO_GROUP_ID=`getent group video | awk -F: '{printf $$3}'` \
	--volume /etc/timezone:/etc/timezone:ro \
	--volume /etc/localtime:/etc/localtime:ro \
	--env DISPLAY \
	--env NVIDIA_VISIBLE_DEVICES=all \
	--gpus all \
	--env NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display \
	--env LD_LIBRARY_PATH=/usr/local/nvidia/lib64 \
	--env SIMULATION=false \
	--env DEVELOPING=true \
	-v /dev/dri:/dev/dri \
	--cap-add=SYS_PTRACE \
	--net=host \
	--privileged \
	--add-host ${PROJECT}-binder-dev:127.0.0.1 \
	--volume $${PWD}/binder/.ais:/root/.ais \
	${DEV_IMAGE_NAME}
	xhost +local:'${PROJECT}-binder-dev'

docker-start-dev-cpu: .upd-params docker-stop-dev
	docker run \
	-h ${PROJECT}-binder-dev \
	--detach \
	--name ${PROJECT}-binder-dev \
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
	--env-file ./binder/${FLEET}-fleet-management/${ROBOT_NAME}.env \
	--env SIMULATION=false \
	--env DEVELOPING=true \
	--env ROS_MASTER_URI=http://bigtop-base:11311 \
	--env ROS_IP=bigtop-base \
	--env ROS_HOSTNAME=bigtop-base \
	-v /dev:/dev \
	-v $${PWD}:${HOME} \
	--env DISPLAY \
	--env VIDEO_GROUP_ID=`getent group video | awk -F: '{printf $$3}'` \
	-v /dev/dri:/dev/dri \
	-v ${HOME}/.ssh:${HOME}/.ssh \
	-v /run/user/`id -u ${USER}`/keyring/ssh:/run/user/`id -u ${USER}`/keyring/ssh \
	--cap-add=SYS_PTRACE \
	--net=host \
	--privileged \
	--add-host ${PROJECT}-binder-dev:127.0.0.1 \
	${DEV_IMAGE_NAME}

docker-start-prod: .upd-params docker-stop-prod
	docker run \
	-h ${PROJECT}-binder-prod \
	--detach \
	--name ${PROJECT}-binder-prod \
	--env COLORFGBG \
	--env TERM \
	-v /dev/shm:/dev/shm \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	--env DISPLAY \
	--env DISPLAY \
	--env VIDEO_GROUP_ID=`getent group video | awk -F: '{printf $$3}'` \
	--volume /etc/timezone:/etc/timezone:ro \
	--volume /etc/localtime:/etc/localtime:ro \
	-v /dev/dri:/dev/dri \
	--cap-add=SYS_PTRACE \
	--net=host \
	--privileged \
	--add-host ${PROJECT}-binder-dev:127.0.0.1 \
	--volume $${PWD}/binder/.ais:/root/.ais \
	${PROD_IMAGE_NAME}
	xhost +local:'${PROJECT}-binder-prod'

docker-start-jupyter:
	docker run \
	-h jupyter-ros \
	--name jupyter-ros \
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
	-v $${PWD}:${HOME} \
	--env DISPLAY \
	--env NVIDIA_VISIBLE_DEVICES=all \
	--gpus all \
	--env NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display \
	--env LD_LIBRARY_PATH=/usr/local/nvidia/lib64 \
	--env VIDEO_GROUP_ID=`getent group video | awk -F: '{printf $$3}'` \
	-v /dev/dri:/dev/dri \
	-v ${HOME}/.ssh:${HOME}/.ssh \
	-v /run/user/`id -u ${USER}`/keyring/ssh:/run/user/`id -u ${USER}`/keyring/ssh \
	--cap-add=SYS_PTRACE \
	--net=host \
	--privileged \
	--add-host jupyter-ros:127.0.0.1 \
	${AWS_ECR_REGISTRY}:jupyter-ros-latest

docker-enter-base: .upd-params
	docker exec -it --env COLORFGBG --env TERM -u ${USER} --workdir ${HOME} ${PROJECT}-binder-base /bin/bash -li

docker-enter-dev: .upd-params
	docker exec -it --env COLORFGBG --env TERM -u ${USER} --workdir ${HOME} ${PROJECT}-binder-dev /bin/bash -li

docker-enter-root: .upd-params
	docker exec -it --env COLORFGBG --env TERM --workdir "/root" ${PROJECT}-binder-prod /bin/bash -li


docker-enter-base-root: .upd-params
	docker exec -it --env COLORFGBG --env TERM --workdir /root ${PROJECT}-binder-base /bin/bash -li

docker-enter-dev-root: .upd-params
	docker exec -it --env COLORFGBG --env TERM --workdir /root ${PROJECT}-binder-dev /bin/bash -li

docker-start-sim: .upd-params docker-stop-dev
	mkdir -p $${PWD}/.ais/logs && mkdir -p $${PWD}/.ais/maps
	docker run --privileged -h  ${PROJECT}-binder-dev --name ${PROJECT}-binder-dev -d --cap-add=SYS_PTRACE \
	  --net=host \
	  --add-host ${PROJECT}-binder-dev:127.0.0.1 \
	  --env TERM \
	  --env DISPLAY \
	  --env VIDEO_GROUP_ID=`getent group video | awk -F: '{printf $$3}'` \
	  --volume /dev/dri:/dev/dri \
	  --volume /dev/input:/dev/input \
	  --gpus all \
	  --env NVIDIA_VISIBLE_DEVICES=all \
	  --env NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display \
	  --env LD_LIBRARY_PATH=/usr/local/nvidia/lib64 \
		--env-file ./binder/simulation.env \
	  --env SIMULATION=true \
	  --env DEVELOPING=false \
	  --volume $${PWD}/.ais:/root/.ais \
	  ${DEV_IMAGE_NAME} bash -c 'source /tmp/run_simulation.sh'
	xhost +local:'${PROJECT}-binder-dev'

docker-start-headless: .upd-params docker-stop-dev ##Start a container running the headless simulation
	docker run  -h ${PROJECT}-binder-dev \
	--detach \
	--name ${PROJECT}-binder-dev \
	--env COLORFGBG \
	--env EMAIL \
	--env GIT_AUTHOR_EMAIL \
	--env GIT_AUTHOR_NAME \
	--env GIT_COMMITTER_EMAIL \
	--env GIT_COMMITTER_NAME \
	--env SSH_AUTH_SOCK \
	--env TERM \
	--env-file ./binder/${PROJECT}-fleet-management/${ROBOT_NAME}.env \
	--env DISPLAY \
	--env SIMULATION=true \
	--env ROS_MASTER_URI=http://localhost:11311 \
	--env ROS_HOSTNAME=localhost \
	--env LAUNCH_GAZEBO=false \
	--env DEVELOPING=false \
	--env ROS_LOG_DIR=~/ \
	--add-host ${PROJECT}-binder-dev:127.0.0.1 \
	   	${DEV_IMAGE_NAME} bash -c 'source /tmp/run_simulation.sh'

docker-start-gtest: .upd-params docker-stop-dev ##Start a container as root to run gtests
	docker run -h ${PROJECT}-binder-dev \
	--detach \
	--name ${PROJECT}-binder-dev \
	--env COLORFGBG \
	--env EMAIL \
	--env GIT_AUTHOR_EMAIL \
	--env GIT_AUTHOR_NAME \
	--env GIT_COMMITTER_EMAIL \
	--env GIT_COMMITTER_NAME \
	--env SSH_AUTH_SOCK \
	--env TERM \
	--env-file ./binder/${PROJECT}-fleet-management/${ROBOT_NAME}.env \
	--env DISPLAY \
	--env SIMULATION=true \
	--env ROS_MASTER_URI=http://localhost:11311 \
	--env ROS_HOSTNAME=localhost \
	--env LAUNCH_GAZEBO=false \
	--env DEVELOPING=false \
	--env ROS_LOG_DIR=~/ \
	--add-host ${PROJECT}-binder-dev:127.0.0.1 \
	   	${DEV_IMAGE_NAME} bash -c '/tmp/sleep_infinity.sh'

docker-run-sim-test: .upd-params ##Runs lab_mission.csv in simulation and returns 0 on completion (must have docker-start-headless container running)
	docker exec -i ${PROJECT}-binder-dev bash -c '/tmp/run_sim_test.sh'

docker-run-g-test: .upd-params ##Runs colcon tests in the robot_ws (must have docker-start-gtest container running)
	docker exec -i ${PROJECT}-binder-dev bash -c 'cd ~/phoenix1 && \
			. /opt/ros/${ROS_DISTRO}/setup.bash && \
			. /opt/underlay_ws/setup.bash && \
			cd ~/phoenix1/robot_ws && \
			colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && \
			. install/setup.bash && \
			cd ~/phoenix1/simulation_ws && \
			colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && \
			. install/setup.bash && \
			cd ~/phoenix1/robot_ws && \
			colcon test --merge-install && \
			colcon test-result --all --verbose'

bring-up: .upd-params
	mkdir -p $${PWD}/.ais/logs && mkdir -p $${PWD}/.ais/maps
	docker run --privileged -h  ${PROJECT}-binder-prod --name ${PROJECT}-binder-prod -d --cap-add=SYS_PTRACE \
	   --net=host \
	   --add-host ${PROJECT}-binder-prod:127.0.0.1 \
	   --env TERM \
	   --env DISPLAY \
	   --env VIDEO_GROUP_ID=`getent group video | awk -F: '{printf $$3}'` \
	   --env-file ./binder/simulation.env \
	   --volume /dev/dri:/dev/dri \
	   --volume /dev/input:/dev/input \
	   --gpus all \
	   --env NVIDIA_VISIBLE_DEVICES=all \
	   --env NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display \
	   --env LD_LIBRARY_PATH=/usr/local/nvidia/lib64 \
	   --volume $${PWD}/.ais:/root/.ais \
	   ${PROD_IMAGE_NAME}
	xhost +local:'${PROJECT}-binder-prod'

run: ## Run the project inside the ADE
	/bin/bash -c '${PROJECT_DIR}/build-resources/entrypoints/docker-entrypoint.bash'
	/bin/bash -c 'web_script_launch.bash'
	/bin/bash -c 'web_script_pub.bash'

.PHONY: help

help: ## Print help for the make targets
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort |  sed -En "s/^[a-zA-Z_-]+:([a-zA-Z_-]+:.*?## .*)/\1/p" | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-50s\033[0m %s\n\n", $$1, $$2}'

run-mission-cmd:
	/bin/bash -c ' \
		source web_script_launch.sh'

pub-mission-cmd:
	/bin/bash -c ' \
		source web_script_pub.sh'

rosdep-keys:
	rosdep keys --from-paths . -i  --rosdistro ${ROS_DISTRO}

rosdep-keys-install:
	rosdep keys --from-paths . --ignore-src -n --rosdistro ${ROS_DISTRO} |  xargs echo -n | xargs rosdep install --rosdistro ${ROS_DISTRO}

rosdep-keys2apt:
	rosdep keys --from-paths . --ignore-src -n --rosdistro ${ROS_DISTRO} | xargs echo -n

GPG: # Update the GPG key of ROS Melodic
	curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
	sudo apt update
