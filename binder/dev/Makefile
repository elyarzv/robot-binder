.PHONY: build
build: ## Build the project in debug mode
	make build-robot
	make build-simulation

.PHONY: build-robot
build-robot: ## Build the project in debug mode
	/bin/bash -c ' \
	. /opt/ros/${ROS_DISTRO}/setup.bash && \
	. /opt/underlay_ws/setup.bash && \
	cd ${HOME}/phoenix1/robot_ws && \
	colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug --merge-install'

.PHONY: build-simulation
build-simulation: ## Build the project in debug mode
	/bin/bash -c ' \
	. /opt/ros/${ROS_DISTRO}/setup.bash && \
	. /opt/underlay_ws/setup.bash && \
	. ${HOME}/phoenix1/robot_ws/install/setup.bash && \
	cd ${HOME}/phoenix1/simulation_ws/ && \
	colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug --merge-install'
