# Makfile Commands    
    
.DEFAULT_GOAL := help    
SHELL=/bin/bash    
    
# Git submodules    
src-update-submodules: ## Update submodules to match their version    
	git submodule sync --recursive    
	git submodule update --init --recursive    
    
# ROS    
    
build: ## Build the project in debug mode    
build-robot-workspace: ## Build the robot workspace in debug mode    
build-simulation-workspace: ## Build the robot workspace in debug mode    
build-rs: build-robot-workspace build-simulation-workspace    
build-release: ## Build the project in release mode    
build-sim-release: ## Build the project in release mode    
clean-robot-ws: ## Clean install, log and build dirs from the workspace    
clean-simulation-ws: ## Clean install, log and build dirs from the workspace  
stop-simulation: ## Stops ros, tmux gzserver and gzclient in order to stop the simulation environment  

## Clean install, log and build dirs from the workspace    
clean-rs: clean-robot-ws clean-simulation-ws     
# Autoware    
build-autoware: ## Build the project in debug mode    
build-robot-workspace-autoware: ## Build the robot workspace in debug mode    
build-simulation-workspace-autoware: ## Build the robot workspace in debug mode    
build-rs-autoware: build-robot-workspace-autoware build-simulation-workspace-autoware    
build-release-autoware: ## Build the project in release mode    
build-sim-release-autoware: ## Build the project in release mode    
build-tests: ## Build the project tests    
build-robot-tests: ## Build the project in debug mode    
build-simulation-tests: ## Build the project in debug mode    
build-tests-results: ## Build the project tests    
build-robot-tests-results:    
build-simulation-tests-results:    
# Docker    
docker-login:    
.build-docker-subbase: ## Build the docker subbase image    
.build-docker-base: ## Build the docker base image    
.build-docker-dev: ## Build the docker dev image    
.build-docker-prod: ## Build the docker prod image    
docker-build-subbase: .upd-params .build-docker-subbase	## build subbase image    
docker-build-base: .upd-params .build-docker-base	## build base image    
docker-build-dev: .upd-params .build-docker-dev  ## build dev image    
docker-build-prod: .upd-params .build-docker-prod  ## build prod image from the current (based on TAG value) dev image  
docker-test-prod: .upd-params ## Run simple test on the local image and check for the required ros nodes and topics    
docker-push-subbase:     
docker-push-base:     
docker-push-dev:     
docker-push-prod:     # pushes the prod image to AWS
docker-pull-base:     
docker-pull-dev:     
docker-pull-jupyter:    
docker-prune:    
docker-remove-no-tags: ## Remove Docker images with no tags    
docker-remove-containers:    
# ROS builds    
ros-dep-install: ## Install ros dependencies for the target $ROS_DISTRO    
rosdep-install-user: ## Install ros dependencies for the target $ROS_DISTRO as $USER    
rosnode-kill:    
docker-stop-base:     
docker-stop-dev:    
docker-start-base:    
docker-start-dev:    
docker-start-prod:      #Starts the production image for CPU
docker-start-robot-cpu:     
docker-start-root:     
docker-start-jupyter:    
docker-enter-base:     
docker-enter-dev:     
docker-enter-prod:      #Enters the production container with root user
docker-enter-base-root:     
docker-enter-dev-root:     
docker-start-sim:     
docker-start-headless: ##Start a container running the headless simulation    
docker-start-gtest: ##Start a container as root to run gtests    
docker-run-sim-test: ##Runs lab_mission.csv in simulation and returns 0 on completion (must have docker-start-headless container     
docker-run-g-test: ##Runs colcon tests in the robot_ws (must have docker-start-gtest container running)    
bring-up:    
run: ## Run the project inside the ADE    
help: ## Print help for the make targets    
run-mission-cmd:    
pub-mission-cmd:      
rosdep-keys:    
rosdep-keys-install:    
rosdep-keys2apt:    
GPG: # Update the GPG key of ROS Melodic    
