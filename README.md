# Phoenix binder

---
## Phoenix binder structure

├── binder  
│   ├── binder/[ade](https://ade-cli.readthedocs.io/en/latest/index.html) ADE docker based tool to ensure that all developers have the same environment  
│   │   ├── binder/ade/[entrypoint](https://bitbucket.org/ais_admin/phoenix-binder/src/main/binder/ade/entrypoint) entrypoint for Dockerfile compatible with ADE   
│   │   └── binder/ade/[env.sh](https://bitbucket.org/ais_admin/phoenix-binder/src/main/binder/ade/env.sh)   
│   ├── binder/base  Requirements for building base docker image   
│   │   └── binder/base/[mt_software_suite_linux-x64_2021.0](https://content.xsens.com/mt-software-suite-download?hsCtaTracking=e7ef7e11-db88-4d9e-b36e-3f937ea4ae15%7Cd6a8454e-6db4-41e7-9f81-f8fc1c4891b3) Xsens software IMU SDK  
│   ├── binder/dev Requirements for building development docker image   
│   │   ├── binder/dev/bashrc  
│   │   └── binder/dev/workspace.sh  
│   ├── binder/[Dockerfile.base](https://bitbucket.org/ais_admin/phoenix-binder/src/main/binder/Dockerfile.base)  Dockerfile for base image    
│   ├── binder/[Dockerfile.dev](https://bitbucket.org/ais_admin/phoenix-binder/src/main/binder/Dockerfile.dev)  Dockerfile for development (and for building code for the production image)    
│   ├── binder/[Dockerfile.prod](https://bitbucket.org/ais_admin/phoenix-binder/src/main/binder/Dockerfile.prod) Dockerfile for production      (no source code on robot)      
│   ├── binder/prod Requirements for building development docker image   
│   │   └── binder/prod/entrypoint  
│   ├── binder/[README.md](https://bitbucket.org/ais_admin/phoenix-binder/src/main/binder/README.md)  
│   └── binder/underlay_ws Worspace use to catch all dependencies that are not actively developed in this project or are used from another source eg. Github   
│       ├── binder/underlay_ws/[README.md](https://bitbucket.org/ais_admin/phoenix-binder/src/main/binder/underlay_ws/README.md)  
│       └── binder/underlay_ws/src  
│           ├── binder/underlay_ws/src/[ais-bigtop-rosbags](https://bitbucket.org/ais_admin/ais-bigtop-rosbags/src/master/)  For automatic recording of rosbags during missions     
│           ├── binder/underlay_ws/src/[ira_laser_tools](https://github.com/iralabdisco/ira_laser_tools)  Provides the node to merge two laser scans      
│           └── binder/underlay_ws/src/phoenix1_drivers  
|               └── binder/underlay_ws/src/phoenix1_drivers/[xsens_ros_mti_driver](https://bitbucket.org/ais_admin/xsens_ros_mti_driver/src/master/)   
├── [CHANGELOG.md](https://bitbucket.org/ais_admin/phoenix-binder/src/main/CHANGELOG.md)  
├── [Makefile](https://bitbucket.org/ais_admin/phoenix-binder/src/main/Makefile)  Provides helpful commands for developers and build tools     
├── [README.md](https://bitbucket.org/ais_admin/phoenix-binder/src/main/README.md)  
├── codepipeline Set of scripts to run pipelines with AWS codebuild and codepipeline   
├── robot_ws Workspace that is used to develop the production software that goes in the robot   
│   └── robot_ws/src  
│       ├── robot_ws/src/[ais_dimming_module](https://bitbucket.org/ais_admin/ais_dimming_module/src/main/)  Controls the individual dimming of the lamps      
│       ├── robot_ws/src/[ais_utilities](https://bitbucket.org/ais_admin/ais_utilities/src/master/)  Utility package for writing code tests     
│       ├── robot_ws/src/[ais_cru_manager](https://bitbucket.org/ais_admin/ais_cru_manager/src/master/)  Package to provide service that advertises the CRU name      
│       ├── robot_ws/src/[ais_job_manager](https://bitbucket.org/ais_admin/ais_job_manager/src/master/)  Lists and saves missions     
│       ├── robot_ws/src/[ais_safety](https://bitbucket.org/ais_admin/ais_safety/src/master/)  Safety node for collision avoidance     
│       ├── robot_ws/src/[ais_messages](https://bitbucket.org/ais_admin/ais_messages/src/noetic-devel/)  Contains custom messages used across AIS     
│       ├── robot_ws/src/[ais_state_machine](https://bitbucket.org/ais_admin/ais_state_machine/src/main/)  State machine and autonomy logic     
│       ├── robot_ws/src/[ais_led_strip_controller](https://bitbucket.org/ais_admin/ais_led_strip_controller/src/main/)  Controller for LED strips      
│       ├── robot_ws/src/[amr_localization](https://bitbucket.org/ais_admin/amr_localization/src/master/)  Localization package providing 1D on-rail localization      
│       ├── robot_ws/src/[phoenix1_bringup](https://bitbucket.org/ais_admin/phoenix1_bringup/src/main/)  Bringup package containing configuration and launch files to start the robot      
│       ├── robot_ws/src/[phoenix1_parameters](https://bitbucket.org/ais_admin/phoenix1_parameters/src/main/)  Robot dimensions and safety parameters used by ais_safety      
│       ├── robot_ws/src/[phoenix_visualization](https://bitbucket.org/ais_admin/phoenix_visualization/src/main/)  Simulation components     
│       └── robot_ws/src/[phoenix_vehicle_model](https://bitbucket.org/ais_admin/phoenix_vehicle_model/src/main/)  Vehical model used in simualtion      
└── simulation_ws Workspace that is used alongside the robot_ws to develop the simulation environment, ther rule of thumb is that if it is not going to be used in the robot goes in the simulation_ws    
    └── simulation_ws/src  
        └── simulation_ws/src/[phoenix1_simulation](https://bitbucket.org/ais_admin/phoenix1_simulation/src/main/)  Simulation bringup package
---

# Simulation

In order to develop inside the container with the share volumes on the host export to variables `SIMULATION=true` and `DEVELOPING=true` the logic behind can be found [here](https://bitbucket.org/ais_admin/phoenix1_bringup/src/8ebc7848f5a46b07551bf58778eba8c24908ad75/scripts/run_bringup.bash#lines-9)  

```
export TAG=develop* --> Please take a look at the next section
export ROBOT_NAME=condor
make docker-pull-dev --> Skip this step if you want to use your local image
```

- If you are running the simulation for the first time, please update the docker image.
```
make docker-update-dev
```

- If you have an NVIDIA GPU on your computer, run this commands under the ais-binders folder:
```
make docker-start-sim
```

- Otherwise, run:
```
make docker-start-sim-cpu
```

- To stop the simulation, run:
```
make docker-stop-dev
```

# Development Workflow of the Simulation with the App

In order to use app in your simulation follow these steps:

- First of all make sure your simulation is runnning (procedure is explained in the "Simulation" section).

- Clone the Deploy Phoenix repository from [this link](https://bitbucket.org/ais_admin/deploy-phoenix/src/master/).

- If you have not installed, install docker-compose and run the following command under Deploy Phoenix repository to run the local websocket:
```
docker-compose -f docker-compose.websocket.sim.yml up -d
```

- run the following command under Deploy Phoenix repository to run the app:
```
docker-compose -f docker-compose.frontend.yml up -d
```

- Use [this link](https://localhost:44583/) for the permission.

- Now the app is running in [https://localhost:3000/](https://localhost:3000/) port.

# Deployment the Simulation with the App

- Run the following comands under phoenix-binder repository:
```
make docker-pull-prod --> Skip this step if you want to use your local image
```

- If you are running the simulation for the first time, please update the docker image.
```
make docker-update-prod
```

- Clone the Deploy Phoenix repository from [this link](https://bitbucket.org/ais_admin/deploy-phoenix/src/master/). Then run the following make command under deploy-phoenix repository:
```
make start-sim
```
# Development Workflow
- Run these commands inside the phoenix binder folder:
```
make docker-start-dev
make docker-enter-dev
make build
source install/setup.bash
```
- Stop the Docker container when you are done:
```
make docker-stop-dev
```

# Development Workflow in the robot
- Run these commands inside the phoenix binder folder as root, [deploy-phoenix](https://bitbucket.org/ais_admin/deploy-phoenix/src/master/?search_id=91e9b41b-f326-44a1-890a-a8df13f5ea9b)should be running as root as well.
```
After bringing up all the containers with the deploy-phoenix stop the phoenix-navigation container with
docker stop phoenix-navigation
cd phoenix-binder (directory where ever it is cloned)
export TAG=develop --> develop is an example for the branch but one can change the branch name
make docker-login -- or make dockerhub-login 
make docker-pull-dev (One can also build the image directly on the robot's computer and export the tag)
sudo su
export TAG=develop --> develop is an example for the branch but one can change the branch name
make docker-start-robot-cpu
make docker-enter-dev
make build-robot-workspace
source robot_ws/install/setup.bash
```
- Stop the Docker container when you are done:
```
make docker-stop-dev
```

# Production Workflow
- [Install Docker Compose](https://docs.docker.com/compose/install/) if you have not.
- [Clone the Deploy Phoenix](https://bitbucket.org/ais_admin/deploy-phoenix/src/master/)
- Inside the deploy-phoenix
  - First login to the ECR registry (https://ais-ugv2.atlassian.net/wiki/spaces/ADE/pages/2907340808/AWS+ECR+Docker+registry)

Then use this commands to start the container in the robot

``` 
make docker-login
make start
make enter
tmux a
```

# Building and Pushing Docker Images

```
make src-update-submodules
make docker-build-base
make docker-push-base
make docker-build-dev
make docker-push-dev
make docker-build-prod
make docker-push-prod
```

# Running Docker Containers

- Running the base Docker image:
```
   make docker-start-base
```
- Running the development Docker image:
```
   make docker-start-dev
```
- Running the production Docker image:
```
   make docker-start-prod
```

# Opening a terminal on Docker Containers

- Entering the base Docker container:
```
   make docker-enter-base
```
- Entering the development Docker container:
```
   make docker-enter-dev
```
- Entering the production Docker container:
```
   make docker-enter-prod
```

# Stopping Docker Containers

- Stopping the base Docker container:
```
   make docker-stop-base
```
- Stopping the development Docker container:
```
   make docker-stop-dev
```
- Stopping the production Docker container:
```
   make docker-stop-prod
```

# Developing inside the container as $USER for the simulation
- Make sure you have updated the submodules
```
make src-update-submodule
```

- Build the whole workspace
```
export TAG=develop* --> Please take a look at the next section
export ROBOT_NAME=condor
make docker-pull-dev --> Skip this step if you want to use your local image
make docker-start-dev --> Use TAG=local if you built the image locally
make docker-enter-dev
make src-update-submodules --> this is to clone and recursively update your submodules.
make build-robot-workspace
source robot_ws/install/setup.bash
make build-simulation-workspace
source simulation_ws/install/setup.bash
export SIMULATION=true
export DEVELOPING=true
roslaunch phoenix1_bringup bringup.launch

```
-  Attention developers
```export TAG=develop``` will pull ```62427299064.dkr.ecr.ca-central-1.amazonaws.com/binders:cartpuller-dev-develop``` from AWS. If you used Jira to create a branch or the branch starts with feature, hotfix, bugfix, or release it will automatically build on the Jenkins pipeline. You can pull this image using: ```export TAG=feature-OCP-100-your-branch``` for the branch: ```feature/OCP-100-your-branch``` and then ```make docker-pull-dev```
For this to work the jenkins build should have finished, otherwise you can `export TAG=develop` or `export TAG=master`
[This is where it is used in Jenkins](https://bitbucket.org/ais_admin/cartpuller-binder/src/2591da86a867c2618c2454da26b7cc4eee9bd9a1/binder/jenkins/Jenkinsfile#lines-25) and [in the ais-makefile](https://bitbucket.org/ais_admin/ais-makefile/src/becabc1dd7adb917727dcb543dfd9e9a70a7f212/Makefile#lines-165)

# Activating and Deactivating SLAM-Toolbox
This instruction provide the required step to activate and deactivate the SLAM algorithm

- To activate SLAM
```
rosservice call /activate_slam_toolbox
```

- To Deactivate SLAM
```
rosservice call /deactivate_slam_toolbox
```


# Infrastructure
The jenkinks server can be found in the following address `http://build.ais-api.com:8080/`

# Note to ADE
As of `2021-09-01` [ADE is compatible with Gitlab and Dockerhub registries](https://gitlab.com/ApexAI/ade-cli/-/blob/master/ade_cli/credentials.py#L90-100), to overcome this build the dev pipeline locally with
```
make docker-login # This command will log you inside the AWS ECR registry
make src-update-submodules
make docker-build-dev # Builds the development image
```
Or download it from the docker registry and tag it again
```
docker pull 062427299064.dkr.ecr.ca-central-1.amazonaws.com/binders:phoenix1-binder-dev
docker tag 062427299064.dkr.ecr.ca-central-1.amazonaws.com/binders:phoenix1-binder-dev binders:phoenix1-binder-dev
```

Then use [ade](https://ade-cli.readthedocs.io/en/latest/#) as usual.  
