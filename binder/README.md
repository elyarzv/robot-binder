Dockerfiles
    - Dockerfile.subbase --> caches all dependencies for underlay_ws and installs dependencies for wrappers.
    - Dockerfile.base --> caces all dependencies for robot_ws and builds underlay_ws.
    - Dockerfile.dev --> build robot_ws and simulation_ws
    - Dockerfile.prod --> copies binaries from Dockerfile.dev and is based on development image