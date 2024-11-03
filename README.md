# cev-sim
Simulator for CEV related vehicles

## Goals

- Write down useful models in Python / C++
- Visualize models in PyGame
- Allow models to modularly decare what their (basic) inputs / outputs are
- Connect models and their I/O to a ROS2 network

Later,
- Add more complicated outputs that interact with environment, like LiDAR and Camera (likely requires a more complicated engine)
- Add visualization in said more complicated engine
- Nice configurability features: changing model parameters, process and sensor noise, and classes to override to let you implement more "fine tuned" features.