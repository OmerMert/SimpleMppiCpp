# SimpleMppiCpp

This is the simple mppi implementation in Cpp. 
Control algorithm is written in Cpp and only for visualize the simulation Python is used.
UDP is used for communication between Cpp and Python.
Simple bath file is created for build and run both Cpp and Python files.

| Parameter | Description                                | Effect of Increasing                                       | Effect of Decreasing                          |
|-----------|--------------------------------------------|------------------------------------------------------------|-----------------------------------------------|
| K         | Number of trajectories sampled per step.   | Smarter, smoother driving (Low FPS).                       | Faster calculation, but "blind" behavior.     |
| T         | Prediction steps into the future.          | Better foresight for curves.                               | late reactions.                               |
| lambda    | Selectivity of the algorithm.              | "Average" behavior, sluggish response.                     | "Greedy" behavior, jerky/unstable.            |
| Sigma     | Variance of the control noise.             | Explores wider paths (can be unstable).                    | Converges to local minima (stuck).            |
| alpha     | Balance between effort vs. tracking.       | (Near 1.0): Aggressive tracking.                           | (Near 0.0): Lazy, fuel-efficient driving.     |


## Usage
```sh
git clone https://github.com/OmerMert/SimpleMppiCpp.git
cd SimpleMppiCpp
build_and_run.bat
```

## Reference
https://github.com/MizuhoAOKI/python_simple_mppi/tree/master