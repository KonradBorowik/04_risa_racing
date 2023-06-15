# 04_risa_racing
First download and follow instructions from https://github.com/amadeuszsz/AWSIM/releases/tag/v1.1.0_f1tenth


Then clone package:</br>
```git clone https://github.com/KonradBorowik/04_risa_racing.git```

Build package:</br>
`colcon build --packages-select 04_risa_racing`

Run after opening Unity simulation to drive:</br>
`ros2 run 04_risa_racing main`</br>
It launches a follow-the-gap type algorythm implemented in main.py. Steering controler is implemented in utils/pid.py.

---
A work in progres is trajectory planning. To see current outcome run:</br>
`ros2 run 04_risa_racing planner`</br>
After converting image of track layout to an array of 0s and 1s (where 1s are obstacles and 0s track) we tried to implement Dijkstra's algorithm to find a path around the circuit. Unfortunately that is where we are stuck for now...
