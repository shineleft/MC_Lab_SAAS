# Chuanlin's Mission_coordination_project

## Step 4: Move one robot to the corresponding flag
In one terminal
```bash
roslaunch evry_project_description simu_robot.launch
```
In another terminal
```bash
roslaunch evry_project_strategy agent_with_pid.launch nbr_robot:=1
```
## Step 5: Implementation of one strategy - timing solution
In one terminal
```bash
roslaunch evry_project_description simu_robot.launch
```
In another terminal
```bash
roslaunch evry_project_strategy agent_with_timing.launch
```

# Lab2: simple and non-robust strategy
## Strategy 1: reactive obstacle avoidance mechanism
In one terminal
```bash
roslaunch evry_project_description simu_robot.launch
```
In another terminal
```bash
roslaunch evry_project_strategy agent_strategy1.launch
```
## Strategy 2: Artificial Potential Field
In one terminal
```bash
roslaunch evry_project_description simu_robot.launch
```
In another terminal
```bash
roslaunch evry_project_strategy agent_strategy2.launch
```
