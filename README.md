# Model_predictive_control

MPC using various optimisation algorithms

### Instructions to run:

- Clone the repository:

  `git clone https://github.com/susiejojo/Model_predictive_control.git`
  
- `cd` into the directory and make a new `/data` directory. This can be done by:

  `mkdir data`
  
- Open in MATLAB and execute `run.m`.
- The simulation images are saved under `/data`. 
- To generate videos, copy `mkmovie.sh` into the `/data` folder and execute the shell script by:

  `./mkmovie.sh`.

### Files and directories:

- `run.m`: main code to generate simulations, and run the optimisation routine.
- `getPreds.m`: used to return predictions based on waypoints for a prediction horizon. Uses L2 norm for cost, L2 regularisation of w.
- `nonhn_pts.m`: used to generate the x,y coordinates given a set of linear and angular velocities.
- `plot_figs.m`: used to plot the simulation frame by frame. Adjusts the heading and position of the agent as given by the optimiser.
- `colnfn.m`: returns collision constraints and lane constraints. Contains flags `has_obstacle` and `has_lane_con` to toggle constraints.
- `mkmovie.sh`: used to generate video from the image frames obtained from MATLAB.
- `/data`: stores the images frame by frame for the simulation. Also on running the video generator code, generates `simulation.mp4`.
- `/results`: stores the simulations obtained during testing. 

### Results:

- All the results below have been obtained with `planning_horizon = 50` and `control_horizon = 10`.
- The green points denote the positions returned by the planner over the planning horizon.
- The blue trajectory is the actual path followed by the bot.
- The black dashed lines denote lane constraints.
- Obstacle has been assumed to be a circle of a given radius.
- Initial w and v: v = 1, w = uniform(-0.06,0.06)
- `vmin = 0`, `vmax = 20`, `wmin = -0.1` `wmax = 0.1`. (Till Video7), `wmin = -0.5`, `wmax = 0.5` for Videos 8,9.
- `MaxFunctionEvaluations = 30000`,`MaxIterations = 10000` for Videos 8 and 9.
- Lane Constraints(where applicable in case of y=x): left => y = x+25, right => y = x-25


| Scenario      | Initial heading | Waypoints | End orientation constraint | Obstacle | Lane Constraint | Video
| ----------- | ------------------| ----------- | -------------------------- | -------- | --------------- | -----
| Move from (0,0) to (125,125) | pi/4 | 50 waypoints, equally spaced | None | None | None | [Video1](https://github.com/susiejojo/Model_predictive_control/blob/master/results/simulation_MPC_50wpts.mp4)
| Move from (0,0) to (80,80)   | 0    | 1 waypoint at goal | None | None | None | [Video2](https://github.com/susiejojo/Model_predictive_control/blob/master/results/simulation_heading0_200.mp4) |
| Move from (0,0) to (80,80)   | pi/4 | 1 waypoint at goal | None | None | None | [Video3](https://github.com/susiejojo/Model_predictive_control/blob/master/results/simulation_directed_heading00.mp4)
| Move from (0,0) to (50,0)    | 0    | 1 waypoint at goal | None | None | None | [Video4](https://github.com/susiejojo/Model_predictive_control/blob/master/results/simulation_stline.mp4)
| Move from (0,0) to (50,0)    | 0    | 1 waypoint at goal | None | (30,0) | None | [Video5](https://github.com/susiejojo/Model_predictive_control/blob/master/results/simulation_stline_obst.mp4)
| Move from (0,0) to (80,80)   | pi/4 | 1 waypoint at goal | 0 | None | None | [Video6](https://github.com/susiejojo/Model_predictive_control/blob/master/results/simulation_endor.mp4)
| Move from (0,0) to (80,80)   | pi/4 | 1 waypoint at goal | None | (30,30) | None | [Video8](https://github.com/susiejojo/Model_predictive_control/blob/master/results/simulation_x%3Dy_obst.mp4)
| Move from (0,0) to (80,80)   | pi/4 | 1 waypoint at goal | None | (30,30) | As in video | [Video9](https://github.com/susiejojo/Model_predictive_control/blob/master/results/simulation_lane_obst_endor.mp4)
| Move from (0,0) to (50,50)   | pi/4 | 1 waypoint at goal | 0    | (30,30) | As in video | [Video10](https://github.com/susiejojo/Model_predictive_control/blob/master/results/simulation_lane_obst_endor.mp4)

### Observations: 

- Uniformly distributed w give better results than fixed w.
- Norm of vector vs norm of matrix
- Adjusting values of vmax and plotting v w.r.t time to ensure a smooth graph
- Setting acceleration constraints to ensure smoothness of v.
- Removing collision constraint once the bot has passed the obstacle and is ahead of it.
- Fine-tuning weights for end orientation constraint
- Adjusting values of wmax for making achieving end orientation constraints feasible.
- Feasilibity of reaching goal vs values of vmax and vmin, time step and planning horizon
- Difference between multiple waypoints and a single waypoint
- fmincon requires manual tuning of MaxIterations and MaxFunctionEvaluations to be able to complete the optimization routine.
- Learnt about fmincon exit codes, and read up fmincon documentation for options parameters.
