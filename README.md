# Model_predictive_control

MPC using various optimisation algorithms

Instructions to run:

- Clone the repository:

  `git clone https://github.com/susiejojo/Model_predictive_control.git`
  
- `cd` into the directory and make a new `/data` directory. This can be done by:

  `mkdir data`
  
- Open in MATLAB and execute `run.m`.
- The simulation images are saved under `/data`. 
- To generate videos, copy `mkmovie.sh` into the `/data` folder and execute the shell script by:

  `./mkmovie.sh`.

Files and directories:

- `run.m`: main code to generate simulations, and run the optimisation routine.
- `getPreds.m`: used to return predictions based on waypoints for a prediction horizon. Uses L2 norm for cost, L2 regularisation of w.
- `nonhn_pts.m`: used to generate the x,y coordinates given a set of linear and angular velocities
- `mkmovie.sh`: used to generate video from the image frames obtained from MATLAB.
- `/results`: stores the images frame by frame for the simulation. Also on running the video generator code, generates `simulation.mp4`.
