# Model_predictive_control

MPC using various optimisation algorithms

Instructions to run:

- Clone the repository:

  `git clone https://github.com/susiejojo/Model_predictive_control.git`
  
- Open in MATLAB and execute `run.m`.

Files:

- `run.m`: main code to generate simulations, and run the optimisation routine.
- `getPreds.m`: used to return predictions based on waypoints for a prediction horizon. Uses L2 norm for cost, L2 regularisation of w.
- `nonhn_pts.m`: used to generate the x,y coordinates given a set of linear and angular velocities
