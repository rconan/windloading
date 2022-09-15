# GMT FEM WIND LOADING

Applies CFD wind loads (forces and moments) to a FEM of the GMT while closing the loop on the mount elevation, azimuth and GIR axes with the mount control system.

The following time series are saved:

 * M1 rigid body motions,
 * M2 rigid body motions,
 * M1 bending modes

 The following environment variables must be set:

  * FEM_REPO: path to the directory with the FEM state space model
  * M1CALIBRATION: path to the directory with the M1 modes to force and force to modes matrices  