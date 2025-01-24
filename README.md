# Vortex Trajectory Tracking Code Repository

_Setup:_
parameters.m: Defines set of physical and control parameters common in all files and functions

_ODE Functions:_
vortexConvection.m: Function to define vortex velocity from vortex position (R^2 -> R^2)
stateDynamics.m: Nonlinear state-space function for the full three-dimentional system (R^3 -> R^3)

_ODE Options:_
timeEvent.m: Option to terminate ODE23s after one second
timeEventLong.m: Option to terminate ODE23s after one minute

_Main Scripts:_
phaseControl.m: Runs control algorithm and generates plots for single system configuration
openLoopErrors.m: Iterates through set of initial conditions and stores open-loop convergence metrics
regionOfConvergence.m: Iterates through set of initial conditions and stores closed-loop convergence metrics
