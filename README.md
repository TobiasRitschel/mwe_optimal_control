# MWE Optimal Control
This repository contains a minimal working example (MWE) of how to solve an optimal control problem in Matlab using a single-shooting approach implemented with fmincon and ode45.

The example system is

dx/dt = -x + u,

and the objective is to minimize

int_t0^tf (y - yref)^2 dt,

where y = x.

The problem contains multiple decision variables. Specifically, the decision variables are the parameters in a zero-order-hold parametrization of the manipulated, i.e., the (constant) value of the manipulated input in each control interval.

## Authors
This code was developed by Tobias K. S. Ritschel from the Technical University of Denmark.
