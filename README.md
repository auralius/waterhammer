# waterhammer
Water hammer simulation and optimal control 

Based on the paper by: 

Chen, T., Xu, C., Lin, Q., Loxton, R., & Teo, K. L. (2015). Water hammer mitigation via PDE-constrained optimization. Control Engineering Practice, 45, 54–63. https://doi.org/10.1016/j.conengprac.2015.08.008

Chen, T., Ren, Z., Xu, C., & Loxton, R. (2015). Optimal boundary control for water hammer suppression in fluid transmission pipelines. Computers & Mathematics with Applications, 69(4), 275–290. https://doi.org/10.1016/j.camwa.2014.11.008

-----------------------------

### 1. test1.m

A constant closure rate is applied to the valve. Simulation runs for 10 seconds. The pipeline is divided into 24 segments (25 nodes). Pressure and velocity data are taken from each node every 1 second.

### 2. test2.m

Optimization is applied to minimize water-hammer effect at the pipe terminus. There are three solvers provided: sqp, patternsearch and [pdfo](https://www.mathworks.com/matlabcentral/fileexchange/75195-pdfo-powell-s-derivative-free-optimization-solvers). For the SQP solver, the derivative of the Jacobian will be computed numerically since the default Jacobian computation provided by MATLAB actually fails the SQP solver.
