# waterhammer
Water hammer simulation and optimal control 

Based on the paper by: 

Chen, T., Xu, C., Lin, Q., Loxton, R., & Teo, K. L. (2015). Water hammer mitigation via PDE-constrained optimization. Control Engineering Practice, 45, 54–63. doi:10.1016/j.conengprac.2015.08.

-----------------------------

The following animations and plot are for the scenario in which the valve is closed at a contant rate (see Section 4 of the paper).

![](https://github.com/auralius/waterhammer/blob/main/waterhammer.gif)

![](https://github.com/auralius/waterhammer/blob/main/pipeline_terminus_pressure.png)

-----------------------------

Next, we will find the optmized valve closing policy. Here, we will use MATLAB fmincon with SQP algorithm.

![](https://github.com/auralius/waterhammer/blob/main/waterhammer-1.png)

![](https://github.com/auralius/waterhammer/blob/main/waterhammer-2.png)

![](https://github.com/auralius/waterhammer/blob/main/waterhammer-3.png)

![](https://github.com/auralius/waterhammer/blob/main/waterhammer-4.png)

![](https://github.com/auralius/waterhammer/blob/main/waterhammer-5.png)
