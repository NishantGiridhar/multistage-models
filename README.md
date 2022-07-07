# multistage-model examples



## 1. Two stage control problem 

We would need a time horizon set T := [0,1,2,3,....$t_f$]

Let's say we have two optimization models, the master problem (denoted by m) and the "simulation" (can be optimization as well) model (denoted by s). 

$$m := $$
   


For trying this out, simply consider any standard linear (or even nonlinear) state space model from the book/literature: 

This is of the form: 
$$x_{t+1}=x_t+\Delta x = Ax_t+bu_t$$
$$y_{t+1}=h(x_{t+1}) = Cx_{t+1}$$
   Out of the inputs ‘u’ consider 1 or more as disturbance (i.e. fixed values in IDAES that one can change as one wishes) and 1 or more ‘u’ as decision variable (i.e. to be optimized).
   
    Simulation approach:

    Say we want to solve for 100 min discretized every 1 min or $t_f$=100 below, 
    - For each ‘i’ you can solve an optimization problem (can be anything as a function of ‘y_i+1’) and then go out of that time index, update ‘u_i+2’ (here only update one or more disturbance variables whichever way you wish by  increasing/decreasing them by 1%, say) and so on and resolve the optimization problem within the loop using the linear/nonlinear model.
    -  This approach will eventually be applied for reinforcement learning-based control of SOFC/SOEC where the model will be replaced by the SOFC/SOEC model. We will do this for the PETSc integrator approach first- but more on that later. Dan has been working on the reinforcement learning approach now where we want to employ this approach. Nishant is also familiar with this SOFC/SOEC code good bit. I am copying both of them in case they have any input.


### Inner process model (mp): In-house NGCC model: 3 inputs-18 outputs
- u[1] : NG flow rate (disturbance variable)
- u[2] : H2 flow rate (control variable)
- u[3] : Something    (cOnstant variable)
