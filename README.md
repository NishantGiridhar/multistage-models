

# EpisodicLearningBlock

- The EpisodicLearningBlock contains the python and pyomo methods that can be used to convert a general dynamic pyomo model into one suitable to solve episodic control problems. 

- The present version is a work in progress will need some handholding functions. A minimal working example for a 3 input 18 output state space model is provided under the `if __name__ == "__main__"` construct.


The following functions are currently implemented:

The first thing to do is to create an instance of the `EpisodicLearningBlock` class. 

Inputs: 
- max_duration (float)

`load_process_model(model_builder, discretize, nfe)` 

Inputs:
- model_builder : function that instantiates a pyomo.DAE model given a continuous set

- discretize : boolean 
- nfe        : Number of finite elements

`initialize_episode()`

__Private function:__ Don't have to call this unless explicitly required

Creates boilerplate structures necessary for episode.

Activates and deactivates necessary constraints.


Can add any instructions that must be completed before simulating an episode

`simulate_step(t_ind, u_c, u_d, u_f)`


__Private function:__ Don't have to call this unless explicitly required

Simulates one timestep by activating and deactivating the necessary constraints

- t_ind : time index to be simulated
- u_c   : control input variable value
- u_d   : disturbance input variable value
- u_f   : fixed input variable value

`simulate_episode(control_model, disturbance_model,fixed_var_value early_termination_criterion, save_json = False)`


- control_model : function that takes in measured variables and a set point and returns control action
- disturbance_model : function that simulates a disturbance at each time point
- fixed_var_value : level(s) of fixed variables
- early_termination_criterion : boolean that must be true when the criterion is met
- save_json : boolean to indicate if results of an episode must be saved 