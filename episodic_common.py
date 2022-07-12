# %%

import pyomo.environ as pyo
from pyomo.dae import ContinuousSet

from NGCC.model.ngcc_3in_18out import create_ngcc_ROM 

from idaes.core.util.model_statistics import degrees_of_freedom


class ControlEpisodeBlock:
    def __init__(self, max_duration):
        self.max_time = max_duration
        self.process_model = None

        self.step_reward = None
        self.episode_reward = {}


    def load_process_model(self, model_builder, discretize, nfe):
        if self.process_model == None:
            self.process_model = pyo.ConcreteModel()
            self.process_model.time = ContinuousSet(bounds = (0, self.max_time))
            model_builder(self.process_model, self.process_model.time)
            m = self.process_model

            if discretize:
                discretizer = pyo.TransformationFactory('dae.finite_difference')
                # nfe = 100
                discretizer.apply_to(m, wrt = m.time, nfe = nfe)

    def initialize_episode(self):
        m = self.process_model
        m.c1.deactivate()
        m.c2.deactivate()
        m.dxdt_disc_eq.deactivate()

        # initial conditions at t = 0
        for i in m.N:
            m.x[0, i].fix(0)
        
        self.step_reward = []


    def simulate_step(self, t_ind, u_c, u_d, u_f):
        m = self.process_model
        for i in m.N:
            m.c1[t_ind,i].activate()
            if t_ind !=0:
                m.dxdt_disc_eq[t_ind,i].activate()
        for i in m.P:
            m.c2[t_ind,i].activate()
        # Add some noise to the first input variable (NG flow)
        # random.seed(123)
        m.u[t_ind,1].fix(u_d)
        # Fix the second variable to the level obtained from the controler
        m.u[t_ind, 2].fix(u_c)
        # The third input variable can be fixed 
        m.u[t_ind,3].fix(u_f)

        assert degrees_of_freedom(m) ==0, f'D.O.F = {degrees_of_freedom(m)}'

        # Solve the timestep 
        solver = pyo.SolverFactory('ipopt')
        solver.solve(m, tee= False)     

        # Fix the evaluated variables at their current values
        for i,j in m.x:
            m.x[t_ind, j].fixed = True
        for i,j in m.y:
            m.y[t_ind, j].fixed = True

        # Deactivate all constraints
        m.c1.deactivate()
        m.c2.deactivate()
        m.dxdt_disc_eq.deactivate()

    def simulate_episode(self, 
                        control_model, 
                        disturbance_model,
                        fixed_var_value, 
                        early_termination_criterion,
                        save_json = False
                        ):

        '''
        control_model --> function: takes measured var values as input, returns control action
        disturbance_model --> function: any function (random, predetermined fn of t etc.) 
        fixed_var_value --> dictionary of values for fixed variables
        early_termination_criterion --> function: returns boolean      
        '''
        self.initialize_episode()

        m =  self.process_model
        
        for t in m.time:
            if t == 0:
                u_c = u_NOM[1][2] 
                u_d = u_NOM[1][1]
                u_f = fixed_var_value

                self.simulate_step(t_ind = t, u_c=u_c, u_d=u_d, u_f=u_f)
                print(f'simulating t = {t} of {m.time.last()}')

    
            elif t >= 1:
                if early_termination_criterion(t) == False:
                    measured_variable = m.y[t-1, 1]()
                    set_point = y_NOM[1][1]
                    u_c = control_model(measured_variable, set_point)
                    u_d = disturbance_model(t)
                    u_f = fixed_var_value

                    self.simulate_step(t_ind = t, u_c=u_c, u_d=u_d, u_f=u_f)
                    print(f'simulating t = {t} of {m.time.last()}')
                elif early_termination_criterion(t) == True:
                    print(f'early termination criterion met at {t}')
        
        print(f'Episode simulated')

        if save_json:
            pass


# Minimal working example

if __name__ == "__main__":
    import pandas as pd
    import random
    from idaes.core.solvers import get_solver


    u_NOM = pd.read_csv('NGCC/model/data/NGCC_H2_Input_100.txt',
                            header=None,index_col=0, sep=None, engine='python')

    y_NOM = pd.read_csv('NGCC/model/data/NGCC_H2_Output_100.txt', 
                                header=None, index_col=0, sep=None, engine='python')

    
    def get_disturbance(t):
        if t <= 20:
            random.seed(32)
            return u_NOM[1][1]*(1 + 0.05*random.gauss(0,1))
        elif t == 30:       
            random.seed(57)
            return  u_NOM[1][1]*(1 + 0.05*random.gauss(0,1))
        elif 60 <= t <= 90:
            random.seed(125)
            return  u_NOM[1][1]*(1 - 0.05*random.gauss(0,1))
        else: 
            return u_NOM[1][1]



    def get_control(measured_variable, set_point):
        mc = pyo.ConcreteModel()

        mc.y = pyo.Var()        # This is the measured variable from mp
        mc.u = pyo.Var()        # This is the output of the control model (manipulated variable u2)

        mc.y_sp = pyo.Var()

        @mc.Constraint()
        def const(mc):
            return mc.u == u_NOM[1][2] - (mc.y - mc.y_sp)/mc.y_sp * u_NOM[1][2]


        mc.y.fix(measured_variable)
        mc.y_sp.fix(set_point)

        solver = pyo.SolverFactory('ipopt')
        solver.solve(mc)

        return mc.u()

    def early_termination_criterion(t):
        return False


    NGCC_block = ControlEpisodeBlock(max_duration = 60)

    NGCC_block.load_process_model(model_builder = create_ngcc_ROM, 
                                discretize = True, nfe = 60)
    
    NGCC_block.simulate_episode(control_model       = get_control,
                                disturbance_model   = get_disturbance,
                                fixed_var_value     = u_NOM[1][3],
                                early_termination_criterion = early_termination_criterion)
                                # TODO: 
    
# %%  
    

