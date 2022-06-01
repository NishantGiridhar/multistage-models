# %% 

import pyomo.environ as pyo
from pyomo.dae import ContinuousSet, DerivativeVar
import pandas as pd
from idaes.core.util.model_statistics import degrees_of_freedom
from idaes.core.solvers import get_solver
import random 
import matplotlib.pyplot as plt
import os


cpath = cpath = os.path.dirname(__file__)
u_NOM = pd.read_csv(f'{cpath}/model/data/NGCC_H2_Input_100.txt',
                        header=None,index_col=0, sep=None, engine='python')
y_NOM = pd.read_csv(f'{cpath}/model/data/NGCC_H2_Output_100.txt', 
                        header=None, index_col=0, sep=None, engine='python')
# The code under this construct will only run if you run this file directly
if __name__ == "__main__":
    from model.ngcc_3in_18out import create_ngcc_ROM


    

    m = pyo.ConcreteModel()
    m.T = ContinuousSet(bounds=(0,100))


    create_ngcc_ROM(m, m.T)

    discretizer = pyo.TransformationFactory('dae.finite_difference')
    discretizer.apply_to(m, wrt = m.T, nfe = 100)
    m.obj = pyo.Objective(expr = 0)

    print('check')
    m.c1.deactivate()
    m.c2.deactivate()
    m.dxdt_disc_eq.deactivate()
    
    # initial conditions at t = 0
    for i in m.N:
        m.x[0, i].fix(0)


def single_step(m, t_ind, u_c, u_d):
    
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
    m.u[t_ind,3].fix(u_NOM[1][3])

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

# This function doesnt return anything but will keep updating the model object 
# as we move along the time horizon

if __name__ == "__main__":
    for t in m.T:
        print(f'{t} of {m.T.last()}')
        single_step(m, t, u_c = u_NOM[1][2])


    # Make some plots if you want 
    plt.plot(m.T, m.u[:,1]())
    plt.xlabel('Time (min)')
    plt.ylabel('NG flowrate')
    plt.title('Disturbance in input')
    plt.show()

    plt.plot(m.T, m.y[:,1]())
    plt.xlabel('Time (min)')
    plt.ylabel('Gross power output')
    plt.title('Disturbance in output')
    plt.show()




# %%
