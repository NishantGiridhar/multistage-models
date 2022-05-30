# %% 

import pyomo.environ as pyo
from pyomo.dae import ContinuousSet, DerivativeVar
import pandas as pd
from idaes.core.util.model_statistics import degrees_of_freedom
from idaes.core.solvers import get_solver
import random 
import matplotlib.pyplot as plt

from models.NGCC.ngcc_3in_18out import create_ngcc_ROM


if __name__ == "__main__":
    u_NOM = pd.read_csv('data/NGCC/NGCC_H2_Input_100.txt',
                            header=None,index_col=0, sep=None, engine='python')
    y_NOM = pd.read_csv('data/NGCC/NGCC_H2_Output_100.txt', 
                            header=None, index_col=0, sep=None, engine='python')

    m = pyo.ConcreteModel()
    m.T = ContinuousSet(bounds=(0,100))


    create_ngcc_ROM(m, m.T)

    discretizer = pyo.TransformationFactory('dae.finite_difference')
    discretizer.apply_to(m, wrt = m.T, nfe = 100)
    m.obj = pyo.Objective(expr = 0)



    m.c1.deactivate()
    m.c2.deactivate()
    m.dxdt_disc_eq

    # initial conditions at t = 0
    for i in m.N:
        m.x[0, i].fix(0)


def single_step(m, t_ind):

    for i in m.N:
        m.c1[t_ind,i].activate()
        if t_ind !=0:
            m.dxdt_disc_eq[t_ind,i].activate()
    for i in m.P:
        m.c2[t_ind,i].activate()

    for i in m.M:
        m.u[t_ind,i].fix(u_NOM[1][i])



    m.u[t_ind,1].fix(u_NOM[1][1]*(1 +0.05*random.gauss(-1,1)))

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
    

for t in m.T:
    print(f'{t} of {m.T.last()}')
    single_step(m, t)


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
