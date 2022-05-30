# %%

import pyomo.environ as pyo
from pyomo.dae import ContinuousSet,  DerivativeVar
from models.NGCC.ngcc_3in_18out import create_ngcc_ROM
import pandas as pd
from idaes.core.util.model_statistics import degrees_of_freedom
from idaes.core.solvers import get_solver

import random 


u_NOM = pd.read_csv('data/NGCC/NGCC_H2_Input_100.txt',
                        header=None,index_col=0, sep=None, engine='python')
y_NOM = pd.read_csv('data/NGCC/NGCC_H2_Output_100.txt', 
                        header=None, index_col=0, sep=None, engine='python')

m = pyo.ConcreteModel()
m.T = ContinuousSet(bounds=(0,100))

create_ngcc_ROM(m, m.T)

discretizer = pyo.TransformationFactory('dae.finite_difference')
discretizer.apply_to(m, wrt = m.T, nfe = 100)


for t,j in m.u:
    m.u[t,j].fix(u_NOM[1][j])
m.x[0,:].fix(0)
    
print(degrees_of_freedom(m))

solver = pyo.SolverFactory('ipopt')

# solver.solve(m, tee= True)


i = 1
for ind in m.u:
    t = ind[0]
    if t in [5, 25, 75, 90]:
        m.u[t,i].fix(u_NOM[1][i]*(1 +0.05*random.gauss(-1,1)))
solver.solve(m, tee= True)

import matplotlib.pyplot as plt

plt.plot(m.T, m.u[:,i]())
plt.xlabel('Time (min)')
plt.ylabel('NG flowrate')
plt.title('Disturbance in input')
plt.show()

plt.plot(m.T, m.y[:,1]())
plt.xlabel('Time (min)')
plt.ylabel('Gross power output')
plt.title('Disturbance in output')
plt.show()


