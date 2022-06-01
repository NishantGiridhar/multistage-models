# %% 
'''
Here we want two models to be solved simultaneously, 
    the outer control model (mc) should take some output variables from from the process model (mp) as an input,
    and use it to predict a future input value for the process model (mp)
'''

import pyomo.environ as pyo
from pyomo.dae import ContinuousSet, DerivativeVar
import pandas as pd
from idaes.core.util.model_statistics import degrees_of_freedom
from idaes.core.solvers import get_solver
import random 
import matplotlib.pyplot as plt


from NGCC.model.ngcc_3in_18out import create_ngcc_ROM
from NGCC.NGCC_one_step import single_step

u_NOM = pd.read_csv('NGCC/model/data/NGCC_H2_Input_100.txt',
                        header=None,index_col=0, sep=None, engine='python')

y_NOM = pd.read_csv('NGCC/model/data/NGCC_H2_Output_100.txt', 
                            header=None, index_col=0, sep=None, engine='python')


solver = pyo.SolverFactory('ipopt')
# %% Process model

mp = pyo.ConcreteModel()
mp.T = ContinuousSet(bounds = (0, 100))

create_ngcc_ROM(mp, mp.T)

discretizer = pyo.TransformationFactory('dae.finite_difference')
discretizer.apply_to(mp, wrt = mp.T, nfe = 100)

# Start of by deactivating all constraints
mp.c1.deactivate()
mp.c2.deactivate()
mp.dxdt_disc_eq.deactivate()

# initial conditions at t = 0
for i in mp.N:
    mp.x[0, i].fix(0)

# %% "Controller" model

mc = pyo.ConcreteModel()

mc.y = pyo.Var()        # This is the measured variable from mp
mc.u = pyo.Var()        # This is the output of the control model (manipulated variable u2)

mc.y_sp = pyo.Var()

@mc.Constraint()
def const(mc):
    return mc.u == u_NOM[1][2] - (mc.y - mc.y_sp)/mc.y_sp * u_NOM[1][2]

def solve_control(control_model, measured_variable, set_point):
    _mc = control_model
    _mc.y.fix(measured_variable)
    _mc.y_sp.fix(set_point)

    solver.solve(_mc)

# %% Putting them together


def disturbance_profile(t):
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

for t in mp.T:
    if t == 0:
       u_c = u_NOM[1][2] 
       u_d = u_NOM[1][1]
    if t >= 1:
        solve_control(mc, mp.y[t-1, 1], set_point = y_NOM[1][1])
        u_c = mc.u()
        u_d = disturbance_profile(t)

    
    single_step(mp, t, u_c, u_d)

print(f'SSE = {sum((mp.y[t,1]() - y_NOM[1][1])**2 for t in mp.T) / 100}')



# %%

# Make some plots if you want 
plt.plot(mp.T, mp.u[:,1]())
plt.xlabel('Time (min)')
plt.ylabel('NG flowrate')
plt.title('Disturbance in input')
plt.show()

plt.plot(mp.T, mp.u[:,2]())
plt.xlabel('Time (min)')
plt.ylabel('H2 flowrate')
plt.title('Control action')
plt.show()

plt.plot(mp.T, mp.y[:,1]())
plt.xlabel('Time (min)')
plt.ylabel('Gross power output')
plt.title('Disturbance in output')
plt.show()





# %%

