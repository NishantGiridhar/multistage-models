# %%
__author__ = "Nishant Giridhar"

import pyomo.environ as pyo
from pyomo.dae import ContinuousSet,  DerivativeVar
import scipy.io
import numpy as np
import pandas as pd
import os

cpath = os.path.dirname(__file__)

# %%
def create_ngcc_ROM(b, time_set):
    mat = scipy.io.loadmat(f'{cpath}/data/NGCC_ROM.mat')
        
    y_NOM = pd.read_csv(f'{cpath}/data/NGCC_H2_Output_100.txt', 
                        header=None, index_col=0, sep=None, engine='python')

    u_NOM = pd.read_csv(f'{cpath}/data/NGCC_H2_Input_100.txt',
                        header=None,index_col=0, sep=None, engine='python')


    A_mat = np.array(mat['A_red'])
    B_mat = np.array(mat['B_red'])
    C_mat = np.array(mat['C_red'])
    D_mat = np.array(mat['D_red'])

    # TODO: replace these to be obtained from the function
    # b = pyo.ConcreteModel()
    b.T = time_set       #ContinuousSet(bounds=(0,100)) 

    # Define index sets
    b.N = pyo.RangeSet(1,54)
    b.M = pyo.RangeSet(1,3)
    b.P = pyo.RangeSet(1,18)

    # Define variables (standard, not deviation)
    b.u = pyo.Var(b.T, b.M)
    b.y = pyo.Var(b.T, b.P)
    b.x = pyo.Var(b.T, b.N)

    b.dxdt = DerivativeVar(b.x, wrt = b.T)

    # Define expressions to calculate deviation variables
    @b.Expression(b.T, b.M)
    def u_dev(b,_t, _m):
        return b.u[_t,_m] - u_NOM[1][_m]

    @b.Expression(b.T, b.P)
    def y_dev(b,_t, _p):
        return b.y[_t,_p] - y_NOM[1][_p]

    # State space model constraints
    @b.Constraint(b.T, b.N)
    def c1(b, t, i):
        return (sum(A_mat[i-1,j-1] * b.x[t, j] for j in b.N) 
                + sum(B_mat[i-1,k-1] * b.u_dev[t,k] for k in b.M)    == b.dxdt[t,i])

    @b.Constraint(b.T, b.P)
    def c2(b,  t,  i):
        return (sum(C_mat[i-1,j-1] * b.x[t,j] for j in b.N)
                + sum(D_mat[i-1,k-1] * b.u_dev[t,k] for k in b.M)  == b.y_dev[t,i])
    


