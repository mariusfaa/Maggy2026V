# Run script to deploy parameters to a LQR-like controller. 
import numpy as np
from scipy.signal import lfilter
from numpy.linalg import lstsq
from scipy import signal
from scipy.linalg import block_diag

import scipy as sp
import sympy as sm
import pandas as pd

from pathlib import Path 

VRFTuning = Path(__file__).resolve()

project_root = VRFTuning.parent.parent

#%%
# ----------------------------
# Define hyperparameters
# -----------------------------

# Specify a reference closed loop transfer function on the form:
# e^(-tau*s)/(1+0.2*t*s)^q
tau = 0   # Time delay
t = 1    # Settling time for the system poles
q = 2   # System order

Ts = 0.01      # Discretisation interval

# Specify a frequency weighting function on the form:
# omega/(omega+s)
omega=100 # Cutoff frequency in the frequency weighting function

simulator = "NMPC" # Options are NMPC and LQR. LQR requires running "sim_notlive_lqr_wrapper.m".

#%%
# ----------------------------
# Functions related to VRFT.
# -----------------------------
# Convert the reference transfer function to discrete time:
def M_cont_to_desc(tau,t,q,Ts):
    den_coeff=np.polynomial.polynomial.polypow([1,0.2*t],q)
    den_coeff=list(reversed(den_coeff)) # Polypow takes coefficients in ascending order, scipy.signal related functions takes them in descending order.
    num_coeff=[1] # Initialise numerator. time delays are added in discrete time.
    M_CT=signal.TransferFunction(num_coeff,den_coeff)
    M_DT=M_CT.to_discrete(Ts, method="bilinear")
    delay_samples = int(round(tau / Ts))
    if delay_samples > 0:
    # Multiply numerator by z^-d (pad with zeros)
        M_DT_num = np.concatenate([np.zeros(delay_samples), (M_DT.num)])
    else:
        M_DT_num = M_DT.num
    M_DT_den = M_DT.den
    return M_DT_num, M_DT_den

# Convert the frequency weighting function to discrete time:
def W_cont_to_desc(omega,Ts):
    W_CT = signal.TransferFunction([omega], [1, omega])
    W_DT = W_CT.to_discrete(Ts, method='bilinear')
    return W_DT.num, W_DT.den

# Construct phi^-1/2. Use a linear fit; will be improved in the future.
def construct_phi(u,Ts=Ts):
    f, pxx = sp.signal.welch(u,fs=1/Ts)
    G = [] # Initialise phi^-1/2
    for i in pxx:
        G.append(1/np.sqrt(i))
    Phi_inv_num = np.polyfit(f,G,1) # Fit a linear curve to G
    Phi_inv_den = [1] 
    return Phi_inv_num, Phi_inv_den

# Function for constructing the main VRFT filter.
def GetFilterCoeff(num,den,lp_num,lp_den,Phi_inv_num,Phi_inv_den):
    x=sm.symbols("x")
    def subConstructPoly(coeff,var=x):
        # Constructs the polynomial coeff[0]+coeff[1]*x+coeff[2]x**2...
        deg=len(coeff)-1
        return sum(c*var**(deg-i) for i,c in enumerate(reversed(coeff)))
    
    def subConstructRational(num,den):
        # Constructs the reference transfer function for internal use
        return subConstructPoly(num)/subConstructPoly(den)
        
    def subGetCoeffs(expr,var=x):
        # Extracts the coefficients of the filter for use with scipy lfilter
        num, den = sm.fraction(sm.simplify(expr))
        num_coeffs = sm.Poly(num, var).all_coeffs()
        den_coeffs = sm.Poly(den, var).all_coeffs()
            
        return list(reversed(num_coeffs)), list(reversed(den_coeffs))
    
    
    M = subConstructRational(num,den) # Construct reference transfer function
    W = subConstructRational(lp_num,lp_den) # Construct Lowpass filter
    Phi_Sqrt_inv = subConstructRational(Phi_inv_num,Phi_inv_den) # Construct Phi^(-1/2)
    
    F = M * (1 - M) * W * Phi_Sqrt_inv# Construct filter used on e_vr(t) and u(t)
    F_num, F_den = subGetCoeffs(F) 
    F_aux = (1 - M) * W * Phi_Sqrt_inv # Auxiliary filter to avoid having to calculate r_v directly, thus avoiding anti-causal filtering in case of time delays.
    aux_num, aux_den = subGetCoeffs(F_aux)
      
    # Convert from sympy float to numpy float
    
    F_num = np.array([float(i) for i in F_num], dtype=float)
    F_den = np.array([float(i) for i in F_den], dtype=float)
    aux_num = np.array([float(i) for i in aux_num], dtype=float)
    aux_den = np.array([float(i) for i in aux_den], dtype=float)
    return F_num, F_den, aux_num, aux_den


#%%
# -----------------------------
# Collect data
# -----------------------------
if simulator == "LQR":
    Data = pd.read_csv("model_output.csv")
    x, y, z = Data["x"], Data["y"], Data["z"]
    a, b, g = Data["alpha"], Data["beta"], Data["gamma"]
    xdot, ydot, zdot = Data["xdot"], Data["ydot"], Data["zdot"]
    adot, bdot, gdot = Data["alphadot"], Data["betadot"], Data["gammadot"]

    u1, u2, u3, u4 = Data["u1"], Data["u2"], Data["u3"], Data["u4"]
elif simulator == "NMPC":
    from scipy import io
    output_location = project_root / "NMPCProject" / "nmpc_results.mat"
    data = io.loadmat(output_location)
    t_sim = data["t"]
    x, y, z = data["x"][0,1:], data["x"][1,1:], data["x"][2,1:]
    a, b, g = data["x"][3,1:], data["x"][4,1:], data["x"][5,1:]
    xdot, ydot, zdot = data["x"][6,1:], data["x"][7,1:], data["x"][8,1:]
    adot, bdot, gdot = data["x"][9,1:], data["x"][10,1:], data["x"][11,1:]
    
    u1, u2, u3, u4 = data["u"][0,:], data["u"][1,:], data["u"][2,:], data["u"][3,:]
else:
    raise Exception("Specify valid simulator type (NMPC or LQR)")
    
#%%
# -----------------------------
# Construct virtual reference and error
# -----------------------------

M_num, M_den = M_cont_to_desc(tau,t,q,Ts) # Use same reference model and frequency weighting for all solonoids for now.

W_num, W_den=W_cont_to_desc(omega,Ts)

Phi_u1_num, Phi_u1_den = construct_phi(u1)
Phi_u2_num, Phi_u2_den = construct_phi(u2)
Phi_u3_num, Phi_u3_den = construct_phi(u3)
Phi_u4_num, Phi_u4_den = construct_phi(u4)

F_num_u1, F_den_u1, aux_num_u1, aux_den_u1 = GetFilterCoeff(M_num,M_den,W_num,W_den,Phi_u1_num,Phi_u1_den)
F_num_u2, F_den_u2, aux_num_u2, aux_den_u2 = GetFilterCoeff(M_num,M_den,W_num,W_den,Phi_u2_num,Phi_u2_den)
F_num_u3, F_den_u3, aux_num_u3, aux_den_u3 = GetFilterCoeff(M_num,M_den,W_num,W_den,Phi_u3_num,Phi_u3_den)
F_num_u4, F_den_u4, aux_num_u4, aux_den_u4 = GetFilterCoeff(M_num,M_den,W_num,W_den,Phi_u4_num,Phi_u4_den)


# Each term in the controller must be filtered by the VRFT filter corresponding to each control input.
# Check controller analysis doc for overview of each state.
x1_l = lfilter(F_num_u1,F_den_u1,x)
x3_l = lfilter(F_num_u3,F_den_u3,x)

xdot1_l = lfilter(F_num_u1,F_den_u1,xdot)
xdot3_l = lfilter(F_num_u3,F_den_u3,xdot)

b1_l = lfilter(F_num_u1,F_den_u1,b)
b3_l = lfilter(F_num_u3,F_den_u3,b)

bdot1_l = lfilter(F_num_u1,F_den_u1,bdot)
bdot3_l = lfilter(F_num_u3,F_den_u3,bdot)

y2_l = lfilter(F_num_u2, F_den_u2,y)
y4_l = lfilter(F_num_u4, F_den_u4,y)

ydot2_l = lfilter(F_num_u2, F_den_u2,ydot)
ydot4_l = lfilter(F_num_u4, F_den_u4,ydot)

a2_l = lfilter(F_num_u2, F_den_u2,a)
a4_l = lfilter(F_num_u4, F_den_u4,a)

adot2_l = lfilter(F_num_u2, F_den_u2,adot)
adot4_l = lfilter(F_num_u4, F_den_u4,adot)

z1_l = lfilter(F_num_u1,F_den_u1,z)
z2_l = lfilter(F_num_u2,F_den_u2,z)
z3_l = lfilter(F_num_u3,F_den_u3,z)
z4_l = lfilter(F_num_u4,F_den_u4,z)

zdot1_l = lfilter(F_num_u1,F_den_u1,zdot)
zdot2_l = lfilter(F_num_u2,F_den_u2,zdot)
zdot3_l = lfilter(F_num_u3,F_den_u3,zdot)
zdot4_l = lfilter(F_num_u4,F_den_u4,zdot)

u1_l = lfilter(F_num_u1,F_den_u1,u1)
u2_l = lfilter(F_num_u2,F_den_u2,u2)
u3_l = lfilter(F_num_u3,F_den_u3,u3)
u4_l = lfilter(F_num_u4,F_den_u4,u4)


# Filtered errors. Only the displacements are controlled.
e_x1 = lfilter(aux_num_u1,aux_den_u1,x) - x1_l
e_x3 = lfilter(aux_num_u3,aux_den_u3,x) - x3_l

e_b1 = lfilter(aux_num_u1,aux_den_u1,b) - b1_l
e_b3 = lfilter(aux_num_u3,aux_den_u3,b) - b3_l

e_y2 = lfilter(aux_num_u2,aux_den_u2,y) - y2_l
e_y4 = lfilter(aux_num_u4,aux_den_u4,y) - y4_l

e_a2 = lfilter(aux_num_u2,aux_den_u2,a) - a2_l
e_a4 = lfilter(aux_num_u4,aux_den_u4,a) - a4_l

e_z1 = lfilter(aux_num_u1,aux_den_u1,z) - z1_l
e_z2 = lfilter(aux_num_u2,aux_den_u2,z) - z2_l
e_z3 = lfilter(aux_num_u3,aux_den_u3,z) - z3_l
e_z4 = lfilter(aux_num_u4,aux_den_u4,z) - z4_l


#%%
# -----------------------------
# Define controller structure C(z, θ)
# -----------------------------
# See control analysis doc for controller structures.
phi_1 = np.column_stack([e_x1,e_z1,e_b1,xdot1_l,zdot1_l,bdot1_l])

phi_2 = np.column_stack([e_y2,e_z2,e_a2,ydot2_l,zdot2_l,adot2_l])

phi_3 = np.column_stack([e_x3,e_z3,e_b3,xdot3_l,zdot3_l,bdot3_l])

phi_4 = np.column_stack([e_y4,e_z4,e_a4,ydot4_l,zdot4_l,adot4_l])

#%%
# -----------------------------
# Calculate LQR-like params and deploy
# -----------------------------

b_lstsq = np.concatenate([u1_l, u2_l, u3_l, u4_l])
a_lstsq = block_diag(phi_1,phi_2,phi_3,phi_4)

theta, _, _, _ = lstsq(a_lstsq,b_lstsq,rcond=None)

print("Tuned coupled controller parameters (θ):",theta)

params = pd.DataFrame(theta.reshape(1,24),columns=["a0","a1","a2","a3","a4","a5",
                                                  "b0","b1","b2","b3","b4","b5",
                                                  "c0","c1","c2","c3","c4","c5",
                                                  "d0","d1","d2","d3","d4","d5",])

params.to_csv("params.csv")
