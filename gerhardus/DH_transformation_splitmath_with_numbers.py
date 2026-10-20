import numpy as np

def dh_matrix(theta, d, a, alpha):
    """calculates the transformation matrix for a single joint using dh parameters."""
    # convert degrees to radians for numpy trig functins
    theta_rad = np.deg2rad(theta)
    alpha_rad = np.deg2rad(alpha)
    
    ct = np.round(np.cos(theta_rad), 5)
    st = np.round(np.sin(theta_rad), 5)
    ca = np.round(np.cos(alpha_rad), 5)
    sa = np.round(np.sin(alpha_rad), 5)
    
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [ 0,     sa,     ca,    d],
        [ 0,      0,      0,    1]
    ])

# link distances exampels 
d1 = 0.05
d2 = 0.10
d3 = 0.15
a4 = 0.20
a5 = 0.25
a6 = 0.05

# joint angles examples
t2, t3, t4, t5, t6 = 0, 0, 0, 0, 0

#  dh table parameters [theta, d, a, alpha]
dh_table = [
    [ 0, d1,  0, 0],  # joint 1
    [t2, d2,  0,  -90],  # joint 2
    [t3, d3,  0,   90],  # joint 3
    [t4,  0, a4,   0],  # joint 4
    [t5,  0, a5, 0],  # joint 5
    [t6,  0, a6,   -90]   # joint 6
]

#  chain multiply and print the blocks
T = np.eye(4) # start with identity matrix

print(" FORWARD KINEMATICS: STEP-BY-STEP VERIFICATION ")

for i, params in enumerate(dh_table):
    theta, d, a, alpha = params
    
    # calculate current joint matrix a_i
    A_i = dh_matrix(theta, d, a, alpha)
    
    # chain multiply: t_0,i = t_0,i-1 * a_i
    T = np.dot(T, A_i)
    
    # extract rotaion (top-left 3x3) and position (top-right 3x1)
    R = T[0:3, 0:3]
    P = T[0:3, 3]
    
    print(f"\nFrame {i+1} (T_0,{i+1}):")
    print("Rotation Matrix (R):")
    print(np.array_str(R, suppress_small=True, precision=3))
    print("Position Vector (p):")
    print(np.array_str(P, suppress_small=True, precision=3))