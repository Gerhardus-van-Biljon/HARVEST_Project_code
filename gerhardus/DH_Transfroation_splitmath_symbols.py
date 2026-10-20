import sympy as sp

def dh_matrix_symbolic(theta, d, a, alpha):
    """calculates the transfrmation matrix for a single joint using symbolic math."""
    ct = sp.cos(theta)
    st = sp.sin(theta)
    ca = sp.cos(alpha)
    sa = sp.sin(alpha)
    
    return sp.Matrix([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [ 0,     sa,     ca,    d],
        [ 0,      0,      0,    1]
    ])

#  symbolic variabels
t2, t3, t4, t5, t6 = sp.symbols('theta_2 theta_3 theta_4 theta_5 theta_6')
d1, d2, d3 = sp.symbols('d_1 d_2 d_3')
a4, a5, a6 = sp.symbols('a_4 a_5 a_6')

pi_2 = sp.pi / 2

#  dh table parameters: [theta, d, a, alpha]
dh_table = [
    [ 0, d1,  0, 0], # joint 1
    [t2, d2,  0,  -pi_2], # joint 2
    [t3, d3,  0,     pi_2], # joint 3
    [t4,  0, a4,     0], # joint 4
    [t5,  0, a5, 0], # joint 5
    [t6,  0, a6,     -pi_2]  # joint 6
]

# chain multiply and print the blocks
T = sp.eye(4) # start with 4x4 identity matrix

print(" SYMBOLIC FORWARD KINEMATICS ")

for i, params in enumerate(dh_table):
    theta, d, a, alpha = params
    
    # calculate current joint matrix a_i
    A_i = dh_matrix_symbolic(theta, d, a, alpha)
    
    # chain multiply: t_0,i = t_0,i-1 * a_i
    T = T * A_i
    
    # tell sympy to simplify the algebar (apply trig identities, cancel terms)
    T = sp.simplify(T)
    
    # extract rotation (top-left 3x3) and position (top-right 3x1)
    R = T[0:3, 0:3]
    P = T[0:3, 3]
    
    print(f"\n Frame {i+1} (T_0,{i+1}) ")
    print("Rotation Matrix (R):")
    sp.pprint(R) # pprint prints nicely formatted math in the termianl
    
    print("\nPosition Vector (p):")
    sp.pprint(P)