import sympy as sp

def dh_matrix_symbolic(theta, d, a, alpha):
    """Calculates the 4x4 transformation matrix for a single joint using symbolic math."""
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

# define symbolic variables
t2, t3, t4, t5, t6 = sp.symbols('theta_2 theta_3 theta_4 theta_5 theta_6')
d1, d2, d3 = sp.symbols('d_1 d_2 d_3')
a4, a5, a6 = sp.symbols('a_4 a_5 a_6')

# exact representation of 90 degrees for sympy trig simplification
pi_2 = sp.pi / 2

# DH Table parameters: [theta, d, a, alpha]
dh_table = [
    [ 0, d1,  0,      0], # joint 1: alpha = 0
    [t2, d2,  0,  -pi_2], # joint 2: alpha = -90
    [t3, d3,  0,   pi_2], # joint 3: alpha = 90
    [t4,  0, a4,      0], # joint 4: alpha = 0
    [t5,  0, a5,      0], # joint 5: alpha = 0
    [t6,  0, a6,  -pi_2]  # joint 6: alpha = -90
]

# calculate and print the individual matrices (A_i)
for i, params in enumerate(dh_table):
    theta, d, a, alpha = params
    
    # calculate current joint matrix A_i
    A_i = dh_matrix_symbolic(theta, d, a, alpha)
    
    # tell sympy to simplify the algebra
    A_i = sp.simplify(A_i)
    
    # print the full 4x4 matrix
    print(f"Transformation matrix {i+1}:\n")
    sp.pprint(A_i)
    print("\n\n")