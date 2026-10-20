import sympy as sp

def dh_matrix_symbolic(theta, d, a, alpha):
    """calculates the transformation matrix for a single joint using symbolic math."""
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

#  define symbolic variabls
t2, t3, t4, t5, t6 = sp.symbols('theta_2 theta_3 theta_4 theta_5 theta_6')
d1, d2, d3 = sp.symbols('d_1 d_2 d_3')
a4, a5, a6 = sp.symbols('a_4 a_5 a_6')


pi_2 = sp.pi / 2

# dh table parameters: [theta, d, a, alpha]
dh_table = [
    [ 0, d1,  0,    0], # a_1
    [t2, d2,  0,  -pi_2], # a_2
    [t3, d3,  0,    pi_2], # a_ 3
    [t4,  0, a4,    0], # a_4
    [t5,  0, a5,    0], # a_5
    [t6,  0, a6,    -pi_2]  # a_6
]

# chain multiply and print the full 4x4 matrices
T = sp.eye(4) # start with 4x4 identity matrix

print(" SYMBOLIC FORWARD KINEMATICS ")

for i, params in enumerate(dh_table):
    theta, d, a, alpha = params
    
    # calculate current joint matrix a_i
    A_i = dh_matrix_symbolic(theta, d, a, alpha)
    
    # chain multiply: t_0,i = t_0,i-1 * a_i
    T = T * A_i
    
    # tell sympi to simplify the algebra
    T = sp.simplify(T)
    
    # print the full 4x4 transfrmation matrix
    print(f"\n Frame {i+1} (T_0,{i+1}) ")
    sp.pprint(T)