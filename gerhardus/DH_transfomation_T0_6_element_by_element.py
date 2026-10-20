import sympy as sp

def dh_matrix_symbolic(theta, d, a, alpha):
    """Calculates the 4x4 transformation matrix for a single joint."""
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

# Define symbolic variables
t2, t3, t4, t5, t6 = sp.symbols('theta_2 theta_3 theta_4 theta_5 theta_6')
d1, d2, d3 = sp.symbols('d_1 d_2 d_3')
a4, a5, a6 = sp.symbols('a_4 a_5 a_6')
pi_2 = sp.pi / 2

# DH Table parameters: [theta, d, a, alpha]
dh_table = [
    [ 0, d1,  0,      0], # joint 1
    [t2, d2,  0,  -pi_2], # joint 2
    [t3, d3,  0,   pi_2], # joint 3
    [t4,  0, a4,      0], # joint 4
    [t5,  0, a5,      0], # joint 5
    [t6,  0, a6,  -pi_2]  # joint 6
]

#  chain multiply to get the final T_0^6 matrix
T = sp.eye(4)
for params in dh_table:
    # *params unpacks the list into theta, d, a, alpha
    T = T * dh_matrix_symbolic(*params)

#  define the naming structure for a standard 4x4 transform
element_names = [
    ['r11', 'r12', 'r13', 'px'],
    ['r21', 'r22', 'r23', 'py'],
    ['r31', 'r32', 'r33', 'pz']
]

print("final T_0,6 Matrix Breakdown\n")

# extract, simpify, and print each element individually
for row in range(3):
    for col in range(4):
        name = element_names[row][col]
        
        # We simplify  elements to prevent large expressions
        equation = sp.simplify(T[row, col]) 
        
        print(f"{name} -> ", end="")
        sp.pprint(equation)
        print("\n") # Add spacing between equations