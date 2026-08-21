import numpy as np
import time

def simulate_ccd_step(current_joints, target_pos, target_rot):
    """
    Simulates the math load of a single CCD iteration.
    Uses vector math (dot/cross products) from tip to base.
    """
    # Simulated dummy calculation representing vector projections
    new_joints = np.copy(current_joints)
    for i in range(5, -1, -1):
        # Fake math operation mimicking a cross product and atan2 calculation
        v1 = np.random.rand(3)
        v2 = np.random.rand(3)
        angle_delta = np.dot(v1, v2) * 0.01 
        new_joints[i] += angle_delta
    return new_joints

def simulate_jacobian_transpose_step(current_joints, target_pos, target_rot):
    """
    Simulates the math load of calculating a 6x6 Jacobian 
    and transposing it (no inversion).
    """
    # simulated dummy calculation mimicking matrix operations
    new_joints = np.copy(current_joints)
    
    #  build a dummy 6x6 Jacobian
    J = np.random.rand(6, 6) 
    
    #  transpose it 
    J_T = J.T 
    
    # Multiply by simulated error vector (6x1)
    error = np.random.rand(6) 
    delta_theta = np.dot(J_T, error) * 0.01
    
    new_joints += delta_theta
    return new_joints

# benchmarking setup 
iterations = 100000
test_joints = np.zeros(6)
target_p = np.array([0.2, 0.1, 0.3])
target_r = np.eye(3)

print(f"Running ESP32 Feasibility Benchmark ({iterations} iterations)")

# benchmark CCD
start_time = time.perf_counter()
for _ in range(iterations):
    test_joints = simulate_ccd_step(test_joints, target_p, target_r)
ccd_time = (time.perf_counter() - start_time) * iterations
print(f"CCD Total Time: {ccd_time:.2f} ms")

#benchmark Jacobian Transpose
start_time = time.perf_counter()
for _ in range(iterations):
    test_joints = simulate_jacobian_transpose_step(test_joints, target_p, target_r)
jac_time = (time.perf_counter() - start_time) * iterations
print(f"Jacobian Transpose Total Time: {jac_time:.2f} ms")