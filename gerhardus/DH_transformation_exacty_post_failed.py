#failed - useles code for project
import numpy as np


def dh_transform(theta, d, a, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)

    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,        sa,       ca,      d],
        [0,         0,        0,      1]
    ])


def forward_kinematics(d1, theta2, theta3, theta4, theta5, theta6,
                        d2, d3, a4, a5, a6, return_all=False):

    A1 = dh_transform(0,      d1, 0,  0)
    A2 = dh_transform(theta2, d2, 0,   -np.pi / 2)
    A3 = dh_transform(theta3, d3, 0,   0)           
    A4 = dh_transform(theta4, 0,  a4,  0)
    A5 = dh_transform(theta5, 0,  a5, -np.pi / 2)
    A6 = dh_transform(theta6, 0,  a6,  0)

    T0_1 = A1
    T0_2 = T0_1 @ A2
    T0_3 = T0_2 @ A3
    T0_4 = T0_3 @ A4
    T0_5 = T0_4 @ A5
    T0_6 = T0_5 @ A6

    if return_all:
        return {
            "A": [A1, A2, A3, A4, A5, A6],
            "T": [T0_1, T0_2, T0_3, T0_4, T0_5, T0_6],
        }
    return T0_6


def extract_position(T):    
    return T[:3, 3]


def extract_orientation(T):
    return T[:3, :3]


def rotation_to_rpy(R):

    pitch = np.arcsin(-R[2, 0])
    if np.isclose(np.cos(pitch), 0):
        # gimbel lock case
        roll = 0
        yaw = np.arctan2(-R[0, 1], R[1, 1])
    else:
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    return np.degrees([roll, pitch, yaw])


def print_pose(label, T):
    pos = extract_position(T)
    rpy = rotation_to_rpy(extract_orientation(T))
    print(f"{label}")
    print(f"  Position (x, y, z): "
          f"({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
    print(f"  Orientation (roll, pitch, yaw) deg: "
          f"({rpy[0]:.2f}, {rpy[1]:.2f}, {rpy[2]:.2f})")


if __name__ == "__main__":
    d2 = 0.10   # base waist offset
    d3 = 0.20   # waist shoulder offset
    a4 = 0.25   # elbow link 1 length
    a5 = 0.20   # elbow link 2 length
    a6 = 0.10   # wrist/drill offset


    d1 = 0.05                     # prismatic slide, meters
    theta2 = np.radians(0)        # waist spin
    theta3 = np.radians(30)       # elbow 1
    theta4 = np.radians(-45)      # elbw2
    theta5 = np.radians(20)       # elbow
    theta6 = np.radians(0)        # drill spin

    result = forward_kinematics(
        d1, theta2, theta3, theta4, theta5, theta6,
        d2, d3, a4, a5, a6,
        return_all=True
    )
    for i, T in enumerate(result["T"], start=1):
        print_pose(f"Frame {i} (T0_{i}):", T)
        print()

    print("=" * 50)
    print_pose("END EFFECTOR (T0_6):", result["T"][-1])