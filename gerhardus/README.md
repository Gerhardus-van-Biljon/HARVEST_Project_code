# Denavit-Hartenberg (DH) Transformation & Forward Kinematics

This project calculates the forward kinematics and Denavit-Hartenberg (DH) transformation matrices for a 6-DOF robotic manipulator using symbolic and numerical methods in Python.

## File Structure

* **DH_transformation_matricses_only.py**: Calculates and prints the individual 4x4 transformation matrices ($A_i$) for each joint symbolically.
* **DH_transfromation_frame_by_frame_symbols.py**: Chain multiplies joint matrices to compute the full 4x4 symbolic transformation matrices step-by-step.
* **DH_transfomation_T0_6_element_by_element.py**: Chain multiplies joint matrices symbolically to obtain the final $T_0^6$ matrix, then simplifies and prints each individual element separately.
* **DH_Transfroation_splitmath_symbols.py**: Chain multiplies joint matrices symbolically and prints rotation matrices and position vectors separately.
* **DH_transformation_frame_by_frame_numbers.py**: Computes numerical 4x4 transformation matrices step-by-step using sample joint values.
* **DH_transformation_splitmath_with_numbers.py**: Computes numerical transformation matrices and prints rotation and position separately.
* **DH_transformation_exacty_post_failed.py**: Experimental/failed script for kinematics calculations.

## Dependencies

* **NumPy**: Used for numerical math and trig functions.
* **SymPy**: Used for symbolic math and simplifications.

## How to Run

Execute any script using a Python interpreter:

```bash
python <script_name>.py
```
