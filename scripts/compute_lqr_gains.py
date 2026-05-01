import numpy as np
import control

def main():
    # System parameters from system_identification.md
    m = 0.318     # mass (kg)
    d = 0.083169  # distance to pivot point (m)
    Iw = 0.000736 # reaction wheel moment of inertia (kg m^2)
    Ip = 0.00118  # whole assembly moment of inertia about pivot (kg m^2)
    Kt = 0.0506   # motor torque constant (Nm/A)
    g = 9.81      # gravity (m/s^2)

    print("System Parameters:")
    print(f"  m: {m} kg")
    print(f"  d: {d} m")
    print(f"  Iw: {Iw} kg m^2")
    print(f"  Ip: {Ip} kg m^2")
    print(f"  Kt: {Kt} Nm/A")

    # Effective inertia of the frame (everything except the wheel rotation)
    I_frame = Ip - Iw
    
    # State space matrices: x = [theta, theta_dot, omega_w]
    # theta: tilt angle (rad)
    # theta_dot: angular velocity (rad/s)
    # omega_w: wheel angular velocity relative to frame (rad/s)
    # Input u: motor current (Amps)
    
    # Equations of motion:
    # (Ip - Iw) * theta_ddot = m*g*d*theta - tau
    # omega_w_dot = tau/Iw - theta_ddot
    # tau = Kt * i
    
    A = np.array([
        [0, 1, 0],
        [(m * g * d) / I_frame, 0, 0],
        [-(m * g * d) / I_frame, 0, 0]
    ])
    
    B = np.array([
        [0],
        [-Kt / I_frame],
        [Kt * Ip / (Iw * I_frame)]
    ])
    
    print("\nSystem Matrices:")
    print("A:\n", A)
    print("B:\n", B)
    
    # LQR Weights
    # Q: State cost
    # [theta, theta_dot, omega_w]
    Q = np.diag([
        1000.0,  # Penalty for tilt error
        10.0,    # Penalty for angular velocity
        0.01     # Penalty for wheel velocity
    ])
    
    # R: Input cost (current in Amps)
    R = np.array([[1.0]])
    
    print("\nLQR Weights:")
    print("Q:\n", Q)
    print("R:\n", R)
    
    # Compute LQR Gains
    # Note: control.lqr returns K such that u = -Kx
    K, S, E = control.lqr(A, B, Q, R)
    
    print("\nComputed LQR Gains (K):")
    print(K)
    
    print("\nClosed-loop Eigenvalues:")
    for e in E:
        print(f"  {e:.4f}")
        
    if all(np.real(E) < 0):
        print("\nResult: System is STABLE")
    else:
        print("\nResult: System is UNSTABLE - check weights or model!")

if __name__ == "__main__":
    main()
