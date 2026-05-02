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
    
    # Augmented state: [theta, theta_dot, omega_w, int_omega_w]
    A_aug = np.zeros((4, 4))
    A_aug[0:3, 0:3] = A
    A_aug[3, 2] = 1.0  # d/dt (int_omega_w) = omega_w
    
    B_aug = np.zeros((4, 1))
    B_aug[0:3, 0] = B.flatten()
    
    print("\nAugmented System Matrices:")
    print("A_aug:\n", A_aug)
    print("B_aug:\n", B_aug)
    
    # LQR Weights
    # Q: State cost
    # [theta, theta_dot, omega_w, int_omega_w]
    Q = np.diag([
        4000.0,  # Penalty for tilt error
        20.0,    # Penalty for angular velocity
        0.0,   # Penalty for wheel velocity
        0.0001   # Penalty for integrated wheel velocity (bias rejection)
    ])
    
    # R: Input cost (current in Amps)
    R = np.array([[.5]])
    
    print("\nLQR Weights:")
    print("Q:\n", Q)
    print("R:\n", R)
    
    # Compute LQR Gains
    # Note: control.lqr returns K such that u = -Kx
    K, S, E = control.lqr(A_aug, B_aug, Q, R)
    
    print("\nComputed LQR Gains (K):")
    print(f'{K[0, 0]:.4f}f, {K[0, 1]:.4f}f, {K[0, 2]:.4f}f, {K[0, 3]:.4f}f')
    
    print("\nClosed-loop Eigenvalues:")
    for e in E:
        print(f"  {e:.4f}")
        
    if all(np.real(E) < 0):
        print("\nResult: System is STABLE")
    else:
        print("\nResult: System is UNSTABLE - check weights or model!")

if __name__ == "__main__":
    main()
