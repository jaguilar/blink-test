import numpy as np
import control

def main():
    # System parameters from system_identification.md
    m = 0.318     # mass (kg)
    d = 0.083169  # distance to pivot point (m)
    Iw = 0.000736 # reaction wheel moment of inertia (kg m^2)
    Ip = 0.00118  # whole assembly moment of inertia about pivot (kg m^2)
    Kt = 0.028     # motor torque constant (Nm/A)
    g = 9.81      # gravity (m/s^2)

    # ETH Zurich Friction Coefficients -- not sure these are really needful
    # but we put them in our model in case. We assume we have similar friction
    # given we are a similar robot.
    Cb = 3.02e-3  # Body dynamic friction (kg m^2 / s)
    Cw = 0.08e-3  # Wheel dynamic friction (kg m^2 / s)

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
        [(m * g * d) / I_frame, -Cb / I_frame, Cw / I_frame],
        [-(m * g * d) / I_frame, Cb / I_frame, -Cw * Ip / (Iw * I_frame)]
    ])
    
    # Linearized B Matrix (Your existing B matrix structurally matched the paper perfectly)
    B = np.array([
        [0],
        [-Kt / I_frame],
        [Kt * Ip / (Iw * I_frame)]
    ])

    C = np.identity(3)
    D = np.zeros((3, 1))
    sys = control.ss(A, B, C, D)
    sys_disc = control.sample_system(sys, Ts=0.005, method='zoh')
    Ad = sys_disc.A
    Bd = sys_disc.B
    print(f'Ad: {str(Ad)}\nBd: {str(Bd)}\n')
    
    # LQR Weights
    # Q: State cost
    Q = np.diag([
        1000.0,  # Penalty for tilt error
        50.0,    # Penalty for angular velocity
        0.01,    # Penalty for wheel velocity
    ])
    
    # R: Input cost (current in Amps)
    R = np.array([[1]])
    
    print("\nLQR Weights:")
    print("Q:\n", Q)
    print("R:\n", R)
    
    # Compute LQR Gains
    # Note: control.lqr returns K such that u = -Kx
    K, S, E = control.dlqr(sys_disc, Q, R)
    
    print("\nComputed LQR Gains (K):")
    print(f'{K[0, 0]:.4f}f, {K[0, 1]:.4f}f, {K[0, 2]:.4f}f')
    
    print("\nClosed-loop Eigenvalues:")
    for e in E:
        print(f"  {e:.4f}")
        
    if all(np.abs(E) < 1.0):
        print("\nResult: System is STABLE")
    else:
        print("\nResult: System is UNSTABLE - check weights or model!")

if __name__ == "__main__":
    main()
