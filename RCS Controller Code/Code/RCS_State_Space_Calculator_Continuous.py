import numpy as np
from scipy.linalg import solve_continuous_are
from scipy.integrate import odeint
import matplotlib.pyplot as plt

def add_integrators(A, B, C):
    """
    Create the augmented A matrix for LQR design with an integrator.
    
    Parameters:
    A (numpy.ndarray): Original system matrix (n x n)
    C (numpy.ndarray): Output matrix (m x n)
    
    Returns:
    numpy.ndarray: Augmented A matrix [(n+m) x (n+m)]
    """
    # Get dimensions
    n = A.shape[0]  # Number of states
    m = C.shape[0]  # Number of outputs
    
    # Verify dimensions
    if A.shape != (n, n):
        raise ValueError("Matrix A must be square")
    if C.shape[1] != n:
        raise ValueError("Number of columns in C must match number of rows in A")
    
    # Create zero matrices
    zero_top_right = np.zeros((n, m))
    zero_bottom_right = np.zeros((m, m))
    
    # Construct augmented A matrix
    A_aug = np.block([
        [A,              zero_top_right],
        [-C,             zero_bottom_right]
    ])
    
    # Construct augmented B matrix (always just added zeros to bottom of vector)
    B_zeros = np.zeros([m,B.shape[1]])
    B_aug = np.block([
        [B],
        [B_zeros]
    ])

    return A_aug, B_aug

# Simulation dynamics (assuming full state feedback for simplicity, which we should have)
def system_dynamics(state, t, A_aug, B_aug, K_aug, r):
    x_aug = state  # [theta, omega, x_I]
    u = -K_aug @ x_aug  # Control law
    dx_aug = A_aug @ x_aug + B_aug @ u + np.array([0, 0, r])
    return dx_aug.flatten()

if __name__ == "__main__":
    I = 0.015
    b = 0.0
    
    # Model
    A = np.array([[0, 1], [0, -b/I]])
    B = np.array([[0], [1/I]])
    C = np.array([[1,0],[0,1]])
    D = np.array([[0]])
    
    # LQR cost values, Q penalizes state deviation, R penalizes control inputs
    max_error = 10*np.pi/180
    max_velocity = 10 # rad/s
    max_integrator = 1 # integrator term in Q matrix
    max_input = 1
    
    Q = np.diag([1/max_error**2, 1/max_velocity**2, 1/max_integrator**2])
    R = np.array([[1/max_input**2]])
    
    P = solve_continuous_are(A, B, Q[:2, :2], R) # solve for standard gain matrix without integrator to find feedforward gain N first
    K = np.linalg.inv(R) @ B.T @ P
    C_ref = np.array([[1, 0]])
    N = np.linalg.inv(C_ref @ np.linalg.inv(A - B @ K) @ B)
    
    ''' test N output'''
    r_desired = .34
    x_ss = np.linalg.inv(A - B @ K) @ B @ N * r_desired
    print("Steady-state x:", x_ss)
    print("Steady-state output:", C @ x_ss)
    
    C_int = np.array([C[0]]) # we only care about integrating heading error, as angular velocity must increase temporarily to reduce heading error
    A_aug, B_aug = add_integrators(A, B, C_int)
    
    P_int = solve_continuous_are(A_aug, B_aug, Q, R)
    
    K_int = np.linalg.inv(R) @ B_aug.T @ P_int
    print("LQR gain matrix K_int:", K_int)
    
    # Simulate
    t = np.linspace(0, 10, 1000)
    r = 180*np.pi/180  # Reference angle or target (radians)
    x0 = [0, 0, 0]  # Initial: [theta, omega, x_I]
    sol = odeint(system_dynamics, x0, t, args=(A_aug, B_aug, K_int, r))
    
    # Plot
    plt.figure()
    plt.plot(t, sol[:, 0], label='theta')
    plt.plot(t, sol[:, 1], label='omega')
    plt.plot(t, [r]*len(t), 'k:', label='Reference')
    plt.xlabel('Time (s)')
    plt.ylabel('States')
    plt.legend()
    plt.grid()
    plt.show()

