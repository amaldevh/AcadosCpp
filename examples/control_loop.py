import sys 
import os 
base_path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(base_path))
from generate_cpp_ocp import generate_code
from quadrotor_model import acados_ocp_solver
import numpy as np
import matplotlib.pyplot as plt
import casadi as ca
import time 

def main():
    mass = 1.0  # kg
    gravity = np.array([0.0, 0.0, -9.81])  # m/s^2
    I = np.diag([0.005, 0.005, 0.009])  # kg*m^2

    # Get the Acados OCP solver
    ocp_solver = acados_ocp_solver(mass, gravity, I)

    # Extract model name and code generation directory
    model_name = ocp_solver.name 
    N_horizon = ocp_solver.N
    output_dir = f'{base_path}/cpp_{model_name}_ocp'
    c_generated_code_dir = ocp_solver.acados_ocp.code_export_directory
    generate_code(model_name, c_generated_code_dir, output_dir)
    print(f"C++ OCP code generated in directory: {output_dir}")
    print(f"Model name: {model_name}, Horizon: {N_horizon}")

    # Closed loop example
    sys.path.append(output_dir)
    module_name = f'{model_name}_ocp_py'
    sim_module_name = f'{model_name}_sim_py'    
    ocp_module = __import__(module_name)
    sim_module = __import__(sim_module_name)
    ocp_instance = ocp_module.ModelOcp()
    sim_instance = sim_module.ModelSim()

    # Initial state
    x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   1.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0])
    xdes = np.array([5.0, 5.0, 5.0, 0.0, 0.0, 0.0,
                     1.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0])
    u0 = np.array([mass * -gravity[2], 0.0, 0.0, 0.0])  # hover thrust


    # Setup integration parameters
    dt = 1e-3  # integration time step
    T_sim = 5.0  # total simulation time
    # set sim dt, by default sim uses tf/N (specified in acados ocp), we need to change
    # it to 1e-3 for better accuracy
    sim_instance.set_dt(dt)
    # update initial state in sim
    sim_instance.set_x0(x0)
    N_sim = int(T_sim / dt)

    # Store states
    states = np.zeros((N_sim+1, len(x0)))
    desired_states = np.tile(xdes, (N_sim+1, 1))
    controls = np.zeros((N_sim, len(u0)))
    states[0, :] = x0
    t_start = time.time()
    for i in range(N_sim):
        u0 = ocp_instance.solve(x0, xdes)
        controls[i, :] = u0
        x0 = sim_instance.step(u0)
        states[i+1, :] = x0
    t_end = time.time()
    print(f"Closed-loop simulation completed in {t_end - t_start:.5f} seconds.")
    print(f"Average time per step: {(t_end - t_start)/N_sim:.5f} s")
    print(f"Average frequency: {N_sim/(t_end - t_start):.2f} Hz ")

    times = np.linspace(0, T_sim, N_sim+1)
    fig, axs = plt.subplots(3,1, figsize=(10,8))
    axs[0].plot(times, states[:,0], label='x')
    axs[0].plot(times, desired_states[:,0], 'r--', label='x_des')
    axs[1].plot(times, states[:,1], label='y')
    axs[1].plot(times, desired_states[:,1], 'r--', label='y_des')
    axs[2].plot(times, states[:,2], label='z')
    axs[2].plot(times, desired_states[:,2], 'r--', label='z_des')
    fig.suptitle('Position')
    axs[0].legend()
    axs[1].legend()
    axs[2].legend()
    fig, axs = plt.subplots(3,1, figsize=(10, 8))

    axs[0].plot(times, states[:,3], label='vx')
    axs[1].plot(times, states[:,4], label='vy')
    axs[2].plot(times, states[:,5], label='vz')
    fig.suptitle('Velocity')
    axs[0].legend()
    axs[1].legend()
    axs[2].legend()
    fig, axs = plt.subplots(4,1, figsize=(10, 8))
    axs[0].plot(times[:-1], controls[:,0], label='thrust')
    axs[1].plot(times[:-1], controls[:,1], label='moment_x')
    axs[2].plot(times[:-1], controls[:,2], label='moment_y')
    axs[3].plot(times[:-1], controls[:,3], label='moment_z')
    fig.suptitle('Controls')
    axs[0].legend()
    axs[1].legend()
    axs[2].legend()
    axs[3].legend()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()