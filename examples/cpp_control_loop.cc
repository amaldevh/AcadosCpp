// Copyright (c) 2026 Amal Dev Haridevan
// SPDX-License-Identifier: MIT

#include <iostream>
#include <fstream>
#include <chrono>
#include <initializer_list>
#include "model_ocp.hh"
#include "model_sim.hh"

using Vector = ModelOcp::Vector;
using VectorArray = ModelOcp::VectorArray;

Vector make_vector(std::initializer_list<double> values)
{
    Vector result(values.size());
    std::size_t index = 0;
    for (double value : values)
        result[index++] = value;
    return result;
}

int main()
{

    ModelOcp ocp;
    ModelSim sim;
    std::ofstream log_file("simulation_log.csv");

    Vector x0 = make_vector({0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0,
                             1.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0});

    Vector u0 = make_vector({0.0, 0.0, 0.0, 0.0});
    Vector xdes = make_vector({5.0, 5.0, 5.0,
                               0.0, 0.0, 0.0,
                               1.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0});
    Vector u_hover = make_vector({9.81, 0.0, 0.0, 0.0});
    double tf = 5.0; // time horizon in seconds
    double dt = 1e-3;

    // set initial state and dt for sim
    sim.set_x0(x0);
    sim.set_dt(dt);
    VectorArray xrefs(ocp.horizon() + 1, xdes);
    VectorArray urefs(ocp.horizon(), u_hover);
    ocp.initialize_guess(x0, u_hover);
    // Log time, state, input, and desired state in CSV format.
    log_file << "time,x,y,z,vx,vy,vz,qw,qx,qy,qz,wx,wy,wz,";
    for (size_t i = 0; i < u0.size(); ++i)
    {
        log_file << "u" << i << ",";
    }
    log_file << "x_des,y_des,z_des,vx_des,vy_des,vz_des,qw_des,qx_des,qy_des,qz_des,wx_des,wy_des,wz_des\n";
    auto log_state = [&log_file](double time, const Vector &x, const Vector &xdes, const Vector &u)
    {
        log_file << time;
        for (std::size_t index = 0; index < x.size(); ++index)
        {
            log_file << "," << x[index];
        }
        for (std::size_t index = 0; index < u.size(); ++index)
        {
            log_file << "," << u[index];
        }
        for (std::size_t index = 0; index < xdes.size(); ++index)
        {
            log_file << "," << xdes[index];
        }
        log_file << '\n';
    };
    int Niters = static_cast<int>(tf / dt);
    auto start_time = std::chrono::high_resolution_clock::now();
    for (int iter = 0; iter < Niters; ++iter)
    {
        // solve OCP to get optimal control
        // Sets the measured stage-0 state, applies the complete reference
        // horizon, and shifts the previous solution as the warm start.
        const Vector &u_opt = ocp.solve(x0, xrefs, urefs);
        x0 = sim.step(u_opt);
        double current_time = (iter + 1) * dt;
        log_state(current_time, x0, xdes, u_opt);
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end_time - start_time;
    std::cout << "Simulation completed in " << elapsed.count() << " seconds." << std::endl;
    std::cout << "Average time per iteration: " << (elapsed.count() / Niters) << " s" << std::endl;
    std::cout << "Average frequency: " << (Niters / elapsed.count()) << " Hz" << std::endl;
    log_file.close();
    return 0;
}
