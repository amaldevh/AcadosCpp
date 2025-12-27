#include <iostream>
#include <fstream>
#include <chrono>
#include "model_ocp.hh"
#include "model_sim.hh"


int main(){

    ModelOcp ocp;
    ModelSim sim;
    std::ofstream log_file("simulation_log.csv");

    std::vector<double> x0 = {0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0,
                            1.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0
                        };

    std::vector<double> u0 = {0.0, 0.0, 0.0, 0.0};
    std::vector<double> xdes = {5.0, 5.0, 5.0,
                             0.0, 0.0, 0.0,
                            1.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0
                        };
    double tf = 5.0; // time horizon in seconds
    double dt = 1e-3;

    // set initial state and dt for sim
    sim.set_x0(x0);
    sim.set_dt(dt);
    // log in csv format time, x, y, z, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz
    // u0, u1, u2, u3, ..., x_des, y_des, z_des, vx_des, vy_des, vz_des, qw_des, qx_des, qy_des, qz_des, wx_des, wy_des, wz_des
    log_file << "time,x,y,z,vx,vy,vz,qw,qx,qy,qz,wx,wy,wz,";
    for (size_t i = 0; i < u0.size(); ++i) {
        log_file << "u" << i << ",";
    }
    log_file << "x_des,y_des,z_des,vx_des,vy_des,vz_des,qw_des,qx_des,qy_des,qz_des,wx_des,wy_des,wz_des\n";
    auto log_state = [&log_file](double time, const std::vector<double>& x, const std::vector<double>& xdes, const std::vector<double>& u) {
        log_file << time << ",";
        for (const auto& xi : x) {
            log_file << xi << ",";
        }
        for (const auto& ui : u) {
            log_file << ui << ",";
        }
        for (const auto& xdi : xdes) {
            log_file << xdi << ",";
        }
        log_file << "\n";
    };
    int Niters = static_cast<int>(tf / dt);
    auto start_time = std::chrono::high_resolution_clock::now();
    for (int iter = 0; iter < Niters; ++iter) {
        // solve OCP to get optimal control
        const std::vector<double>& u_opt = ocp.solve(x0, xdes);
        x0 = sim.step(u_opt);
        double current_time = iter * dt;
        log_state(current_time, x0, xdes, u_opt);
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end_time - start_time;
    std::cout << "Simulation completed in " << elapsed.count() << " seconds." << std::endl;
    std::cout<<"Average time per iteration: " << (elapsed.count() / Niters) << " s" << std::endl;
    std::cout<<"Average frequency: " << (Niters / elapsed.count()) << " Hz" << std::endl;
    log_file.close();
    return 0;
}