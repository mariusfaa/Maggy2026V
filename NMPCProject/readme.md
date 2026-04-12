tips for installation

Install gcc-13, default gcc for wsl is pretty old, gcc-13 might give a bit faster compiletime

```
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt update
sudo apt install gcc-13 g++-13
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-13 13 --slave /usr/bin/g++ g++ /usr/bin/g++-13
```

```
mkdir -p build
cd build
cmake -DACADOS_WITH_QPOASES=ON -DACADOS_WITH_DAQP=ON -DACADOS_WITH_OPENMP=ON ..
make install -j14
```


TODOS - things i want to research compare for my thesis:
It is importaint to note that our sim_solver does not need to be very fast, here we ONLY care about accuracy, have found that magnet.n = 16 and n_radial=1 for the accurate model with ERK, stages=4,steps=1 to be a good solver, with timestep = 0.0001s to get good simulations. but maybe we find something better after integrator study.
* Integrator accuracy study (`analysis/integratorStudy.m`)
    - **Single-step approach**: Sample 500 random (x, u) points near the operating region, take ONE integration step with each acados config, compare against ode15s single-step reference using the SAME model (n=16). This isolates pure numerical integration error from model fidelity error.
    - Sweeps ERK/IRK x stages(1-4) x steps(1,2,4,8) at dt = [0.1ms, 1ms, 2ms, 3ms]
    - Figures: accuracy-vs-time Pareto, error/timing heatmaps, error distribution boxplots, RMSE-vs-dt curves
    - Note: a previous trajectory-based approach showed all integrators identical — because model discretization error dominated. The single-step approach reveals actual numerical differences.
* Model fidelity study (`analysis/magnetNStudy.m`)
    - **Part A**: Proves n_axial has NO effect on model accuracy for the Accurate model (sweep n_axial=[1,3,7,11,21] at n=16 — all identical)
    - **Part B**: Sweeps magnet.n = [2,4,6,8,12,16,20,24,32,48,64] with n_axial=1 fixed
    - Fixed ERK(4,1) integrator at dt=1ms, same 500-point single-step methodology
    - Reference: ode15s at n=80, n_axial=21 (highest fidelity)
    - Key finding: accuracy saturates at n=16 — doubling to n=32/64 gives zero improvement at 2x/4x cost
    - Figures: n_axial invariance proof, accuracy+cost vs n, error distributions, all-state errors vs n
* Jacobian analysis of system
    - analyse the jacobians of A,B system to potentially come with ideas for fast "linearization" approaches for sol mpc, maybe worth taking a look? or just good for thesis idk
* Observability and Controllability of system
    - should be addressed so we can explain why it is feasable to control the system.
    - would be interesting to find some way to find the "minimal" required control rate, can we find out at what rate the system oscillate or something to back the fact that control rates under 1000hz induce a lot of vibrations that dampen very slow
* LMPC,SOLNMPC,NMPC
    - Compare things like settling time, cost, terminal cost to compare different controllers
    - Computational time
    - find "zone" of controllable space (maybe by trial and error)
    - robustness to disturbance (apply a force and see how controllers handle it)
* NMPC RTI Scheme analysis
    - what does it do, and how does it compare to normal nmpc