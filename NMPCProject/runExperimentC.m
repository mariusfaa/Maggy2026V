function runExperimentC(exp_id, exp_N, exp_Tf, exp_we)
% RUNEXPERIMENTC  One sub-experiment of Experiment C (SQP_RTI).
%   matlab -batch "runExperimentC('C1', 15, 0.15, 10)"

if ischar(exp_N);  exp_N =str2double(exp_N);  end
if ischar(exp_Tf); exp_Tf=str2double(exp_Tf); end
if ischar(exp_we); exp_we=str2double(exp_we); end

exp_dt=exp_Tf/exp_N;
exp_name=sprintf('C_%s',exp_id);
exp_desc=sprintf('SQP_RTI N=%d Tf=%.2f dt=%.1fms W_e=Q*%d',exp_N,exp_Tf,exp_dt*1e3,exp_we);
fprintf('=== Experiment C — %s ===\n  %s\n\n',exp_id,exp_desc);

%% --- Setup ---
acados_root='/home/mariujf/acados';
project_root='/home/mariujf/Maggy2026V/NMPCProject';
setenv('ACADOS_SOURCE_DIR',acados_root);
setenv('ENV_ACADOS_INSTALL_DIR',acados_root);
setenv('ACADOS_INSTALL_DIR',acados_root);
addpath(fullfile(acados_root,'interfaces','acados_matlab_octave'));
addpath(fullfile(acados_root,'external','jsonlab'));
addpath(fullfile(acados_root,'external','casadi-matlab'));
addpath(genpath(fullfile(project_root,'model_implementations')));
addpath(genpath(fullfile(project_root,'system_parameters')));
addpath(genpath(fullfile(project_root,'utilities')));
import casadi.*

nx=10; nu=4;
[xEq,uEq,x_r,xdot_r,u_sym,f_expl_r,paramsFast] = build_model();
Q=diag([1e2,1e2,1e3,1e3,1e3,1e1,1e1,1e1,1e1,1e1]); R=eye(nu); n_sbx=5;
lbx_c=[-0.025;-0.025;0.015;-0.35;-0.35];
ubx_c=[0.025;0.025;0.055;0.35;0.35]; idx_c=1:5;
dx_dir=[0.005;-0.008;0.010;0.15;-0.10;0.01;-0.01;0.0;0.2;-0.3];
pert_scale=0.05; x_init=xEq+pert_scale*dx_dir; sim_steps=200;

%% === Plant simulator (dt matches this experiment's MPC period) ===
plant_solver = build_plant(sprintf('mplant_%s',exp_id), x_r, u_sym, xdot_r, f_expl_r, exp_dt);

%% === OCP solver (SQP_RTI, with experiment-specific N, Tf, W_e) ===
ocp_solver = build_ocp(sprintf('mlev_%s',exp_id), x_r, u_sym, xdot_r, f_expl_r, ...
    'IRK', 4, 10, exp_N, exp_Tf, Q, R, n_sbx, xEq, uEq, lbx_c, ubx_c, exp_we);

%% === Main simulation ===
[PX,PU,Pstat,Psqp,Pqp,Pt,Perr,Ppos,Pang,Pslack,n_actual,diverged] = ...
    run_sim(ocp_solver, plant_solver, x_init, xEq, uEq, exp_N, nx, nu, sim_steps);

%% === Usability metrics ===
[usab, settling, roa_max] = compute_usability( ...
    PX, PU, Perr, Ppos, Pang, Pslack, xEq, uEq, exp_dt, n_actual, ...
    lbx_c, ubx_c, idx_c, pert_scale, dx_dir, ...
    ocp_solver, plant_solver, exp_N, diverged);

%% === Print ===
print_summary(exp_id, diverged, n_actual, sim_steps, Pstat, Pt, Psqp, ...
    Perr, usab, settling, roa_max);

%% === Save ===
sd = build_save_struct(exp_name, exp_desc, 'SQP_RTI', 'IRK', 4, 10, ...
    exp_N, exp_Tf, exp_dt, nx, nu, paramsFast, pert_scale, exp_we, ...
    PX, PU, Pstat, Psqp, Pqp, Pt, Perr, Ppos, Pang, Pslack, ...
    n_actual, settling, roa_max, usab, xEq, uEq, Q, R, lbx_c, ubx_c, idx_c);
fname=sprintf('nmpc_results_%s.mat',lower(exp_name));
save(fname,'-struct','sd'); fprintf('\nSaved: %s\n',fname);
end

% =====================================================================
%  HELPER FUNCTIONS
% =====================================================================

function [xEq,uEq,x_r,xdot_r,u_sym,f_expl_r,paramsFast] = build_model()
    import casadi.*
    nx_full=12; nx=10; nu=4;
    x_full=SX.sym('x_full',nx_full); u_sym=SX.sym('u',nu);
    params=struct(); parameters_maggy_V4;
    paramsFast=params;
    cf=computeSolenoidRadiusCorrectionFactor(paramsFast,'fast');
    paramsFast.solenoids.r=cf*paramsFast.solenoids.r;
    f_expl_full=maglevSystemDynamicsCasADi(x_full,u_sym,paramsFast);
    f_func=casadi.Function('f_full',{x_full,u_sym},{f_expl_full});
    z_var=SX.sym('z_eq');
    accel=f_func([0;0;z_var;zeros(9,1)],zeros(nu,1));
    nlp=struct('x',z_var,'f',accel(9)^2);
    sol_eq=nlpsol('eq','ipopt',nlp,struct('ipopt',struct('print_level',0)));
    sol=sol_eq('x0',0.030,'lbx',0.015,'ubx',0.060);
    zEq=full(sol.x); uEq=zeros(nu,1); xEq=[0;0;zEq;zeros(7,1)];
    fprintf('Equilibrium z = %.6f m\n',zEq);
    x_r=SX.sym('x_r',nx); xdot_r=SX.sym('xdot_r',nx);
    dx=f_func([x_r(1:5);0;x_r(6:8);x_r(9:10);0],u_sym);
    f_expl_r=[dx(1:5);dx(7:11)];
end

function ps = build_plant(name, x_r, u_sym, xdot_r, f_expl_r, dt)
    sim=AcadosSim(); sim.model.name=name;
    sim.model.x=x_r; sim.model.u=u_sym; sim.model.xdot=xdot_r;
    sim.model.f_impl_expr=xdot_r-f_expl_r;
    sim.solver_options.Tsim=dt; sim.solver_options.integrator_type='IRK';
    sim.solver_options.num_stages=4; sim.solver_options.num_steps=10;
    ps=AcadosSimSolver(sim);
end

function os = build_ocp(name, x_r, u_sym, xdot_r, f_expl_r, ...
        integ_type, n_stages, n_steps, N, Tf, Q, R, n_sbx, xEq, uEq, lbx_c, ubx_c, we_scale)
    nu=length(uEq);
    ocp=AcadosOcp(); ocp.model.name=name;
    ocp.model.x=x_r; ocp.model.u=u_sym; ocp.model.xdot=xdot_r;
    ocp.model.f_impl_expr=xdot_r-f_expl_r; ocp.model.f_expl_expr=f_expl_r;
    ocp.solver_options.N_horizon=N; ocp.solver_options.tf=Tf;
    ocp.solver_options.integrator_type=integ_type;
    ocp.solver_options.sim_method_num_stages=n_stages;
    ocp.solver_options.sim_method_num_steps=n_steps;
    ocp.solver_options.nlp_solver_type='SQP_RTI';
    ocp.solver_options.nlp_solver_max_iter=1;
    ocp.solver_options.qp_solver='PARTIAL_CONDENSING_HPIPM';
    ocp.solver_options.qp_solver_iter_max=200;
    ocp.solver_options.qp_solver_warm_start=1;
    ocp.solver_options.hessian_approx='GAUSS_NEWTON';
    ocp.solver_options.regularize_method='CONVEXIFY';
    ocp.cost.cost_type='NONLINEAR_LS'; ocp.cost.cost_type_0='NONLINEAR_LS';
    ocp.cost.cost_type_e='NONLINEAR_LS';
    ocp.cost.W=blkdiag(Q,R); ocp.cost.W_0=blkdiag(Q,R);
    ocp.cost.W_e=Q*we_scale;
    ocp.model.cost_y_expr=[x_r;u_sym]; ocp.model.cost_y_expr_0=[x_r;u_sym];
    ocp.model.cost_y_expr_e=x_r;
    ocp.cost.yref=[xEq;uEq]; ocp.cost.yref_0=[xEq;uEq]; ocp.cost.yref_e=xEq;
    ocp.constraints.idxbu=0:nu-1; ocp.constraints.lbu=-ones(nu,1); ocp.constraints.ubu=ones(nu,1);
    ocp.constraints.idxbx=[0,1,2,3,4]; ocp.constraints.lbx=lbx_c; ocp.constraints.ubx=ubx_c;
    ocp.constraints.idxsbx=0:4;
    ocp.cost.Zl=1e3*ones(n_sbx,1); ocp.cost.Zu=1e3*ones(n_sbx,1);
    ocp.cost.zl=1e2*ones(n_sbx,1); ocp.cost.zu=1e2*ones(n_sbx,1);
    ocp.constraints.x0=xEq;
    os=AcadosOcpSolver(ocp);
    fprintf('OCP solver ready (SQP_RTI, %s %d/%d, N=%d, W_e=Q*%d).\n',...
        integ_type,n_stages,n_steps,N,we_scale);
end

function [PX,PU,Pstat,Psqp,Pqp,Pt,Perr,Ppos,Pang,Pslack,n_actual,diverged] = ...
        run_sim(ocp_solver, plant_solver, x_init, xEq, uEq, N, nx, nu, sim_steps)
    x_current=x_init;
    for k=0:N; a=k/N; ocp_solver.set('x',(1-a)*x_current+a*xEq,k); end
    for k=0:N-1; ocp_solver.set('u',uEq,k); end

    PX=zeros(nx,sim_steps+1); PX(:,1)=x_current;
    PU=zeros(nu,sim_steps);
    Pstat=zeros(1,sim_steps); Psqp=zeros(1,sim_steps); Pqp=zeros(1,sim_steps);
    Pt=zeros(8,sim_steps);  % rows: tot,lin,sim,glob,qp,reg,sim_ad,sim_la
    Perr=zeros(1,sim_steps+1); Ppos=zeros(1,sim_steps+1); Pang=zeros(1,sim_steps+1);
    Pslack=zeros(1,sim_steps);
    Perr(1)=norm(x_current-xEq);
    Ppos(1)=norm(x_current(1:3)-xEq(1:3))*1e3;
    Pang(1)=rad2deg(norm(x_current(4:5)-xEq(4:5)));

    fprintf('Running simulation (%d steps)...\n',sim_steps);
    for i=1:sim_steps
        ocp_solver.set('constr_x0',x_current); ocp_solver.solve();
        status=ocp_solver.get('status'); sqp_iter=ocp_solver.get('sqp_iter');
        u_applied=ocp_solver.get('u',0); u_applied=max(min(u_applied,1),-1);
        Pt(1,i)=ocp_solver.get('time_tot'); Pt(2,i)=ocp_solver.get('time_lin');
        Pt(5,i)=ocp_solver.get('time_qp_sol'); Pt(6,i)=ocp_solver.get('time_reg');
        try Pt(3,i)=ocp_solver.get('time_sim');    catch; Pt(3,i)=NaN; end
        try Pt(7,i)=ocp_solver.get('time_sim_ad'); catch; Pt(7,i)=NaN; end
        try Pt(8,i)=ocp_solver.get('time_sim_la'); catch; Pt(8,i)=NaN; end
        try Pt(4,i)=ocp_solver.get('time_glob');   catch; Pt(4,i)=NaN; end
        try qr=ocp_solver.get('qp_iter'); qp_it=sum(qr(:)); catch; qp_it=NaN; end
        all_sl=0;
        for k=0:N
            try sl=ocp_solver.get('sl',k); su=ocp_solver.get('su',k);
                all_sl=[all_sl;abs(sl(:));abs(su(:))]; %#ok<AGROW>
            catch; end
        end
        plant_solver.set('x',x_current); plant_solver.set('u',u_applied);
        plant_solver.solve(); x_next=plant_solver.get('xn');
        xt=ocp_solver.get('x'); ut=ocp_solver.get('u');
        for k=0:N; if k<N; ocp_solver.set('x',xt(:,k+2),k); else; ocp_solver.set('x',xEq,k); end; end
        for k=0:N-1; if k<N-1; ocp_solver.set('u',ut(:,k+2),k); else; ocp_solver.set('u',uEq,k); end; end

        PX(:,i+1)=x_next; PU(:,i)=u_applied;
        Pstat(i)=status; Psqp(i)=sqp_iter; Pqp(i)=qp_it;
        Perr(i+1)=norm(x_next-xEq);
        Ppos(i+1)=norm(x_next(1:3)-xEq(1:3))*1e3;
        Pang(i+1)=rad2deg(norm(x_next(4:5)-xEq(4:5)));
        Pslack(i)=max(all_sl);

        if i<=3||mod(i,50)==0||i==sim_steps
            fprintf('  Step %3d: st=%d sqp=%d t=%.1fms z=%.4fmm |e|=%.2e\n',...
                i,status,sqp_iter,Pt(1,i)*1e3,x_next(3)*1e3,Perr(i+1));
        end
        if abs(x_next(3))>0.5||max(abs(x_next(4:5)))>pi||any(isnan(x_next))||any(isinf(x_next))
            fprintf('  *** DIVERGED step %d ***\n',i); break;
        end
        x_current=x_next;
    end
    n_actual=min(i,sim_steps); diverged=n_actual<sim_steps;
end

function [usab, settling, roa_max] = compute_usability( ...
        PX, PU, Perr, Ppos, Pang, Pslack, xEq, uEq, dt, n_actual, ...
        lbx_c, ubx_c, idx_c, pert_scale, dx_dir, ...
        ocp_solver, plant_solver, N, diverged)
    usab.diverged = diverged;
    usab.peak_pos_mm  = max(Ppos(1:n_actual+1));
    usab.peak_ang_deg = max(Pang(1:n_actual+1));
    usab.peak_z_dev_mm = max(abs(PX(3,1:n_actual+1)-xEq(3)))*1e3;
    usab.control_effort = sum(sum(PU(:,1:n_actual).^2,1))*dt;
    usab.pct_saturated = 100*sum(any(abs(PU(:,1:n_actual))>0.999,1))/n_actual;
    n_viol=0; max_viol=0;
    for j=1:n_actual+1
        for c=1:length(idx_c)
            v=max([lbx_c(c)-PX(idx_c(c),j),PX(idx_c(c),j)-ubx_c(c),0]);
            if v>0; n_viol=n_viol+1; max_viol=max(max_viol,v); end
        end
    end
    usab.n_hard_viol=n_viol; usab.max_hard_viol=max_viol;

    initial_err=Perr(1); thr_pct=[0.01,0.02,0.05];
    settling=NaN(size(thr_pct));
    for s=1:length(thr_pct)
        thr=thr_pct(s)*initial_err;
        for j=1:n_actual+1
            if all(Perr(j:n_actual+1)<=thr); settling(s)=(j-1)*dt; break; end
        end
    end

    roa_scales=[0.1,0.2,0.5,1.0]; roa_steps=50;
    roa_max=pert_scale;
    fprintf('\nRoA sweep...\n');
    for r=1:length(roa_scales)
        sc=roa_scales(r); x_roa=xEq+sc*dx_dir;
        for k=0:N; a=k/N; ocp_solver.set('x',(1-a)*x_roa+a*xEq,k); end
        for k=0:N-1; ocp_solver.set('u',uEq,k); end
        x_t=x_roa; ok=true;
        for j=1:roa_steps
            ocp_solver.set('constr_x0',x_t); ocp_solver.solve();
            u_t=ocp_solver.get('u',0); u_t=max(min(u_t,1),-1);
            plant_solver.set('x',x_t); plant_solver.set('u',u_t);
            plant_solver.solve(); x_t=plant_solver.get('xn');
            xt=ocp_solver.get('x'); ut=ocp_solver.get('u');
            for k=0:N; if k<N; ocp_solver.set('x',xt(:,k+2),k); else; ocp_solver.set('x',xEq,k); end; end
            for k=0:N-1; if k<N-1; ocp_solver.set('u',ut(:,k+2),k); else; ocp_solver.set('u',uEq,k); end; end
            if abs(x_t(3))>0.5||max(abs(x_t(4:5)))>pi||any(isnan(x_t))||any(isinf(x_t))
                ok=false; break;
            end
        end
        if ok; roa_max=sc; fprintf('  scale=%.2f: STABLE (|e|=%.2e)\n',sc,norm(x_t-xEq));
        else; fprintf('  scale=%.2f: DIVERGED step %d\n',sc,j); break; end
    end
end

function print_summary(id, diverged, n_actual, sim_steps, Pstat, Pt, Psqp, Perr, usab, settling, roa_max)
    tv=Pt(1,1:n_actual); stv=Pstat(1:n_actual);
    fprintf('\n--- %s Summary ---\n',id);
    fprintf('  Diverged:      %s\n',yn_(diverged));
    fprintf('  Steps:         %d/%d  Conv: %.0f%%\n',n_actual,sim_steps,100*sum(stv==0)/n_actual);
    fprintf('  Solve time:    mean=%.2f med=%.2f max=%.2f ms\n',mean(tv)*1e3,median(tv)*1e3,max(tv)*1e3);
    fprintf('  Peak pos:      %.3f mm   Peak ang: %.3f deg\n',usab.peak_pos_mm,usab.peak_ang_deg);
    fprintf('  Ctrl effort:   %.4f   Saturated: %.1f%%\n',usab.control_effort,usab.pct_saturated);
    fprintf('  Hard viol:     %d (max %.2e)\n',usab.n_hard_viol,usab.max_hard_viol);
    fprintf('  Settle 1%%:    %.0f ms   5%%: %.0f ms\n',settling(1)*1e3,settling(3)*1e3);
    fprintf('  RoA max:       %.2f\n',roa_max);
end

function sd = build_save_struct(exp_name, exp_desc, nlp_type, integ_type, n_stages, n_steps, ...
        N, Tf, dt, nx, nu, paramsFast, pert_scale, we_scale, ...
        PX, PU, Pstat, Psqp, Pqp, Pt, Perr, Ppos, Pang, Pslack, ...
        n_actual, settling, roa_max, usab, xEq, uEq, Q, R, lbx_c, ubx_c, idx_c)
    sd=struct();
    sd.experiment_name=exp_name; sd.experiment_desc=exp_desc;
    sd.timestamp=datestr(now,'yyyy-mm-dd_HH-MM-SS');
    sd.config.nlp_solver_type=nlp_type; sd.config.integrator_type=integ_type;
    sd.config.sim_method_num_stages=n_stages; sd.config.sim_method_num_steps=n_steps;
    sd.config.qp_solver='PARTIAL_CONDENSING_HPIPM'; sd.config.qp_solver_iter_max=200;
    sd.config.hessian_approx='GAUSS_NEWTON'; sd.config.regularize_method='CONVEXIFY';
    sd.config.cost_type='NONLINEAR_LS'; sd.config.N=N; sd.config.Tf=Tf; sd.config.dt=dt;
    sd.config.nx=nx; sd.config.nu=nu; sd.config.magnet_n=paramsFast.magnet.n;
    sd.config.perturbation_scale=pert_scale; sd.config.we_scale=we_scale;
    sim_steps=size(PU,2);
    sd.t=(0:sim_steps)*dt; sd.x=PX; sd.u=PU;
    sd.status=Pstat; sd.sqp_iter=Psqp; sd.qp_iter=Pqp;
    sd.timing.time_tot=Pt(1,:); sd.timing.time_lin=Pt(2,:);
    sd.timing.time_sim=Pt(3,:); sd.timing.time_glob=Pt(4,:);
    sd.timing.time_qp_sol=Pt(5,:); sd.timing.time_reg=Pt(6,:);
    sd.timing.time_sim_ad=Pt(7,:); sd.timing.time_sim_la=Pt(8,:);
    sd.tracking.err_norm=Perr; sd.tracking.err_pos=Ppos; sd.tracking.err_ang=Pang;
    sd.constraints.max_slack=Pslack;
    sd.constraints.n_hard_violations=usab.n_hard_viol;
    sd.constraints.max_violation=usab.max_hard_viol;
    sd.settling.thresholds_pct=[1,2,5]; sd.settling.times_s=settling;
    sd.roa.scales_tested=[pert_scale,0.1,0.2,0.5,1.0];
    sd.roa.max_stable=roa_max; sd.roa.test_steps=50;
    sd.usability=usab;
    tv=Pt(1,1:n_actual); sv=Psqp(1:n_actual); qv=Pqp(1:n_actual); stv=Pstat(1:n_actual);
    sd.summary.n_actual=n_actual; sd.summary.all_converged=all(stv==0);
    sd.summary.solve_time_mean_ms=mean(tv)*1e3; sd.summary.solve_time_median_ms=median(tv)*1e3;
    sd.summary.solve_time_std_ms=std(tv)*1e3; sd.summary.solve_time_max_ms=max(tv)*1e3;
    sd.summary.solve_time_min_ms=min(tv)*1e3;
    sd.summary.solve_time_p95_ms=prctile_(tv,95)*1e3;
    sd.summary.solve_time_p99_ms=prctile_(tv,99)*1e3;
    sd.summary.sqp_iter_mean=mean(sv); sd.summary.qp_iter_mean=nanmean_(qv);
    sd.summary.time_lin_mean_ms=nanmean_(Pt(2,1:n_actual))*1e3;
    sd.summary.time_sim_mean_ms=nanmean_(Pt(3,1:n_actual))*1e3;
    sd.summary.time_qp_sol_mean_ms=nanmean_(Pt(5,1:n_actual))*1e3;
    sd.summary.time_reg_mean_ms=nanmean_(Pt(6,1:n_actual))*1e3;
    sd.summary.final_err_norm=Perr(n_actual+1);
    sd.summary.final_err_pos_mm=Ppos(n_actual+1);
    sd.summary.final_err_ang_deg=Pang(n_actual+1);
    sd.summary.settling_1pct_ms=settling(1)*1e3;
    sd.summary.settling_2pct_ms=settling(2)*1e3;
    sd.summary.settling_5pct_ms=settling(3)*1e3;
    sd.summary.max_soft_slack=max(Pslack(1:n_actual));
    sd.summary.pct_status_0=100*sum(stv==0)/n_actual;
    sd.summary.roa_max_stable=roa_max;
    sd.xEq=xEq; sd.uEq=uEq; sd.Q=Q; sd.R=R; sd.params=paramsFast;
end

function s=yn_(b); if b;s='YES';else;s='NO';end; end
function m=nanmean_(x); x=x(~isnan(x)); if isempty(x);m=NaN;else;m=mean(x);end; end
function p=prctile_(x,pct); x=sort(x(~isnan(x)));n=length(x);if n==0;p=NaN;return;end
    r=(pct/100)*n;lo=max(floor(r),1);hi=min(ceil(r),n);
    if lo==hi;p=x(lo);else;p=x(lo)+(r-lo)*(x(hi)-x(lo));end
end