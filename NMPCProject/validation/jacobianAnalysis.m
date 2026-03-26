%%
%% jacobianAnalysis — Sensitivity analysis of Jacobian elements A(x,u), B(x,u)
%
% Sweeps the reduced 10-state Fast dynamics over TWO operating envelopes:
%   1. Wide  (±5mm, ±5 deg) — worst-case, covers large perturbations
%   2. Tight (±1mm, ±1 deg) — typical controlled operation near equilibrium
%
% For each envelope the script computes statistics of every Jacobian element
% and classifies it as ZERO, STATIC, SLOW, or DYNAMIC based on its
% coefficient of variation.  The comparison between envelopes reveals
% whether the Jacobian is "well-behaved enough" for a partial-update or
% periodic-relinearization MPC scheme.

clear; clc; close all;

%% ================================================================
%% Setup
%% ================================================================

addpath(genpath('system_parameters'));
addpath(genpath('utilities'));
addpath(genpath('model_matlab'));
addpath(genpath('model_reduced_casadi'));

out_dir = 'figures';
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

font_name  = 'Times New Roman';
font_ax    = 9;
font_lab   = 10;
font_leg   = 8;

%% ================================================================
%% Build numeric Jacobian function
%% ================================================================

params = load_params(MaglevModel.Fast);
[zEq, ~, ~, ~] = computeSystemEquilibria(params, MaglevModel.Fast);

nx = 10;
nu = 4;

xEq_12 = [0; 0; zEq(1); zeros(9,1)];
uEq = zeros(nu, 1);
xEq = xEq_12([1:5, 7:11]);

f_12 = @(x12, u) maglevSystemDynamics(x12, u, params, MaglevModel.Fast);
keep = [1:5, 7:11];

    function [A, B] = computeJacobian(x10, u, f12, keep, delta)
        x12 = [x10(1:5); 0; x10(6:10); 0];
        h12 = @(xx, uu) xx;
        [A12, B12, ~, ~] = finiteDifferenceLinearization(f12, h12, x12, u, delta);
        A = A12(keep, keep);
        B = B12(keep, :);
    end

delta = 1e-7;

[A_eq, B_eq] = computeJacobian(xEq, uEq, f_12, keep, delta);
fprintf('Equilibrium eigenvalues (real): %s\n', mat2str(real(eig(A_eq))', 4));

state_names = {'x','y','z','\phi','\theta','v_x','v_y','v_z','\omega_x','\omega_y'};
input_names = {'u_1','u_2','u_3','u_4'};

cv_threshold_static = 0.01;   % < 1%  -> static
cv_threshold_slow   = 0.10;   % < 10% -> slow update

%% ================================================================
%% Define two envelopes
%% ================================================================

envelopes = struct();

% --- Wide envelope: ±5mm, ±5 deg ---
envelopes(1).name = 'Wide ($\pm$5\,mm, $\pm$5$^\circ$)';
envelopes(1).tag  = 'wide';
envelopes(1).pos  = 5e-3;
envelopes(1).ang  = 5 * pi/180;
envelopes(1).vel  = 0.1;
envelopes(1).omg  = 10;
envelopes(1).umax = 1;

% --- Tight envelope: ±1mm, ±1 deg ---
envelopes(2).name = 'Tight ($\pm$1\,mm, $\pm$1$^\circ$)';
envelopes(2).tag  = 'tight';
envelopes(2).pos  = 1e-3;
envelopes(2).ang  = 1 * pi/180;
envelopes(2).vel  = 0.02;
envelopes(2).omg  = 2;
envelopes(2).umax = 0.5;

N_samples = 2000;
rng(42);

%% ================================================================
%% Sweep both envelopes
%% ================================================================

for ie = 1:numel(envelopes)
    env = envelopes(ie);
    fprintf('\n========== ENVELOPE: %s ==========\n', env.tag);

    range_lo = [-env.pos; -env.pos; zEq(1)-env.pos; -env.ang; -env.ang;
                -env.vel; -env.vel; -env.vel; -env.omg; -env.omg;
                -env.umax; -env.umax; -env.umax; -env.umax];
    range_hi = [ env.pos;  env.pos; zEq(1)+env.pos;  env.ang;  env.ang;
                 env.vel;  env.vel;  env.vel;  env.omg;  env.omg;
                 env.umax;  env.umax;  env.umax;  env.umax];

    % Stratified random sampling
    lhs_raw = (repmat((0:N_samples-1)', 1, nx+nu) + rand(N_samples, nx+nu)) / N_samples;
    for col = 1:nx+nu
        lhs_raw(:,col) = lhs_raw(randperm(N_samples), col);
    end
    samples = lhs_raw .* (range_hi - range_lo)' + range_lo';

    A_all = zeros(nx, nx, N_samples);
    B_all = zeros(nx, nu, N_samples);

    fprintf('Sampling %d points...\n', N_samples);
    tic;
    for k = 1:N_samples
        x_k = samples(k, 1:nx)';
        u_k = samples(k, nx+1:end)';
        [A_all(:,:,k), B_all(:,:,k)] = computeJacobian(x_k, u_k, f_12, keep, delta);
        if mod(k, 500) == 0, fprintf('  %d/%d (%.1f s)\n', k, N_samples, toc); end
    end
    fprintf('Done in %.1f s\n', toc);

    % --- Statistics ---
    A_mean = mean(A_all, 3);  A_std = std(A_all, 0, 3);
    A_min  = min(A_all,[],3); A_max = max(A_all,[],3);
    A_range = A_max - A_min;
    A_cv = A_std ./ max(abs(A_mean), 1e-12);
    A_cv(abs(A_mean) < 1e-6 & A_std < 1e-6) = 0;

    B_mean = mean(B_all, 3);  B_std = std(B_all, 0, 3);
    B_min  = min(B_all,[],3); B_max = max(B_all,[],3);
    B_range = B_max - B_min;
    B_cv = B_std ./ max(abs(B_mean), 1e-12);
    B_cv(abs(B_mean) < 1e-6 & B_std < 1e-6) = 0;

    % --- Classify ---
    n_z = 0; n_s = 0; n_w = 0; n_d = 0;
    fprintf('\n--- A matrix ---\n');
    fprintf('%-8s  %12s  %12s  %12s  %8s  Class\n', 'Elem', 'Mean', 'Std', 'Range', 'CV');
    fprintf('%s\n', repmat('-', 1, 70));
    for i = 1:nx
        for j = 1:nx
            m=A_mean(i,j); s=A_std(i,j); cv=A_cv(i,j); r=A_range(i,j);
            if abs(m)<1e-6 && s<1e-6, n_z=n_z+1; continue; end
            if cv < cv_threshold_static, cls='STATIC'; n_s=n_s+1;
            elseif cv < cv_threshold_slow, cls='SLOW'; n_w=n_w+1;
            else, cls='DYNAMIC'; n_d=n_d+1; end
            fprintf('A(%2d,%2d)  %12.4e  %12.4e  %12.4e  %8.4f  %s\n',i,j,m,s,r,cv,cls);
        end
    end
    fprintf('A: %d zero, %d static, %d slow, %d dynamic\n', n_z, n_s, n_w, n_d);

    nzb=0; nsb=0; nwb=0; ndb=0;
    fprintf('\n--- B matrix ---\n');
    fprintf('%-8s  %12s  %12s  %12s  %8s  Class\n', 'Elem', 'Mean', 'Std', 'Range', 'CV');
    fprintf('%s\n', repmat('-', 1, 70));
    for i = 1:nx
        for j = 1:nu
            m=B_mean(i,j); s=B_std(i,j); cv=B_cv(i,j); r=B_range(i,j);
            if abs(m)<1e-6 && s<1e-6, nzb=nzb+1; continue; end
            if cv < cv_threshold_static, cls='STATIC'; nsb=nsb+1;
            elseif cv < cv_threshold_slow, cls='SLOW'; nwb=nwb+1;
            else, cls='DYNAMIC'; ndb=ndb+1; end
            fprintf('B(%2d,%2d)  %12.4e  %12.4e  %12.4e  %8.4f  %s\n',i,j,m,s,r,cv,cls);
        end
    end
    fprintf('B: %d zero, %d static, %d slow, %d dynamic\n', nzb, nsb, nwb, ndb);

    total = nx*nx + nx*nu;
    fprintf('\n=== %s: dynamic=%d/%d (%.0f%%), slow=%d, static=%d, zero=%d ===\n', ...
        env.tag, n_d+ndb, total, 100*(n_d+ndb)/total, n_w+nwb, n_s+nsb, n_z+nzb);

    % --- Correlation analysis (non-NaN only) ---
    A_flat = reshape(A_all, nx*nx, N_samples)';
    B_flat = reshape(B_all, nx*nu, N_samples)';
    X_s = samples(:, 1:nx);
    U_s = samples(:, nx+1:end);

    fprintf('\n--- Correlations (dynamic elements, |r| > 0.3) ---\n');
    for i = 1:nx
        for j = 1:nx
            if abs(A_mean(i,j))<1e-6 && A_std(i,j)<1e-6, continue; end
            if A_cv(i,j) < cv_threshold_slow, continue; end
            elem = A_flat(:, (i-1)*nx+j);
            for s = 1:nx
                c = corrcoef(X_s(:,s), elem);
                if ~isnan(c(1,2)) && abs(c(1,2)) > 0.3
                    fprintf('  A(%d,%d) ~ %s: r = %+.3f\n', i,j, state_names{s}, c(1,2));
                end
            end
        end
    end
    for i = 1:nx
        for j = 1:nu
            if abs(B_mean(i,j))<1e-6 && B_std(i,j)<1e-6, continue; end
            if B_cv(i,j) < cv_threshold_slow, continue; end
            elem = B_flat(:, (i-1)*nu+j);
            for s = 1:nx
                c = corrcoef(X_s(:,s), elem);
                if ~isnan(c(1,2)) && abs(c(1,2)) > 0.3
                    fprintf('  B(%d,%d) ~ %s: r = %+.3f\n', i,j, state_names{s}, c(1,2));
                end
            end
        end
    end

    % Store for plotting
    envelopes(ie).A_mean = A_mean; envelopes(ie).A_std = A_std;
    envelopes(ie).A_cv   = A_cv;   envelopes(ie).A_range = A_range;
    envelopes(ie).B_mean = B_mean; envelopes(ie).B_std = B_std;
    envelopes(ie).B_cv   = B_cv;   envelopes(ie).B_range = B_range;
    envelopes(ie).n_dynamic = n_d + ndb;
    envelopes(ie).n_slow    = n_w + nwb;
    envelopes(ie).n_static  = n_s + nsb;
    envelopes(ie).n_zero    = n_z + nzb;
end

%% ================================================================
%% Helper functions
%% ================================================================

    function fig = mkfig(w_cm, h_cm)
        fig = figure('Units','centimeters','Position',[2 2 w_cm h_cm], ...
                     'PaperUnits','centimeters','PaperSize',[w_cm h_cm], ...
                     'PaperPosition',[0 0 w_cm h_cm], 'Color','w');
    end

    function style_ax(ax, fn, fs_ax, fs_lab)
        set(ax, 'FontName', fn, 'FontSize', fs_ax, ...
                'TickDir','out','TickLength',[0.015 0.015], ...
                'LineWidth',0.6,'Box','on');
        set(ax.XLabel,'FontSize',fs_lab);
        set(ax.YLabel,'FontSize',fs_lab);
    end

    function savefig(fig, name, out_dir)
        fpath = fullfile(out_dir, [name '.pdf']);
        exportgraphics(fig, fpath, 'ContentType', 'vector');
        fprintf('  Saved: %s\n', fpath);
    end

%% ================================================================
%% Figure 1: Jacobian magnitude (wide envelope)
%% ================================================================

fprintf('\n--- Generating figures ---\n');
fig1 = mkfig(17, 8);

subplot(1,2,1);
imagesc(log10(max(abs(envelopes(1).A_mean), 1e-15)));
colorbar; colormap(gca, 'parula');
title('$\log_{10}|\bar{A}_{ij}|$', 'Interpreter', 'latex');
xticks(1:nx); xticklabels(state_names); xtickangle(45);
yticks(1:nx); yticklabels(state_names);
xlabel('State $j$', 'Interpreter', 'latex');
ylabel('Equation $i$', 'Interpreter', 'latex');
style_ax(gca, font_name, font_ax, font_lab);

subplot(1,2,2);
imagesc(log10(max(abs(envelopes(1).B_mean), 1e-15)));
colorbar; colormap(gca, 'parula');
title('$\log_{10}|\bar{B}_{ij}|$', 'Interpreter', 'latex');
xticks(1:nu); xticklabels(input_names);
yticks(1:nx); yticklabels(state_names);
xlabel('Input $j$', 'Interpreter', 'latex');
ylabel('Equation $i$', 'Interpreter', 'latex');
style_ax(gca, font_name, font_ax, font_lab);
savefig(fig1, 'jac_magnitude', out_dir);

%% ================================================================
%% Figure 2: CV comparison — wide vs tight (side by side for A)
%% ================================================================

fig2 = mkfig(17, 8);
env_titles = {'Wide envelope', 'Tight envelope'};

for ie = 1:2
    subplot(1,2,ie);
    cv_vis = min(envelopes(ie).A_cv, 2);
    cv_vis(abs(envelopes(ie).A_mean) < 1e-6 & envelopes(ie).A_std < 1e-6) = NaN;

    h = imagesc(cv_vis);
    set(h, 'AlphaData', ~isnan(cv_vis));
    set(gca, 'Color', [0.9 0.9 0.9]);
    colorbar; caxis([0 2]);
    colormap(gca, flipud(hot));
    title(sprintf('CV of $A_{ij}$ --- %s', env_titles{ie}), 'Interpreter', 'latex');
    xticks(1:nx); xticklabels(state_names); xtickangle(45);
    yticks(1:nx); yticklabels(state_names);
    xlabel('State $j$', 'Interpreter', 'latex');
    ylabel('Equation $i$', 'Interpreter', 'latex');
    style_ax(gca, font_name, font_ax, font_lab);

    hold on;
    for i = 1:nx
        for j = 1:nx
            if isnan(cv_vis(i,j)), continue; end
            if cv_vis(i,j) < cv_threshold_static
                text(j,i,'S','HorizontalAlignment','center','FontSize',7,'Color',[0 0.5 0]);
            elseif cv_vis(i,j) < cv_threshold_slow
                text(j,i,'W','HorizontalAlignment','center','FontSize',7,'Color',[0.8 0.6 0]);
            else
                text(j,i,'D','HorizontalAlignment','center','FontSize',7,'Color',[0.8 0 0]);
            end
        end
    end
end
savefig(fig2, 'jac_cv_A_comparison', out_dir);

%% ================================================================
%% Figure 3: CV comparison — wide vs tight (side by side for B)
%% ================================================================

fig3 = mkfig(17, 8);
for ie = 1:2
    subplot(1,2,ie);
    cv_vis = min(envelopes(ie).B_cv, 2);
    cv_vis(abs(envelopes(ie).B_mean) < 1e-6 & envelopes(ie).B_std < 1e-6) = NaN;

    h = imagesc(cv_vis);
    set(h, 'AlphaData', ~isnan(cv_vis));
    set(gca, 'Color', [0.9 0.9 0.9]);
    colorbar; caxis([0 2]);
    colormap(gca, flipud(hot));
    title(sprintf('CV of $B_{ij}$ --- %s', env_titles{ie}), 'Interpreter', 'latex');
    xticks(1:nu); xticklabels(input_names);
    yticks(1:nx); yticklabels(state_names);
    xlabel('Input $j$', 'Interpreter', 'latex');
    ylabel('Equation $i$', 'Interpreter', 'latex');
    style_ax(gca, font_name, font_ax, font_lab);

    hold on;
    for i = 1:nx
        for j = 1:nu
            if isnan(cv_vis(i,j)), continue; end
            if cv_vis(i,j) < cv_threshold_static
                text(j,i,'S','HorizontalAlignment','center','FontSize',7,'Color',[0 0.5 0]);
            elseif cv_vis(i,j) < cv_threshold_slow
                text(j,i,'W','HorizontalAlignment','center','FontSize',7,'Color',[0.8 0.6 0]);
            else
                text(j,i,'D','HorizontalAlignment','center','FontSize',7,'Color',[0.8 0 0]);
            end
        end
    end
end
savefig(fig3, 'jac_cv_B_comparison', out_dir);

%% ================================================================
%% Figure 4: Ranked bar chart — tight envelope
%% ================================================================

fig4 = mkfig(17, 7);
Am = envelopes(2).A_mean; As = envelopes(2).A_std; Acv = envelopes(2).A_cv;
Bm = envelopes(2).B_mean; Bs = envelopes(2).B_std; Bcv = envelopes(2).B_cv;

elems = {}; cvs = [];
for i=1:nx, for j=1:nx
    if abs(Am(i,j))<1e-6 && As(i,j)<1e-6, continue; end
    elems{end+1}=sprintf('A(%d,%d)',i,j); cvs(end+1)=Acv(i,j); %#ok<AGROW>
end, end
for i=1:nx, for j=1:nu
    if abs(Bm(i,j))<1e-6 && Bs(i,j)<1e-6, continue; end
    elems{end+1}=sprintf('B(%d,%d)',i,j); cvs(end+1)=Bcv(i,j); %#ok<AGROW>
end, end

[cvs_sorted, si] = sort(cvs, 'descend');
elems_sorted = elems(si);

n_show = min(30, numel(cvs_sorted));
colors = zeros(n_show, 3);
for k=1:n_show
    if cvs_sorted(k)>=cv_threshold_slow, colors(k,:)=[0.85 0.33 0.10];
    elseif cvs_sorted(k)>=cv_threshold_static, colors(k,:)=[1.0 0.75 0.0];
    else, colors(k,:)=[0.0 0.65 0.3]; end
end

ax = axes; hold on;
b = barh(1:n_show, cvs_sorted(1:n_show));
b.FaceColor='flat'; b.CData=colors; b.EdgeColor='none';
yticks(1:n_show); yticklabels(elems_sorted(1:n_show));
set(gca, 'YDir', 'reverse');
xlabel('Coefficient of Variation', 'Interpreter', 'latex');
title('Jacobian element variability --- tight envelope', 'Interpreter', 'latex');
xline(cv_threshold_static, 'g--', 'LineWidth', 1, 'Label', '1\%', 'FontSize', font_leg);
xline(cv_threshold_slow, '--', 'Color', [0.8 0.6 0], 'LineWidth', 1, 'Label', '10\%', 'FontSize', font_leg);
style_ax(ax, font_name, font_ax, font_lab);
savefig(fig4, 'jac_ranked_tight', out_dir);

%% ================================================================
%% Figure 5: Summary comparison bar — wide vs tight
%% ================================================================

fig5 = mkfig(8.5, 7);
cats = categorical({'Zero','Static','Slow','Dynamic'});
cats = reordercats(cats, {'Zero','Static','Slow','Dynamic'});
counts = [envelopes(1).n_zero, envelopes(1).n_static, envelopes(1).n_slow, envelopes(1).n_dynamic;
          envelopes(2).n_zero, envelopes(2).n_static, envelopes(2).n_slow, envelopes(2).n_dynamic];
bar(cats, counts');
legend({'Wide', 'Tight'}, 'Location', 'northwest', 'FontSize', font_leg);
ylabel('Number of elements');
title('Jacobian classification by envelope', 'Interpreter', 'latex');
style_ax(gca, font_name, font_ax, font_lab);
savefig(fig5, 'jac_envelope_comparison', out_dir);

%% ================================================================
%% Save
%% ================================================================

save('jacobian_analysis.mat', 'envelopes', 'A_eq', 'B_eq', 'xEq', 'uEq', ...
    'state_names', 'input_names', 'N_samples', ...
    'cv_threshold_static', 'cv_threshold_slow');
fprintf('\nAnalysis saved to jacobian_analysis.mat\n');
fprintf('\n=== Jacobian analysis complete ===\n');
