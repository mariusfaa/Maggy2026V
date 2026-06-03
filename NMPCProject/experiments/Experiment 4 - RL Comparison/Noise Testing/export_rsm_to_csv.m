%% export_rsm_to_csv.m
% =========================================================================
%  Export rsm_noise_results.mat to CSVs for plotting in a separate TeX
%  workspace (pgfplots / TikZ). Reproduces the data behind:
%     - ofat_sensitivity.fig       -> ofat_runs.csv + ofat_summary.csv
%     - rsm_contours_yrms.fig      -> rsm_runs.csv  + rsm_contour_grid.csv
%  Also writes meta.csv with factor decoding info.
%
%  Run from C:\Users\mariujf\noise_testing
% =========================================================================
clearvars; clc;

mat_file = 'rsm_noise_results.mat';
out_dir  = 'csv_export';
if ~exist(out_dir, 'dir'); mkdir(out_dir); end

S = load(mat_file);

results       = S.results;
ofat          = S.ofat;
factor_names  = S.factor_names;
log_center    = S.log_center;
step_per_unit = S.step_per_unit;
y_rms_base    = S.y_rms_base;
ofat_thr      = S.ofat_thresholds;

%% --- 1. Raw RSM runs ----------------------------------------------------
% One row per (design point, replicate). Columns:
%   design, rep, f1..f4 (coded -1/0/+1), sigma_pos..sigma_ang (real units),
%   seed, y_rms, y_maxpos, y_diverged, log10_y_rms
results_out = results;
results_out.log10_y_rms = log10(max(results.y_rms, 1e-8));
writetable(results_out, fullfile(out_dir, 'rsm_runs.csv'));
fprintf('Wrote %s  (%d rows)\n', fullfile(out_dir,'rsm_runs.csv'), height(results_out));

%% --- 2. RSM contour grids -----------------------------------------------
% Refit the same quadratic model used in noiseRSM_reducedOrder.m, then
% evaluate it on a 40x40 grid for each of the 6 factor pairs (other two
% factors fixed at coded 0). This is exactly the data behind the contourf
% panels in rsm_contours_yrms.fig.
results.log_y_rms = log10(max(results.y_rms, 1e-8));
surv = results.y_diverged == 0;

mdl = fitlm(results(surv,:), ...
    ['log_y_rms ~ f1 + f2 + f3 + f4 + ' ...
     'f1:f2 + f1:f3 + f1:f4 + f2:f3 + f2:f4 + f3:f4 + ' ...
     'f1^2 + f2^2 + f3^2 + f4^2']);

pair_list = nchoosek(1:4, 2);
ngrid     = 40;
gx        = linspace(-1, 1, ngrid);
[GX, GY]  = meshgrid(gx, gx);

grid_rows = [];   % columns: pair_id, fa_idx, fb_idx, fa_coded, fb_coded, log10_y_rms
for k = 1:size(pair_list,1)
    i1 = pair_list(k,1);
    i2 = pair_list(k,2);
    fvals = zeros(numel(GX), 4);
    fvals(:, i1) = GX(:);
    fvals(:, i2) = GY(:);
    pred = table(fvals(:,1), fvals(:,2), fvals(:,3), fvals(:,4), ...
        'VariableNames', {'f1','f2','f3','f4'});
    z = predict(mdl, pred);

    block = [repmat(k, numel(z), 1), ...
             repmat(i1, numel(z), 1), repmat(i2, numel(z), 1), ...
             GX(:), GY(:), z];
    grid_rows = [grid_rows; block]; %#ok<AGROW>
end
grid_T = array2table(grid_rows, ...
    'VariableNames', {'pair_id','fa_idx','fb_idx','fa_coded','fb_coded','log10_y_rms'});
writetable(grid_T, fullfile(out_dir, 'rsm_contour_grid.csv'));
fprintf('Wrote %s  (%d rows = %d pairs x %dx%d grid)\n', ...
    fullfile(out_dir,'rsm_contour_grid.csv'), height(grid_T), size(pair_list,1), ngrid, ngrid);

% Pair lookup table so the TeX side knows which factor is which axis
pair_T = array2table([(1:size(pair_list,1))', pair_list], ...
    'VariableNames', {'pair_id','fa_idx','fb_idx'});
pair_T.fa_name = factor_names(pair_T.fa_idx)';
pair_T.fb_name = factor_names(pair_T.fb_idx)';
writetable(pair_T, fullfile(out_dir, 'rsm_contour_pairs.csv'));
fprintf('Wrote %s\n', fullfile(out_dir,'rsm_contour_pairs.csv'));

%% --- 3. OFAT raw runs ---------------------------------------------------
% One row per (factor, sigma level, replicate).
ofat_out = ofat;
ofat_out.factor_name = factor_names(ofat.factor)';
writetable(ofat_out, fullfile(out_dir, 'ofat_runs.csv'));
fprintf('Wrote %s  (%d rows)\n', fullfile(out_dir,'ofat_runs.csv'), height(ofat_out));

%% --- 4. OFAT per-factor summary (medians + IQR + P(div)) ----------------
% This is what each subplot in ofat_sensitivity.fig actually plots.
sum_rows = [];
for f = 1:4
    rows = ofat.factor == f;
    lvls = unique(ofat.sigma(rows));
    for li = 1:numel(lvls)
        m  = rows & ofat.sigma == lvls(li);
        rr = ofat.y_rms(m & ~isnan(ofat.y_rms));
        if isempty(rr)
            med = NaN; p25 = NaN; p75 = NaN;
        else
            med = median(rr);
            p25 = quantile(rr, 0.25);
            p75 = quantile(rr, 0.75);
        end
        pdiv = mean(ofat.y_diverged(m));
        sum_rows = [sum_rows; f, li, lvls(li), med, p25, p75, pdiv, sum(m)]; %#ok<AGROW>
    end
end
sum_T = array2table(sum_rows, 'VariableNames', ...
    {'factor','level_idx','sigma','y_rms_median','y_rms_p25','y_rms_p75','p_diverged','n_reps'});
sum_T.factor_name = factor_names(sum_T.factor)';
writetable(sum_T, fullfile(out_dir, 'ofat_summary.csv'));
fprintf('Wrote %s  (%d rows)\n', fullfile(out_dir,'ofat_summary.csv'), height(sum_T));

%% --- 5. Metadata --------------------------------------------------------
meta = table((1:4)', factor_names', log_center', ofat_thr, ...
    'VariableNames', {'factor_idx','factor_name','log10_center','sigma_thr_p50'});
writetable(meta, fullfile(out_dir, 'meta_factors.csv'));

scalars = table({'y_rms_base'; 'step_per_unit'; 'n_replicates'; 'sim_steps'}, ...
                [y_rms_base; step_per_unit; S.n_replicates; S.sim_steps], ...
    'VariableNames', {'name','value'});
writetable(scalars, fullfile(out_dir, 'meta_scalars.csv'));
fprintf('Wrote %s, %s\n', fullfile(out_dir,'meta_factors.csv'), fullfile(out_dir,'meta_scalars.csv'));

fprintf('\nDone. CSVs are in %s\\\n', out_dir);
