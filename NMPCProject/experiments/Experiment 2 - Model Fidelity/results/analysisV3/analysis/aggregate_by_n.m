function A = aggregate_by_n(T, value_cols)
% AGGREGATE_BY_N  Per-(variant, n) mean / median / min / max / count of value_cols.
%
%   Only rows with completed == 1 contribute to the aggregation of numeric
%   value_cols. A success_count column is added regardless.

    if ~iscell(value_cols)
        value_cols = cellstr(value_cols);
    end

    variants = unique(string(T.variant));
    rows = {};
    for v = 1:numel(variants)
        Tv = T(string(T.variant) == variants(v), :);
        ns = unique(Tv.magnet_n);
        for k = 1:numel(ns)
            Tn = Tv(Tv.magnet_n == ns(k), :);
            ok = Tn(double(Tn.completed) == 1, :);
            row = struct();
            row.variant = variants(v);
            row.magnet_n = ns(k);
            row.n_runs = height(Tn);
            row.success_count = height(ok);
            for c = 1:numel(value_cols)
                col = value_cols{c};
                if ~ismember(col, Tn.Properties.VariableNames)
                    continue;
                end
                vals = double(ok.(col));
                vals = vals(~isnan(vals));
                if isempty(vals)
                    row.([col '_mean'])   = NaN;
                    row.([col '_median']) = NaN;
                    row.([col '_min'])    = NaN;
                    row.([col '_max'])    = NaN;
                else
                    row.([col '_mean'])   = mean(vals);
                    row.([col '_median']) = median(vals);
                    row.([col '_min'])    = min(vals);
                    row.([col '_max'])    = max(vals);
                end
            end
            rows{end+1} = row; %#ok<AGROW>
        end
    end
    A = struct2table([rows{:}]);
end
