function ctrl_colors = thesisControllerColors()
%THESISCONTROLLERCOLORS  Shared controller colors for thesis plots.
%   Palette follows dare_vs_nodare.m and is reused across all plot scripts.

ctrl_colors = containers.Map();
ctrl_colors('lqr')     = [0.50 0.50 0.50];   % gray baseline
ctrl_colors('lmpc')    = [0.00 0.45 0.74];   % blue
ctrl_colors('solnmpc') = [0.85 0.33 0.10];   % orange
ctrl_colors('nmpc')    = [0.47 0.67 0.19];   % green
end
