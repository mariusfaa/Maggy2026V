function col_shade = thesisShadeColor(col, idx, n)
%THESISSHADECOLOR  Lighter-to-darker shades of one base color.
%   Useful when a figure compares horizons for the same controller.

shade = 0.3 + 0.7 * (idx - 1) / max(n - 1, 1);
col_shade = 1 - shade * (1 - col);
end
