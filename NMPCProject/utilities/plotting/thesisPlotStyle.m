function style = thesisPlotStyle()
%THESISPLOTSTYLE  Shared plotting constants for thesis figures.
%   Keep text large enough for single-column and near full-width LaTeX use.

style.FontName        = 'Arial';
style.AxisFontSize    = 11;
style.LabelFontSize   = 12;
style.TitleFontSize   = 14;
style.LegendFontSize  = 10;
style.LineWidth       = 1.7;
style.ThinLineWidth   = 1.2;
style.MarkerSize      = 6;
style.AxisLineWidth   = 0.8;
style.GridAlpha       = 0.18;
style.MinorGridAlpha  = 0.08;
style.FigureColor     = 'w';
style.FigureScale     = 1.25;

% Export sizes in centimeters. Vector PDFs preserve sharpness; these sizes
% make the text readable when included in LaTeX without heavy zooming.
style.FigureSize.single = style.FigureScale * [16, 10];
style.FigureSize.small  = (style.FigureScale/2) * [16, 10];
style.FigureSize.wide   = style.FigureScale * [18, 10];
style.FigureSize.tall   = style.FigureScale * [18, 14];
style.FigureSize.grid   = style.FigureScale * [19, 13];
style.FigureSize.overview = style.FigureScale * [24, 16];
end
