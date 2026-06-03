function save_fig(fig, out_dir, name)
% SAVE_FIG  Save a figure as both PNG (300 dpi) and PDF (vector) under out_dir.
    if ~isfolder(out_dir)
        mkdir(out_dir);
    end
    png_path = fullfile(out_dir, [name '.png']);
    pdf_path = fullfile(out_dir, [name '.pdf']);
    set(fig, 'Color', 'w');
    try
        exportgraphics(fig, png_path, 'Resolution', 300);
        exportgraphics(fig, pdf_path, 'ContentType', 'vector');
    catch
        print(fig, png_path, '-dpng', '-r300');
        print(fig, pdf_path, '-dpdf', '-bestfit');
    end
end
