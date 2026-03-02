load_params(MaglevModel.Accurate);

%% Constants
r   = params.magnet.r;
l   = params.magnet.l;
I   = 1;
mu0 = 1e-6;

%% 2D Grid (y = 0 slice)
N = 45;
range = 1;

xrange = r*2;
zrange = l*1.1;
range = max(xrange,zrange)

x = linspace(-range, range, N);
z = linspace(-range, range, N);

[X,Z] = meshgrid(x,z);
Y = zeros(size(X));   % y = 0 plane

%% Allocate
Bx = zeros(size(X));
Bz = zeros(size(X));

%% Compute field
rmax = 0
for k = 1:numel(X)

    xx = X(k);
    yy = 0;      % slice condition
    zz = Z(k);

    [phi,rho] = cart2pol(xx,yy);

    rmax = max(rmax, rho)

    % Skip inside magnet volume
    if abs(zz) < l/2 && rho < r
        continue
    end

    %[brho, bz] = computeFieldCircularCurrentSheetPolarSmart(rho, zz, r, l, mu0);
    
    % Use the fact that
    % brho(rho, -z) = -brho(rho, z) — odd in z
    % bz(rho, -z) = bz(rho, z) — even in z
    % this halfs our lookuptable
    [brho, bz] = computeFieldCircularCurrentSheetPolarSmart(rho, abs(zz), r, l, mu0);
    if zz < 0
        brho = -brho;
    end


    brho = brho * I;
    bz   = bz   * I;

    % Polar → Cartesian
    [bx,~,~] = pol2cart(phi,brho,0);

    Bx(k) = bx;
    Bz(k) = bz;
end

%% Magnitude
Bmag = sqrt(Bx.^2 + Bz.^2);

%% Normalize (unit arrows)
eps_val = 1e-12;
Bx_n = Bx ./ (Bmag + eps_val);
Bz_n = Bz ./ (Bmag + eps_val);

arrowScale = 0.002;
Bx_plot = Bx_n * arrowScale;
Bz_plot = Bz_n * arrowScale;

%% Plot
figure;
hold on;

% Background magnitude
imagesc(x,z,Bmag);
set(gca,'YDir','normal');
colormap(jet);
colorbar;

% Quivers (NO autoscaling)
quiver(X,Z,Bx_plot,Bz_plot,0,'k');

xlabel('x');
ylabel('z');
title('2D Slice at y = 0');
axis equal;
grid on;