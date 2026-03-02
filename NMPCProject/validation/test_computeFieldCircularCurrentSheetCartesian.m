load_params(MaglevModel.Accurate);

%% Constants
r   = params.magnet.r;
l   = params.magnet.l;
I   = 1;
mu0 = 1e-6;

%% Grid
N = 10;
range = 0.1;

x = linspace(-range, range, N);
y = linspace(-range, range, 3);
z = linspace(-range, range, N);

[X,Y,Z] = meshgrid(x,y,z);

%% Allocate
Bx = zeros(size(X));
By = zeros(size(X));
Bz = zeros(size(X));
Bmag = zeros(size(X));

%% Compute field
for k = 1:numel(X)

    xx = X(k);
    yy = Y(k);
    zz = Z(k);

    [phi,rho,~] = cart2pol(xx,yy,0);

    if abs(zz) < l && abs(rho) < r
        continue
    end


    % [bphi1, brho1, bz1] = computeFieldCircularWirePolar(phi, rho, zz, r, mu0);
    % [bphi2, brho2, bz2] = computeFieldCircularCurrentSheetPolar(phi, rho, zz, r, l, mu0);
    % bphi = bphi1 - bphi2;
    % brho = brho1 - brho2;
    % bz = bz1 - bz2;

    [brho, bz] = computeFieldCircularCurrentSheetPolarSmart(rho, zz, r, l, mu0);
    brho = brho * I;
    bz = bz * I;

    % Polar → Cartesian
    [bx,by,~] = pol2cart(phi,brho,0);

    Bx(k) = bx;
    By(k) = by;
    Bz(k) = bz;
end

%% Normalize to length = 1
eps_val = 1e-12;
scale = 0.01;


Bmag = sqrt(Bx.^2 + By.^2 + Bz.^2);

Bx = Bx .* scale;
By = By .* scale;
Bz = Bz .* scale;

Bx_n = Bx ./ (Bmag + eps_val);
By_n = By ./ (Bmag + eps_val);
Bz_n = Bz ./ (Bmag + eps_val);

%% Plot
figure;
hold on;

% Important: turn OFF autoscaling
quiver3(X,Y,Z,Bx_n,By_n,Bz_n,0,'k');  % 0 = no autoscale

% Now color using magnitude via scatter overlay
scatter3(X(:),Y(:),Z(:),20,Bmag(:),'filled');

colormap(jet);
colorbar;

axis equal
grid on
xlabel('x')
ylabel('y')
zlabel('z')
title('Normalized 3D Field (Arrow length = 1, Color = |B|)')
view(3)