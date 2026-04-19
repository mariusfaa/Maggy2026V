function plotCoordinateFrame(x, y, z, theta, phi, yaw, scale)
    % Inputs: (x,y,z) origin, (theta,phi,yaw) Euler angles in degrees, scale
    
    % 1. Calculate Rotation Matrix (Z-Y-X sequence)
    t = (theta); p = (phi); y_rad = (yaw);
    
    Rx = [1 0 0; 0 cos(t) -sin(t); 0 sin(t) cos(t)];
    Ry = [cos(p) 0 sin(p); 0 1 0; -sin(p) 0 cos(p)];
    Rz = [cos(y_rad) -sin(y_rad) 0; sin(y_rad) cos(y_rad) 0; 0 0 1];
    
    R = Rz * Ry * Rx; % Combined rotation matrix
    
    % 2. Define Axis Vectors
    origin = [x; y; z];
    axes_dir = R * eye(3) * scale;
    
    hold on; grid on; axis equal; view(3);
    colors = ['r', 'g', 'b'];
    labels = {'X', 'Y', 'Z'};
    
    for i = 1:3
        % Plot Main Axis Arrow
        quiver3(x, y, z, axes_dir(1,i), axes_dir(2,i), axes_dir(3,i), ...
            'Color', colors(i), 'LineWidth', 2, 'MaxHeadSize', 0.5);
        
        % Plot Label at the tip
        text(x + axes_dir(1,i)*1.1, y + axes_dir(2,i)*1.1, z + axes_dir(3,i)*1.1, ...
            labels{i}, 'Color', colors(i), 'FontWeight', 'bold');
            
        % 3. Plot Circular Rotation Arrow around each axis
        drawRotationArrow(origin, R(:,i), colors(i), scale);
    end
end

function drawRotationArrow(origin, axis_vec, col, scale)
    % Generate a circle in the plane perpendicular to axis_vec
    t = linspace(0, 1.5*pi, 50); % 270-degree arc
    r = scale * 0.2;            % Radius of circular arrow
    
    % Find two vectors perpendicular to axis_vec to define the plane
    if abs(axis_vec(1)) < 0.9
        perp1 = cross(axis_vec, [1;0;0]);
    else
        perp1 = cross(axis_vec, [0;1;0]);
    end
    perp1 = perp1 / norm(perp1);
    perp2 = cross(axis_vec, perp1);
    
    % Calculate circle points
    center = origin + axis_vec * (scale * 0.5); % Move circle to mid-axis
    circle = center + r * (cos(t).*perp1 + sin(t).*perp2);
    
    % Plot arc
    plot3(circle(1,:), circle(2,:), circle(3,:), 'Color', col, 'LineStyle', '--');
    
    % Add arrowhead to the end of the arc
    tip = circle(:, end);
    tangent = -sin(t(end))*perp1 + cos(t(end))*perp2;
    quiver3(tip(1), tip(2), tip(3), tangent(1)*0.01, tangent(2)*0.01, tangent(3)*0.01, ...
        'Color', col, 'MaxHeadSize', 5, 'LineWidth', 1.5);
end