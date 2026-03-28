function filename = getFilename(type, N, dt)
filename = sprintf('%s_%dms_N%d.mat', type, round(dt*1e3), N);
end
