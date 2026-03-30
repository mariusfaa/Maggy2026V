function files = getFiles(types,N,dt)

NN = numel(types) * numel(N) * numel(dt);

files = cell(NN, 1);
n = 1;
for i = 1:numel(types)
    for j = 1:numel(N)
        for k = 1:numel(dt)
            folder = '.';
            files{n} = sprintf("%s/%s",folder, getFilename(types{i}, N(j), dt(k)));
            n = n + 1;
        end
    end
end
files = string(files);

end