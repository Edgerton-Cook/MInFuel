function [X,Y,Z] = plotcones(angle, x, y, z, scale)

n = length(x);
t = tand(angle);

X = cell(1,n);
Y = cell(1,n);
Z = cell(1,n);

for i = 1:n
    [cx,cy,cz] = cylinder([0,t]);
    X{i} = ( cx * scale ) + x(i);
    Y{i} = ( cy * scale ) + y(i);
    Z{i} = ( cz * scale ) + z(i);
    clearvars cx cy cz
end

end