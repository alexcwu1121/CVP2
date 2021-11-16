function [pd] = ParseMat(filename, l_off, r_off)
    % ParseMat.m
    %   Read space delimited 3D and 2D points from text file
    %
    % Arguments:
    %   filename: name of point file
    %
    % Returns:
    %   pd: matrix of points

    pd = [];
    file = fopen(filename);
    tline = fgetl(file);
    while ischar(tline)
        pd = [pd,split(tline)];
        tline = fgetl(file);
    end
    fclose(file);
    pd = str2double(pd);
    pd = pd(1+l_off:end-r_off,:);
end