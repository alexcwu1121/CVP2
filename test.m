clear all;
close all;

for i=1:1
    double_driver("resources/pts_3D.txt",...
        "resources/pts_2D_left.txt",...
        "resources/pts_2D_right.txt", i)
end