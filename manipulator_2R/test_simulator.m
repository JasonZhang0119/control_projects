clear; clc; close all;

param.L1 = 1.0;
param.L2 = 0.8;

q = [pi/4; -pi/3];

options.title_text = "2R Robot Visualization Test";
options.show_frame = true;

Plot2RRobot(q, param, options);