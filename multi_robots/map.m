
clear 
close all
clc

map = binaryOccupancyMap(1.8288, 7.9248);
% rng('default');
% map = mapClutter;

planner = plannerAStarGrid(map);

goal = [0.4 0.5];
start = [1.3 5.3];
path =plan(planner,start,goal,'world');

show(planner)
