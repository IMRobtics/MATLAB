% Clear previous variables
close all;
clear all;
clc;

% load Fuzzy Inference Systems (FIS) files
LamdaFuzzyController = readfis('LamdaFuzzyController');
%FuzzyCoperativeController = readfis('FuzzyCoperativeController_Mod(17-12-14)');
%FuzzyCoperativeController = readfis('FuzzyCoperativeController_Mod(6-9-2015)');
FuzzyCoperativeController = readfis('FuzzyCoperativeController_Mod(18-10-2015)');

% Experiment#: 1 = Different Paths, 2 = Same Paths 
Experiment = 2;
Tstop = 100;

%DC Motor Parameters

R = 3.07;
L = 0.04;
J = 0.0294;
B = 0.0141;
Ke = 0.65;
Kt = 0.65;

% Controller

Kp=50;

%UniCycle Mobile Robot

Len = 0.2;     %m
r = 0.045;      %m

% Robot initial Position & Orientation
if (Experiment == 1)
    R1_xi = 0.01;%0.0;
    R1_yi = 11.4303;%10.2;
    R1_phi = -pi/4;%pi/4;%0.0;
%     R1_xi = 0.01;%0.0;
%     R1_yi = 11.0747;%10.2;
%     R1_phi = -pi/4;%pi/4;%0.0;

    R2_xi = 0.01;%0.0;
    R2_yi = 6.7207;%4.4;
    R2_phi = -pi/4;%pi/4;%0.0;

    R3_xi = 0.01;%0.0;
    R3_yi = 2.0489;%0.46;
    R3_phi = -pi/4;%pi/4;%0.0;
elseif (Experiment == 2)
    R1_xi = 1.0;
    R1_yi = 11.0;
    R1_phi = -pi/2;

    R2_xi = -0.5;
    R2_yi = 8.0;
    R2_phi = 0.0;

    R3_xi = 1.5;
    R3_yi = -1.0;
    R3_phi = 0.0;%-pi/4;
elseif (Experiment == 3)
    R1_xi = 1.0;
    R1_yi = -1.5;
    R1_phi = 0.0;

    R2_xi = -1.0;
    R2_yi = 4.0;
    R2_phi = 0.0;

    R3_xi = 2.0;
    R3_yi = -1.2;
    R3_phi = 0.0;
elseif (Experiment == 4)
    R1_xi = 0.01;%2.0;
    R1_yi = 1.5837;%-1.2;
    R1_phi = 0.0;

    R2_xi = -1.0;
    R2_yi = 4.0;
    R2_phi = 0.0;

    R3_xi = 2.0;
    R3_yi = -1.2;
    R3_phi = 0.0;
elseif (Experiment == 5)
    R1_xi = 2.0;%0.0;
    R1_yi = 0.9;%0.397;
    R1_phi = 0.0;

    R2_xi = -1.0;
    R2_yi = 4.0;
    R2_phi = 0.0;

    R3_xi = 2.0;
    R3_yi = -1.2;
    R3_phi = 0.0;
end
