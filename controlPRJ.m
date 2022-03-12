%   AEROSPACE CONTROL SYSTEM PROJECT -- AY 2021/2022
%   authors:
%       * Mate-Erik Moni 
%       * Antonio Pucciarelli 
%       * Atefeh Esmaelizadeh Rostam
%   
%   problem description: 
%   --- analysis, design and verification of a quadrirotor, ANT-R, single axis attitude control system 
%
%   this program calls:
%   --- lateral dynamics system function study  
%   --- system P + PID plant tuning 
%   --- robust stability analysis 
%   --- total system verification 
%    
%   !!! import first all the directories !!!

clear 
close all 
clc

% importing \src folder 
% --- in \src are stored all the program and the functions 
thisPath = pwd;
addpath(append(thisPath, '\src\'));

% importing \data folder 
% --- in \data are stored the lateral dynamics properties of the ANT-R quadrirotor
addpath(append(thisPath, '\data\'));

%% G(s) study -> system lateral dynamics study 
run Gstudy; 

%% system tuning 
run feedbackDesign; 

%% removing path 
% removing \src path 
rmpath(append(thisPath, '\src\'));

% removing \data path 
rmpath(append(thisPath, '\data\'));
