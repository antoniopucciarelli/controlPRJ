function [gain, control1, control2] = controlWeight() 
%   AEROSPACE CONTROL SYSTEM PROJECT -- AY 2021/2022
%   authors:
%       * Mate-Erik Moni 
%       * Antonio Pucciarelli 
%       * Atefeh Esmaelizadeh Rostam
%   
%   problem description: 
%   --- analysis, design and verification of a quadrirotor, ANT-R, single axis attitude control system 
%
%   this program: 
%   --- tuning sensitivity weight to be used in the overall plant tuning via systune  
%   --- compute: A, M, omega_b that allow to satisfy the 2nd order response of the system 
%
%   input:
%   --- damp: 2nd order function damping ratio 
%   --- omega_n: 2nd order function natural frequency 
%   --- args: boolean value for the print of results
%   
%   output:
%   --- A: low frequency value of the sensitivity 
%   --- M: M >= 1/damping ratio (damp)
%   --- omega_b: lowerbound bandwidth sensitivity
%