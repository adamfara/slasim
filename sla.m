%% 
% Adam Farabaugh
% Penn Electric Racing
% SLA suspension rigger
%
% Input inboard points and outboard points at ride height
% Solve for point locations at some deflection
%
% Actuation is ignored, i.e. pushrod/bellcrank location not known
% These can be integrated later through motion ratio math
%
% Coordinate system is SAE J670 - Z down, X forward, Y outboard
% Assume we do this for the right side, can flip for left

function [ opts ] = sla( ipts, ride )
    % ipts: 
    % uaf - U A ARM, Front BJ
    % uar - U A ARM, Rear BJ
    % uao - U A ARM, Outer BJ
    % laf - L A ARM, Front BJ
    % lar - L A ARM, Rear BJ
    % lao - L A ARM, Outer BJ
    % whc - Wheel Center
    % wcp - Wheel Contact Patch Point
    % tri - Tie Rod, Inner BJ
    % tro - Tie Rod, Outer BJ