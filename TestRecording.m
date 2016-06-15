clear all
close all

%A = Recording('Data/20160330_ArmSleeveProtocolTest1.h5', 60, 1480, 1, 'L');
%A = Recording('Data/20160406-Subject1.h5', 25, 40, 1, 'R');
A = Recording('Data/20160606-Subject2.h5', 25, 90, 1, 'R');
%A = Recording('Data/20160606-Subject2.h5', 69, 90, 1, 'R');

A = A.calcEverything();

A.drawEverything();
%A.drawTheta(0);
%A.drawSimplifiedTheta(0);
% A.drawJointHists();