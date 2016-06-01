clear all
close all


%R = Recording('Data/20160330_ArmSleeveProtocolTest1.h5', 60, 1480, 1, 'L');
%R = Recording('Data/20160406-Subject1.h5', 25, 40, 1, 'R')
L = Recording('Data/20160406-Subject1.h5', 25, 150, 1, 'L')

L = L.calcEverything()

L.drawEverything()