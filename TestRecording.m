clear all
close all


%R = Recording('Data/20160330_ArmSleeveProtocolTest1.h5', 60, 1480, 1, 'L');
R = Recording('Data/20160406-Subject1.h5', 60, 150, 1, 'R')

R = R.calcEverything()

R.drawEverything()