clear all
close all


%R = Recording('Data/20160330_ArmSleeveProtocolTest1.h5', 60, 1480, 1, 'L');
R = Recording('Data/20160330_ArmSleeveProtocolTest1.h5', 60, 120, 1, 'L')

R = R.calcEverything()

R.drawEverything()