% Range Estimation Exercise

% TODO : Find the Bsweep of chirp for 1 m resolution

deltaR = 1;
c = 3*10^8;
bSweep = c/2*deltaR;

% TODO : Calculate the chirp time based on the Radar's Max Range
rangeMax = 300;
Ts = 5.5*2*rangeMax/c;

% TODO : define the frequency shifts 
frequencyShift = [0, 1.1e6, 13e6, 24e6];
calculated_range = c * Ts * frequencyShift / (2 * bSweep);

% Display the calculated range
disp(calculated_range);
