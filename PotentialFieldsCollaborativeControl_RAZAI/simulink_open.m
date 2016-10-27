clc
clear
model = 'Virtual.mdl';
load_system(model)
a = sim(model);
b = simout.signals.values;
c = simout1.signals.values;

plot(b,c);