% Unless you specify the 'Color' property when you plot,
% plots are plotted according to the 'ColorOrder' property of the axes.
% This demo shows how you can change the default color order of plots.

clc;    % Clear the command window.
close all;  % Close all figures (except those of imtool.)
clear;  % Erase all existing variables.
workspace;  % Make sure the workspace panel is showing.
fontSize = 18;

% Make 20 plots, with 25 data points in each plot.
numberOfDataSets = 20;
x = 1:25;
y = rand(numberOfDataSets, length(x));
% These y would all be on top of each other.
% Separate the plots vertically.
offsets = repmat((1:numberOfDataSets)', [1, length(x)]);
y = y + offsets; 

% Get the initial set of default plot colors.
initialColorOrder = get(gca,'ColorOrder') % Initial

% See what the colors look like when plotted:
subplot(2, 1, 1);
plot(x,y, 'LineWidth', 3);
grid on;
caption = sprintf('%d plots with the Initial Default Color Order (Note the repeating colors)', numberOfDataSets);
title(caption, 'FontSize', fontSize);
xlabel('X', 'FontSize', fontSize);
ylabel('Y', 'FontSize', fontSize);

% Enlarge figure to full screen.
set(gcf, 'units','normalized','outerposition',[0 0 1 1]); % Maximize figure.
% Give a name to the title bar.
set(gcf,'name','Image Analysis Demo','numbertitle','off')

choice = menu('Which ColorOrder do you want?', 'jet', 'random', 'hsv', 'hot', 'cool', 'spring', 'summer',...
	'autumn', 'winter', 'lines', 'gray', 'bone', 'copper', 'pink');

% Make a new axes:
subplot(2, 1, 2);

% Create a new colormap that will define the new default color order property.
switch choice
	case 1
		newDefaultColors = jet(numberOfDataSets);
	case 2
		newDefaultColors = rand(numberOfDataSets, 3);
	case 3
		newDefaultColors = hsv(numberOfDataSets);
	case 4
		newDefaultColors = hot(numberOfDataSets);
	case 5
		newDefaultColors = cool(numberOfDataSets);
	case 6
		newDefaultColors = spring(numberOfDataSets);
	case 7
		newDefaultColors = summer(numberOfDataSets);
	case 8
		newDefaultColors = autumn(numberOfDataSets);
	case 9
		newDefaultColors = winter(numberOfDataSets);
	case 10
		newDefaultColors = lines(numberOfDataSets);
	case 11
		newDefaultColors = gray(numberOfDataSets);
	case 12
		newDefaultColors = bone(numberOfDataSets);
	case 13
		newDefaultColors = copper(numberOfDataSets);
	otherwise
		newDefaultColors = pink(numberOfDataSets);
end
% Note: You can build your own custom order if you want, 
% just make up a array with numberOfDataSets rows and 3 columns
% where each element is in the range 0-1.

% Apply the new default colors to the current axes.
set(gca, 'ColorOrder', newDefaultColors, 'NextPlot', 'replacechildren');

% Now get the new set of default plot colors.
% Verify it changed by printing out the new default color set to the command window.
newColorOrder = get(gca,'ColorOrder')

% Now plot the datasets with the changed default colors.
plot(x,y, 'LineWidth', 3);
grid on;
caption = sprintf('%d plots with the New Default Color Order', numberOfDataSets);
title(caption, 'FontSize', fontSize);
xlabel('X', 'FontSize', fontSize);
ylabel('Y', 'FontSize', fontSize);

msgbox('Done with ColorOrder demo!');