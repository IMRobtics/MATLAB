%% An Example of UDP broadcasting from MATLAB
% UDP allows broadcasting of messages to multiple UDP clients.
% This example uses MATLAB to broadcast, and receive, messages from 
% one or more devices that are on the same subnet. 
% To execute this example, you need one or more Arduinos with an 
% ethernet shield and connected to the same subnet as your computer.
% The Arduinos need to be programmed with a sketch to listen to UDP 
% broadcast messages, do something interesting (like blink an LED)
% and to respond to the broadcasted messages. 

% Copyright 2016 The MathWorks, Inc.

%% Example setup
% One or more Arduinos with the sketch, MATLABUDPBroadcastExample.ino.
% Each of the Arduinos Digital inputs pin 12 is connected to an LED through
% a 330 Ohm resistor to ground. 

%% Configure MATLAB UDP object that will be used to broadcast messages
% In this example one UDP object will broadcast messages to all devices 
% on the same subnet of the LAN on which your computer is. 
           
% Get localhost address of the computer
% [~,hostName] = system('225.0.0.37');
% % Convert hostname to fully-qualified domain name
% [~,hostAddress] = resolvehost([strtrim(hostName) '.dhcp.mathworks.com']);
% Compute broadcast address. This assumes a subnet mask of 255.255.255.0,
% which implies that address xxx.xxx.xxx.255 is the broadcast address
expression = '(?<=\d+[.]\d+[.]\d+[.])\d+';
broadcastAddress = regexprep('192.168.100.172', expression, '255')
% Create the UDP connection to broadcast messages on port 31416
sharedPort = 12345;
udpSender = udp(broadcastAddress, sharedPort,...
                'LocalPort', sharedPort);
% Enable port sharing to allow multiple clients on the same PC to bind to 
% the same port
% udpSender.EnablePortSharing = 'on';
udpSender.Terminator = 'CR';
udpSender.BytesAvailableFcnMode = 'terminator';
udpSender.BytesAvailableFcn = @(~,~)fprintf('Message "%s" at %s\n', fgetl(udpSender), datestr(now));
fopen(udpSender);

%% Setup a simple UI control to allow the user to send messages on button push
f = figure('Name','UDP Broadcast Example',...
           'NumberTitle','off',...
           'Position',[200 400 250 50],...
           'dockcontrols','off',...
           'Toolbar','none',...
           'menubar','none');

btn = uicontrol('Style', 'pushbutton', ...
                'String', 'Broadcast UDP message',...
                'Position',[25 15 200 25],...
                'Callback', @(~,~)fwrite(udpSender, ['MATLAB:BlinkIt' 13]));

% Wait until the user closes the figure. Note that callbacks will be
% executed in the background.
uiwait(f);    
fread(udpSender)
%% Clean up
fclose(udpSender);
delete(udpSender);
clear udpSender;
