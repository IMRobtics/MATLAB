% socket = java.net.MultiSocket(12345);
% socket.joinGroup(java.net.InetAddress.getByName('225.0.0.37'));
% socket.setReuseAddress(1);
% 
% packet = java.net.DatagramPacket(zeros(1, intmax('uint16'), 'int8'), intmax('uint16'));
% 
% socket.receive(packet);
% 
% socket.leaveGroup(InetAddress.getByName(streamIP));
% socket.close;
% 
% msg = packet.getData;
% msg = msg(1:packet.getLength);


% judp('send',1,'192.168.100.194',int8('Howdy!'))
% mssg=judp('receive',12345,100);char(mssg')

sharedPort = 8888;
u = udp('192.168.100.172', sharedPort,'LocalPort', sharedPort);
fopen(u)
t=1;
u
while t<10
    fwrite(u,t);
    a = fread(u,10)
    t = t+1;
end
fclose(u)