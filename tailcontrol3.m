clear all; 
fclose(instrfind);
servoa=arduino('COM9');
gyroa=serial('COM3','BaudRate',9600);
fopen(gyroa);
% stopasync(gyroa);
readasync(gyroa);
s=servo(servoa,'D8');
phi=0;

%wait for value to stabilize
ts=0;
tic;
while toc<20;
    fwrite(gyroa,'1');
    yaw=fscanf(gyroa,'%i');
    display(yaw);
    ts=toc;
    display(ts);
end
display('stabilized');

fwrite(gyroa,'1');
initial_yaw=fscanf(gyroa,'%i');
prev_yaw=initial_yaw;
x=1;
t=0.02;
params=sys_params();
des_state = desiredstate();
while x<500
    tic;
    fwrite(gyroa,'1');
    yaw=fscanf(gyroa,'%i');
    syaw=size(yaw);
    t1=toc;
    angle=yaw-initial_yaw;
    angveloc=(yaw-prev_yaw)/t;
    display(yaw);
    display(angle);
    display(angveloc);
    prev_yaw=yaw;
    theta(1)=deg2rad(angle);
    theta(2)=deg2rad(angveloc);
    %inertia and lever arm changes when phi (servo angle) changes value
    %obtained by rough calculation 
    inertia=0.00015-0.00005*cos(phi);
    lc=0.06*sin(theta(1))-0.02*sin(theta(1)+phi);
    moment=params.mass*params.grav*lc*theta(1)-inertia*(-params.Kd*(theta(2)-des_state.omega)-params.Kp*(theta(1)-des_state.theta));
    t2=toc;
    syms y; % solve equation
    sol=asin(moment-0.001*sin(theta(1))/0.0035/(-params.grav))-theta(1);
    radphi=wrapToPi(double(sol));
    phi=floor(rad2deg(radphi));
    display(phi);
    t3=toc;
    servowrite=(phi+90)/180;
    writePosition(s,servowrite);
    x=x+1;
    t=toc;
end