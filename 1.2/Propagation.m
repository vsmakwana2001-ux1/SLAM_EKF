function newpos = Propagation(pos_car, vel_cmd, w)
global dt
% Position
x = pos_car(1);
y = pos_car(2);
psi = pos_car(3);

% Velocity command
speed      = vel_cmd(1) + w(1);
psi_dot    = vel_cmd(2) + w(2);

% Propagation equation
newpos(1,1) = x+dt*(speed*cos(psi+psi_dot*dt)) ;
newpos(2,1) = y+dt*(speed*sin(psi+psi_dot*dt)) ;
newpos(3,1) = psi+ dt*psi_dot ;

end
