function R = RPY_to_Rot(RPY_angles)
g=RPY_angles(1);%roll
b=RPY_angles(2);%pitch
a=RPY_angles(3);%yaw

Rr=[1,0,0;0,cos(g),-sin(g);0,sin(g),cos(g)];
Rp=[cos(b),0,sin(b);0,1,0;-sin(b),0,cos(b)];
Ry=[cos(a),-sin(a),0;sin(a),cos(a),0;0,0,1];
R=Ry*Rp*Rr;%final rotation matrix
end