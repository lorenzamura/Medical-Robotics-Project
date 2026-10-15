function T = transP(dx,dy,dz)
% matrice di pure traslazione

T = eye(4);
T(1,4) = dx;
T(2,4) = dy;
T(3,4) = dz;
