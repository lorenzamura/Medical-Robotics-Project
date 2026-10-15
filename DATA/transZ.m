function T = transZ(gamma)
% rotazione elementare su Z
% ATTENZIONE: l'angolo alfa deve essere in [°]

T = [cosd(gamma) -sind(gamma) 0 0;
    sind(gamma) cosd(gamma) 0 0;
    0 0 1 0;
    0 0 0 1];
