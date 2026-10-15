function T = transY(beta)
% rotazione elementare su Y
% ATTENZIONE: l'angolo alfa deve essere in [°]

T = [cosd(beta) 0 sind(beta) 0;
    0 1 0 0;
    -sind(beta) 0 cosd(beta) 0;
    0 0 0 1];
