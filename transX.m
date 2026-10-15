function T = transX(alfa)
% rotazione elementare su X
% ATTENZIONE: l'angolo alfa deve essere in [°]

T = [1 0 0 0;
    0 cosd(alfa) -sind(alfa) 0;
    0 sind(alfa) cosd(alfa) 0;
    0 0 0 1];
