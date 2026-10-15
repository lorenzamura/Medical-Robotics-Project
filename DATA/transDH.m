function T = transDH(alfa,a,theta,d)
% calcola la matrice di trasformazione con DH
% gli angoli devono essere espressi in gradi

T = transX(alfa)*transP(a,0,0)*transZ(theta)*transP(0,0,d);
