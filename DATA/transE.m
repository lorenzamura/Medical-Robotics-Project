function T = transE(x,y,z,alfa,beta,gamma)
% calcola la matrice di trasformazione con traslazione + Eulero
% gli angoli devono essere espressi in gradi

T = transP(x,y,z)*transZ(alfa)*transY(beta)*transZ(gamma);
