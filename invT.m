function T12 = invT(T21)
% calcola la matrice di rotazione inversa

% matrice di rotazione inversa
R12 = T21(1:3,1:3)';
O12 = -R12*T21(1:3,4);

% matrice di trasformazione
T12 = eye(4);
T12(1:3,1:3) = R12;
T12(1:3,4) = O12;