%% funzione per la cinematica inversa del polso del DaVinci
function qfin = invkinDaVinci(D,m,q)

% calcolo della matrice T40

m.T40 = m.T10*m.T21*m.T32*m.T43;

% estrazione paarametri DH

a4 = D.alphai(5);
d5 = D.di(5);
a6 = D.ai(7);
a7 = D.ai(8);
a8 = D.ai(9);
L9 = D.di(9);
Ltool = D.Ltool;

% definizione della terna RCM rispetto a world

m.Trcmw = D.T0w*m.T40*transX(a4+90)*transP(0,d5+a7,0);

% determinazione delle coordinate del punto target

P = D.Pt;

% calcolo delle coordinate di P nel sistema RCM

Prcm = invT(m.Trcmw)*P;

% calcolo dei valori di q5, q6, quindi q7, q8 e infine di q9

q5 = atan2d(-Prcm(3),Prcm(1))+180;
q6 = atan2d(Prcm(2),Prcm(3)/sind(q5))+asind(a8/a6);
q7 = (3*180)/2-q6;
q8 = q6 - asind(a8/a6);
q9 = - L9 + sqrt(a6^2-a8^2) + sqrt(Prcm(1)^2+Prcm(2)^2+Prcm(3)^2) - Ltool;
qfin = [q(1) q(2) q(3) q(4) q5 q6 q7 q8 q9];