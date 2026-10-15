function disframe(T,L,m)
% disegna una terna definita da T rispetto al sistema del disegno

% estremi dei segmenti da disegnare
Pw = T*[[0;0;0;1] [L;0;0;1] [0;L;0;1] [0;0;L;1]];

% marker assi (se non fornito in ingresso, mette a 'none')
if nargin<3
    m = 'none';
end

% linea asse X
line(Pw(1,[1 2]),Pw(2,[1 2]),Pw(3,[1 2]),'color','r','linewidth',1,'marker',m)
% linea asse Y
line(Pw(1,[1 3]),Pw(2,[1 3]),Pw(3,[1 3]),'color','g','linewidth',1,'marker',m)
% linea asse Z
line(Pw(1,[1 4]),Pw(2,[1 4]),Pw(3,[1 4]),'color','b','linewidth',1,'marker',m)