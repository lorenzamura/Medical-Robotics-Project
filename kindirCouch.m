%% funzione per la cinematica diretta RoboCouch
function m = kindirCouch(q,d)

% trasformazioni relative [Ti,i-1]
m.T10 = transDH(d.alphai(1),d.ai(1),q(1),d.di(1));
m.T21 = transDH(d.alphai(2),d.ai(2),q(2),d.di(2));
m.T32 = transDH(d.alphai(3),d.ai(3),q(3),d.di(3));
m.T43 = transDH(d.alphai(4),d.ai(4),q(4),d.di(4));
m.T54 = transDH(d.alphai(5),d.ai(5),q(5),d.di(5));
m.T65 = transDH(d.alphai(6),d.ai(6),q(6),d.di(6));

% trasformazioni assolute
m.T1w = d.T0w*m.T10;
m.T2w = m.T1w*m.T21;
m.T3w = m.T2w*m.T32;
m.T4w = m.T3w*m.T43;
m.T5w = m.T4w*m.T54;
m.T6w = m.T5w*m.T65;

% trasformazione tool-world [Ttw]
m.Ttw = m.T6w*d.Ttn; % soluzione problema cinematico diretto di posizione
