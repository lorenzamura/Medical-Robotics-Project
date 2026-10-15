%% funzione per la cinematica diretta DaVinci
function m = kindirDaVinci(q,d)

% definizione delle trasformazioni relative

m.T10 = transDH(d.alphai(1),d.ai(1),0,d.di(1)+q(1));
m.T21 = transDH(d.alphai(2),d.ai(2),q(2),d.di(2));
m.T32 = transDH(d.alphai(3),d.ai(3),q(3),d.di(3));
m.T43 = transDH(d.alphai(4),d.ai(4),q(4),d.di(4));
m.T54 = transDH(d.alphai(5),d.ai(5),q(5),d.di(5));
m.T65 = transDH(d.alphai(6),d.ai(6),q(6),d.di(6));
m.T76 = transDH(d.alphai(7),d.ai(7),q(7),d.di(7));
m.T87 = transDH(d.alphai(8),d.ai(8),q(8),d.di(8));
m.T98 = transDH(d.alphai(9),d.ai(9),0,d.di(9)+q(9));

% definizione delle matrici di trasformazione relative

m.T1w = d.T0w*m.T10;
m.T2w = m.T1w*m.T21;
m.T3w = m.T2w*m.T32;
m.T4w = m.T3w*m.T43;
m.T5w = m.T4w*m.T54;
m.T6w = m.T5w*m.T65;
m.T7w = m.T6w*m.T76;
m.T8w = m.T7w*m.T87;
m.T9w = m.T8w*m.T98;

m.T40 = m.T10*m.T21*m.T32*m.T43;

a4 = d.alphai(5);
d5 = d.di(5);
a7 = d.ai(8);

m.Trcmw = d.T0w*m.T40*transX(a4+90)*transP(0,d5+a7,0);

% definizione della matrice Ttw

m.Ttw = m.T9w*d.Ttn;