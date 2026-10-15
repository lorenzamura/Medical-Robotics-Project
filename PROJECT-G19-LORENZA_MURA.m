%% PROGETTO ROBOTICA MEDICA

% BORDIN MARTA
% MANFRINI MATILDE MARIE
% MURA LORENZA

clear all
close all
clc

%% CARICAMENTO DATI
% Importazione dati DaVinci
load("DaVinci.mat")
datiDaVinci=dati;
base1=base;
link1=link;

% Importazione dati RoboCouch
load("Couch.mat")
datiCouch=dati;
base2=base;
dati2=dati;
link2=link;

% Posizione base robot rispetto a world
datiDaVinci.T0w = transE(0,0,0,0,0,0);
datiCouch.T0w = transE(350,2200,0,0,0,0);

% Posizione del tool rispetto alla flangia robot
datiDaVinci.Ttn = transP(0,0,datiDaVinci.Ltool);
datiCouch.Ttn = transP(0,0,0);

% Punti robot DaVinci
q = [0 -90 180 180 -90 45 0 0 0];   
q(7) = 270-q(6);
q(8) = q(6)-asind(datiDaVinci.ai(9)/datiDaVinci.ai(7));

% Punti robot Couch
Q = [0 0 90 -90 90 90];

% Calcolo matrici di trasformazione
mat1 = kindirDaVinci(q,datiDaVinci)
mat2 = kindirCouch(Q,datiCouch)

% Punto target
datiCouch.P1 = [100;500;70;1];  % Assi x,y,z da regolare
% Opzionalmente calcolo della posizione iniziale nel world (per controllo)
datiCouch.Pt = mat2.Ttw * datiCouch.P1;
datiDaVinci.Pt = datiCouch.Pt;  % Inizializza il target per DaVinci


%% CREAZIONE DELLA FIGURA
hf = figure(1);
clf
hf.MenuBar = 'none';
hf.Name = sprintf('Robot DAVINCI e ROBOCOUCH');
hf.NumberTitle = 'off';
hf.Color = 'w';

% Assi
axis equal
view(30,20)
grid on
rotate3d on
xlabel 'x[mm]'
ylabel 'y[mm]'
zlabel 'z[mm]'
title 'Robot DaVinci e RoboCouch'
xlim(2000*[-0.7 2.5])
ylim(2000*[-1 2.0])
zlim(1500*[0 2])
hold on

%% DISEGNO DAVINCI

% Terne di riferimento
L = 200;     %[mm]
disframe(eye(4),2*L)            % world
disframe(datiDaVinci.T0w,L,'.') % base robot (0)
disframe(mat1.T1w,L)            % link 1
disframe(mat1.T2w,L)            % link 2
disframe(mat1.T3w,L)            % link 3
disframe(mat1.T4w,L)            % link 4
disframe(mat1.T5w,L)            % link 5
disframe(mat1.T6w,L)            % link 6
disframe(mat1.T7w,L)            % link 7
disframe(mat1.T8w,L)            % link 8
disframe(mat1.T9w,L)            % link 9
disframe(mat1.Ttw,L,'.')        % tool

% Disegno base robot
P00 = base1.P;
P0w = datiDaVinci.T0w*P00;
patch('faces',base1.faces,'vertices',P0w(1:3,:)',...
     'facecolor',0.95*[0 0.5 1],'facealpha',0.6,...
     'edgecolor','none')

% Disegno link 1
P11 = link1(1).P;
P1w = mat1.T1w*P11;
patch('faces',link1(1).faces,'vertices',P1w(1:3,:)',...
     'facecolor',0.95*[1 1 1],'facealpha',0.6,...
     'edgecolor','none')

% Disegno link 2
P22 = link1(2).P;
P2w = mat1.T2w*P22;
patch('faces',link1(2).faces,'vertices',P2w(1:3,:)',...
     'facecolor',0.95*[1 1 1],'facealpha',0.6,...
     'edgecolor','none')

% Disegno link 3
P33 = link1(3).P;
P3w = mat1.T3w*P33;
patch('faces',link1(3).faces,'vertices',P3w(1:3,:)',...
     'facecolor',0.95*[1 1 1],'facealpha',0.6,...
     'edgecolor','none')

% Disegno link 4
P44 = link1(4).P;
P4w = mat1.T4w*P44;
patch('faces',link1(4).faces,'vertices',P4w(1:3,:)',...
     'facecolor',0.95*[1 1 1],'facealpha',0.6,...
     'edgecolor','none')

% Disegno link 5
P55 = link1(5).P;
P5w = mat1.T5w*P55;
patch('faces',link1(5).faces,'vertices',P5w(1:3,:)',...
     'facecolor',0.95*[1 1 1],'facealpha',0.6,...
     'edgecolor','none')

% Disegno link 6
P66 = link1(6).P;
P6w = mat1.T6w*P66;
patch('faces',link1(6).faces,'vertices',P6w(1:3,:)',...
     'facecolor',0.95*[1 1 1],'facealpha',0.6,...
     'edgecolor','none')

% Disegno link 7
P77 = link1(7).P;
P7w = mat1.T7w*P77;
patch('faces',link1(7).faces,'vertices',P7w(1:3,:)',...
     'facecolor',0.95*[1 1 1],'facealpha',0.6,...
     'edgecolor','none')

% Disegno link 8
P88 = link1(8).P;
P8w = mat1.T8w*P88;
patch('faces',link1(8).faces,'vertices',P8w(1:3,:)',...
     'facecolor',0.95*[1 1 1],'facealpha',0.6,...
     'edgecolor','none')

% Disegno link 9
P99 = link1(9).P;
P9w = mat1.T9w*P99;
patch('faces',link1(9).faces,'vertices',P9w(1:3,:)',...
     'facecolor',0.95*[1 1 1],'facealpha',0.6,...
     'edgecolor','none')

hold on


%% DISEGNO ROBOCOUCH

% Terne di riferimento
disframe(eye(4),2*L)            % world
disframe(datiCouch.T0w,L,'.')   % base robot (0)
disframe(mat2.T1w,L)            % link 1
disframe(mat2.T2w,L)            % link 2
disframe(mat2.T3w,L)            % link 3
disframe(mat2.T4w,L)            % link 4
disframe(mat2.T5w,L)            % link 5
disframe(mat2.T6w,L)            % link 6
disframe(mat2.Ttw,L,'.')        % tool

% Disegno base robot
P00 = base2.P;
P0w = datiCouch.T0w*P00;
patch('faces',base2.faces,'vertices',P0w(1:3,:)',...
     'facecolor',0.95*[0 0.5 1],'facealpha',0.6,...
     'edgecolor','none')

% Disegno link 1
P11 = link2(1).P;
P1w = mat2.T1w*P11;
patch('faces',link2(1).faces,'vertices',P1w(1:3,:)',...
     'facecolor',0.95*[1 1 1],'facealpha',0.6,...
     'edgecolor','none')

% Disegno link 2
P22 = link2(2).P;
P2w = mat2.T2w*P22;
patch('faces',link2(2).faces,'vertices',P2w(1:3,:)',...
     'facecolor',0.95*[1 1 1],'facealpha',0.6,...
     'edgecolor','none')

% Disegno link 3
P33 = link2(3).P;
P3w = mat2.T3w*P33;
patch('faces',link2(3).faces,'vertices',P3w(1:3,:)',...
     'facecolor',0.95*[1 1 1],'facealpha',0.6,...
     'edgecolor','none')

% Disegno link 4
P44 = link2(4).P;
P4w = mat2.T4w*P44;
patch('faces',link2(4).faces,'vertices',P4w(1:3,:)',...
     'facecolor',0.95*[1 1 1],'facealpha',0.6,...
     'edgecolor','none')

% Disegno link 5
P55 = link2(5).P;
P5w = mat2.T5w*P55;
patch('faces',link2(5).faces,'vertices',P5w(1:3,:)',...
     'facecolor',0.95*[1 1 1],'facealpha',0.6,...
     'edgecolor','none')

% Disegno lettino
P66 = tool.P;
P6w = mat2.T6w*P66;
patch('faces',tool.faces,'vertices',P6w(1:3,:)',...
     'facecolor',0.5*[1 1 1],'facealpha',0.6,...
     'edgecolor','none')

% Disegno target
plot3(datiCouch.Pt(1), datiCouch.Pt(2), datiCouch.Pt(3), 'm*', 'MarkerSize', 6, 'LineWidth', 1);
text(datiCouch.Pt(1), datiCouch.Pt(2), datiCouch.Pt(3), '   T', 'Color', 'm', 'FontSize', 10);

light
drawnow


%% SIMULAZIONE 1: RoboCouch si posiziona in modo generico (moto P-P ai giunti)

% definizione del punto iniziale e del punto finale per lo spostamento del RoboCouch
Q0 = Q;                      % [°]/[mm]  
Qf = Q0 + [0 0 -90 0 0 0];   % [°]/[mm]

% Calcolo dello spostamento complessivo
dQ=Qf-Q0;

[Q0; Qf; dQ]

% Pianificazione del moto 

% Vettore dei tempi normalizzati tau
T=2;                   % [s]
dt = 0.05;             % [s]
t = [0:dt:T];          % [s]

N = length(t);
tau=t/T;               

% Inizializzazione video
video = VideoWriter('video DAVINCI e ROBOCOUCH','MPEG-4');
video.FrameRate = (N-1)/T;
video.Quality = 50;
video.open;

% Legge adimensionale normalizzata sigma: polinoio cubico
sigma = 3*tau.^2 - 2*tau.^3;     
sigmap = 6*tau - 6*tau.^2;        
sigmapp = 6 - 12*tau;          

[tau' sigma' sigmap' sigmapp' t']

% Coefficienti adimensionali
cv = max(sigmap);
cap = max(sigmapp);      % positivo
cam = abs(min(sigmapp)); % negativo

% Traiettoria dei giunti Qi(t)
Q1 = Q0(1) + dQ(1)*sigma;
Q2 = Q0(2) + dQ(2)*sigma;
Q3 = Q0(3) + dQ(3)*sigma;
Q4 = Q0(4) + dQ(4)*sigma;
Q5 = Q0(5) + dQ(5)*sigma;
Q6 = Q0(6) + dQ(6)*sigma;

Q = [Q1' Q2' Q3' Q4' Q5' Q6'];

% Velocità e accelerazione dei giunti
Qp = sigmap'*dQ/T;
Qpp = sigmapp'*dQ/(T^2);


% Inizializzazione matrice punti traiettoria
PtrjC = zeros(4,N);
Qsim = zeros(N, length(Q0));

% Simulazione
for k = 1:N
  s = sigma(k);

  % Interpolazione giunti (P-P in spazio giunti)
  Qk = Q0 + (Qf - Q0) * s;  

  % Definizione del punto robot al k-esimo istante
  Qsim(k,:) = Qk;

  % Calcolo matrice di trasformazione
  matkC = kindirCouch(Qk, datiCouch);

  % Origine terna tool
  PtrjC(:,k) = matkC.Ttw(:,4);

  % Target fissato al lettino: trasformo P1 nella world attuale
  Ptw = matkC.Ttw * datiCouch.P1;    % target in world (segue il tool del Couch)
  datiDaVinci.Pt = Ptw;              % aggiorno il target che il DaVinci deve seguire

  % Disegno
  hf = figure(2);
  % set(gcf, 'Units', 'pixels', 'Position', [100 100 3840 2160]);   % aumento risoluzione
  clf
  hf.MenuBar = 'none';
  hf.Name = sprintf('Robot DAVINCI e ROBOCOUCH');
  hf.NumberTitle = 'off';
  hf.Color = 'w';
  cla
  hold on

  % Assi
  axis equal
  view(30,20)
  grid on
  rotate3d on
  xlabel 'x[mm]'
  ylabel 'y[mm]'
  zlabel 'z[mm]'
  title 'Simulazione DaVinci e RoboCouch'
  xlim(2000*[-0.7 2.5])
  ylim(2000*[-1 2.0])
  zlim(1500*[0 2])
  hold on

  % Disegno DaVinci
  disframe(eye(4),2*L)        % world
  disframe(datiDaVinci.T0w,L) % base robot (0)
  disframe(mat1.T1w,L)        % link 1...
  disframe(mat1.T2w,L)
  disframe(mat1.T3w,L)
  disframe(mat1.T4w,L)
  disframe(mat1.T5w,L)
  disframe(mat1.T6w,L)
  disframe(mat1.T7w,L)
  disframe(mat1.T8w,L)
  disframe(mat1.T9w,L)
  disframe(mat1.Ttw,L,'.')    % tool

  % Base DaVinci 
  P00 = base1.P;
  P0w = datiDaVinci.T0w*P00;
  patch('faces',base1.faces,'vertices',P0w(1:3,:)',...
         'facecolor',0.95*[0 0.5 1],'facealpha',0.6,...
         'edgecolor','none')

  % Link DaVinci
  for ii = 1:9
      Pii = link1(ii).P;
      Piw = mat1.(['T' num2str(ii) 'w']) * Pii;
      patch('faces',link1(ii).faces,'vertices',Piw(1:3,:)',...
         'facecolor',0.95*[1 1 1],'facealpha',0.6,...
         'edgecolor','none')
  end

  % Disegno RoboCouch
  disframe(datiCouch.T0w,L) % base robot (0)
  disframe(matkC.T1w,L)     % link 1...
  disframe(matkC.T2w,L)
  disframe(matkC.T3w,L)
  disframe(matkC.T4w,L)
  disframe(matkC.T5w,L)
  disframe(matkC.T6w,L)
  disframe(matkC.Ttw,L,'.') % tool 

  % Base Couch
  Pb0 = base2.P;
  Pbw = datiCouch.T0w*Pb0;
  patch('faces',base2.faces,'vertices',Pbw(1:3,:)',...
         'facecolor',0.95*[0 0.5 1],'facealpha',0.6,...
         'edgecolor','none')

  % Link Couch
  for ii = 1:length(link2)
      Pii = link2(ii).P;
      Piw = matkC.(['T' num2str(ii) 'w']) * Pii;
      patch('faces',link2(ii).faces,'vertices',Piw(1:3,:)',...
         'facecolor',0.95*[1 1 1],'facealpha',0.6,...
         'edgecolor','none')
  end

  % Disegno lettino
  Ptool = tool.P;
  Ptoolw = matkC.T6w * Ptool;
  patch('faces',tool.faces,'vertices',Ptoolw(1:3,:)',...
         'facecolor',0.5*[1 1 1],'facealpha',0.6,...
         'edgecolor','none')

  % Disegno punto target
  plot3(Ptw(1), Ptw(2), Ptw(3), 'm*', 'MarkerSize', 6, 'LineWidth', 1);
  text(Ptw(1), Ptw(2), Ptw(3), '   T', 'Color', 'm', 'FontSize', 10);

  light
  drawnow
  pause(0.1)   % regola la velocità della simulazione

  % grab immagine
  IM = getframe(hf);
  writeVideo(video,IM.cdata);

end

matCouchFinal = matkC;


%% SIMULAZIONE 2: struttura portante del DaVinci si posiziona (moto P-P ai giunti)

% definizione del punto iniziale e del punto finale per lo spostamento della struttura portante
q0 = q;  % [°]/[mm]
qf = q0 + [650 90 -180 90 0 0 0 0 0]; % [°]/[mm]
qf(7) = 270-qf(6);
qf(8) = qf(6)-asind(datiDaVinci.ai(9)/datiDaVinci.ai(7));

% Calcolo dello spostamento complessivo
dq = qf-q0;

[q0; qf; dq]

% Pianificazione del moto 

% Vettore dei tempi normalizzati tau
T=2;                   % [s]
dt = 0.05;             % [s]
t = [0:dt:T];          % [s]
N = length(t);
tau=t/T; 

% Legge adimensionale normalizzata
sigma = 3*tau.^2 - 2*tau.^3;     
sigmap = 6*tau - 6*tau.^2;        
sigmapp = 6 - 12*tau;          

[tau' sigma' sigmap' sigmapp' t']

% Coefficienti adimensionali
cv = max(sigmap);
cap = max(sigmapp);      % positivo
cam = abs(min(sigmapp)); % negativo

% Traiettoria dei giunti qi(t)
q1 = q0(1) + dq(1)*sigma;
q2 = q0(2) + dq(2)*sigma;
q3 = q0(3) + dq(3)*sigma;
q4 = q0(4) + dq(4)*sigma;
q5 = q0(5) + dq(5)*sigma;
q6 = q0(6) + dq(6)*sigma;
q7 = q0(7) + dq(7)*sigma;
q8 = q0(8) + dq(8)*sigma;
q9 = q0(9) + dq(9)*sigma;

q = [q1' q2' q3' q4' q5' q6' q7' q8' q9'];

% Velocità e accelerazione dei giunti
qp = sigmap'*dq/T;
qpp = sigmapp'*dq/(T^2);

% Inizializzazione matrice punti traiettoria
PtrjD = zeros(4,N);
qsim = zeros(N, length(q0));

% Simulazione
for k = 1:N
  s = sigma(k);

  % Interpolazione giunti (P-P in spazio giunti)
  qk = q0 + (dq) * s;   

  % Definizione del punto robot al k-esimo istante
  qsim(k,:) = qk;

  % Analisi cinematica diretta di posizione
  matkD = kindirDaVinci(qk, datiDaVinci);

  % Origine terna tool
  PtrjD(:,k) = matkD.Ttw(:,4);

  % Disegno
  cla
  hold on

  % Disegno DaVinci
  disframe(eye(4),2*L)        % world
  disframe(datiDaVinci.T0w,L) % base robot (0)
  disframe(matkD.T1w,L)       % link 1...
  disframe(matkD.T2w,L)
  disframe(matkD.T3w,L)
  disframe(matkD.T4w,L)
  disframe(matkD.T5w,L)
  disframe(matkD.T6w,L)
  disframe(matkD.T7w,L)
  disframe(matkD.T8w,L)
  disframe(matkD.T9w,L)
  disframe(matkD.Ttw,L,'.')   % tool

  % Base DaVinci
  P00 = base1.P;
  P0w = datiDaVinci.T0w*P00;
  patch('faces',base1.faces,'vertices',P0w(1:3,:)',...
         'facecolor',0.95*[0 0.5 1],'facealpha',0.6,...
         'edgecolor','none')

  % Link DaVinci
  for ii = 1:9
      Pii = link1(ii).P;
      Piw = matkD.(['T' num2str(ii) 'w']) * Pii;
      patch('faces',link1(ii).faces,'vertices',Piw(1:3,:)',...
         'facecolor',0.95*[1 1 1],'facealpha',0.6,...
         'edgecolor','none')
  end

  % Disegno RoboCouch
  disframe(datiCouch.T0w,L)     % base robot (0)
  disframe(matCouchFinal.T1w,L) % link 1...
  disframe(matCouchFinal.T2w,L)
  disframe(matCouchFinal.T3w,L)
  disframe(matCouchFinal.T4w,L)
  disframe(matCouchFinal.T5w,L)
  disframe(matCouchFinal.T6w,L)
  disframe(matCouchFinal.Ttw,L,'.') % tool 

  % Base Couch
  Pb0 = base2.P;
  Pbw = datiCouch.T0w*Pb0;
  patch('faces',base2.faces,'vertices',Pbw(1:3,:)',...
         'facecolor',0.95*[0 0.5 1],'facealpha',0.6,...
         'edgecolor','none')

  % Link Couch
  for ii = 1:length(link2)
      Pii = link2(ii).P;
      Piw = matCouchFinal.(['T' num2str(ii) 'w']) * Pii;
      patch('faces',link2(ii).faces,'vertices',Piw(1:3,:)',...
         'facecolor',0.95*[1 1 1],'facealpha',0.6,...
         'edgecolor','none')
  end

  % Disegno lettino
  Ptool = tool.P;
  Ptoolw = matCouchFinal.T6w * Ptool;
  patch('faces',tool.faces,'vertices',Ptoolw(1:3,:)',...
         'facecolor',0.5*[1 1 1],'facealpha',0.6,...
         'edgecolor','none')

  % Disegno punto target
  plot3(datiDaVinci.Pt(1), datiDaVinci.Pt(2), datiDaVinci.Pt(3), 'm*', 'MarkerSize', 6, 'LineWidth', 1);
  text(datiDaVinci.Pt(1), datiDaVinci.Pt(2), datiDaVinci.Pt(3), '   T', 'Color', 'm', 'FontSize', 10);

  light
  drawnow
  pause(0.1)   % regola la velocità della simulazione

  % grab immagine
  IM = getframe(hf);
  writeVideo(video,IM.cdata);

end


%% SIMULAZIONE 3a: polso del DaVinci si posiziona con l'utensile fuori dal paziente (moto P-P ai giunti)

% definizione del punto iniziale e del punto finale per lo spostamento del polso
q0_1 = qf;  % [°]/[mm]

% analisi cinematica inversa di posizione
qf_1 = invkinDaVinci(datiDaVinci, matkD, qf);  % [°]/[mm]
qf_1(9) = qf(9);

% Calcolo dello spostamento complessivo
dq_1 = qf_1 - q0_1;

% Traiettoria dei giunti qi_1(t)
q1_1 = q0_1(1) + dq_1(1)*sigma;
q2_1 = q0_1(2) + dq_1(2)*sigma;
q3_1 = q0_1(3) + dq_1(3)*sigma;
q4_1 = q0_1(4) + dq_1(4)*sigma;
q5_1 = q0_1(5) + dq_1(5)*sigma;
q6_1 = q0_1(6) + dq_1(6)*sigma;
q7_1 = q0_1(7) + dq_1(7)*sigma;
q8_1 = q0_1(8) + dq_1(8)*sigma;
q9_1 = q0_1(9) + dq_1(9)*sigma;

q_1 = [q1_1' q2_1' q3_1' q4_1' q5_1' q6_1' q7_1' q8_1' q9_1'];

% Velocità e accelerazione dei giunti
qp_1 = sigmap'*dq_1/T;
qpp_1 = sigmapp'*dq_1/(T^2);

% Inizializzazione matrice punti traiettoria
PtrjD_1 = zeros(4,N);
qsim_1 = zeros(N, length(q0_1));

% Simulazione
for k = 1:N

   s = sigma(k);

   % Interpolazione giunti (P-P in spazio giunti)
   qk_1 = q0_1 + (dq_1) * s;  

   % Definizione del punto robot al k-esimo istante
   qsim_1(k,:) = qk_1;

   % Analisi cinematica diretta di posizione
   matkD_1 = kindirDaVinci(qk_1, datiDaVinci);

   % Origine terna tool
   PtrjD_1(:,k) = matkD_1.Ttw(:,4);

   % Disegno
   cla
   hold on

   % Disegno DaVinci
   L = 200;
   disframe(eye(4),2*L)        % world
   disframe(datiDaVinci.T0w,L) % base robot (0)
   disframe(matkD_1.T1w,L)     % link 1...
   disframe(matkD_1.T2w,L)
   disframe(matkD_1.T3w,L)
   disframe(matkD_1.T4w,L)
   disframe(matkD_1.T5w,L)
   disframe(matkD_1.T6w,L)
   disframe(matkD_1.T7w,L)
   disframe(matkD_1.T8w,L)
   disframe(matkD_1.T9w,L)
   disframe(matkD_1.Ttw,L,'.') % tool

   % Base DaVinci
   P00 = base1.P;
   P0w = datiDaVinci.T0w*P00;
   patch('faces',base1.faces,'vertices',P0w(1:3,:)',...
         'facecolor',0.95*[0 0.5 1],'facealpha',0.6,...
         'edgecolor','none')

   % Link DaVinci
   for ii = 1:9
       Pii = link1(ii).P;
       Piw = matkD_1.(['T' num2str(ii) 'w']) * Pii;
       patch('faces',link1(ii).faces,'vertices',Piw(1:3,:)',...
         'facecolor',0.95*[1 1 1],'facealpha',0.6,...
         'edgecolor','none')
   end

   % Disegno RoboCouch
   disframe(datiCouch.T0w,L)     % base robot (0)
   disframe(matCouchFinal.T1w,L) % link 1...
   disframe(matCouchFinal.T2w,L)
   disframe(matCouchFinal.T3w,L)
   disframe(matCouchFinal.T4w,L)
   disframe(matCouchFinal.T5w,L)
   disframe(matCouchFinal.T6w,L)
   disframe(matCouchFinal.Ttw,L,'.') % tool 

   % Base Couch
   Pb0 = base2.P;
   Pbw = datiCouch.T0w*Pb0;
   patch('faces',base2.faces,'vertices',Pbw(1:3,:)',...
       'facecolor',0.95*[0 0.5 1],'facealpha',0.6,...
       'edgecolor','none')

   % Link Couch
   for ii = 1:length(link2)
       Pii = link2(ii).P;
       Piw = matCouchFinal.(['T' num2str(ii) 'w']) * Pii;
       patch('faces',link2(ii).faces,'vertices',Piw(1:3,:)',...
           'facecolor',0.95*[1 1 1],'facealpha',0.6,...
           'edgecolor','none')
   end

   % Disegno lettino
   Ptool = tool.P;
   Ptoolw = matCouchFinal.T6w * Ptool;
   patch('faces',tool.faces,'vertices',Ptoolw(1:3,:)',...
       'facecolor',0.5*[1 1 1],'facealpha',0.6,...
       'edgecolor','none')

   % Disegno punto target
   plot3(datiDaVinci.Pt(1), datiDaVinci.Pt(2), datiDaVinci.Pt(3), 'm*', 'MarkerSize', 6, 'LineWidth', 1);
   text(datiDaVinci.Pt(1), datiDaVinci.Pt(2), datiDaVinci.Pt(3), '   T', 'Color', 'm', 'FontSize', 10);

   light
   drawnow
   pause(0.1)   % regola la velocità della simulazione

   % grab immagine
   IM = getframe(hf);
   writeVideo(video,IM.cdata);

end


%% SIMULAZIONE 3b: utensile del DaVinci inserito fino al punto target (moto P-P ai giunti)

% Definizione dei punti iniziale e finali per lo spostamento del giunto 9
q0_2 = qf_1;  % [°]/[mm]

% Analisi cinematica inversa di posizione
qf_2 = invkinDaVinci(datiDaVinci, matkD_1, q0_2);  % [°]/[mm]

% Giunti 5,6,7 e 8 vengono mantenuti costanti
qf_2(5) = q0_2(5);
qf_2(6) = q0_2(6);
qf_2(7) = q0_2(7);
qf_2(8) = q0_2(8);

% Calcolo dello spostamento complessivo
dq_2 = qf_2 - q0_2;

% Traiettoria dei giunti qi_2(t)
q1_2 = q0_2(1) + dq_2(1)*sigma;
q2_2 = q0_2(2) + dq_2(2)*sigma;
q3_2 = q0_2(3) + dq_2(3)*sigma;
q4_2 = q0_2(4) + dq_2(4)*sigma;
q5_2 = q0_2(5) + dq_2(5)*sigma;
q6_2 = q0_2(6) + dq_2(6)*sigma;
q7_2 = q0_2(7) + dq_2(7)*sigma;
q8_2 = q0_2(8) + dq_2(8)*sigma;
q9_2 = q0_2(9) + dq_2(9)*sigma;

q_2 = [q1_2' q2_2' q3_2' q4_2' q5_2' q6_2' q7_2' q8_2' q9_2'];

% Velocità e accelerazione dei giunti
qp_2 = sigmap'*dq_2/T;
qpp_2 = sigmapp'*dq_2/(T^2);

% Inizializzazione matrice punti traiettoria
PtrjD_2 = zeros(4,N);
qsim_2 = zeros(N, length(q0_2));

% Simulazione 
for k= 1:N

   s = sigma(k);

   % Interpolazione giunti (P-P in spazio giunti)
   qk_2 = q0_2 + (dq_2) * s;  

   % Definizione del punto robot all'i-esimo istante
   qsim_2(k, :) = qk_2;

   % Analisi cinematica diretta di posizione
   matkD_2 = kindirDaVinci(qk_2, datiDaVinci);

   % Origine terna tool
   PtrjD_2(:,k) = matkD_2.Ttw(:,4);

   % Disegno
   cla
   hold on

   % Disegno DaVinci
   L = 200;
   disframe(eye(4),2*L)        % world
   disframe(datiDaVinci.T0w,L) % base robot (0)
   disframe(matkD_2.T1w,L)     % link 1...
   disframe(matkD_2.T2w,L)  
   disframe(matkD_2.T3w,L)  
   disframe(matkD_2.T4w,L)  
   disframe(matkD_2.T5w,L)  
   disframe(matkD_2.T6w,L)  
   disframe(matkD_2.T7w,L)  
   disframe(matkD_2.T8w,L)  
   disframe(matkD_2.T9w,L)  
   disframe(matkD_2.Ttw,L,'.') % tool

   % Base DaVinci
   P00 = base1.P;
   P0w = datiDaVinci.T0w*P00;
   patch('faces',base1.faces,'vertices',P0w(1:3,:)',...
         'facecolor',0.95*[0 0.5 1],'facealpha',0.6,...
         'edgecolor','none')

   % Link DaVinci
   for ii = 1:9
       Pii = link1(ii).P;
       Piw = matkD_2.(['T' num2str(ii) 'w']) * Pii;
       patch('faces',link1(ii).faces,'vertices',Piw(1:3,:)',...
         'facecolor',0.95*[1 1 1],'facealpha',0.6,...
         'edgecolor','none')
   end

   % Disegno RoboCouch
   disframe(datiCouch.T0w,L)        % base robot (0)
   disframe(matCouchFinal.T1w,L)
   disframe(matCouchFinal.T2w,L)
   disframe(matCouchFinal.T3w,L)
   disframe(matCouchFinal.T4w,L)
   disframe(matCouchFinal.T5w,L)
   disframe(matCouchFinal.T6w,L)
   disframe(matCouchFinal.Ttw,L,'.') % tool

   % Base Couch
   Pb0 = base2.P;
   Pbw = datiCouch.T0w*Pb0;
   patch('faces',base2.faces,'vertices',Pbw(1:3,:)',...
       'facecolor',0.95*[0 0.5 1],'facealpha',0.6,...
       'edgecolor','none')

   % Link Couch
   for ii = 1:length(link2)
       Pii = link2(ii).P;
       Piw = matCouchFinal.(['T' num2str(ii) 'w']) * Pii;
       patch('faces',link2(ii).faces,'vertices',Piw(1:3,:)',...
           'facecolor',0.95*[1 1 1],'facealpha',0.6,...
           'edgecolor','none')
   end

   % Disegno lettino
   Ptool = tool.P;
   Ptoolw = matCouchFinal.T6w * Ptool;
   patch('faces',tool.faces,'vertices',Ptoolw(1:3,:)',...
       'facecolor',0.5*[1 1 1],'facealpha',0.6,...
       'edgecolor','none')

   % Disegno punto target
   plot3(datiDaVinci.Pt(1), datiDaVinci.Pt(2), datiDaVinci.Pt(3), 'm*', 'MarkerSize', 6, 'LineWidth', 1);
   text(datiDaVinci.Pt(1), datiDaVinci.Pt(2), datiDaVinci.Pt(3), '   T', 'Color', 'm', 'FontSize', 10);

   light
   drawnow
   pause(0.1)   % regola la velocità della simulazione

   % grab immagine
   IM = getframe(hf);
   writeVideo(video,IM.cdata);

end


%% SIMULAZIONE 4: movimento verso un nuovo target con utensile inserito

% Definisco nuovo punto target  
datiCouch.P2 = [50;700;70;1];  % Assi x,y,z da regolare

% Opzionalmente calcolo della posizione iniziale nel world (per controllo)
datiCouch.Pt2 = matkC.Ttw * datiCouch.P2;
datiDaVinci.Pt = datiCouch.Pt2;  % Inizializza il target per DaVinci

% Definisco punti di partenza e punto finale 
q0_3 = qf_2;
qf_3 = invkinDaVinci(datiDaVinci,matkD_2,q0_3);

% Calcolo dello spostamento complessivo
dq_3 = qf_3 - q0_3;

% Traiettoria dei giunti qi_3(t)
q1_3 = q0_3(1) + dq_3(1)*sigma;
q2_3 = q0_3(2) + dq_3(2)*sigma;
q3_3 = q0_3(3) + dq_3(3)*sigma;
q4_3 = q0_3(4) + dq_3(4)*sigma;
q5_3 = q0_3(5) + dq_3(5)*sigma;
q6_3 = q0_3(6) + dq_3(6)*sigma;
q7_3 = q0_3(7) + dq_3(7)*sigma;
q8_3 = q0_3(8) + dq_3(8)*sigma;
q9_3 = q0_3(9) + dq_3(9)*sigma;

q_3 = [q1_3' q2_3' q3_3' q4_3' q5_3' q6_3' q7_3' q8_3' q9_3'];

% Calcolo di velocità e accelerazione dei giunti
qp_3 = sigmap'*dq_3/T;
qpp_3 = sigmapp'*dq_3/(T^2);

% Inizializzazione matrice punti traiettoria
PtrjD_3 = zeros(4,N);
qsim_3 = zeros(N, length(q0_3));

% Simulazione 
for k= 1:N

   s = sigma(k);

   % Interpolazione giunti (P-P in spazio giunti)
   qk_3 = q0_3 + (dq_3) * s;  

   % Definizione del punto robot all'i-esimo istante
   qsim_3(k, :) = qk_3;

   % Analisi cinematica diretta di posizione
   matkD_3 = kindirDaVinci(qk_3, datiDaVinci);

   % Origine terna tool
   PtrjD_3(:,k) = matkD_3.Ttw(:,4);

   % Disegno
   cla
   hold on

   % Disegno DaVinci
   L = 200;
   disframe(eye(4),2*L)        % world
   disframe(datiDaVinci.T0w,L) % base robot (0)
   disframe(matkD_3.T1w,L)     % link 1...
   disframe(matkD_3.T2w,L)  
   disframe(matkD_3.T3w,L)  
   disframe(matkD_3.T4w,L)  
   disframe(matkD_3.T5w,L)  
   disframe(matkD_3.T6w,L)  
   disframe(matkD_3.T7w,L)  
   disframe(matkD_3.T8w,L)  
   disframe(matkD_3.T9w,L)  
   disframe(matkD_3.Ttw,L,'.') % tool

   % Base DaVinci
   P00 = base1.P;
   P0w = datiDaVinci.T0w*P00;
   patch('faces',base1.faces,'vertices',P0w(1:3,:)',...
         'facecolor',0.95*[0 0.5 1],'facealpha',0.6,...
         'edgecolor','none')

   % Link DaVinci
   for ii = 1:9
       Pii = link1(ii).P;
       Piw = matkD_3.(['T' num2str(ii) 'w']) * Pii;
       patch('faces',link1(ii).faces,'vertices',Piw(1:3,:)',...
         'facecolor',0.95*[1 1 1],'facealpha',0.6,...
         'edgecolor','none')
   end

   % Disegno RoboCouch
   disframe(datiCouch.T0w,L)        % base robot (0)
   disframe(matCouchFinal.T1w,L)
   disframe(matCouchFinal.T2w,L)
   disframe(matCouchFinal.T3w,L)
   disframe(matCouchFinal.T4w,L)
   disframe(matCouchFinal.T5w,L)
   disframe(matCouchFinal.T6w,L)
   disframe(matCouchFinal.Ttw,L,'.') % tool

   % Base Couch
   Pb0 = base2.P;
   Pbw = datiCouch.T0w*Pb0;
   patch('faces',base2.faces,'vertices',Pbw(1:3,:)',...
       'facecolor',0.95*[0 0.5 1],'facealpha',0.6,...
       'edgecolor','none')

   % Link Couch
   for ii = 1:length(link2)
       Pii = link2(ii).P;
       Piw = matCouchFinal.(['T' num2str(ii) 'w']) * Pii;
       patch('faces',link2(ii).faces,'vertices',Piw(1:3,:)',...
           'facecolor',0.95*[1 1 1],'facealpha',0.6,...
           'edgecolor','none')
   end

   % Disegno lettino
   Ptool = tool.P;
   Ptoolw = matCouchFinal.T6w * Ptool;
   patch('faces',tool.faces,'vertices',Ptoolw(1:3,:)',...
       'facecolor',0.5*[1 1 1],'facealpha',0.6,...
       'edgecolor','none')

   % Disegno punto target
   plot3(datiDaVinci.Pt(1), datiDaVinci.Pt(2), datiDaVinci.Pt(3), 'y*', 'MarkerSize', 6, 'LineWidth', 1);
   text(datiDaVinci.Pt(1), datiDaVinci.Pt(2), datiDaVinci.Pt(3), '   T', 'Color', 'y', 'FontSize', 10);

   light
   drawnow
   pause(0.1)   % regola la velocità della simulazione

   % grab immagine
   IM = getframe(hf);
   writeVideo(video,IM.cdata);

end


%% SIMULAZIONE 5: movimento di estrazione utensile

% Definisco punti di partenza e punto finale 
q0_4 = qf_3;
qf_4 = qf_3;
qf_4(9) = qf_1(9);

% Calcolo dello spostamento complessivo
dq_4 = qf_4 - q0_4;

% Traiettoria dei giunti qi_4(t)
q1_4 = q0_4(1) + dq_4(1)*sigma;
q2_4 = q0_4(2) + dq_4(2)*sigma;
q3_4 = q0_4(3) + dq_4(3)*sigma;
q4_4 = q0_4(4) + dq_4(4)*sigma;
q5_4 = q0_4(5) + dq_4(5)*sigma;
q6_4 = q0_4(6) + dq_4(6)*sigma;
q7_4 = q0_4(7) + dq_4(7)*sigma;
q8_4 = q0_4(8) + dq_4(8)*sigma;
q9_4 = q0_4(9) + dq_4(9)*sigma;

q_4 = [q1_4' q2_4' q3_4' q4_4' q5_4' q6_4' q7_4' q8_4' q9_4'];

% Calcolo di velocità e accelerazione dei giunti
qp_4 = sigmap'*dq_4/T;
qpp_4 = sigmapp'*dq_4/(T^2);
PtrjD_4 = zeros(4,N);
qsim_4 = zeros(N, length(q0_4));

% Simulazione 
for k= 1:N

   s = sigma(k);

   % Interpolazione giunti (P-P in spazio giunti)
   qk_4 = q0_4 + (dq_4) * s; 

   % Definizione del punto robot all'i-esimo istante
   qsim_4(k, :) = qk_4;

   % Analisi cinematica diretta di posizione
   matkD_4 = kindirDaVinci(qk_4, datiDaVinci);
   PtrjD_4(:,k) = matkD_4.Ttw(:,4);

   % Disegno
   cla
   hold on

   % Disegno DaVinci
   L = 200;
   disframe(eye(4),2*L)        % world
   disframe(datiDaVinci.T0w,L) % base robot (0)
   disframe(matkD_4.T1w,L)     % link 1...
   disframe(matkD_4.T2w,L) 
   disframe(matkD_4.T3w,L) 
   disframe(matkD_4.T4w,L)  
   disframe(matkD_4.T5w,L)  
   disframe(matkD_4.T6w,L)  
   disframe(matkD_4.T7w,L) 
   disframe(matkD_4.T8w,L)  
   disframe(matkD_4.T9w,L)  
   disframe(matkD_4.Ttw,L)     % tool

   % Base DaVinci
   P00 = base1.P;
   P0w = datiDaVinci.T0w*P00;
   patch('faces',base1.faces,'vertices',P0w(1:3,:)',...
         'facecolor',0.95*[0 0.5 1],'facealpha',0.6,...
         'edgecolor','none')

   % Link DaVinci
   for ii = 1:9
       Pii = link1(ii).P;
       Piw = matkD_4.(['T' num2str(ii) 'w']) * Pii;
       patch('faces',link1(ii).faces,'vertices',Piw(1:3,:)',...
         'facecolor',0.95*[1 1 1],'facealpha',0.6,...
         'edgecolor','none')
   end

   % Disegno RoboCouch
   disframe(datiCouch.T0w,L)     % base robot (0)
   disframe(matCouchFinal.T1w,L) % link 1...
   disframe(matCouchFinal.T2w,L)
   disframe(matCouchFinal.T3w,L)
   disframe(matCouchFinal.T4w,L)
   disframe(matCouchFinal.T5w,L)
   disframe(matCouchFinal.T6w,L)
   disframe(matCouchFinal.Ttw,L,'.') % tool del Couch

   % Base Couch
   Pb0 = base2.P;
   Pbw = datiCouch.T0w*Pb0;
   patch('faces',base2.faces,'vertices',Pbw(1:3,:)',...
       'facecolor',0.95*[0 0.5 1],'facealpha',0.6,...
       'edgecolor','none')

   % Link Couch
   for ii = 1:length(link2)
       Pii = link2(ii).P;
       Piw = matCouchFinal.(['T' num2str(ii) 'w']) * Pii;
       patch('faces',link2(ii).faces,'vertices',Piw(1:3,:)',...
           'facecolor',0.95*[1 1 1],'facealpha',0.6,...
           'edgecolor','none')
   end

   % Disegno lettino
   Ptool = tool.P;
   Ptoolw = matCouchFinal.T6w * Ptool;
   patch('faces',tool.faces,'vertices',Ptoolw(1:3,:)',...
       'facecolor',0.5*[1 1 1],'facealpha',0.6,...
       'edgecolor','none')

   % Disegno punto target
   plot3(datiDaVinci.Pt(1), datiDaVinci.Pt(2), datiDaVinci.Pt(3), 'y*', 'MarkerSize', 6, 'LineWidth', 1);
   text(datiDaVinci.Pt(1), datiDaVinci.Pt(2), datiDaVinci.Pt(3), '   T', 'Color', 'y', 'FontSize', 10);

   light
   drawnow
   pause(0.1)

   % grab immagine
   IM=getframe(hf);
   writeVideo(video,IM.cdata);

end

% Chiusura video
video.close

%% Grafici pianificazione

% Funzione normalizzata
figure(3)
clf
subplot(3,1,1)
plot(tau,sigma)
title('Grafici della funzione normalizzata')
grid on
xlabel('tau [ ]')
ylabel('sigma [ ]')
subplot(3,1,2)
plot(tau,sigmap)
grid on
xlabel('tau [ ]')
ylabel('sigmap [ ]')
subplot(3,1,3)
plot(tau,sigmapp)
grid on
xlabel('tau [ ]')
ylabel('sigmapp [ ]')

% Traiettorie dei giunti nello spostamento del RoboCouch
figure(4)
clf
subplot(3,1,1)
plot(t,Q(:,1:4))
title('Grafici andamenti Qi(t) del RoboCouch')
grid on
xlabel('t[s]')
ylabel('Qi[°,mm]')
legend({'Q(1)','Q(2)','Q(3)','Q(4)'})
subplot(3,1,2)
plot(t,Qp(:,1:4))
grid on
xlabel('t[s]')
ylabel('Qpi[°/s,mm/s]')
subplot(3,1,3)
plot(t,Qpp(:,1:4))
grid on
xlabel('t[s]')
ylabel('Qppi[°/s^2,mm/s^2]');

% Traiettorie dei giunti nello spostamento della struttura portante del DaVinci
figure(5)
clf
subplot(3,1,1)
plot(t,q(:,1:4))
title('Grafici andamenti qi(t) della struttura portante del DaVinci')
grid on
xlabel('t[s]')
ylabel('qi[°,mm]')
legend({'q(1)','q(2)','q(3)','q(4)'})
subplot(3,1,2)
plot(t,qp(:,1:4))
grid on
xlabel('t[s]')
ylabel('qpi[°/s,mm/s]')
subplot(3,1,3)
plot(t,qpp(:,1:4))
grid on
xlabel('t[s]')
ylabel('qppi[°/s^2,mm/s^2]');

% Traiettorie dei giunti nello spostamento del polso del DaVinci
figure(6)
subplot(3,1,1)
plot(t,q_1(:,5:8))
title('Grafici andamenti q_1i(t) del polso del DaVinci')
grid on
xlabel('t[s]')
ylabel('q_1i[°,mm]')
legend({'q_1(5)','q_1(6)','q_1(7)','q_1(8)'})
subplot(3,1,2)
plot(t,qp_1(:,5:8))
grid on
xlabel('t[s]')
ylabel('q_1pi[°/s,mm/s]')
subplot(3,1,3)
plot(t,qpp_1(:,5:8))
grid on
xlabel('t[s]')
ylabel('qpp_1i[°/s^2,mm/s^2]');

% Traiettorie dei giunti nello spostamento dell'utensile del DaVinci
figure(7)
subplot(3,1,1)
plot(t,q_2(:,9))
title('Grafici andamenti spostamento utensile del DaVinci')
grid on
xlabel('t[s]')
ylabel('q_2i[°,mm]')
legend({'q_2(9)'})
subplot(3,1,2)
plot(t,qp_2(:,9))
grid on
xlabel('t[s]')
ylabel('qp_2i[°/s,mm/s]')
subplot(3,1,3)
plot(t,qpp_2(:,9))
grid on
xlabel('t[s]')
ylabel('qpp_2i[°/s^2,mm/s^2]');

% Traiettorie dei giunti nello spostamento del DaVinci nel nuovo punto target
figure(8)
subplot(3,1,1)
plot(t,q_3(:,5:8))
title('Grafici andamenti spostamento del DaVinci nel nuovo punto target')
grid on
xlabel('t[s]')
ylabel('q_3i[°,mm]')
legend({'q_3(5)','q_3(6)','q_3(7)','q_3(8)'})
subplot(3,1,2)
plot(t,qp_3(:,5:8))
grid on
xlabel('t[s]')
ylabel('qp_3i[°/s,mm/s]')
subplot(3,1,3)
plot(t,qpp_3(:,5:8))
grid on
xlabel('t[s]')
ylabel('qpp_3i[°/s^2,mm/s^2]');

% Traiettorie dei giunti nel'estrazione dell'utensile del DaVinci
figure(9)
subplot(3,1,1)
plot(t,q_4(:,9))
title('Grafici andamenti estrazione utensile del DaVinci')
grid on
xlabel('t[s]')
ylabel('q_4i[°,mm]')
legend({'q_4(9)'})
subplot(3,1,2)
plot(t,qp_4(:,9))
grid on
xlabel('t[s]')
ylabel('qp_4i[°/s,mm/s]')
subplot(3,1,3)
plot(t,qpp_4(:,9))
grid on
xlabel('t[s]')
ylabel('qpp_4i[°/s^2,mm/s^2]');

% Grafici andamento nel tempo di x,y,z origine terna tool del DaVinci
figure(10)
subplot(3,1,1)
plot(t,PtrjD_2(1,:))
title('Grafici andamento nel tempo di x,y,z origine terna tool del DaVinci')
grid on
xlabel('t[s]')
ylabel('x[mm]')
legend({'x'})
subplot(3,1,2)
plot(t,PtrjD_2(2,:))
grid on
xlabel('t[s]')
ylabel('y[mm]')
legend({'y'})
subplot(3,1,3)
plot(t,PtrjD_2(3,:))
grid on
xlabel('t[s]')
ylabel('z[mm]')
legend({'z'});

