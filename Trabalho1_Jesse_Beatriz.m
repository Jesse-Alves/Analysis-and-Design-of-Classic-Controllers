% Trabalho 1 Controle - Jesse e Beatriz
clc;clear all;close all;

% Sistema com Realimentacao unitaria Via simulink
sim('Trabalho1_Jesse_Beatriz_simulink') %Executar arquivo simulink
plot(tout,y)
grid on
title('SIMULINK - Sistema com Realimentacao unitaria e Entrada Degrau unitario')

% Planta escolhida
disp('Planta do Sistema')
ft = tf([45],[1 15 59 45]);% Polos em -9,-5 e -1
Funcao_de_Tranferencia = zpk(ft)

% ===> ESTABILIDADE

% Novos polos: -10, -2,5+1,66i , -2,5-1,66i
ft_realimentado = tf([45],[1 15 59 90]) 
disp('Verificacao de Estabilidade: 1 - Estavel  0 - Instavel')
estab = isstable(ft_realimentado)

disp('Os polos mudam do sistema com realimentacao unitaria')
disp('Mas continuam no SPE, portanto o sistema é estavel')
zpk(ft_realimentado)


% ===> SENSIBILIDADE
disp('A SENSIBILIDADE É DADA POR:')

s = 0; %Quando s é zero
Sensibilidade = (1)/(1 + 1*((45)/((s+9)*(s+5)*(s+1))))


% ========================= 1 METODO DE SINTONIA ========================= 
% =========================                      ========================= 
figure;
plot(tout,y_metodo1)
grid on
axis([0 5 0 1.5])
hold on

%Ponto recolhidos do grafico com ginput()
t_reta = [0.3444 0.5561 0.7208];
y_reta = [0.0953 0.2231 0.3293];

%Fomarcao da reta
p = polyfit(t_reta,y_reta,1);
y_reta = p(1)*tout + p(2);
plot(tout,y_reta,'k')
title('DETERMINACAO DOS PARAMETROS K,L e T')

%Definicao dos parametros K,L e T

%Ao recolher os pontos com a funcao ginput()
K = 1;
L = 0.1999;
T = 1.6029;

% Comparando o modelo real com o aproximado
figure;
plot(tout,y_metodo1_1)
grid on
title('COMPARACAO DO MODELO REAL E MODELO APROXIMADO')
legend('Valor de K','Modelo Real','Modelo Aproximado')

% ===> VALORES TABELADOS DOS GANHOS - METODO 1

% CONTROLADOR P
disp('CONTROLADOR P')
Kp1 = T/(K*L)

% CONTROLADOR PI
disp('CONTROLADOR PI')
KP1 = (0.9*T)/(K*L)
KI1 = KP1/(L/0.3)

% CONTROLADOR PID
disp('CONTROLADOR PID')
kp1 = (1.2*T)/(K*L)
ki1 = kp1/(2*L)
kd1 = kp1*0.5*L
return

% ========================= 2 METODO DE SINTONIA ========================= 
% =========================                      ========================= 
figure;
plot(tout,y_metodo2)
grid on

% Ganho Critico
disp('Ganho Critico')
Kcr = 18.61

% O periodo critico foi calculado pela funcao ginput()
disp('Periodo Critico')
Pcr = 1.2853 - 0.4721

% ===> VALORES TABELADOS DOS GANHOS - METODO 2

% CONTROLADOR P
disp('CONTROLADOR P')
Kp2 = 0.5*Kcr

% CONTROLADOR PI
disp('CONTROLADOR PI')
KP2 = 0.45*Kcr
KI2 = (KP2)/(Pcr/1.2)

% CONTROLADOR PID
disp('CONTROLADOR PID')
kp2 = 0.6*Kcr
ki2 = (kp2)/(0.5*Pcr)
kd2 = kp2*(0.125*Pcr)
