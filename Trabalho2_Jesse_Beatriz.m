% Trabalho 2 Controle - Jesse e Beatriz
clc;clear all;close all;

% Planta escolhida
disp('Planta do Sistema')
FT = tf([45],[1 15 59 45]);% Polos em -9,-5 e -1
Funcao_de_Tranferencia = zpk(FT)


% ========== PID -> METODO 1 DE SINTONIA ================ 
% ==========      SEM AJUSTE FINO        ================ 

disp('Pelo Metodo 1 temos os GANHOS')

kp1 = 9.6222;
ki1 = 24.0676;
kd1 = 0.9617;

disp('Logo temos kp1(1 + 1/s*Ti1 + sTd1)')
kp1
Ti1 = kp1/ki1
Td1 = kd1/kp1

disp('O fun��o de Transfer�ncia do controlador ser�:')
PID = tf([kd1 kp1 ki1],[1 0])

disp('O sistema')
KGH = series(PID,FT)


% ========================= PID -> METODO 1 DE SINTONIA ========================= 
% =========================      COM AJUSTE FINO        ========================= 


disp('PID COM AJUSTE FINO')
kp2 = 12;
ki2 = 5;
kd2 = 2;

disp('Logo temos kp1(1 + 1/s*Ti1 + sTd1)')
kp2
Ti2 = kp2/ki2
Td2 = kd2/kp2

disp('O fun��o de Transfer�ncia do controlador ser�:')
PID_fino = tf([kd2 kp2 ki2],[1 0])

disp('O sistema')
KGH_fino = series(PID_fino,FT)


% ======= Compara��o entre os zeros e os polos ======

disp('Compara��o dos polos')
planta = pole(FT)
pid_fino = pole(feedback(KGH,1))
pid_ajustado = pole(feedback(KGH_fino,1))


disp('Compara��o dos zeros')
planta = zero(FT)
malha_fechada = zero(feedback(KGH,1))
pid_ajustado = zero(feedback(KGH_fino,1))


% ===> An�lise do Lugar das Ra�zes

rlocus(FT)
title('Lugar das ra�zes da planta')
grid on

% PID SEM ajuste Fino
figure;
rlocus(KGH)
title('Lugar das ra�zes com controladores PID')
grid on
hold on

% PID COM ajuste Fino
rlocus(KGH_fino)
legend('PID','PID_fino')

% CONCLUS�O

% No PID ajustado o lugar das raizes est� mais para a esquerda
% O que indica um sigma maior em m�dulo => Um menor sobressinal
% Al�m de ter um angulo beta menor => Maior amortecimento => .
% => Menos oscila��o no transit�rio

% Os graficos abaixo provam isso:
figure;
[y,t] = step(feedback(KGH,1));
plot(t,y)
grid on
hold on

[y1,t1] = step(feedback(KGH_fino,1));
plot(t1,y1)

%S� PID
figure;
rlocus(KGH)
title('PID')
grid on

%S� PID Fino
figure;
rlocus(KGH_fino)
title('PID Fino')
grid on


% ===================================================== 
% ==== Calculo do Lugar das Raizes no papel ===========
% =====================================================

%PID sem ajuste fino

% Ponto de partida do eixo Real
syms s
derivada = diff(-((s*(s+9)*(s+5)*(s+1))/(43.28*(s+5.03)*(s+4.97))))
raizes = double(solve(derivada == 0,s))


%PID Com ajuste fino
derivada2 = diff(-((s*(s+9)*(s+5)*(s+1))/(90*(s+5.55)*(s+0.45))))
raizes2 = double(solve(derivada2 == 0,s))

