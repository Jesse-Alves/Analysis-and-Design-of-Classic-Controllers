% Controlador por ATRASO de fase
clear all;close all;clc;

% Planta escolhida
disp('Planta do Sistema')
FT = tf([45],[1 15 59 45]);% Polos em -9,-5 e -1
Funcao_de_Tranferencia = zpk(FT)

%Polos originais da planta em MF:
FTmf = feedback(FT,1);
polos_originais = pole(FTmf);

% Critérios de Desempenho

% Estacionario
disp('VALOR DE AJUSTE FINO 0.05')
ess = input('Insira o valor máximo de Erro Estacionároio desejado -> '); % 0.001
Kp_desejado = (1/ess)-1

%Calculando o Kp do sistema original
s = 0;
Kp_antigo = ((45)/((s+9)*(s+5)*(s+1))); %FTma quando s=0

%Calculando o valor de Beta
beta = Kp_desejado/Kp_antigo

% Escolhemos arbitrariamente um valor para o zero
% do compensador, perto da origem

valor_maximo = abs(real(polos_originais(2)))
disp('VALOR DE AJUSTE FINO 0.435')
zero_lag = input('Escolha um Zero para o controlador PERTO DE ZERO-> ');%0.4 por exemplo
T = 1/zero_lag;

disp('Valores para teste de Alocação')
zero_lag
polo_lag = 1/(beta*T)


% ========================================
% ====== Testando esse polo e zero =======
% ========================================

% fi = atand((abs(real(polos_originais(2))) - zero_lag)/(abs(imag(polos_originais(2)))));
% gama = atand((abs(real(polos_originais(2))) - polo_lag)/(abs(imag(polos_originais(2))))); 
% theta = gama - fi;

sigma = abs(real(polos_originais(2)));
wd = abs(imag(polos_originais(2)));
theta = atand((wd)/(-sigma+zero_lag)) - atand((wd)/(-sigma+polo_lag))

if theta>-5 & theta<0
    disp('Angulo Adequado para o projeto. O valor de Theta é:')
    theta
    
    disp('O ganho Kc vale:')
    s1 = polos_originais(2);
    planta = ((45)/((s1+9)*(s1+5)*(s1+1)));
    Kc = (1)/abs(((s1+zero_lag)/(s1+polo_lag))*planta)
    
    disp('A FT do compensador de Atraso será:')
    Gc = tf([Kc Kc*zero_lag],[1 polo_lag])
    FT_ma = series(Gc,FT);
    FT_mf = feedback(FT_ma,1);    
    FT_mf = zpk(FT_mf)
    disp('Os polos de Malha Fechada do sistema')
    polos_mf = pole(FT_mf)
    stepinfo(FT_mf)
    step(FT_mf)    
    grid on  
    
    disp('Teste do Erro Estacionario')
    sim('lagging_compensator_simulink')
    Erro_Estacionario = Erro_Estacionario(end)    
    
    if Erro_Estacionario <= ess
        disp('O Erro estacionario atendeu ao Criterio')
    else
        disp('O Erro estacionario NÃO ATENDIDO')
    end    
    
else    
    disp('Angulo não adequado, Escolher outro valor de zero para o controlador')
end
