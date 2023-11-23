% Controlador por AVANÇO E ATRASO de fase
clear all;close all;clc;

% =============   CASO 1    =============
% =======  Beta DIFERENTE DE Gama   =======

% Planta escolhida
disp('Planta do Sistema')
polinomio = poly([-9 -5 -1]);% Polos em -9,-5 e -1
FT = tf([45],polinomio);
polos_planta = pole(FT);
Funcao_de_Tranferencia = zpk(FT)


% =========================================
% =======  Critérios de Desempenho  =======
% =========================================

% Transitório
disp('VALOR DE AJUSTE FINO 30%')
Sobressinal_max = input('Insira o máximo SOBRESSINAL desejado em % -> ') 
Sobressinal_max = (Sobressinal_max)/100;

disp('VALOR DE AJUSTE FINO 2 ou 4 ou 6')
ts = input('Insira o Tempo de Acomodação (TS) máximo desejado -> ');

% Reaproveitando os projetos já feitos

%Controlador de Avanço
disp('Mínimo valor para Sigma')
sigma_min = 4/ts

disp('VALOR DE AJUSTE FINO 4 ou 2.5 ou 2')
sigma = input('Escolha um sigma maior que o sigma mínimo e do que o menor polo da planta -> ')


syms wd_max
wd_Max = solve(exp(-(sigma*pi)/(wd_max)) == Sobressinal_max);
wd_Max = double(wd_Max)

disp('VALOR DE AJUSTE FINO 10 ou 6 ou 5')
wd = input('Escolha um wd menor que o wd máximo -> ')

disp('Os polos dominants que serão alocados são:')
s1 = -sigma + i*wd
s2 = -sigma - i*wd

% Cálculo da Deficiencia Angular - A PLANTA SEM ZEROS
deficiencia_angular = 180;
for k = 1:size(polos_planta,1)
    
    if abs(real(polos_planta(k))) > abs(real(s1))
        
        theta_i = atand((imag(s1))/(abs(real(polos_planta(k)))-abs(real(s1))));
        deficiencia_angular = deficiencia_angular - theta_i;
        
    elseif abs(real(polos_planta(k))) < abs(real(s1))  
        
        theta_i = 180 - atand((imag(s1))/(abs(real(s1))-abs(real(polos_planta(k)))));
        deficiencia_angular = deficiencia_angular - theta_i;
    
    elseif abs(real(polos_planta(k))) == abs(real(s1))
        theta_i = 90;
        deficiencia_angular = deficiencia_angular - theta_i;  
    end        
end

if deficiencia_angular >=0
    disp('Deficiencia angular positiva, por favor escolhe outros polos para Alocar')
    return
end    

% Determinação do Zero e do Polo do Controlador de Avanço
if sigma > abs(max(polos_planta))    
    zero_adv = abs(max(polos_planta));
    ang1 = atand((sigma-abs(max(polos_planta)))/(wd));
    polo_adv = sigma + wd*tand(-deficiencia_angular - ang1);
    
elseif sigma == abs(max(polos_planta))
    zero_adv = abs(max(polos_planta));
    polo_adv = sigma + wd*tand(-deficiencia_angular);     
else
    disp('Sigma escolhido é menor que o menor polo, escolha outro')
    return
end

disp('O ganho Kc pelo Metodo 2 vale:')
planta = ((45)/((s1+9)*(s1+5)*(s1+1))); % Planta pessoal
Kc = (1)/abs(((s1+zero_adv)/(s1+polo_adv))*planta)

G_adv = tf([Kc Kc*zero_adv],[1 polo_adv])

disp('Parametros')
Kc
T1 = 1/zero_adv
gama = T1*polo_adv

%=========================================================================

disp('=== Controlador de Atraso ===')
disp(' ')
disp(' ')

% =========================================
% =======  Critério de Desempenho  =======
% =========================================

% Estacionario
disp('VALOR DE AJUSTE FINO 0.13 ou 0.15 ou 0.05')
ess = input('Insira o valor máximo de ERRO ESTACIONÁRIO desejado -> '); 
Kp_desejado = (1/ess)-1

%Calculando o Kp do sistema original
s = 0
Kp_antigo = ((45)/((s+9)*(s+5)*(s+1))); %FTma quando s=0

%Calculando o valor de Beta
beta = (Kp_desejado*gama)/(Kc*Kp_antigo)

% Escolhemos arbitrariamente um valor para o zero
% do compensador, perto da origem
disp('VALOR DE AJUSTE FINO 1.3 ou 0.8 ou 0.55')
zero_lag = input('Escolha um Zero para o controlador perto da Origem-> ');

T2 = 1/zero_lag;

disp('Valores para teste de Alocação')
zero_lag
polo_lag = 1/(beta*T2)


% ========================================
% ====== Testando esse polo e zero =======
% ========================================

% CONDIÇÃO DE FASE

% fi = atand((abs(real(s1)) - zero_lag)/(abs(imag(s1))));
% alfa = atand((abs(real(s1)) - polo_lag)/(abs(imag(s1)))); 
% theta = alfa - fi;
theta = atand((wd)/(-sigma+zero_lag)) - atand((wd)/(-sigma+polo_lag))

%CONDIÇÃO DE MÓDULO
modulo = abs((s1+zero_lag)/(s1+polo_lag));


if theta>-5 & theta<0 & modulo<1.2 & modulo > 0.8
    disp('Angulo Adequado para o projeto. O valor de Theta é:')
    theta
    disp('Modulo Adequado para o projeto. O valor do Modulo é:')
    modulo
    
    disp('A FT do compensador de Atraso será:')
    G_lag = tf([1 zero_lag],[1 polo_lag])
    
    disp('A FT do Controlador será:')
    Gc = series(G_adv,G_lag)
    
    FT_ma = series(Gc,FT);
    disp('A FT de Malha Fechada será:')
    FT_mf = feedback(FT_ma,1);    
    FT_mf = zpk(FT_mf)
    disp('Os polos que foram alocados são:')
    s1
    s2
    disp('Os polos de Malha Fechada do sistema')
    polos_mf = pole(FT_mf)
    step(FT_mf)
    grid on 
      
    infor = stepinfo(FT_mf)
    infor = struct2array(infor);
    
    disp('O zero do controlador cancela com o Polo do Sistema')
    FT_ma = zpk(FT_ma)
    
    Sobressinal_do_sistema = infor(5)
    Tempo_acomodacao = infor(2)    
    
    disp('Teste do Erro Estacionario')
    sim('adv_lag_simulink')
    Erro_Estacionario = Erro_Estacionario(end)
    
    if Erro_Estacionario <= (ess+0.001)
        disp('O Erro estacionario atendeu ao Criterio')
        
    else
        disp('O Erro estacionario NÃO ATENDIDO')
    end    
    
else    
    disp('Angulo não adequado, Escolher outro valor de zero para o controlador')
    theta
end
