% Controlador por AVANÇO E ATRASO de fase
clear all;clc;

% =============   CASO 2    =============
% =======  Beta IGUAL DE Gama   ===========

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
% Sobressinal_max = input('Insira o máximo SOBRESSINAL desejado em % -> ') 
Sobressinal_max = 30;
Sobressinal_max = (Sobressinal_max)/100;

disp('VALOR DE AJUSTE FINO 2 ou 4 ou 6')
ts = 2;
% ts = input('Insira o Tempo de Acomodação (TS) máximo desejado -> ');


% =========================================
% =======  Controlador de Avanço  =========
% =========================================
disp('Mínimo valor para Sigma')
sigma_min = 4/ts

disp('VALOR DE AJUSTE FINO 4 ou 2.5 ou 2')
sigma = 4;
% sigma = input('Escolha um sigma maior que o sigma mínimo e do que o menor polo da planta -> ')


syms wd_max
wd_Max = solve(exp(-(sigma*pi)/(wd_max)) == Sobressinal_max);
wd_Max = double(wd_Max)

disp('VALOR DE AJUSTE FINO 10 ou 6 ou 5')
wd = 10;
% wd = input('Escolha um wd menor que o wd máximo -> ')

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


% Estacionário
disp('VALOR DE AJUSTE FINO 0.13 ou 0.15 ou 0.05')
ess = 0.01;
% ess = input('Insira o valor máximo de ERRO ESTACIONÁRIO desejado -> '); 
Kp_desejado = (1/ess)-1


disp('Determinando o Kc do Controlador')
s = 0
Kp_antigo = ((45)/((s+9)*(s+5)*(s+1)));
Kc = Kp_desejado/Kp_antigo


disp('Escolha valores iniciais para o zero e polo do controlador de avanço')
disp(' ')
disp(' ')
% zero_adv = input('Escolha o zero')
% disp(' ')
% polo_adv = input('Escolha o polo')
zero_adv = 4
polo_adv = 39.8730

controlador = (s1 + zero_adv)/(s1 + polo_adv);
planta = ((45)/((s1+9)*(s1+5)*(s1+1)));
modulo_inicial = abs(Kc*planta*controlador)
Kc

% % Determinação automática do Zero e Polo do Controlador
% if modulo_inicial > 1.1
%    %Seria subindo o valor do zero, para abaixar o modulo 
% %    chave = 0;
%    for c = 1:30
%         zero_adv = zero_adv + 0.1;
%         alfa = atand((sigma-zero_adv)/(wd));
%         polo_adv = sigma + wd*tand(-deficiencia_angular - alfa);        
%         
%         %Conferir valor do Modulo e Fase
%         controlador = (s1 + zero_adv)/(s1 + polo_adv);
%         modulo(c) = abs(Kc*planta*controlador);
%         fase(c) = (180/pi)*(angle(controlador));
%    
%    end
%    
% elseif modulo_inicial < 0.9
%     chave = 0;
%     while chave == 0
%         zero_adv = zero_adv - 0.1;
%         
%         % ==> Calculo do polo       
%         % Caso o polo seja maior que sigma
%             %...
%         % Caso o polo seja menor que sigma
%         alfa = atand((sigma-zero_adv)/(wd));
%         polo_adv = sigma + wd*tand(-deficiencia_angular - alfa);        
%         
%         %Conferir valor do Modulo e Fase
%         controlador = (s1 + zero_adv)/(s1 + polo_adv);
%         modulo = abs(Kc*planta*controlador);
%         fase = (180/pi)*(angle(controlador));
%         if modulo>0.99 & modulo<1.01 & fase>(-deficiencia_angular-0.05) & fase<(-deficiencia_angular+0.05)
%            chave = 1;
%         end      
%     end    
% end
% 
% disp('O zero e o polo abaixo')
% zero_adv
% polo_adv
% return

% 
% disp('Atende o critério de modulo e fase')
% modulo
% fase

disp('A função de Transferencia do Controlador é')
G_adv = tf([Kc Kc*zero_adv],[1 polo_adv])
T1 = 1/zero_adv
gama = T1*polo_adv
beta = gama

%========================================================================
%========================================================================

% =========================================
% =======  Controlador de Atraso  =========
% =========================================


% =========================================
% == Escolhendo um Zero arbitrariamente  ==
% =========================================
zero_lag = 0.7

T2 = 1/zero_lag;
disp('Valores para teste de Alocação')
zero_lag
polo_lag = 1/(beta*T2)

%Testando condição de Modulo e Fase
theta = atand((wd)/(-sigma+zero_lag)) - atand((wd)/(-sigma+polo_lag))
mod = abs((s1+zero_lag)/(s1+polo_lag))


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

Sobressinal_do_sistema = infor(5)
Tempo_acomodacao = infor(2)  

sim('adv_lag_igual_simulink')
Erro_Estacionario = Erro_Estacionario(end)
