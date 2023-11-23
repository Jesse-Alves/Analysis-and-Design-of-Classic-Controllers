% Controlador por avanço de fase
clear all;close all;clc;

% Planta escolhida
disp('Planta do Sistema')
FT = tf([45],[1 15 59 45]);% Polos em -9,-5 e -1
polos_planta = pole(FT);
Funcao_de_Tranferencia = zpk(FT)

% Critérios de Desempenho

% Transitório
disp('VALOR DE AJUSTE FINO 20')
Sobressinal_max = input('Insira o máximo sobressinal desejado em % -> ') %Sobressinal < 20%
Sobressinal_max = (Sobressinal_max)/100;

disp('VALOR DE AJUSTE FINO 2')
ts = input('Insira o Tempo de Acomodação máximo desejado -> '); % Tempo de Acomodação < 5 segundos

disp('Mínimo valor para Sigma')
sigma_min = 4/ts

disp('VALOR DE AJUSTE FINO 2.5')
sigma = input('Escolha um sigma maior que o sigma mínimo e do que o menor polo da planta -> ')

syms wd_max
wd_Max = solve(exp(-(sigma*pi)/(wd_max)) == Sobressinal_max);
wd_Max = double(wd_Max)

disp('VALOR DE AJUSTE FINO 4')
wd = input('Escolha um wd menor que o wd máximo -> ')

disp('Os polos dominantes que serão alocados são:')
s1 = -sigma + i*wd
s2 = -sigma - i*wd


% Olhar se estes polos são alcançados com a mudança do 
% ganho proporcional do sistema, isto é, se estes polos já
% pertencem ao lugar das raízes do sistema.


% Se não, prossiga o projeto para a Alocação


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
      

% % =======  METODO  =======
% % =======     1    =======
% 
% Determinação do polo e zero a ser alocado
zero_cont1 = 3.87;
polo_cont1 = 5.75;

% Determinação do Ganho Kc

disp('O ganho Kc vale:')
planta = ((45)/((s1+9)*(s1+5)*(s1+1)));
Kc = (1)/abs(((s1+zero_cont1)/(s1+polo_cont1))*planta)

%disp('')

disp('A FT do compensador de avanço será:')
Gc = tf([Kc Kc*zero_cont1],[1 polo_cont1])

FT_ma = series(Gc,FT)
FT_mf = feedback(FT_ma,1);

% Comparativo no Lugar das Raizes
rlocus(FT)
hold on
rlocus(FT_mf)
grid on
title('Lugar das raízes so Sistema com e Sem controlador')
legend('Planta','Compensador de Avanço')


% Resultado do Controlador
FT_mf = zpk(FT_mf)
polos_mf = pole(FT_mf)
infor = stepinfo(FT_mf)
infor = struct2array(infor);
[y1,t1] = step(FT_mf);
figure;
plot(t1,y1)
title('Respostas do Sistema com os Controladores de Avanço')
grid on
hold on

% Resultado do Controle
Sobressinal_max = Sobressinal_max*100
Sobressinal_do_sistema = infor(5)
Ts_max = ts
Tempo_acomodacao = infor(2)
sim('advance_compensator_simulink')
Erro_Estacionario = Erro_Estacionario(end)


% =======  METODO  =======
% =======     2    =======

% Determinação do Zero e do Polo do Controlador de Avanço
if sigma > abs(max(polos_planta))    
    zero_cont2 = abs(max(polos_planta));
    ang1 = atand((sigma-abs(max(polos_planta)))/(wd));
    polo_cont2 = sigma + wd*tand(-deficiencia_angular - ang1);
else
    disp('Sigma escolhido é menor que o menor polo, escolha outro')
    return
end

disp('O ganho Kc pelo Metodo 2 vale:')
planta = ((45)/((s1+9)*(s1+5)*(s1+1)));
Kc2 = (1)/abs(((s1+zero_cont2)/(s1+polo_cont2))*planta)

disp('A FT do compensador de Atraso será:')
Gc2 = tf([Kc2 Kc2*zero_cont2],[1 polo_cont2])

FT_ma2 = series(Gc2,FT);
FT_mf2 = feedback(FT_ma2,1)

% ======> Resultado do Controlador 2
disp('Atendeu os Critérios')
polos_mf = pole(FT_mf2)
[y2,t2] = step(FT_mf2);
plot(t2,y2)
legend('MÉTODO 1','MÉTODO 2')
grid on

infor2 = stepinfo(FT_mf2)
infor2 = struct2array(infor2);

disp('O zero do controlador cancela com o Polo do Sistema')
FT_ma2 = zpk(FT_ma2)

% Resultado do Controle 2
Sobressinal_max
Sobressinal_do_sistema2 = infor2(5)
Ts_max = ts
Tempo_acomodacao2 = infor2(2)  
sim('advance_compensator2_simulink')
Erro_Estacionario2 = Erro_Estacionario2(end)
