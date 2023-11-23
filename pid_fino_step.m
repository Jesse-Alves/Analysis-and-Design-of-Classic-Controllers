disp('PID COM AJUSTE FINO')
kp2 = 12;
ki2 = 5;
kd2 = 2;

disp('O função de Transferência do controlador será:')
PID_fino = tf([kd2 kp2 ki2],[1 0])

disp('O sistema')
KGH_fino = series(PID_fino,FT)
[y1,t1] = step(feedback(KGH_fino,1));
plot(t1,y1)
grid on
