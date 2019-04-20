[N,T,Z,F,Hfull,mX0,PX0,Qw,Rv,X] = simulationDonnees(1);
[d s] = kalman(N,T,Z,F,Hfull,mX0,PX0,Qw,Rv,X);
subplot(1,2,1)
plot(s(2:50,1),s(2:50,2))
hold on
plot(s(2:50,3),s(2:50,4),'r')
hold on
plot(s(2:50,5),s(2:50,6),'y')
hold off
xlim([-5 5])
ylim([-5 5])
title('X estimé avec filtre de Kalman')
subplot(1,2,2)
plot(X(1,2:50),X(2,2:50))
hold on
plot(X(3,2:50),X(4,2:50),'r')
hold on
plot(X(5,2:50),X(6,2:50),'y')
title('X sans filtre de Kalman')
xlim([-5 5])	
ylim([-5 5])
