% 
% Kalman equivalent filters
%
% Author: Francesco Achille
%
% Date: 5/06/2010
% 
% Requirements: Control System Toolbox (lsim and tf functions)
%


clear all;
close all;
clc;

Ts=0.01; %Sampling time
v=1; % target velocity [m/s]
s0=0; % Initial target posotion [m]
t=0:Ts:10; %Time
std_noise=0.1; %meas noise std [m]
var_noise=std_noise^2; 

%Original signal and noise affected measurement 
signal=t*v+s0; 
meas=signal+std_noise*randn(size(signal))

%Process noise variance (use this to modify the filter bandwidth)
var_process=0.1; 

%Kalman filter init

%constant matrix
Rn=var_noise; %noise covariance matrix
Qn=[0 0;0 var_process]; %Process noise covariance matrix
sig=[1 Ts; 0 1]; %Plant (constant velocity) matrix
M=[1 0]; %Mauasurement matrix
H=M;

pos_s(1)=meas(1);
vel_s(1)=0; %not used
pos_s(2)=meas(2);
vel_s(2)=(pos_s(2)-pos_s(1))/Ts;

X_piu=[pos_s(2);vel_s(2)]; %initial state estimate
P_piu=[std_noise sqrt(2*var_noise)/Ts]'*[std_noise sqrt(2*var_noise)/Ts]; %initial error covariance matrix esitmate

%temp variables for plot 
K_pos(1)=1;K_pos(2)=1;
K_vel(1)=1;K_vel(2)=1;

%%
%Kalman filter 
for(i=3:length(signal))
X_meno=sig*X_piu;
P_meno=sig*P_piu*sig'+Qn;
Kk=P_meno*H'*(H*P_meno*H'+Rn)^(-1);
K_pos(i)=Kk(1,1); %memorize result
K_vel(i)=Kk(2,1); 
X_piu=X_meno+Kk*(meas(i)-H*X_meno);
P_piu=([1 0;0 1]-Kk*H)*P_meno;
pos_s(i)=X_piu(1,1); 
vel_s(i)=X_piu(2,1); 
end
%%

%Steady-state kalman gains
g=K_pos(length(K_pos));
h=K_vel(length(K_vel));

%Equivalent filters equations
filtro_eq_ang=tf([g Ts*h-g 0],[1 g+Ts*h-2 1-g],Ts); %relation estimated_postion/measured_position
filtro_eq_w=tf([h -h 0],[1 g+Ts*h-2 1-g],Ts); %relation estimated_velocity/measured_position
%filtro_eq_ww=tf([h -h 0],[1 g+Ts*h-2 1-g],Ts)*tf([Ts],[1 -1],Ts); %relation estimated_velocity/diff(measured_position)

%Filtering
pos_eq=lsim(filtro_eq_ang,meas,t);
vel_eq=lsim(filtro_eq_w,meas,t);




%%
%Results plots
figure;
plot(t,pos_s-pos_eq');
legend('Difference Kalman Est. Pos - Filter Est. Pos')
figure;
plot(t,vel_s-vel_eq');
legend('Difference Kalman Est. Vel - Filter Est. Vel')

figure;
bode(filtro_eq_w);
title('Equivalent filter for velocity')

figure;
bode(filtro_eq_ang);
title('Equivalent filter for position')

figure;
plot(t,K_vel,t,zeros(size(K_vel))+h);
legend('Kalman velocity gain','value used for filter set-up')
xlabel('time [s]');
ylabel('Gain');

figure;
plot(t,K_pos,t,zeros(size(K_pos))+g);
legend('Kalman position gain','value used for filter set-up')
xlabel('time [s]');
ylabel('Gain');

figure;
plot(t,[Ts diff(signal)/Ts],t,vel_s,t,vel_eq);
legend('True velocity','Kalman Est. Velocity','Filter estimated velocity');
title('Velocity Estimate results');
xlabel('Time [s]');
Ylabel('Velocity [m\s]');

figure;
plot(t,signal,t,meas,t,pos_s,t,pos_eq);
legend('True position','Measured position','Kalman estimated position','Filter estimated position');
title('Position estimate results');
xlabel('Time [s]');
Ylabel('Position [m]');

figure;
plot(t,signal,t,meas,t,pos_s,t,pos_eq);
xlim([0 1])
ylim([-0.5 1])
legend('True position','Measured position','Kalman estimated position','Filter estimated position');
title('Position estimate results - Startup');
xlabel('Time [s]');
Ylabel('Position [m]');


