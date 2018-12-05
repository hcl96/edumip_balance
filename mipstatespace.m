%% eduMIP State Space Control
clear;clc;
u = 0;  % no input
dt = 0.001;  %simulation time step
t_max = 10000;    %simulate for 10 seconds
%% Defining parameter constants
A1=2.7058*10^-4;
A2=2.9192*10^-4;
B1=2.9192*10^-4;
B2=6.7255*10^-4;
B3=-0.0842;
C1=(B1-(B2*A1)/A2)/(1+(B2/A2));
C2=B3/(1+B2/A2);
C3=(B2-(B1*A2)/A1)/(1+B1/A1);
C4=B3/(1+B1/A1);
G=35.555;
s=0.003;
k_motor=0.003/1760;
%% Constructing state space model
A=[0,1,0,0;0,(-2*G^2*k_motor)/(C1),-C2,2*G^2*k_motor;0,0,0,1;0,(-2*G^2*k_motor)/(C3),-C4,(-2*G^2*k_motor)];
B=[0;-2*G*s;0;-2*G*s];
C=[1,0,0,0;0,0,1,0];
D=[0,0]';
sysC=ss(A,B,C,D); %CT state space
dt=0.01; %10mS processor clock speed
sysD=c2d(sysC,dt,'zoh'); %DT state space
%% Controllability and Observability
eigA=eig(sysD.A)
fprintf('A has eigenvalues on/outside unit circle, so is unstable\n')
ctrbA=rank(ctrb(A,B));
obsvC=rank(obsv(A,C));
fprintf('AB has full rank of %d: "Controllable" & BC has full rank of  %d: "Observable"\n',ctrbA,obsvC');
[U,S,V]=svd(ctrb(A,B));
%% Controller Design
Ktarget=[0.999,0.9991,0.9992,0.9993]; %arbitrary choosing eigenvalues < 1
K_d1=place(sysD.A,sysD.B,Ktarget)
Q=eye(4); Q(1,1)=10; Q(2,2)=100; %highest weight on maintaining balance
R=0.01; %low effort cost
K_d2=lqr(sysD,Q,R)
eig(sysD.A-sysD.B*K_d2)
%% Simulate Controller
K_c=lqr(A,B,Q,R);
sysD_bal=ss(A-B*K_c,[],C,[]); %create balanced ss
x0=[deg2rad(10),0,0,0]; %initial condition with 10 degrees tilt
initial(sysD_bal,x0,0:0.001:10)



