%% Homework # 1
clc;clear all
%% Define the system
model_name = "hwk_1";
sim_time = 10;

% Create a state variable 's'
% s = zpk(0, [], 1)
s = tf('s');
G = tf([1],[0.0025 0.1 1]);
Gd_term = 0.5;
H = tf([200],[10 1]);

Plant = G*H;
G_d = Gd_term*H;

%% PID Controller

% Transient-Response Specifications

% Overshoot less than 5%
percent_Mp = 5;
damping_ratio = -log(percent_Mp/100)/(sqrt(pi^2+(log(percent_Mp/100))^2)); % For a desirable transient response,damping ratio must be between 0.4 and 0.8.


% Find beta using damping ratio
beta = atan(sqrt(1-damping_ratio^2)/damping_ratio);

% Rise time -- Should be less than 0.3
t_rise = 0.2;
w_d = (pi - beta)/t_rise;
w_n = w_d/(sqrt(1-damping_ratio^2));

% Settling time (usually 2% or 5%)
t_s_2per = 4/(damping_ratio*w_n); % --> For 2% criterion
t_s_5per = 3/(damping_ratio*w_n); % --> For 5% criterion

% We shall see that the maximum overshoot and the rise time conflict with
% each other!!

% Bandwidth is related to natural frequency by
bandwith = w_n*sqrt((1-2*damping_ratio^2)+sqrt(4*damping_ratio^4-4*damping_ratio^2+2));

% Desired Phase margin
PM = atan(2*damping_ratio/(sqrt(-2*damping_ratio^2+sqrt(1+4*damping_ratio^4))));
PM_approx = 100*damping_ratio; % Approximation for phase margin
PM_deg = rad2deg(PM);

% % GM, PM , Phase and gain crossover frequency
% [Gm,pm,wcp,wcg] = margin(sys);
% GmdB = 20*log10(Gm);
% results = [GmdB pm wcp wcg];

%% Initial PID Design with Sisotool
sisotool(Plant)

Pid_tf = tf([0.07483 0.8525 1.388],[1 0]);
[Kp,Ki,Kd] = piddata(Pid_tf);
K4 = Pid_tf;
%% Controller with Loop Shaping
K1 = 0.5;
K2 = tf([K1 2*K1],[1 0]);
K3 = K2* tf([0.05 1],[0.005 1]);


Controller_names = ["K1","K2","K3","PID"];
Controllers = [K1 K2 K3 K4];
%% Simulation for controllers and inputs
block_path = strcat(model_name,'/Constant1');

% for i = 1:4
%     % Results for the first controller
%     set_param(block_path,'Value',string(i));
%     sim(model_name,sim_time);
%     saveas(gcf,'K1_Controller.png')
%     
% end

Prefilter = tf([1],[0.14 1]);

%%  NYQUIST PLOT FOR K1
    L = Plant*K1;
    h = nyquistplot(L);
    zoomcp(h)
    % Sensitivity
    S = 1/(1 + L);
    margin(S)
    saveas(gcf,'Sensitivity K1')
    
    %Complementary sensitivity function
    T = (1/(1+L))*L;
    margin(T)
    saveas(gcf,'Complementary Sensitivity K1')
    
    KS = K1*S;
    margin(KS)
    saveas(gcf,'KS for K1')
    
    SGd = Controllers(i)*(Gd_term*H);
    margin(SGd)
    saveas(gcf,'SGd for K1')
    
%% NYQUIST PLOT FOR K2
    L = Plant*K2;
    h = nyquistplot(L);
    zoomcp(h);
    
    % Sensitivity
    S = 1/(1 + L);
    margin(S)
    saveas(gcf,'Sensitivity K2')
    
    %Complementary sensitivity function
    T = (1/(1+L))*L;
    margin(T)
    saveas(gcf,'Complementary Sensitivity K2')
    
    KS = Controllers(i)*S;
    margin(KS)
    saveas(gcf,'KS for K2')
    
    SGd = Controllers(i)*(Gd_term*H);
    margin(SGd)
    saveas(gcf,'SGd for K2')
%% NYQUIST PLOT FOR K3
    L = Plant*K3;
    h = nyquistplot(L);
    zoomcp(h);
    
    % Sensitivity
    S = 1/(1 + L);
    margin(S)
    saveas(gcf,'Sensitivity K3')
    
    %Complementary sensitivity function
    T = (1/(1+L))*L;
    margin(T)
    saveas(gcf,'Complementary Sensitivity K3')
    
    
    KS = Controllers(i)*S;
    margin(KS)
    saveas(gcf,'KS for K3')
    
    SGd = Controllers(i)*(Gd_term*H);
    margin(SGd)
    saveas(gcf,'SGd for K3')
%% NYQUIST PLOT FOR pid
    L = Plant*Prefilter*K4;
    h = nyquistplot(L);
    zoomcp(h);
    
    % Sensitivty
    S = 1/(1 + L);
    margin(S)
    saveas(gcf,'Sensitivity PID')
    
    %Complementary sensitivity function
    T = (1/(1+L))*L;
    margin(T)
    saveas(gcf,'Complementary Sensitivity PID')
    
    KS = Controllers(i)*S;
    margin(KS)
    saveas(gcf,'KS for PID')
    
    SGd = Controllers(i)*(Gd_term*H);
    margin(SGd)
    saveas(gcf,'SGd for PID')
%%
 
for i = 1:length(Controllers)
%% Sensitivty
    if i==4 % for pid there is a prefilter
        L = Plant*Prefilter*Controllers(i);
        S = 1/(1 + L); % Sensitivity 
    else
        L = Plant.*Controllers(i);
        S = 1/(1 + L); % Sensitivity 
    end
    margin(S)
    saveas(gcf,Controller_names(i))
%% Complementary sensitivity function
    T = (1/(1+L))*L;
    margin(T)
    hold all
    %saveas(gcf,strcat(Controller_names(i),'_T'))

    KS = Controllers(i)*S;
    margin(KS)
    saveas(gcf,strcat(Controller_names(i),'KS'))
    
    SGd = Controllers(i)*(Gd_term*H);
    margin(SGd)
    saveas(gcf,strcat(Controller_names(i),'SGd'))

end
% Your first thought might be to reduce overall gain of the open-loop
% system. This would bring the gain curve down and increase margin.


%% MATLAB program to synthesize H_inf controller

clc;clear;close all;
s = tf('s');
% Uses the Mu-toolbox
G=nd2sys(1,conv([10 1],conv([0.05 1],[0.05 1])),200); % Plant is G.
Gd = 100/(10*s + 1);
Gd = nd2sys(1, cell2mat(Gd.Denominator),100); % Disturbance plant is Gdp.
% M=1.5; wb=10; A=1.e-4; Wp = nd2sys([1/M wb], [1 wb*A]); Wu = 1; % Weights.
M=1.5; wb=10; A=1.e-4; Ws = nd2sys([1/M wb], [1 wb*A]); Wks = 1; % Weights.

%Creating the generalized plant P
systemnames = 'G Gd Ws Wks';
inputvar = '[d(1); r(1); u(1)]'; %all inputs are scalar, r(2) would be a 2dim signal
outputvar = '[Ws; Wks; r-G-Gd]';
input_to_G = '[u]';
input_to_Gd = '[d]';
input_to_Ws = '[r-G-Gd]';
input_to_Wks = '[u]';
sysoutname = 'P';
cleanupsysic = 'yes';
sysic

%
% Find H-infinity optimal controller:
%
nmeas=1; nu=1; gmn=0.5; gmx=20; tol=0.001;
[K,CL,gopt] = hinfsyn(P,nmeas,nu,gmn,gmx,tol);

w = logspace(-4,6,50);
CLw = vsvd(frsp(CL,w));
figure(1); vplot('liv,m',CLw);
title('singular values of weighted closed loop system');

%generate typical transfer matrices
[type,out,in,n] = minfo(G);
I = eye(out);
S = minv(madd(I,mmult(G,K))); %sensitivity
T = msub(I,S); %complementary sensitivity
KS = mmult(K,S); %input to G
GK = mmult(G,K); %loop transfer function

%singular values as a function of frequency
Sw = vsvd(frsp(S,w));
Tw = vsvd(frsp(T,w));
Kw = vsvd(frsp(K,w));
KSw = vsvd(frsp(KS,w));
GKw = vsvd(frsp(GK,w));

%Plot singular value plots
%Note: if desired, you can change vplot to plot the amplitude in dB. Type
%edit vplot and uncomment the appropriate lines in the code
figure(2); vplot('liv,lm',Sw,'-',Tw,'--',GKw,'-.');
title('\sigma(S(jw)) (solid), \sigma(T(jw)) (dashed) and \sigma(GK(jw)) (dashdot)');
xlabel('Frequency [rad/sec]'); ylabel('Amplitude')
figure(3); vplot('liv,lm',Kw);
title('\sigma(K(jw))');
xlabel('Frequency [rad/sec]'); ylabel('Amplitude')

%Did we get what we asked for?
Sd = minv(Ws); Sdw = vsvd(frsp(Sd,w)); %"desired" sensitivity
KSd = minv(Wks); KSdw = vsvd(frsp(KSd,w)); %"desired" output
figure(4); vplot('liv,lm',Sw,'-',Sdw,'--');
title('\sigma(S(jw)) (solid) and \sigma(Ws^{-1}(jw)) (dashed)');
xlabel('Frequency [rad/sec]'); ylabel('Amplitude')
figure(5); vplot('liv,lm',KSw,'-',KSdw,'--')
title('\sigma(KS(jw)) (solid) and \sigma(Wks^{-1}(jw)) (dashed)');
xlabel('Frequency [rad/sec]'); ylabel('Amplitude')


%Finally the step response
reference = 1; tfinal = 1; step = 0.01;
y = trsp(T,reference,tfinal,step);
u = trsp(KS,reference,tfinal,step);
figure(6); subplot(2,1,1); vplot('iv,d',y);
title('Step response'); ylabel('y');
subplot(2,1,2); vplot('iv,d',u);
ylabel('u'); xlabel('time');

%% Perturb all plant parameters by 10% 
% https://www.mathworks.com/help/robust/ref/ureal.html
p1 = ureal('p1',200,'Percentage',10); 
p2 = ureal('p2',0.025,'Percentage',10);
p3 = ureal('p3',1.002,'Percentage',10);
p4 = ureal('p4',10.1,'Percentage',10);
p5 = ureal('p5',1,'Percentage',10);

uncertain_plant = tf([p1],[p2 p3 p4 p5]);

%% Controller K1
xx = feedback(K1*uncertain_plant,1);
nyquist(xx)
saveas(gcf,'Uncertainty for K1 - Nyquist')
step(xx)
saveas(gcf,'Uncertainty for K1 - Step')

%% Controller K2
yy = feedback(K2*uncertain_plant,1);
nyquist(yy)
saveas(gcf,'Uncertainty for K2 - Nyquist')
step(yy)
saveas(gcf,'Uncertainty for K2 - Step')

%% Controller K3

yy = feedback(K3*uncertain_plant,1);
nyquist(yy)
saveas(gcf,'Uncertainty for K3 - Nyquist')
step(yy)
saveas(gcf,'Uncertainty for K3 - Step')

%% Controller PID

zz = feedback(K4*Prefilter*uncertain_plant,1);
nyquist(zz)
saveas(gcf,'Uncertainty for PID - Nyquist')
step(zz)
saveas(gcf,'Uncertainty for PID - Step')