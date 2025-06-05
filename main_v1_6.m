% clear;
close all;

% This is the complete working version for the descent of 747 trajectory

folderpath = "D:/Test_Shanoop/main_v1_6/"; 

if ~exist(folderpath, 'dir')
    mkdir(folderpath);
end


%% Descent formulation version 1.6

% objective function J = minimize tf
% states X = [s,u,y,v,z,w,theta,beta,utheta,upsi]
% Use chebyshev as well as fLGR as required
% Initial guess is taken from the previous solution with 
% J = 1

% Algorithm used - "interior-point"
% Solution mainly depend on the initial guess

%%

load density_data.mat
hVal = sort(hVal);
rhoVal = sort(rhoVal,'descend');
param.hVal   = [-50e3;hVal;150e3];
param.rhoVal = [1.2;rhoVal;1e-4];


load Temp_data.mat
hValT = sort(hVal(1:500:end));
TVal  = sort(TempVal(1:500:end),'descend');
param.hValT = [-50e3;hValT;150e3];
param.TVal  = [TVal(1);TVal;TVal(end)];


%%
% data   = readtable("output_descent_stage1_booster0_07012025-095802_state_Agniban_configuration_2024_3_747_R_pl5_sim_databook_v1_descent.csv");
% data1  = readtable("output_descent_stage1_booster0_07012025-095802_Agniban_configuration_2024_3_747_R_pl5_sim_databook_v1_descent (2).csv");

t  = data.t;
xI = data.x_I;
yI = data.y_I;
zI = data.z_I;
uI = data.v_x_I;
vI = data.v_y_I;
wI = data.v_z_I;

altitude = data.altitude;
MI = data1.Mass_kg_;

rend = [xI(end);yI(end);zI(end)];

tcr_pitch = data.tcr_pitch;
tcr_yaw   = data.tcr_yaw;

lpi_pitch = data.lpi_pitch;
lpi_yaw   = data.lpi_yaw;

alpha = data1.aoa_deg_;
phia  = data1.Aero_Phi;

t  = t(136800:end);
xI = xI(136800:end);
yI = yI(136800:end);
zI = zI(136800:end);
uI = uI(136800:end);
vI = vI(136800:end);
wI = wI(136800:end);

altitude = altitude(136800:end);
MI       = MI(136800:end);
tcr_pitch = tcr_pitch(136800:end);
tcr_yaw   = tcr_yaw(136800:end);

lpi_pitch = lpi_pitch(136800:end);
lpi_yaw   = lpi_yaw(136800:end);

tI = t-t(1);

%% Normalized parameters

N = 64;

wgts = clencurt(N);

[tau,D]  = Dleg(N);
tau = flip(tau);
D = -D;

%%

Tmg    =  1.86;

param.Isp = 260;
param.m0  = 3300;
param.mp  = 800;
param.Tmg = Tmg;

g = 9.8066;
param.g    = g;

param.d  = 1.3;
param.Cd = 2.2;

RE          = 2.0925741*10^7*0.3048;      % Equatorial radius  of Earth in ft
RP          = 2.0855590*10^7*0.3048;      % Polar radius in ft
Omega_p     = 7.29211*10^(-5);            % Rotation rate of Earth  in rad/s
mu          = 1.4076539*10^16*0.0283168466;%3.986004418*10^(14);        %6.67430*10^(-11);           % Gravitational constant in ft^3/s^2
J2          = 1.082639*10^(-3);           % Gravtitational parameters to consider the oblateness of Earth
J3          = -2.565*10^(-6);
J4          = -1.608*10^(-6);

param.RE       = RE;
param.RP       = RP;
param.Omega_p  = Omega_p;
param.mu       = mu;
param.J2       = J2;
param.J3       = J3;
param.J4       = J4;


	


latitude  =  11.9925331633138;          % Launch site geodetic latitude  in degress + for N and - for S
longitude =  81.9334427870699;         % Launch site inertial longitude in degrees + for E and - for W
azimuth   = -50.8202786130912;


lat  = deg2rad(latitude);
long = deg2rad(longitude);
azim = deg2rad(azimuth);

Reci2tcr = eci2tcr(lat,long);

param.r11 = Reci2tcr(1,1);param.r12 = Reci2tcr(1,2);param.r13 = Reci2tcr(1,3);
param.r21 = Reci2tcr(2,1);param.r22 = Reci2tcr(2,2);param.r23 = Reci2tcr(2,3);
param.r31 = Reci2tcr(3,1);param.r32 = Reci2tcr(3,2);param.r33 = Reci2tcr(3,3);


%% Initial conditions

% AT SEPARATION

% x0 =  980022.402487852;
% y0 =  6170139.12507793;
% z0 =  1485713.21750536;
% u0 = -975.172827354291;
% v0 =  1266.21109402094;
% w0 = -440.345017221349; 
% 
% 
% 
% theta0 = deg2rad(-46.0936385899027);
% beta0  = deg2rad(-0.375526003769536	);

% AT APOGEE

x0 =  833851.625810815;
y0 =  6256501.20700102;
z0 =  1405256.11339669;
u0 = -1157.58050143751;
v0 =  0.745626331808077;
w0 = -733.289655957345; 

	
theta1 = deg2rad(154.056683265998);
beta1  = deg2rad(134.808614023642);



m0  =  param.m0;

ic0 = [x0;u0;y0;v0;z0;w0;m0;theta1;beta1];

%% Soft landing conditions
		

xf =  680343.127073116;
uf = -452.298273234552;

yf =  6202633.55835012;
vf =  49.617808928807;

zf =  1316594.60372594;
wf =  0.00493060507737834;

mf  = 2500;%m0 - param.mp;
	
	



thetaF = deg2rad(90.3784445809234);
betaF  = deg2rad(156.413485089011);

icf = [xf;uf;yf;vf;zf;wf;mf;thetaF;betaF];


rIf    = sqrt(xf^2 + yf^2 + zf^2);
k     = (RE/RP)^2;
lat_cf = asin(zf./rIf);
RSf    = RE*(1 + (k-1)*sin(lat).^2).^(-0.5);

Hf = rIf - RSf;     


%% Initial Guess

t0  = 0;
tfg = tI(end);
Tcheck = tfg/2*(1+tau);

Xg = interp1(tI,xI,Tcheck);
Yg = interp1(tI,yI,Tcheck);
Zg = interp1(tI,zI,Tcheck);
Ug = interp1(tI,uI,Tcheck);
Vg = interp1(tI,vI,Tcheck);
Wg = interp1(tI,wI,Tcheck);
Mg = interp1(tI,MI,Tcheck);

thetag = deg2rad(interp1(tI,tcr_pitch,Tcheck));
betag  = deg2rad(interp1(tI,tcr_yaw,Tcheck));

uthetag = 2/tfg*D*thetag;
ubetag  = 2/tfg*D*betag;

options = optimoptions('fmincon','Display', 'iter', 'Algorithm', 'interior-point','MaxFunctionEvaluations', 5000000,'MaxIterations',50000,...
                        'FunctionTolerance',1e0,'OptimalityTolerance',1e0, ...
                        'BarrierParamUpdate','predictor-corrector','InitBarrierParam',0.01);

% options = optimoptions('fmincon', 'Algorithm', 'sqp','MaxIterations', 5000,'MaxFunctionEvaluations',5000000, 'TolFun', 1e-6, 'TolX', 1e-6, 'UseParallel', true, 'Display', 'iter');

X0 = linspace(ic0(1), icf(1), N+1)'; 
U0 = linspace(ic0(2), icf(2), N+1)';
Y0 = linspace(ic0(3), icf(3), N+1)';
V0 = linspace(ic0(4), icf(4), N+1)';
Z0 = linspace(ic0(5), icf(5), N+1)';
W0 = linspace(ic0(6),icf(6), N+1)';
M0 = linspace(ic0(7), 3000, N+1)'; 

theta0  = linspace(ic0(8),icf(8),N+1)';
utheta0 = linspace(-deg2rad(1),deg2rad(1),N+1)';

beta0   = linspace(ic0(9),icf(9),N+1)';
ubeta0  = linspace(-deg2rad(1),deg2rad(1),N+1)';



% P0   = [X0; U0; Y0; V0; Z0;W0; Mg; theta0; beta0; utheta0;ubeta0; tf]; 

P0   = [Xg; Ug; Yg; Vg; Zg;Wg; Mg; thetag; betag;uthetag;ubetag; tfg]; 

%% Bounds
 
% Lower Bound

XL = -inf*ones(N+1,1); 
UL = -inf*ones(N+1,1);
YL = -inf*ones(N+1,1);
VL = -inf*ones(N+1,1);
ZL = -inf*ones(N+1,1);
WL = -inf*ones(N+1,1);
ML = -inf*ones(N+1,1);

thetaL = deg2rad(85)*ones(N+1,1);
betaL  =  deg2rad(90)*ones(N+1,1);


% XL = Xg - abs(10/sqrt(2));%
% UL = Ug - abs(10/sqrt(2));% 
% YL = Yg - abs(10/sqrt(2));%
% VL = Vg - abs(10/sqrt(2));
% ZL = Zg - abs(10/sqrt(2));
% WL = Wg - abs(10/sqrt(2));


% thetaL  = thetag - deg2rad(sqrt(10/sqrt(2)));
% betaL   = betag  - deg2rad(sqrt(10/sqrt(2)));


uthetaL = -deg2rad(1)*ones(N+1,1);
ubetaL  = -deg2rad(0)*ones(N+1,1);
 

tfL = 0;

PL   = [XL; UL; YL; VL; ZL; WL; ML; thetaL; betaL; uthetaL; ubetaL; tfL]; 

% Upper bounds

XU =  inf*ones(N+1,1); 
UU =  inf*ones(N+1,1);%inf*ones(N+1,1);
YU =  inf*ones(N+1,1);
VU =  inf*ones(N+1,1);
ZU =  inf*ones(N+1,1);
WU =  inf*ones(N+1,1);%inf*ones(N+1,1);

MU =  inf*ones(N+1,1);
thetaU  = deg2rad(180)*ones(N+1,1);
betaU   = deg2rad(180)*ones(N+1,1);


% XU = Xg + 10/sqrt(2);
% UU = Ug + 10/sqrt(2);
% YU = Yg + 10/sqrt(2);
% VU = Vg + 10/sqrt(2);
% ZU = Zg + 10/sqrt(2);
% WU = Wg + 10/sqrt(2);

% 
% thetaU  = thetag + deg2rad(sqrt(10/sqrt(2)));
% betaU   = betag  + deg2rad(sqrt(10/sqrt(2)));

uthetaU = deg2rad(0.5)*ones(N+1,1);
ubetaU  = deg2rad(1)*ones(N+1,1);

% alpha_tU =  deg2rad(179.9)*ones(N+1,1);
% phi_AU   =  deg2rad(179.9)*ones(N+1,1);


tfU = inf;

PU   = [XU; UU; YU; VU; ZU; WU; MU; thetaU; betaU; uthetaU;ubetaU; tfU]; 



%% solving
% 
load interior_point_result1.mat
% P0 = Popt;

Jfun    = @(P) objFn(P,N,wgts);
nonlcon = @(P) nonldy(P,D,N,ic0,icf,param); 

tic
[Popt,~,flg] = fmincon(Jfun,P0,[],[],[],[],PL,PU,nonlcon,options);
toc

X  = Popt(1:N+1,1); 
U  = Popt(N+2:2*N+2);
Y  = Popt(2*N+3:3*N+3);
V  = Popt(3*N+4:4*N+4);
Z  = Popt(4*N+5:5*N+5);
W  = Popt(5*N+6:6*N+6);
M  = Popt(6*N+7:7*N+7);

theta  = Popt(7*N+8:8*N+8);   
beta   = Popt(8*N+9:9*N+9);

utheta = Popt(9*N+10:10*N+10);
ubeta  = Popt(10*N+11:11*N+11);

tf     = Popt(end);
T  = -param.Isp*g*2/tf*D*M;
Tout  = t0 + (tf - t0)/2*(1+tau);

filename = fullfile(folderpath,"main_v1_6.csv");

yout     = [X,Y,Z,U,V,W,theta,beta,utheta,ubeta];
data_opt = array2table([Tout, yout], 'VariableNames', ...
    {'Times_s', 'X_m', 'Y_m', 'Z_m', 'U_mpers', 'V_mpers', 'W_m_s','tcr_pitch_rad','tcr_yaw_rad','pitch_rate_rad','yaw_rate_rad'});


writetable(data_opt,filename);


%%

tspan  = linspace(t0,tf,500)';
thetai  = spline(Tout,theta,tspan);
uthetai = spline(Tout,utheta,tspan);

betai   = spline(Tout,beta,tspan);
ubetai  = spline(Tout,ubeta,tspan);


Xi = spline(Tout,X,tspan);
Yi = spline(Tout,Y,tspan);
Zi = spline(Tout,Z,tspan);
Ui = spline(Tout,U,tspan);
Vi = spline(Tout,V,tspan);
Wi = spline(Tout,W,tspan);


disp(["tf = ", num2str(tf)])
disp(["Mf = ", num2str(M(end))])

Cont = [];
odefun = @(t,Y) ecidynamics(t,Y,U,param);


%%
figure(1)
plot(Tout,X/1000,'o','DisplayName','X_{PS}')
hold on
plot(tspan,Xi/1000,'-.','DisplayName','X_{interp}')
plot(tI,xI/1000,'r-','DisplayName','X_{ref}')
yline(xf/1000,'k-.','DisplayName','X_{f}')
xlabel("T (s)")
ylabel("X (km)")
grid minor
ylim([(min(X)/1000)-10,(max(X)/1000)+10])
legend('Location','best')
fullf1 = fullfile(folderpath,"X.fig");
savefig(fullf1)

figure(2)
plot(Tout,Y/1000,'o','DisplayName','Y_{PS}')
hold on
plot(tspan,Yi/1000,'-.','DisplayName','Y_{interp}')
plot(tI,yI/1000,'r-','DisplayName','Y_{ref}')
yline(yf/1000,'k-.','DisplayName','Y_{f}')
xlabel("T (s)")
ylabel("Y (km)")
grid minor
ylim([(min(Y)/1000)-10,(max(Y)/1000)+10])
legend('Location','best')
fullf1 = fullfile(folderpath,"Y.fig");
savefig(fullf1)


figure(3)
plot(Tout,Z/1000,'o','DisplayName','Z_{PS}')
hold on
plot(tspan,Zi/1000,'-.','DisplayName','Z_{interp}')
plot(tI,zI/1000,'r-','DisplayName','Z_{ref}')
yline(zf/1000,'k-.','DisplayName','Z_{f}')
xlabel("T (s)")
ylabel("Z (km)")
grid minor
ylim([(min(Z)/1000)-10,(max(Z)/1000)+10])
legend('Location','best')
fullf1 = fullfile(folderpath,"Z.fig");
savefig(fullf1)


figure(4)
plot(Tout,U,'o','DisplayName','U_{PS}')
hold on
plot(tspan,Ui,'-.','DisplayName','U_{interp}')
plot(tI,uI,'r-','DisplayName','U_{ref}')
yline(uf,'k-.','DisplayName','U_{f}')
xlabel("T (s)")
ylabel("U (m/s)")
ylim([(min(uI))-50,(max(uI))+50])
grid minor
legend('Location','best')
fullf1 = fullfile(folderpath,"U.fig");
savefig(fullf1)


figure(5)
plot(Tout,V,'o','DisplayName','V_{PS}')
hold on
plot(tspan,Vi,'-.','DisplayName','V_{interp}')
plot(tI,vI,'r-','DisplayName','V_{ref}')
yline(vf,'k-.','DisplayName','V_{f}')
xlabel("T (s)")
ylabel("V (m/s)")
ylim([(min(Vi))-50,(max(Vi))+50])
grid minor
legend('Location','best')
fullf1 = fullfile(folderpath,"V.fig");
savefig(fullf1)



figure(6)
plot(Tout,W,'o','DisplayName','W_{PS}')
hold on
plot(tspan,Wi,'-.','DisplayName','W_{interp}')
plot(tI,wI,'r-','DisplayName','W_{ref}')
yline(wf,'k-.','DisplayName','W_{f}')
xlabel("T (s)")
ylabel("W (m/s)")
ylim([(min(wI))-50,(max(wI))+50])
grid minor
legend('Location','best')
fullf1 = fullfile(folderpath,"W.fig");
savefig(fullf1)



figure(7)
plot(Tout,M,'DisplayName','M')
grid minor
ylabel("M (kg)")
xlabel("t(s)")
ylim([(min(M)-50)-5,(max(M))+50])
legend('Location','best')
fullf1 = fullfile(folderpath,"M.fig");
savefig(fullf1)


figure(8)
plot(Tout,T/1000,'DisplayName',"Thrust (kN)")
grid minor
xlabel("t(s)")
ylabel("T (kN)")
ylim([(min(T)/1000)-5,(max(T)/1000)+5])
legend('Location','best')
fullf1 = fullfile(folderpath,"T.fig");
savefig(fullf1)


figure(9)
plot(Tout,rad2deg(theta),'o','DisplayName','\theta_{PS}')
hold on
plot(tspan,rad2deg(thetai),'-.','DisplayName','\theta_{interp}')
plot(tI,tcr_pitch,'r-','DisplayName','\theta_{ref}')
grid minor
xlabel("T (s)")
ylabel(" pitch (deg)")
legend('Location','best')
fullf1 = fullfile(folderpath,"theta.fig");
savefig(fullf1)


figure(10)
plot(Tout,rad2deg(beta),'*','DisplayName','\beta_{PS}')
hold on
plot(tspan,rad2deg(betai),'-.','DisplayName','\beta_{interp}')
plot(tI,tcr_yaw,'r-','DisplayName','\beta_{ref}')
grid minor
xlabel("T (s)")
ylabel(" yaw (deg)")
legend('Location','best')
fullf1 = fullfile(folderpath,"beta.fig");
savefig(fullf1)



figure(11)
plot(Tout,rad2deg(utheta),'o','DisplayName','u_{\theta_{PS}}')
hold on
plot(tspan,rad2deg(uthetai),'-.','DisplayName','u_{\theta_{interp}}')
plot(Tout,rad2deg(ubeta),'*','DisplayName','u_{\beta_{PS}}')
plot(tspan,rad2deg(ubetai),'-.','DisplayName','u_{\beta_{interp}}')
xlabel("T(s)")
ylabel("(deg/s)")
legend('Location','best')
fullf1 = fullfile(folderpath,"rates.fig");
savefig(fullf1)


figure(12)
plot3(Y/1000,X/1000,Z/1000,'o','DisplayName','PS')
hold on
plot3(Yi/1000,Xi/1000,Zi/1000,'-.','DisplayName','interp')
plot3(yf/1000,xf/1000,zf/1000,'go','DisplayName','Land')
plot3(y0/1000,x0/1000,z0/1000,'bo','DisplayName','Start')
plot3(yI/1000,xI/1000,zI/1000,'r-','DisplayName','Reference')
xlabel("X km")
ylabel("Y km")
zlabel("Z km")
legend('Location','best')
fullf1 = fullfile(folderpath,"trajectory.fig");
savefig(fullf1)

rI    = sqrt(X.^2 + Y.^2 + Z.^2);
k     = (RE/RP)^2;
lat_c = asin(Z./rI);
RS    = RE*(1 + (k-1)*sin(lat_c).^2).^(-0.5);

H = rI - RS;     

figure(13)
plot(Tout,H/1000,'b-.','DisplayName','PS')
hold on
plot(tI,altitude/1000,'r-','DisplayName','Ref')
xlabel("Time(s)")
ylabel("Height(km)")
legend()
fullf1 = fullfile(folderpath,"altitude.fig");
savefig(fullf1)


%% functions

function J = objFn(P,N,w)

M      = P(6*N+7:7*N+7);
utheta = P(9*N+10:10*N+10);
ubeta  = P(10*N+11:11*N+11);



tf = P(end);
% J = -M(end)/M(1); 
% J = -M(end) ;% 0.7*sum (utheta) + 0.3*sum(ubeta)/10;
% J = 0.7*sum (utheta) + 0.3*sum(ubeta);
% J =  0.5*tf*( sum((w*utheta.^2)) + sum((w*ubeta.^2)));
% J =  0.5*tf*( sum((w*M.^2)));% + sum((w*ubeta.^2)));
% J = 1;
J = tf;
end

function [c,ceq] = nonldy(P,D,N,ic0,icf,param)

X      = P(1:N+1,1);
U      = P(N+2:2*N+2);
Y      = P(2*N+3:3*N+3);
V      = P(3*N+4:4*N+4);
Z      = P(4*N+5:5*N+5);
W      = P(5*N+6:6*N+6);
M      = P(6*N+7:7*N+7);
theta  = P(7*N+8:8*N+8);
beta   = P(8*N+9:9*N+9);

utheta = P(9*N+10:10*N+10);
ubeta  = P(10*N+11:11*N+11);
% 
% alpha_t = P(11*N+12:12*N+12);
% phi_A   = P(12*N+13:13*N+13);

tf     = P(end); 


X0 = ic0(1);
U0 = ic0(2);
Y0 = ic0(3);
V0 = ic0(4);
Z0 = ic0(5);
W0 = ic0(6);
M0 = ic0(7);

theta0 = ic0(8);
beta0  = ic0(9); 

XF = icf(1);
UF = icf(2);
YF = icf(3);
VF = icf(4);
ZF = icf(5);
WF = icf(6);
MF = icf(7);

thetaF = icf(8);
betaF = icf(9);


Isp = param.Isp;
g    = param.g;
m0   = param.m0;

Tmax = param.Tmg*m0*g;

r11 = param.r11; r12 = param.r12; r13 = param.r13;
r21 = param.r21; r22 = param.r22; r23 = param.r23;
r31 = param.r31; r32 = param.r32; r33 = param.r33;


T1 = -(Isp*g*((2/tf)*D*M));
T2 = 0;
T3 = 0;

Tx = T2.*(r21*cos(beta) - r11*sin(beta)) + T1.*(r11*cos(beta).*cos(theta) - r31*sin(theta) + r21*sin(beta).*cos(theta)) + T3.*(r31*cos(theta) + r11*cos(beta).*sin(theta) + r21*sin(beta).*sin(theta));
Ty = T2.*(r22*cos(beta) - r12*sin(beta)) + T1.*(r12*cos(beta).*cos(theta) - r32*sin(theta) + r22*sin(beta).*cos(theta)) + T3.*(r32*cos(theta) + r12*cos(beta).*sin(theta) + r22*sin(beta).*sin(theta));
Tz = T2.*(r23*cos(beta) - r13*sin(beta)) + T1.*(r13*cos(beta).*cos(theta) - r33*sin(theta) + r23*sin(beta).*cos(theta)) + T3.*(r33*cos(theta) + r13*cos(beta).*sin(theta) + r23*sin(beta).*sin(theta));

rvec = [X,Y,Z];
gval = gravity_fn(rvec,param);

gx = gval(:,1);
gy = gval(:,2);
gz = gval(:,3);

RE = param.RE;
RP = param.RP;

rI    = sqrt(X.^2 + Y.^2 + Z.^2);
k     = (RE/RP)^2;
lat_c = asin(Z./rI);
RS    = RE*(1 + (k-1)*sin(lat_c).^2).^(-0.5);

H = rI - RS;     

hVal   = param.hVal;
rhoVal = param.rhoVal;

rho = interp1(hVal,rhoVal,H);

omega = param.Omega_p;

uA = U + omega*Y;
vA = V - omega*X;
wA = W;

ub = uA.*(r11*cos(beta).*cos(theta) - r31*sin(theta) + r21*sin(beta).*cos(theta)) + ...
     vA.*(r12*cos(beta).*cos(theta) - r32*sin(theta) + r22*sin(beta).*cos(theta)) + ...
     wA.*(r13*cos(beta).*cos(theta) - r33*sin(theta) + r23*sin(beta).*cos(theta));
vb = uA.*(r21*cos(beta) - r11*sin(beta)) + vA.*(r22*cos(beta) - r12*sin(beta)) + ...
     wA.*(r23*cos(beta) - r13*sin(beta));
wb = uA.*(r31*cos(theta) + r11*cos(beta).*sin(theta) + r21*sin(beta).*sin(theta)) + ...
     vA.*(r32*cos(theta) + r12*cos(beta).*sin(theta) + r22*sin(beta).*sin(theta)) + ...
     wA.*(r33*cos(theta) + r13*cos(beta).*sin(theta) + r23*sin(beta).*sin(theta));


% ub = wA.*(r13*cos(theta) - r11*sin(theta)) + uA.*(r11*cos(beta).*cos(theta) - r12*sin(beta) + r13*cos(beta).*sin(theta)) + vA.*(r12*cos(beta) + r11*sin(beta).*cos(theta) + r13*sin(beta).*sin(theta));
% vb = wA.*(r23*cos(theta) - r21*sin(theta)) + uA.*(r21*cos(beta).*cos(theta) - r22*sin(beta) + r23*cos(beta).*sin(theta)) + vA.*(r22*cos(beta) + r21*sin(beta).*cos(theta) + r23*sin(beta).*sin(theta));
% wb = wA.*(r33*cos(theta) - r31*sin(theta)) + uA.*(r31*cos(beta).*cos(theta) - r32*sin(beta) + r33*cos(beta).*sin(theta)) + vA.*(r32*cos(beta) + r31*sin(beta).*cos(theta) + r33*sin(beta).*sin(theta));

Vt = sqrt(ub.^2 + vb.^2 + wb.^2);

alpha_t = acos(ub./Vt);
phi_A   = atan(vb./wb);

% for j = 1:N+1
%     alpha_t(j) = max(170.001*pi/180,min(179.995*pi/180,alpha_t(j))); 
% end

hValT = param.hValT;
TVal  = param.TVal;
Temp  = interp1(hValT,TVal,H);
V_sound = sqrt(1.4*287*Temp);
Mach = Vt./V_sound;


[Ca,Cs,Cn] = aero_interp(Mach,alpha_t,phi_A);


% max(alpha_t*180/pi)
% max(Mach)
% Cd = param.Cd;
d  = param.d;
S  = pi/4*d^2;

FDbx  = -0.5*rho.*Vt.^2*S.*Ca;
FDby  =  0.5*rho.*Vt.^2*S.*Cs;
FDbz  = -0.5*rho.*Vt.^2*S.*Cn;

FDx = FDby.*(r21*cos(beta) - r11*sin(beta)) + FDbx.*(r11*cos(beta).*cos(theta) - r31*sin(theta) + r21*sin(beta).*cos(theta)) + FDbz.*(r31*cos(theta) + r11*cos(beta).*sin(theta) + r21*sin(beta).*sin(theta));
FDy = FDby.*(r22*cos(beta) - r12*sin(beta)) + FDbx.*(r12*cos(beta).*cos(theta) - r32*sin(theta) + r22*sin(beta).*cos(theta)) + FDbz.*(r32*cos(theta) + r12*cos(beta).*sin(theta) + r22*sin(beta).*sin(theta));
FDz = FDby.*(r23*cos(beta) - r13*sin(beta)) + FDbx.*(r13*cos(beta).*cos(theta) - r33*sin(theta) + r23*sin(beta).*cos(theta)) + FDbz.*(r33*cos(theta) + r13*cos(beta).*sin(theta) + r23*sin(beta).*sin(theta));



ceq(1:N+1,1)       = (2/tf)*D*X - U;
ceq(N+2:2*N+2,1)   = M.*((2/tf)*D*U - gx) - Tx - FDx;
ceq(2*N+3:3*N+3,1) = (2/tf)*D*Y - V;
ceq(3*N+4:4*N+4,1) = M.*((2/tf)*D*V - gy) - Ty - FDy;
ceq(4*N+5:5*N+5,1) = 2/tf*D*Z - W;
ceq(5*N+6:6*N+6,1) = M.*((2/tf)*D*W - gz) - Tz - FDz;

ceq(6*N+7:7*N+7)   = 2/tf*D*theta - utheta;
ceq(7*N+8:8*N+8)   = 2/tf*D*beta - ubeta;

ceq(8*N+9,1)        = X(1) - X0;
ceq(8*N+10,1)        = U(1) - U0;
ceq(8*N+11,1)        = Y(1) - Y0;
ceq(8*N+12,1)       = V(1) - V0;
ceq(8*N+13,1)       = Z(1) - Z0;
ceq(8*N+14,1)       = W(1) - W0;
 
ceq(8*N+15,1)       = M(1) - M0;

ceq(8*N+16,1)      = X(N+1) - XF;
ceq(8*N+17,1)      = U(N+1) - UF;
ceq(8*N+18,1)      = Y(N+1) - YF;
ceq(8*N+19,1)      = V(N+1) - VF;
ceq(8*N+20,1)      = Z(N+1) - ZF;
ceq(8*N+21,1)      = W(N+1) - WF;
% 
ceq(8*N+22,1)      = theta(1) - theta0;
ceq(8*N+23,1)      = beta(1)  - beta0;
ceq(8*N+24,1)      = theta(N+1) - thetaF;
ceq(8*N+25,1)      = beta(N+1)  - betaF;

% ceq(8*N+26:8*N+50,1) = T1(1:25,1) - 0;
% ceqT = (T1-Tmax).*T1; 

% ceq = [ceq;ceqT];
c(1:N+1,1)          =   -T1;
c(N+2:2*N+2,1)      =    T1   - Tmax;
c(2*N+3:3*N+2) = T1(1:end-1) - T1(2:end);

% % c = [];
c          =  [c; -M + MF];
end



function [x, D] = cheb(N)

theta = pi * (0:N) / N;
x = cos(theta)'; % nodes
c = [2; ones(N-1, 1); 2] .* (-1).^(0:N)';

X = repmat(x, 1, N+1);
dX = X - X'; 

D = (c * (1 ./ c)') ./ (dX + eye(N+1));
D = D - diag(sum(D, 2));

end


function [w] = clencurt(N)
theta = pi*(0:N)'/N; 
% x = cos(theta);
w = zeros(1,N+1); 
ii = 2:N; 
v = ones(N-1,1);
if mod(N,2)==0
    w(1) = 1/(N^2-1);
    w(N+1) = w(1);
    for k=1:N/2-1
        v = v - 2*cos(2*k*theta(ii))/(4*k^2-1); 
    end
    v = v - cos(N*theta(ii))/(N^2-1);
else
    w(1) = 1/N^2; w(N+1) = w(1);
    for k=1:(N-1)/2
        v = v - 2*cos(2*k*theta(ii))/(4*k^2-1); 
    end
end
w(ii) = 2*v/N;
end

function g_val = gravity_fn(r,param)

x = r(:,1); y = r(:,2); z = r(:,3);

mu = param.mu;
RE = param.RE;
J2 = param.J2;
J3 = param.J3;
J4 = param.J4;

r1  = sqrt(x.^2+y.^2+z.^2);
R   = RE./r1;
Z   = z./RE;
J   = 3/2*J2;
H   = 5/2*J3;
D   = 35/8*J4;





gx  = -mu*x./(r1.^3).*Pfun(z,r1,J,R,Z,H,D);
gy  = -mu*y./(r1.^3).*Pfun(z,r1,J,R,Z,H,D);
gz  = -mu./(r1.^3).*((1+J*(R.^2).*(3-5*Z.^2)).*z + H*(R.^3)./r1.*(6*z.^2 - 7*(z.^2).*Z.^2 - 3/5*(r1.^2)) + ...
                      D*(R.^4).*(15/7-10*(Z.^2) + 9*(Z.^4)).*z);

g_val = [gx,gy,gz];

end

function R = eci2lpi(phi,theta,lam)
% This function is used to convert from Earth Centered Inertial Frame to
% Launch Frame
% It takes as theta as Launch point longitude
%             phi   as Launch point latitude
%             lam   as Launch point azimuth



r11 = cos(phi)*cos(theta);
r12 = cos(phi)*sin(theta);
r13 = sin(phi);

r21 = sin(phi)*cos(theta)*sin(lam)-cos(lam)*sin(theta);
r22 = cos(lam)*cos(theta) + sin(lam)*sin(phi)*sin(theta);
r23 = -sin(lam)*cos(phi);

r31 = -sin(lam)*sin(theta) - cos(lam)*sin(phi)*cos(theta);
r32 = sin(lam)*cos(theta) - cos(lam)*sin(phi)*sin(theta);
r33 = cos(lam)*cos(phi);

R = [r11,r12,r13;r21,r22,r23;r31,r32,r33];

end

function p = Pfun(z,r,J,R,Z,H,D)

p = (1 + J*R.^2.*(1-5*Z.^2) + H*(R.^3)./r.*(3-7*Z.^2).*z + D*(R.^4).*(9*Z.^4 - 6*Z.^2 + 3/7));

end



function [x,D] = Dleg(N)


Np = N+1;
xc = cos(pi*(0:N)/N)';
xu = linspace(-1,1,Np)';

if N<3
    x = xc;
else
    x = sin(pi*xu)./(4*N) + xc;
end

xold = 2;

P = zeros(Np,Np);


while (max(abs(x-xold))> eps)
    
    xold = x;

    P(:,1) = 1;
    P(:,2) = x;

    for k = 2:N
        
        P(:,k+1) = 1/k*((2*k-1)*x.*P(:,k) - (k-1)*P(:,k-1));

    end

    x = xold - (x.*P(:,Np) - P(:,N))./(Np*P(:,Np));

end

X     = repmat(x,1,Np);
Xdiff = X - X' + eye(Np);

L                    = repmat(P(:,Np),1,Np);
L(1:Np+1:Np*Np)      = 1;

D               = L./(Xdiff.*L');
D(1:Np+1:Np*Np) = 0;
D(1,1)          =  N*Np/4;
D(Np,Np)        = -N*Np/4;

end


function dY = ecidynamics(t,Y,U,param)

x = Y(1);
y = Y(2);
z = Y(3);
u = Y(4);
v = Y(5);
w = Y(6);

tSpan      = U(:,1);
thetaSpan  = U(:,2);
betaSpan   = U(:,3);
ThrsutSpan = U(:,4);
MSpan      = U(:,5);

% alphaSpan  = U(:,6);
% phiASpan   = U(:,6);

theta = spline(tSpan,thetaSpan,t);
beta  = spline(tSpan,betaSpan,t);
phi   = 0;

% alpha_t  = spline(tSpan,alphaSpan,t);
% phi_A    = spline(tSpan,phiASpan,t);

T1 = interp1(tSpan,ThrsutSpan,t);
T2 = 0;
T3 = 0;

m  = interp1(tSpan,MSpan,t);

RE = param.RE;
RP = param.RP;

rvec = [x,y,z];

rI    = sqrt(x^2 + y^2 + z^2);
k     = (RE/RP)^2;
lat_c = asin(z/rI);
RS    = RE*(1 + (k-1)*sin(lat_c)^2)^(-0.5);

H = rI - RS;     

hVal   = param.hVal;
rhoVal = param.rhoVal;

rho = interp1(hVal,rhoVal,H);

omega = param.Omega_p;

gI   = gravity_fn(rvec,param);

gIx  = gI(1);
gIy  = gI(2);
gIz  = gI(3);

uA   = u + omega*y;
vA   = v - omega*x;
wA   = w;


r11 = param.r11; r12 = param.r12; r13 = param.r13;
r21 = param.r21; r22 = param.r22; r23 = param.r23;
r31 = param.r31; r32 = param.r32; r33 = param.r33;

Reci2tcr  = [r11,r12,r13;r21,r22,r23;r31,r32,r33];
Rtcr2body = tcr2body(phi,theta,beta);
Reci2body = Rtcr2body*Reci2tcr;
Rbody2eci = Reci2body';

VB = Reci2body*[uA;vA;wA];

ub = VB(1);
vb = VB(2);
wb = VB(3);
Vb = sqrt(sum(VB.^2));


alpha_t = acos(ub/Vb);
phi_A   = atan(vb/wb);


% alpha_t = max(170.01*pi/180,min(179.99*pi/180,alpha_t));
% rad2deg(alpha_t)

hValT   = param.hValT;
TVal    = param.TVal;
Temp    = interp1(hValT,TVal,H);
V_sound = sqrt(1.4*287*Temp);

Mach    = Vb/V_sound;


[Ca,Cs,Cn] = aero_interp(Mach,alpha_t,phi_A);

d  = param.d;
S  = pi/4*d^2;

FDbx  = -0.5*rho.*Vb^2*S*Ca;
FDby  =  0.5*rho.*Vb^2*S*Cs;
FDbz  = -0.5*rho.*Vb^2*S*Cn;

FDI   = Rbody2eci*[FDbx;FDby;FDbz];
FDIx = FDI(1);
FDIy = FDI(2);
FDIz = FDI(3);

TI   = Rbody2eci*[T1;T2;T3];
TIx  = TI(1);
TIy  = TI(2);
TIz  = TI(3);

dY(1,1) = u;
dY(2,1) = v;
dY(3,1) = w;

dY(4,1) = gIx + 1/m*( FDIx + TIx );
dY(5,1) = gIy + 1/m*( FDIy + TIy );
dY(6,1) = gIz + 1/m*( FDIz + TIz );

% if isnan(dY)==true
%     Mach
%     H
%     Temp
%     rad2deg(alpha_t)
%     rad2deg(phi_A)
% 
% end

end


