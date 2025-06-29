%% Clean up
if ~(exist('PMS_NO_CLEAR', 'var') && PMS_NO_CLEAR)
    close all
    clear
    clc
end

%% Control Strategy Selection
% Select control strategy: 1 = Centralized, 2 = Decentralized, 3 = Distributed
Control_Strategy = 1; % Change to 2 or 3 for other strategies
Control_Strategy_Labels = ["Centralized", "Decentralized", "Distributed"];

%% Base Parameters
run('parameters/base.m')
run('parameters/signal_conditioning.m')
run('parameters/gel50.m')
run('parameters/abr_pr_no_fb.m')

%% PLL Parameters
PLL.Kp = 5;                             % Proportional gain
PLL.Ki = 500;                           % Integral gain
PLL.f_init = 50;                        % Initial frequency [Hz]
PLL.h0 = 1;
PLL.h1 = 1;
PLL.L = 30;
PLL.f0 = 50;

%% Inverter Control Options
INV_Mode = 18; % BTU_PR_OuterCurrent
INV_Mode_Labels = ["PR voltage control", "DQ voltage control", ...
    "PR current control", "DQ current control", ...
    "DC voltage control", "DC current control", ...
    "BTU-PR Voltage Control", "BTU-PR Current Control", ...
    "BTU-DQ Voltage Control", "BTU-DQ Current Control", ...
    "Droop Control (DQ VC based)", ...
    "Electric Load","BTU_DQ_InnerCurrent_Control","BTU_DQ_OuterVoltage_Control","BTU_DQ_OuterCurrent_Control",...
    "IJ_01_PR_InnerCurrent","IJ_02_PR_Voltage","IJ_03_PR_OuterCurrent"];
INV.Pfilter = 1/(2*pi*50); % Filter time constant
INV.Island_Mode = 0; % 0: PLL uses microgrid voltage, 1: internal reference

%% Load Inverter Parameters
load('generated_parameters/IJ_03_PR_OuterCurrent.mat')

%% Inverter Setpoints and Rate Limits
INV.Igrid_ref = 0;
INV.EL.Pref = 0;
INV.EL.Qref = 0;
INV.Setpoint.Rate_Limiter_Vref = 100;
INV.Setpoint.Rate_Limiter_Iref = 20;
INV.SC_Rate_Limiter_Iref = 50; % Higher for supercapacitor
InnerC.Setpoint.RateLim = 5;
InnerC.Setpoint.RateLim_V = 40;
INV.initial_vref_slope = 0.25;
INV.initial_iref_slope = 0.5;
INV.U_INV_Ref_sat = 245*sqrt(2);

%% Swing Equation Parameters (GE)
GEL50.M = 0.1; % Inertia (s^2)
GEL50.D = 0.05; % Damping (s)
GEL50.Wref = 50*2*pi; % Nominal frequency
GEL50.SwingPref = 10000; % Initial mechanical power (W)
GEL50.Fnum = [0 0.0041];
GEL50.Fden = [1.0000 -0.9959];
GEL50.WgridHz = 50;
GEL50.SwingMode = 1;

%% Control Strategy Specific Parameters
switch Control_Strategy
    case 1 % Centralized
        % DG1 (Supercapacitor) - Primary
        INV.DG1.droop.KP = 100; % Droop gain (W/Hz)
        INV.DG1.droop.omega_ref = 2*pi*50;
        INV.DG1.droop.P_ref = 0;
        INV.DG1.droop.omega_sat = 0.04*2*pi;
        INV.DG1.droop.tau_P_filt = 0.1;
        INV.DG1.enable_droop = 1;
        INV.DG1.enable_powerPI = 0;
        
        % DG2 (Fuel Cell) - Secondary
        INV.DG2.Secondary_Kp = 0;
        INV.DG2.Secondary_Ki = 200; % W/s
        INV.DG2.P_ref = 0;
        INV.DG2.enable_powerPI = 1;
        INV.DG2.enable_droop = 0;
        AUX.DG2.Secondary_PI = tf([INV.DG2.Secondary_Ki],[1 0]);
        AUX.DG2.Secondary_d_PI = c2d(AUX.DG2.Secondary_PI, Tsc, 'zoh');
        INV.DG2.Secondary_Num = AUX.DG2.Secondary_d_PI.Numerator{1};
        INV.DG2.Secondary_Den = AUX.DG2.Secondary_d_PI.Denominator{1};
        
        % DG3 (Battery) - Tertiary
        INV.DG3.Kt = 0.5; % Proportional gain
        INV.DG3.P_ref = 0;
        INV.DG3.enable_powerPI = 0;
        INV.DG3.enable_droop = 0;
        
    case 2 % Decentralized
        % DG1 (Supercapacitor) - Primary
        INV.DG1.droop.KP = 100; % Same droop gain
        INV.DG1.droop.omega_ref = 2*pi*50;
        INV.DG1.droop.P_ref = 0;
        INV.DG1.droop.omega_sat = 0.04*2*pi;
        INV.DG1.droop.tau_P_filt = 0.1;
        INV.DG1.enable_droop = 1;
        INV.DG1.enable_powerPI = 0;
        
        % DG2 (Fuel Cell) - Secondary
        INV.DG2.Secondary_Kp = 0;
        INV.DG2.Secondary_Ki = 200; % Local PI control
        INV.DG2.P_ref = 0;
        INV.DG2.enable_powerPI = 1;
        INV.DG2.enable_droop = 0;
        AUX.DG2.Secondary_PI = tf([INV.DG2.Secondary_Ki],[1 0]);
        AUX.DG2.Secondary_d_PI = c2d(AUX.DG2.Secondary_PI, Tsc, 'zoh');
        INV.DG2.Secondary_Num = AUX.DG2.Secondary_d_PI.Numerator{1};
        INV.DG2.Secondary_Den = AUX.DG2.Secondary_d_PI.Denominator{1};
        
        % DG3 (Battery) - Tertiary
        INV.DG3.Kt = 0.5; % Local estimation
        INV.DG3.P_ref = 0;
        INV.DG3.enable_powerPI = 0;
        INV.DG3.enable_droop = 0;
        
    case 3 % Distributed
        % DG1 (Supercapacitor) - Primary
        INV.DG1.droop.KP = 100; % Consensus-adjusted droop
        INV.DG1.droop.omega_ref = 2*pi*50;
        INV.DG1.droop.P_ref = 0;
        INV.DG1.droop.omega_sat = 0.04*2*pi;
        INV.DG1.droop.tau_P_filt = 0.1;
        INV.DG1.enable_droop = 1;
        INV.DG1.enable_powerPI = 0;
        INV.DG1.consensus_Kp = 50; % Consensus gain
        INV.DG1.consensus_Ki = 100;
        
        % DG2 (Fuel Cell) - Secondary
        INV.DG2.Secondary_Kp = 0;
        INV.DG2.Secondary_Ki = 200;
        INV.DG2.P_ref = 0;
        INV.DG2.enable_powerPI = 1;
        INV.DG2.enable_droop = 0;
        INV.DG2.consensus_Kp = 50;
        INV.DG2.consensus_Ki = 100;
        AUX.DG2.Secondary_PI = tf([200],[1 0]);
        AUX.DG2.Secondary_d_I = c2d(AUX.DG2.Secondary_PI, Tsc, 'zoh');
        INV.DG2.Secondary_Num = AUX.DG2.Secondary_d_PI.Numerator{1};
        INV.DG2.Secondary_Den = AUX.DG2.Secondary_d_PI.Denominator{1};
        
        % DG3 (Battery) - Tertiary
        INV.DG3.Kt = 0.5;
        INV.DG3.P_ref = 0;
        INV.DG3.enable_powerPI = 0;
        INV.DG3.enable_droop = 0;
        INV.DG3.consensus_Kp = 50;
        INV.DG3.consensus_Ki = 100;
end

%% Display Configuration
disp('Parameter script finished!')
disp(['Control Strategy: ' Control_Strategy_Labels(Control_Strategy)])
disp(['INV_Mode: ' num2str(INV_Mode) ' (' INV_Mode_Labels(INV_Mode) ')'])