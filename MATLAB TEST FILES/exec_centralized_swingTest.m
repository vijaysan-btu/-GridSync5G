%% Centralized Swing Equation Test Execution Script
clc
gui = GUIRemote();

% Device IDs
sc = 1; % Supercapacitor (Conv1)
bat = 3; % Battery (Conv3)
fc = 4; % Fuel Cell (Conv4)
ge = 5; % Grid Emulator

%% Initialize System
gui.connect([sc, bat, fc, ge])
gui.load(sc, "Model_03_PR_Igrid.mldatx")
gui.load(bat, "Model_03_PR_Igrid.mldatx")
gui.load(fc, "Model_03_PR_Igrid.mldatx")
gui.load(ge, "GEL50_Model.mldatx")
gui.sdi_clear()
gui.log_delete_all

%% Set Calibration
gui.setparam(sc, "Calibration_Target_Nbr", 1)
gui.setparam(bat, "Calibration_Target_Nbr", 3)
gui.setparam(fc, "Calibration_Target_Nbr", 4)

%% Start System
gui.exec_start([sc, bat, fc, ge])
gui.go_ready([sc, bat, fc])
gui.GE_go_ready()
gui.GE_go_run()
gui.go_idle([sc, bat, fc])
gui.go_grid([sc, bat, fc])

%% Set Initial Parameters
run('parameters_swing_test.m') % Ensure Control_Strategy = 1
gui.setparam(ge, "GEL50.SwingMode", 1)
gui.setparam(ge, "GEL50.PA_Setpoint_L1", 180)
gui.setparam(ge, "GEL50.PA_Setpoint_L2", 180)
gui.setparam(ge, "GEL50.PA_Setpoint_L3", 180)
gui.setparam(ge, "GEL50.M", 0.1)
gui.setparam(ge, "GEL50.D", 0.05)
gui.setparam(ge, "GEL50.SwingPref", 10000)
gui.setparam(sc, "INV.droop.KP", 100)
gui.setparam(sc, "INV.enable_droop", 1)
gui.setparam(sc, "INV.enable_powerPI", 0)
gui.setparam(sc, "INV.Setpoint.Rate_Limiter_Iref", 50)
gui.setparam(fc, "INV.enable_powerPI", 0) % Initially disable secondary
gui.setparam(fc, "INV.Secondary_Ki", 200)
gui.setparam(bat, "INV.EL.Pref", 0)
gui.setparam(bat, "INV.Kt", 0.5)
gui.setparam(sc, "INV.U_INV_Ref_sat", 245*sqrt(2))
gui.setparam(fc, "INV.U_INV_Ref_sat", 245*sqrt(2))
gui.setparam(bat, "INV.U_INV_Ref_sat", 245*sqrt(2))

%% Test Procedure
% Step 0: Wait for stabilization
pause(2)

% Step 1: Load Change at t=5s
pause(3)
gui.setparam(bat, "INV.EL.Pref", -10000) % -10 kW load on DG3
pause(5)

% Step 2: Secondary Control at t=10s
gui.setparam(fc, "INV.enable_powerPI", 1)
pause(10)

% Step 3: Tertiary Control at t=20s
P_load = -10000;
P1 = 2000; % Approximate
P2 = 3000;
P3_star = INV.DG3.Kt * (P_load - P1 - P2);
gui.setparam(bat, "INV.EL.Pref", P3_star)
pause(10)

% Reset
gui.setparam(bat, "INV.EL.Pref", 0)
pause(2)

%% Stop System
gui.exec_stop([sc, bat, fc, ge])
disp('Centralized swing test completed.')