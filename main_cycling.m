%% Predictive Simulations of Human Gait

% This script starts the predictive simulation of human movement. The
% required inputs are necessary to start the simulations. Optional inputs,
% if left empty, will be taken from getDefaultSettings.m.

clear
close all
clc
% path to the repository folder
[pathRepo,~,~] = fileparts(mfilename('fullpath'));
% path to the folder that contains the repository folder
[pathRepoFolder,~,~] = fileparts(pathRepo);

%% Initialize S
addpath(fullfile(pathRepo,'DefaultSettings'))

[S] = initializeSettings('LaiArnold_TwistLimit');

%% Settings

% name of the subject
S.subject.name = 'LaiArnold_TwistLimit';

% path to folder where you want to store the results of the OCP
% S.misc.save_folder  = fullfile(pathRepoFolder,'PredSimResults',[S.subject.name '_multifibre'],'tact_sensitivity');
S.misc.save_folder  = fullfile(pathRepoFolder,'PredSimResults',[S.subject.name]);

% either choose "quasi-random" or give the path to a .mot file you want to use as initial guess
% TODO: THE INITIAL GUESS WILL LIKELY HAVE TO BE UPDATED TO EITHER A "HOT
% START" OR "QUASI-RANDOM" CYCLING GUESS.
% S.solver.IG_selection = fullfile(S.misc.main_path,'OCP','IK_Guess_Full_cycling.mot');
S.solver.IG_selection = fullfile(S.misc.main_path,'OCP','IK_Guess_Full_cycling_Lai.mot'); 
S.solver.IG_selection_gaitCyclePercent = 100;
% S.solver.IG_selection = 'quasi-random';

% Set options for multi motor unit (MMU) muscle model
S.multifibre.use_multifibre_muscles = false;
S.multifibre.NFibre = 2;
S.multifibre.vMmax_range = [6 10]; % Range of max contraction velocities as multiple of optimal fibre lengths
% S.multifibre.tact_range = [0.01 0.02]; % Range of activation time constants 
S.multifibre.beta = 0.6; % deactivation time constants are given by tact * (1 / beta).
S.multifibre.smeta = linspace(1.5, 2.5, S.multifibre.NFibre); % most efficient to least

% Set options for parameter shifts
% fibre_type_shift = 0.5;
% S.param_shift.slow_to_fast = fibre_type_shift;
% S.param_shift.fast_to_slow = fibre_type_shift;

% Set cost functional weights
% S.weights.a = 1000; % Reduced to half of the original cost, as this weight is multiplied by a sum over twice as many activations.

% Set number of threads
S.solver.N_threads = 6;

S.misc.gaitmotion_type = 'HalfGaitCycle'; % FullGaitCycle

% Visualize bounds
% S.misc.visualize_bounds = true;

% give the path to the osim model of your subject
osim_path = fullfile(pathRepo,'Subjects',S.subject.name,[S.subject.name '.osim']);

% Run simulations as batch jobs, such that multiple simulations can run at
% the same time.
S.solver.run_as_batch_job = false;

S.misc.task = 'cycling';
% S.misc.forward_velocity = 4.5;

% S.metabolicE.model = 'MinettiAlexander';

% S.bounds.t_final.lower = 0.01;
S.misc.default_msk_geom_bounds = 'default_msk_geom_bounds_cycling.csv';
% S.bounds.default_coordinate_bounds = 'Running_Coordinate_Bounds.csv';
S.bounds.default_coordinate_bounds = 'LaiArnold_Cycling_Coordinate_Bounds.csv';
S.subject.default_coord_lim_torq_coeff = 'default_coord_lim_torq_coeff_Lai.csv';
S.OpenSimADOptions.verbose_mode = true;

% TODO: ADD DEFAULTS TO DEFAULT SETTING FUNCTION
S.cycling.rpm = 80;
S.bounds.FPedal.lower = -1000; % N
S.bounds.FPedal.upper = 1000; % N
S.bounds.alpha_crank.lower = -100;
S.bounds.alpha_crank.upper = 100;

% S.cycling.min_crank_omega = -0.1;
S.cycling.power = 200; % watt
% 
S.cycling.tol_omega = 1e-2;
gear_ratio = 52/17;
S.cycling.tau_eff = -1*(2.125 * gear_ratio + 1.379*1e-4 * gear_ratio^3 * S.cycling.rpm^2);
S.cycling.I_eff = 3.456 * 1e-3 + 10.442 * gear_ratio^2;

% S.post_process.load_prev_opti_vars = true;
% S.post_process.rerun = true;
% S.misc.result_filename = 'gait1018_cycling_v78';

% S.misc.msk_geom_n_samples = 2500;
% S.misc.threshold_lMT_fit = 0.01;
% S.misc.threshold_dM_fit = 0.01;
% S.misc.poly_order.upper = 5;
% S.misc.poly_order.lower = 3;

S.solver.run_as_batch_job = false;

%% Run predictive simulations
if S.solver.run_as_batch_job
    
    if strcmp(S.subject.name, 'LaiArnold2D')
        % Parameters
        m_shift = [0.9, 1.0, 1.10, 1.20 1.30];

        % Muscle passive stiffness shift
        S.misc.save_folder  = fullfile(pathRepoFolder,'PredSimResults',[S.subject.name],'quadStiffnessShift');
        for i = 1:size(m_shift, 1)
            S.subject.muscle_pass_stiff_shift = {{'rect_fem','vas_lat', 'vas_med', 'vas_int'},m_shift(i)};
            [savename] = runPredSim(S, osim_path);
        end
    else
        [savename] = runPredSim(S, osim_path);
    end
    
else
    [savename] = runPredSim(S, osim_path);
end

%% Plot results
% see .\PlotFigures\run_this_file_to_plot_figures.m for more

if ~S.solver.run_as_batch_job

    % set path to reference result
    % result_paths{1} = fullfile(pathRepo,'Tests','ReferenceResults',...
    %     'Falisse_et_al_2022','Falisse_et_al_2022_paper.mat');
    
    % set path to saved result
    result_paths{1} = fullfile(S.misc.save_folder,[savename '.mat']);
    reference_path = ...
        'C:\Users\nml-p\Documents\Projects\PredictiveCycling\ReferenceData\Dick_et_al_2016\PredSimReference\ReferenceData_80rpm_200w.mat';

    % Cell array with legend name for each result
    legend_names = {'single fibre'};

    % add path to subfolder with plotting functions
    addpath(fullfile(pathRepo,'PlotFigures'))

    figure_settings(1).name = 'lMtilde';
    figure_settings(1).dofs = {'muscles_r'};
    figure_settings(1).variables = {'lMtilde'};
    figure_settings(1).savepath = [];
    figure_settings(1).filetype = {};

    figure_settings(2).name = 'FT';
    figure_settings(2).dofs = {'muscles_r'};
    figure_settings(2).variables = {'FT'};
    figure_settings(2).savepath = [];
    figure_settings(2).filetype = {};

    figure_settings(3).name = 'Fpass';
    figure_settings(3).dofs = {'muscles_r'};
    figure_settings(3).variables = {'Fpass'};
    figure_settings(3).savepath = [];
    figure_settings(3).filetype = {};

    % % call plotting function
    plot_figures(result_paths, legend_names, figure_settings);
    plot_cycling(result_paths, reference_path, legend_names);

end
