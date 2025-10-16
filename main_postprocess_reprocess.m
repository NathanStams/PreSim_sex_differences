%% Predictive Simulations of Human Gait - Reprocess

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

[S] = initializeSettings('DHondt_et_al_2024_3seg');

%% Settings

% name of the subject
S.subject.name = 'DHondt_et_al_2024_3seg';

% path to folder where you want to store the results of the OCP
S.misc.save_folder  = fullfile(pathRepoFolder,'PredSimResults',[S.subject.name '_multifibre'],'fibre_shift', ...
    'Walk_1_33_ms', 'QR_guess');
S.misc.result_filename = 'DHondt_et_al_2024_3seg_06p_type_I';

% give the path to the osim model of your subject
osim_path = fullfile(pathRepo,'Subjects',S.subject.name,[S.subject.name '.osim']);

% Run simulations as batch jobs, such that multiple simulations can run at
% the same time.
S.solver.run_as_batch_job = false;

S.post_process.rerun = 1;

%% Run predictive simulations
if S.solver.run_as_batch_job
    % fibre_type_shift = fliplr(linspace(0.1, 1, 5))';
    % fibre_type_shift = [0.775; 0.5500; 0.3250; 0.1000];
    fibre_type_shift = 0.1;
    % n_meshes = [40 50 60 75 100 125];
    % speeds = [0.8 1.33 2.25 3.0 4.5]';
    % tacts = [0.005 0.015; 0.015 0.025; 0.025 0.035; 0.025 0.045; 0.035 0.055; 0.045 0.065];
    % vmax = [0.5 0.75 1 1.25 1.5]';
    % vmax = [5 6 7 8 9]';
    % tacts = [0.05 0.015 0.025 0.035 0.045]';
    % tol = [2 3 4 5 6]';
    % w_Edot = [1 100 500 1000 2000]';
    for i = 1:size(fibre_type_shift, 1)

        % S.misc.forward_velocity = speeds(i);
        % S.solver.N_meshes = n_meshes(i);
        S.param_shift.slow_to_fast = fibre_type_shift(i);
        % S.param_shift.fast_to_slow = fibre_type_shift(i);
        % S.multifibre.tact_range = tacts(i, :);
        % S.multifibre.vMmax_range = [5 10] .* vmax(i);
        % S.misc.custom_vMmax = vmax(i);
        % S.misc.custom_tact = tacts(i);
        % S.solver.tol_ipopt = tol(i);
        % S.weights.E = w_Edot(i);

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
    
    % Cell array with legend name for each result
    legend_names = {'two_fibre'};
    
    % add path to subfolder with plotting functions
    addpath(fullfile(pathRepo,'PlotFigures'))
    
    figure_settings(1).name = 'Qs';
    figure_settings(1).dofs = {'all_coords'};
    figure_settings(1).variables = {'Qs'};
    figure_settings(1).savepath = [];
    figure_settings(1).filetype = {};

    % call plotting function
    plot_figures(result_paths, legend_names, figure_settings);

end
