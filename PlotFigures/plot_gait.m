function plot_gait(sim_paths, ref_path, legend_names, varargin)

addpath('C:\Users\nml-p\Documents\Projects\PredSim\PreProcessing\');

if nargin > 3
    save_path = varargin{1};
    file_type = varargin{2};
    plot_reference = varargin{3};
else
    plot_reference = true;
end

% Load reference data if provided
load(ref_path, 'ref');

% Load first simulation data
load(sim_paths{1}, 'R');

% Define reference and simulation parameters
variables = {'kinematics', 'kinetics'};
cmap = parula(length(sim_paths));

% Plot reference data
for c = variables
    if plot_reference
        t_ref = ref.(c{:}).GC_percent;
        dofs = intersect(R.colheaders.coordinates, ref.colheaders.coordinates);
    else
        dofs = sort(R.colheaders.coordinates);
    end

    if strcmp(c, {'kinematics'})
        dofs(contains(dofs, {'_l', 'arm_', 'elbow_'})) = [];
    else
        dofs(contains(dofs, {'_l', 'pelvis', 'arm_', 'elbow_'})) = [];
    end

    fig = figure('Name',c{:},'Position',get(0,'ScreenSize'));
    tiledlayout('flow');

    if plot_reference
        for i = 1:length(dofs)
            nexttile;

            % Get experimental mean and sd
            idx = strcmp(ref.colheaders.coordinates, dofs{i});
            Y = ref.(c{:}).mean(:, idx);
            SD = ref.(c{:}).std(:, idx);

            % Use fill function to draw
            x_long = [t_ref fliplr(t_ref)];
            y_long = [(Y - 2*SD)' flipud(Y + 2*SD)'];
            fill(x_long, y_long, 'k', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
            hold on;

            % Plot average
            plot(t_ref, Y, 'LineStyle', '-', 'Color', [0 0 0 0.5], 'LineWidth', 0.5);
        end
    end

    % Plot simulation data on top
    for i = 1:length(sim_paths)
        % Load correct simulation data
        load(sim_paths{i}, 'R');

        if plot_reference
            t_sim = linspace(ref.(c{:}).GC_percent(1), ref.(c{:}).GC_percent(end), size(R.kinematics.Qs, 1));
        else
            t_sim = linspace(0, 100, size(R.kinematics.Qs, 1));
        end

        for j = 1:length(dofs)
            nexttile(j);
            % Plot simulation
            if strcmp(c, 'kinematics')
                idx = strcmp(R.colheaders.coordinates, dofs{j});
                plot(t_sim, R.kinematics.Qs(:, idx), 'LineStyle', '-', 'Color', cmap(i, :), 'LineWidth', 1);
            else
                idx = strcmp(R.colheaders.coordinates, dofs{j});
                plot(t_sim, R.kinetics.T_ID(:, idx), 'LineStyle', '-', 'Color', cmap(i, :), 'LineWidth', 1);
            end
            hold on;

            % Annotate
            title(replace(dofs{j}, '_', ' '));
            xlabel('Cycle (%)');
        end
        lns = get(gca, 'Children');
        p(i) = lns(1);
    end
    leg = legend(p(:), legend_names, 'Orientation', 'Horizontal', 'Interpreter', 'latex');
    leg.Layout.Tile = 'south';

    % Save plot
    if exist('save_path', 'var')
        exportgraphics(fig, fullfile(save_path, [c{:} '.' file_type]));
        close(fig);
    end
end

% GRFs
fig = figure('Name', 'GRF','Position',get(0,'ScreenSize'));
tiledlayout('flow');

dofs = ref.colheaders.grf;
dofs(contains(dofs, {'time'})) = [];

if plot_reference
    t_ref = ref.GRF.GC_percent;
    for i = 1:length(dofs)
        nexttile;

        % Get experimental data
        idx = strcmp(ref.colheaders.grf, dofs{i});
        Y = ref.GRF.mean(:, idx);
        SD = ref.GRF.std(:, idx);

        % Use fill function to draw
        x_long = [t_ref fliplr(t_ref)];
        y_long = [(Y - 2*SD)' flipud(Y + 2*SD)'];
        fill(x_long, y_long, 'k', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
        hold on;

        % Plot average
        plot(t_ref, Y, 'LineStyle', '-', 'Color', [0 0 0 0.5], 'LineWidth', 0.5);
    end
end

% Plot simulation data on top
for i = 1:length(sim_paths)
    % Load correct simulation data
    load(sim_paths{i}, 'R');

    if plot_reference
        t_sim = linspace(ref.GRF.GC_percent(1), ref.GRF.GC_percent(end), size(R.ground_reaction.GRF_r, 1));
    else
        t_sim = linspace(0, 100, size(R.ground_reaction.GRF_r, 1));
    end

    for j = 1:length(dofs)
        nexttile(j);
        tmp = strsplit(dofs{j}, '_');
        fld = strjoin(tmp([1 end]), '_');
        idx = strcmp(R.colheaders.GRF, ref.mapping.grf.(strjoin(tmp(1:2), '_')));

        % Plot simulation
        plot(t_sim, R.ground_reaction.(fld)(:, idx), 'LineStyle', '-', 'Color', cmap(i, :), 'LineWidth', 1);
        hold on;

        % Annotate
        title(replace(dofs{j}, '_', ' '));
        xlabel('Cycle (%)');
    end
    lns = get(gca, 'Children');
    p(i) = lns(1);
end
leg = legend(p(:), legend_names, 'Orientation', 'Horizontal', 'Interpreter', 'latex');
leg.Layout.Tile = 'south';

% Save plot
if exist('save_path', 'var')
    exportgraphics(fig, fullfile(save_path, ['GRF.' file_type]));
    close(fig);
end

% Excitation
fig = figure('Name', 'a','Position',get(0,'ScreenSize'));
tiledlayout('flow');

if plot_reference
    muscles = fieldnames(ref.mapping.emg);
    st_ratio = getSlowTwitchRatios(muscles);
    t_ref = ref.EMG.GC_percent;

    load(sim_paths{1}, 'R');

    for i = 1:length(muscles)
        nexttile;

        % Get experimental data
        idx = find(strcmp(ref.colheaders.muscles, ref.mapping.emg.(muscles{i})), 1);
        Y_nsc = ref.EMG.mean(:, idx);
        SD_nsc = ref.EMG.std(:, idx);
        ref_max = max(Y_nsc);

        if R.S.multifibre.use_multifibre_muscles
            m_idx = reshape(1:length(R.colheaders.muscles)*2, R.S.multifibre.NFibre, [])';
            a_weighted = R.muscles.a(:, m_idx(idx, 1)) .* st_ratio(i) +  ...
                R.muscles.a(:, m_idx(idx, 2)) .* (1 - st_ratio(i));
            sim_max = max(a_weighted);
        else
            sim_max = max(R.muscles.a(:, idx));
        end

        % Scale emg
        emg_scale = sim_max / ref_max;
        Y = Y_nsc .* emg_scale;
        SD = SD_nsc .* emg_scale;

        % Use fill function to draw
        x_long = [t_ref fliplr(t_ref)];
        y_long = [(Y - 2*SD)' flipud(Y + 2*SD)'];
        fill(x_long, y_long, 'k', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
        hold on;

        % Plot average
        plot(t_ref, Y, 'LineStyle', '-', 'Color', [0 0 0 0.5], 'LineWidth', 0.5);
    end
end

if ~plot_reference
    load(sim_paths{1}, 'R');
    muscles = sort(R.colheaders.muscles);
    st_ratio = getSlowTwitchRatios(muscles);
end

% Plot simulation data on top
for i = 1:length(sim_paths)
    % Load correct simulation data
    if ~(i == 1)
        load(sim_paths{i}, 'R');
    end

    if plot_reference
    t_sim = linspace(ref.EMG.GC_percent(1), ref.EMG.GC_percent(end), size(R.muscles.a, 1));
    else
        t_sim = linspace(0, 100, size(R.muscles.a, 1));
    end

    for j = 1:length(muscles)
        nexttile(j);
        idx = find(strcmp(R.colheaders.muscles, muscles{j}));

        % Calculate weighted average
        if R.S.multifibre.use_multifibre_muscles
            m_idx = reshape(1:length(R.colheaders.muscles)*2, R.S.multifibre.NFibre, [])';
            if isfield(R.S, 'param_shift')
                if isfield(R.S.param_shift, 'slow_to_fast')
                    Y = R.muscles.a(:, m_idx(idx, 1)) .* (st_ratio(i)*R.S.param_shift.slow_to_fast) +  ...
                        R.muscles.a(:, m_idx(idx, 2)) .* (1 - (st_ratio(i))*R.S.param_shift.slow_to_fast);
                else
                    Y = R.muscles.a(:, m_idx(idx, 1)) .* (1 - (1 - st_ratio(i))*R.S.param_shift.fast_to_slow) +  ...
                        R.muscles.a(:, m_idx(idx, 2)) .* (1 - (1 - (1 - st_ratio(i))*R.S.param_shift.fast_to_slow));
                end
            else
                Y = R.muscles.a(:, m_idx(idx, 1)) .* st_ratio(i) +  ...
                        R.muscles.a(:, m_idx(idx, 2)) .* (1 - st_ratio(i));
            end
        else
            Y = R.muscles.a(:, idx);
        end

        % Plot simulation
        plot(t_sim, Y, 'LineStyle', '-', 'Color', cmap(i, :), 'LineWidth', 1);
        hold on;

        % Annotate
        title(replace(muscles{j}, '_', ' '));
        xlabel('Cycle (%)');
        ylim([0 1]);
    end
    lns = get(gca, 'Children');
    p(i) = lns(1);
end

leg = legend(p(:), legend_names, 'Orientation', 'Horizontal', 'Interpreter', 'latex');
leg.Layout.Tile = 'south';

% Save plot
if exist('save_path', 'var')
    exportgraphics(fig, fullfile(save_path, ['activations.' file_type]));
    close(fig);
end

end