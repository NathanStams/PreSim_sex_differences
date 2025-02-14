function plot_cycling(sim_paths, ref_path, legend_names, varargin)

if nargin > 3
    save_path = varargin{1};
    file_type = varargin{2};
end

% Load reference data
load(ref_path, 'R');
ref = R;

% Load first simulation data
load(sim_paths{1}, 'R');

% Define reference and simulation parameters
variables = {'Qs', 'pedals'};
cmap = hsv(length(sim_paths));
t_ref = 0:100;
t_sim = linspace(0, 100, size(R.kinematics.Qs, 1));

% Plot reference data
for c = variables
    if strcmp(c, 'Qs')
        dofs = intersect(R.colheaders.coordinates, fieldnames(ref.Qs));
    else
        dofs = intersect(fieldnames(R.pedal_reaction), fieldnames(ref.pedals));
    end

    fig = figure('Name',c{:});
    tiledlayout();

    for i = 1:length(dofs)
        nexttile;

        % Get experimental mean and sd
        Y = ref.(c{:}).(dofs{i}).female_avg;
        SD = ref.(c{:}).(dofs{i}).female_sd;

        if contains(dofs(i), {'knee', 'velocity'})
            Y = Y * -1;
        end

        % Use fill function to draw
        x_long = [t_ref fliplr(t_ref)];
        y_long = [(Y - SD)' flipud(Y + SD)'];
        fill(x_long, y_long, 'k', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
        hold on;

        % Plot average
        plot(t_ref, Y, 'LineStyle', '-', 'Color', [0 0 0 0.5], 'LineWidth', 0.5);

        % Annotate
        title(replace(dofs{i}, '_', ' '));
        xlabel('Cycle (%)');
    end

    % Plot simulation data on top
    for i = 1:length(sim_paths)
        % Load correct simulation data
        if ~(i == 1)
            load(sim_paths{i}, 'R');
        end

        for j = 1:length(dofs)
            nexttile(j);
            % Plot simulation
            idx = strcmp(R.colheaders.coordinates, dofs{j});
            if strcmp(c, 'Qs')
                plot(t_sim, R.kinematics.Qs(:, idx), 'LineStyle', '-', 'Color', cmap(i, :), 'LineWidth', 1);
            else
                plot(t_sim, R.pedal_reaction.(dofs{j}), 'LineStyle', '-', 'Color', cmap(i, :), 'LineWidth', 1);
            end
            
        end
        lns = get(gca, 'Children');
        p(i) = lns(1);
    end
    leg = legend(p(:), legend_names, 'Orientation', 'Horizontal');
    leg.Layout.Tile = 'south';

    % Save plot
    if exist('save_path', 'var')
        exportgraphics(fig, fullfile(save_path, [c{:} '.' file_type]));
        close(fig);
    end
end
end