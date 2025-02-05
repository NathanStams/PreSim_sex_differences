function plot_cycling(sim_path, varargin)

% Parse varargin
if nargin > 1
    ref_path = varargin{:};
    load(ref_path, 'R');
    ref = R;
end

% Load simulaton data
load(sim_path, 'R');

% Plot Qs
figure('Name','Qs');
tiledlayout();
t = 1:100;
coords = R.colheaders.coordinates;
for i = 1:length(coords)
    nexttile;
    
    % Get experimental mean and sd
    Y = ref.Qs.(coords{i}).female_avg(2:end);
    SD = ref.Qs.(coords{i}).female_sd(2:end);

    if contains(coords(i), {'knee'})
        Y = Y * -1;
    end

    % Use fill function to draw
    x_long = [t fliplr(t)];
    y_long = [(Y - SD)' flipud(Y + SD)'];
    fill(x_long, y_long, 'k', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
    hold on;

    % Plot average 
    plot(t, Y, 'LineStyle', '-', 'Color', [0 0 0 0.5], 'LineWidth', 0.5);

    % Plot simulation
    idx = strcmp(coords, coords{i});
    plot(t, R.kinematics.Qs(:, idx), 'LineStyle', '-', 'Color', 'r', 'LineWidth', 1);

    % Annotate
    title(replace(coords{i}, '_', ' '));
    xlabel('Cycle (%)');
end

% Plot pedal parameters
figure('Name','Pedal params');
tiledlayout();
t = 1:100;
vars = intersect(fieldnames(R.pedal_reaction), fieldnames(ref.pedals));
for i = 1:length(vars)
    nexttile;
    
    % Get experimental mean and sd
    Y = ref.pedals.(vars{i}).female_avg(2:end);
    SD = ref.pedals.(vars{i}).female_sd(2:end);
    
    if contains(vars(i), {'velocity'})
        Y = Y * -1;
    end

    % Use fill function to draw
    x_long = [t fliplr(t)];
    y_long = [(Y - SD)' flipud(Y + SD)'];
    fill(x_long, y_long, 'k', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
    hold on;

    % Plot average 
    plot(t, Y, 'LineStyle', '-', 'Color', [0 0 0 0.5], 'LineWidth', 0.5);

    % Plot simulation
    plot(t, R.pedal_reaction.(vars{i}), 'LineStyle', '-', 'Color', 'r', 'LineWidth', 1);

    % Annotate
    title(replace(vars{i}, '_', ' '));
    xlabel('Cycle (%)');
end


end