function [R] = PostProcess_pedal_force(model_info, f_casadi, R)
% --------------------------------------------------------------------------
% PostProcess_pedal_force
%   This function reorganizes the pedal forces such that they are given
%   individually for each leg
% 
% INPUT:
%   - model_info -
%   * structure with all the model information based on the OpenSim model
%
%   - f_casadi -
%   * Struct containing all casadi functions.
%
%   - R -
%   * struct with simulation results
%
% OUTPUT:
%   - R -
%   * struct with simulation results
% 
% Original author: Torstein Daehlin
% Original date: 15/Jan/2025
%
% Last edit by: 
% Last edit date: 
% --------------------------------------------------------------------------

N = size(R.Pedals.Force, 1);

import casadi.*

F  = external('F',replace(fullfile(R.S.misc.subject_path,R.S.misc.external_function),'\','/'));

Foutk_opt = zeros(N,F.nnz_out);

for i = 1:N
    % Create zero input vector for external function
    F_ext_input = zeros(model_info.ExtFunIO.input.nInputs,1);
    % Assign Qs
    F_ext_input(model_info.ExtFunIO.input.Qs.all,1) = R.kinematics.Qs_rad(i,:);
    % Assign Qdots
    F_ext_input(model_info.ExtFunIO.input.Qdots.all,1) = R.kinematics.Qdots_rad(i,:);
    % Assign Qdotdots (A)
    F_ext_input(model_info.ExtFunIO.input.Qdotdots.all,1) = R.kinematics.Qddots_rad(i,:);
    % Assign pedal forces
    F_ext_input([model_info.ExtFunIO.input.Forces.pedal_force_r ...
        model_info.ExtFunIO.input.Forces.pedal_force_l],1) = R.Pedals.Force(i, :);

    % Evaluate external function
    res = F(F_ext_input);

    Foutk_opt(i,:) = full(res);
end

% Assign pedal forces
R.pedal_reaction.force_r = R.Pedals.Force(:, 1:3);
R.pedal_reaction.force_l = R.Pedals.Force(:, 4:6);

% Assign point offorce application
R.pedal_reaction.cop_r = Foutk_opt(:, model_info.ExtFunIO.position.pedal_r);
R.pedal_reaction.cop_l = Foutk_opt(:, model_info.ExtFunIO.position.pedal_l);

% Calculate crank torque
R.pedal_reaction.crank_tau = cross(R.pedal_reaction.cop_r, ... 
   -R.pedal_reaction.force_r) + cross(R.pedal_reaction.cop_l, ... 
   -R.pedal_reaction.force_l);

% Calculate crank angular velocity
R.pedal_reaction.cop_vel_r = Foutk_opt(:, model_info.ExtFunIO.velocity.pedal_r);
R.pedal_reaction.cop_vel_l = Foutk_opt(:, model_info.ExtFunIO.velocity.pedal_l);

for i = 1:N
    R.pedal_reaction.omega_r(i, :) = cross(R.pedal_reaction.cop_r(i, :), R.pedal_reaction.cop_vel_r(i, :)) / ...
        norm(R.pedal_reaction.cop_r(i, :))^2;
    R.pedal_reaction.omega_l(i, :) = cross(R.pedal_reaction.cop_l(i, :), R.pedal_reaction.cop_vel_l(i, :)) / ...
        norm(R.pedal_reaction.cop_l(i, :))^2;    
end
R.pedal_reaction.Crank_velocity = mean([R.pedal_reaction.omega_r(:, 3) R.pedal_reaction.omega_l(:, 3)], 2);

% Calculate crank power
R.pedal_reaction.Crank_Power_total = R.pedal_reaction.crank_tau(:, 3) .* R.pedal_reaction.Crank_velocity;

% Calculate normal and tangential pedal forces
r_normal_hat = R.pedal_reaction.cop_r(:,1:2) ./ vecnorm(R.pedal_reaction.cop_r(:,1:2), 2, 2);
tmp_r = cross([zeros(size(R.pedal_reaction.Crank_velocity, 1),2) R.pedal_reaction.Crank_velocity], ...
    [R.pedal_reaction.cop_r(:,1:2) zeros(size(R.pedal_reaction.cop_r,1),1)], 2);
r_tangent_hat = tmp_r(:, 1:2) ./ vecnorm(tmp_r(:, 1:2), 2, 2);

l_normal_hat = R.pedal_reaction.cop_l(:,1:2) ./ vecnorm(R.pedal_reaction.cop_l(:,1:2), 2, 2);
tmp_l = cross([zeros(size(R.pedal_reaction.Crank_velocity, 1),2) R.pedal_reaction.Crank_velocity], ...
    [R.pedal_reaction.cop_l(:,1:2) zeros(size(R.pedal_reaction.cop_l,1),1)], 2);
l_tangent_hat = tmp_l(:, 1:2) ./ vecnorm(tmp_l(:, 1:2), 2, 2);

for i = 1:size(R.pedal_reaction.force_r, 1)
    R_r = [r_tangent_hat(i, :)' r_normal_hat(i, :)'];
    R_l = [l_tangent_hat(i, :)' l_normal_hat(i, :)'];

    F_r(i, :) = (R_r' * -R.pedal_reaction.force_r(i, 1:2)')';
    F_l(i, :) = (R_l' * -R.pedal_reaction.force_l(i, 1:2)')';
end

R.pedal_reaction.R_force_tangential = F_r(:, 1);
R.pedal_reaction.R_force_normal = F_r(:, 2);

R.pedal_reaction.L_force_tangential = F_l(:, 1);
R.pedal_reaction.L_force_normal = F_l(:, 2);

end