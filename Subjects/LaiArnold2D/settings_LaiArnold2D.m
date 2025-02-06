% --------------------------------------------------------------------------
% Settings for LaiArnold_2017 that deviate from the PredSim defaults
%
% Original author: Lars D'Hondt
% Original date: 16/July/2024
% Last edited by: Torstein Daehlin
% Last edit on: 09/Jan/2025
% --------------------------------------------------------------------------

% Subject name
S.subject.name = 'LaiArnold2D';

% Solver options
S.solver.N_meshes = 50;
S.solver.tol_ipopt = 3;
S.solver.constr_viol_tol_ipopt = 5;

% Metabolic cost options
S.metabolicE.tanh_b = 100;

% The model has no arms, so set arm base to empty
S.subject.base_joints_arms = []; 

% Achilles tendon stiffness
S.subject.tendon_stiff_scale = {{'soleus','lat_gas', 'med_gas'},0.5};

% Muscle passive stiffness shift
% S.subject.muscle_pass_stiff_scale = {{'rect_fem'},0.75,{'vas_lat', 'vas_med', 'vas_int'},0.85};

tmp = mfilename('fullpath');
[f_path, ~, ~] = fileparts(tmp);
load(fullfile(f_path,'crank_parameters.mat'));
tol = 0.005; % tolerance for play between foot and pedal

% Define the point at which force is applied to the pedal
S.bounds.points(1).body = 'calcn_r';
S.bounds.points(1).point_in_body = r_pedal_calcn;
S.bounds.points(1).name = 'pedal_r';

S.bounds.points(2).body = 'calcn_l';
S.bounds.points(2).point_in_body = l_pedal_calcn;
S.bounds.points(2).name = 'pedal_l';

% Add forces at these points
S.OpenSimADOptions.input3DBodyForces(1).body = 'calcn_r';
S.OpenSimADOptions.input3DBodyForces(1).point_in_body = r_pedal_calcn;
S.OpenSimADOptions.input3DBodyForces(1).name = 'pedal_force_r';
S.OpenSimADOptions.input3DBodyForces(1).reference_frame = 'ground';

S.OpenSimADOptions.input3DBodyForces(2).body = 'calcn_l';
S.OpenSimADOptions.input3DBodyForces(2).point_in_body = l_pedal_calcn;
S.OpenSimADOptions.input3DBodyForces(2).name = 'pedal_force_l';
S.OpenSimADOptions.input3DBodyForces(2).reference_frame = 'ground';

S.OpenSimADOptions.export3DVelocities(1).body = 'calcn_r';
S.OpenSimADOptions.export3DVelocities(1).point_in_body = r_pedal_calcn;
S.OpenSimADOptions.export3DVelocities(1).name = 'pedal_r';

S.OpenSimADOptions.export3DVelocities(2).body = 'calcn_l';
S.OpenSimADOptions.export3DVelocities(2).point_in_body = l_pedal_calcn;
S.OpenSimADOptions.export3DVelocities(2).name = 'pedal_l';

% We enforce the pedaling kinematics with 5 constraints ensuring that the
% pedal has to remain a constant distance from the crank rotation centre
% (located at the origin or the global coordinate system) in the xy
% (sagittal plane) plane and a constant distance from the crank rotation
% centre in z (the medio-lateral direction). Additionally, the left and
% right pedal are constrained to remain a 2 * crank length distance from
% each other at all times.
S.bounds.distanceConstraints(1).point1 = 'pedal_r';
S.bounds.distanceConstraints(1).point2 = 'ground';
S.bounds.distanceConstraints(1).direction = 'xy';
S.bounds.distanceConstraints(1).lower_bound = foot_crank_dist_xy - tol;
S.bounds.distanceConstraints(1).upper_bound = foot_crank_dist_xy + tol;

S.bounds.distanceConstraints(2).point1 = 'pedal_l';
S.bounds.distanceConstraints(2).point2 = 'ground';
S.bounds.distanceConstraints(2).direction = 'xy';
S.bounds.distanceConstraints(2).lower_bound = foot_crank_dist_xy - tol;
S.bounds.distanceConstraints(2).upper_bound = foot_crank_dist_xy + tol;

S.bounds.distanceConstraints(3).point1 = 'pedal_r';
S.bounds.distanceConstraints(3).point2 = 'pedal_l';
S.bounds.distanceConstraints(3).direction = 'xy';
S.bounds.distanceConstraints(3).lower_bound = 2 * foot_crank_dist_xy - tol;
S.bounds.distanceConstraints(3).upper_bound = 2 * foot_crank_dist_xy + tol;

% Remove ground contact variables, as the model has no gorund contact
% model.
S.OpenSimADOptions.exportGRFs = false;
S.OpenSimADOptions.exportSeparateGRFs = false;
S.OpenSimADOptions.exportGRMs = false;
S.OpenSimADOptions.exportContactPowers = false;

% % to prevent body segments from clipping into eachother MAY NOT BE
% NEEDED AT ALL
% S.bounds.distanceConstraints(1).point1 = 'calcn_r';
% S.bounds.distanceConstraints(1).point2 = 'calcn_l';
% S.bounds.distanceConstraints(1).direction = 'xz';
% S.bounds.distanceConstraints(1).lower_bound = 0.09;
% S.bounds.distanceConstraints(1).upper_bound = 2;
% 
% S.bounds.distanceConstraints(2).point1 = 'hand_r';
% S.bounds.distanceConstraints(2).point2 = 'femur_r';
% S.bounds.distanceConstraints(2).direction = 'xz';
% S.bounds.distanceConstraints(2).lower_bound = 0.18;
% S.bounds.distanceConstraints(2).upper_bound = 2;
% 
% S.bounds.distanceConstraints(3).point1 = 'hand_l';
% S.bounds.distanceConstraints(3).point2 = 'femur_l';
% S.bounds.distanceConstraints(3).direction = 'xz';
% S.bounds.distanceConstraints(3).lower_bound = 0.18;
% S.bounds.distanceConstraints(3).upper_bound = 2;
% 
% S.bounds.distanceConstraints(4).point1 = 'tibia_r';
% S.bounds.distanceConstraints(4).point2 = 'tibia_l';
% S.bounds.distanceConstraints(4).direction = 'xz';
% S.bounds.distanceConstraints(4).lower_bound = 0.11;
% S.bounds.distanceConstraints(4).upper_bound = 2;
% 
% S.bounds.distanceConstraints(5).point1 = 'toes_r';
% S.bounds.distanceConstraints(5).point2 = 'toes_l';
% S.bounds.distanceConstraints(5).direction = 'xz';
% S.bounds.distanceConstraints(5).lower_bound = 0.1;
% S.bounds.distanceConstraints(5).upper_bound = 2;

% S.subject.muscle_pass_stiff_shift =...
%     {{'soleus','_gas','per_','tib_','_dig_','_hal_','FDB'},0.9};
% S.subject.tendon_stiff_scale = {{'soleus','_gas'},0.5};

% S.OpenSimADOptions.export3DPositions(1).body = 'tibia_l';
% S.OpenSimADOptions.export3DPositions(1).point_in_body = [0, -0.012, 0];
% S.OpenSimADOptions.export3DPositions(1).name = 'left_shin';
% S.OpenSimADOptions.export3DPositions(2).body = 'tibia_r';
% S.OpenSimADOptions.export3DPositions(2).point_in_body = [0, -0.012, 0];
% S.OpenSimADOptions.export3DPositions(2).name = 'right_shin';
% 
% S.OpenSimADOptions.export3DVelocities(1).body = 'tibia_l';
% S.OpenSimADOptions.export3DVelocities(1).point_in_body = [0, -0.012, 0];
% S.OpenSimADOptions.export3DVelocities(1).name = 'left_shin';
% S.OpenSimADOptions.export3DVelocities(2).body = 'tibia_r';
% S.OpenSimADOptions.export3DVelocities(2).point_in_body = [0, -0.012, 0];
% S.OpenSimADOptions.export3DVelocities(2).name = 'right_shin';





