function PrepareCyclingModel()

import org.opensim.modeling.*;

% Load model
model = Model('LaiArnold2017_lowerBody_scaled.osim');
s = model.initSystem();

% We use experimental data from Lai at al. 2018 to determine the distances
% from the crank centre to the point of force application under the pedals
mrks = MarkerData('..\..\ReferenceData\Files_fSub03\static_2013_11_12_130603_001_3d_replace.trc');
frame = mrks.getFrame(0);
mrks.getNumMarkers();
for i = 0:mrks.getNumMarkers()-1
    marker_name = string(mrks.getMarkerNames().get(i));
    idx = mrks.getMarkerIndex(marker_name);
    markers.(marker_name) = osimVec3ToArray(frame.getMarker(idx)) ./ 1000;
end
crank_ctr = mean([markers.lCrankCtr; markers.rCrankCtr]);

% pedals
l_pedal_global = mean([markers.lCrankDist; markers.lPedalAxis]);
r_pedal_global = mean([markers.rCrankDist; markers.rPedalAxis]);
ground = model.getGround();
calcn_l = model.getBodySet().get('calcn_l');
calcn_r = model.getBodySet().get('calcn_r');
pedal_calcn_tmp = mean([osimVec3ToArray(ground.findStationLocationInAnotherFrame(s, osimVec3FromArray(l_pedal_global), calcn_l)); ...
    osimVec3ToArray(ground.findStationLocationInAnotherFrame(s, osimVec3FromArray(r_pedal_global), calcn_r))]);
l_pedal_calcn = [pedal_calcn_tmp(1) pedal_calcn_tmp(2) 0];
r_pedal_calcn = [pedal_calcn_tmp(1) pedal_calcn_tmp(2) 0];

% We also use the experimental data to calculate hte crank length
tmp_l = osimVec3ToArray(calcn_l.findStationLocationInGround(s, osimVec3FromArray(l_pedal_calcn))) - crank_ctr;
tmp_r = osimVec3ToArray(calcn_r.findStationLocationInGround(s, osimVec3FromArray(r_pedal_calcn))) - crank_ctr;
foot_crank_dist_xy = mean([norm(tmp_l(1:2)) norm(tmp_r(1:2))]);
foot_crank_dist_z = mean([abs(tmp_l(3)) abs(tmp_r(3))]);
save('crank_parameters.mat', 'l_pedal_calcn', 'r_pedal_calcn', 'foot_crank_dist_xy','foot_crank_dist_z');

% We fix the pelvis COM to the seat by prescribing a constant pelvis
% position. 
pelvis = model.getBodySet().get('pelvis');
pelvis_in_ground = osimVec3ToArray(pelvis.findStationLocationInGround(s, Vec3(0))) - crank_ctr;

ground_pelvis = CustomJoint.safeDownCast(model.updJointSet().get('ground_pelvis'));
translation_1 = ground_pelvis.updSpatialTransform().upd_translation1();
x_fun = Constant(pelvis_in_ground(1));
translation_1.setFunction(x_fun);

translation_2 = ground_pelvis.updSpatialTransform().upd_translation2();
y_fun = Constant(pelvis_in_ground(2));
translation_2.setFunction(y_fun);

translation_3 = ground_pelvis.updSpatialTransform().upd_translation3();
z_fun = Constant(0);
translation_3.setFunction(z_fun);

tol = 1e-2;
ground_pelvis.getCoordinate(3).set_default_value(pelvis_in_ground(1));
ground_pelvis.getCoordinate(4).set_default_value(pelvis_in_ground(2));
ground_pelvis.getCoordinate(5).set_default_value(0);

model.finalizeConnections();
model.initSystem();
model.print('LaiArnold_2017.osim');

end