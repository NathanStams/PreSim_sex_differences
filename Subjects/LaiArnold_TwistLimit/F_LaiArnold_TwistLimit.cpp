#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/WeldJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/PlanarJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/Joint.h>
#include <OpenSim/Simulation/SimbodyEngine/SpatialTransform.h>
#include <OpenSim/Simulation/SimbodyEngine/CustomJoint.h>
#include <OpenSim/Common/LinearFunction.h>
#include <OpenSim/Common/PolynomialFunction.h>
#include <OpenSim/Common/MultiplierFunction.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Simulation/Model/SmoothSphereHalfSpaceForce.h>
#include "SimTKcommon/internal/recorder.h"

#include <iostream>
#include <iterator>
#include <random>
#include <cassert>
#include <algorithm>
#include <vector>
#include <fstream>

using namespace SimTK;
using namespace OpenSim;

constexpr int n_in = 2; 
constexpr int n_out = 1; 
constexpr int nCoordinates = 15; 
constexpr int NX = nCoordinates*2; 
constexpr int NU = 21; 
constexpr int NR = 39; 

template<typename T> 
T value(const Recorder& e) { return e; }; 
template<> 
double value(const Recorder& e) { return e.getValue(); }; 

SimTK::Array_<int> getIndicesOSInSimbody(const Model& model) { 
	auto s = model.getWorkingState(); 
	const auto svNames = model.getStateVariableNames(); 
	SimTK::Array_<int> idxOSInSimbody(s.getNQ()); 
	s.updQ() = 0; 
	for (int iy = 0; iy < s.getNQ(); ++iy) { 
		s.updQ()[iy] = SimTK::NaN; 
		const auto svValues = model.getStateVariableValues(s); 
		for (int isv = 0; isv < svNames.size(); ++isv) { 
			if (SimTK::isNaN(svValues[isv])) { 
				s.updQ()[iy] = 0; 
				idxOSInSimbody[iy] = isv/2; 
				break; 
			} 
		} 
	} 
	return idxOSInSimbody; 
} 

SimTK::Array_<int> getIndicesSimbodyInOS(const Model& model) { 
	auto idxOSInSimbody = getIndicesOSInSimbody(model); 
	auto s = model.getWorkingState(); 
	SimTK::Array_<int> idxSimbodyInOS(s.getNQ()); 
	for (int iy = 0; iy < s.getNQ(); ++iy) { 
		for (int iyy = 0; iyy < s.getNQ(); ++iyy) { 
			if (idxOSInSimbody[iyy] == iy) { 
				idxSimbodyInOS[iy] = iyy; 
				break; 
			} 
		} 
	} 
	return idxSimbodyInOS; 
} 

template<typename T>
int F_generic(const T** arg, T** res) {

	// Definition of model.
	OpenSim::Model* model;
	model = new OpenSim::Model();

	// Definition of bodies.
	OpenSim::Body* pelvis;
	pelvis = new OpenSim::Body("pelvis", 40.84270236909712536999, Vec3(-0.07070017454502323939, 0.00000000000000000000, 0.00000000000000000000), Inertia(0.09192125803640867343, 0.07788270014563418442, 0.05177277082011733783, 0., 0., 0.));
	model->addBody(pelvis);

	OpenSim::Body* femur_r;
	femur_r = new OpenSim::Body("femur_r", 8.05003614489175234326, Vec3(0.00000000000000000000, -0.16816138016093473695, 0.00000000000000000000), Inertia(0.11339261465182806643, 0.02972427762717822983, 0.11957458692186798133, 0., 0., 0.));
	model->addBody(femur_r);

	OpenSim::Body* tibia_r;
	tibia_r = new OpenSim::Body("tibia_r", 3.07371144883765357037, Vec3(0.00000000000000000000, -0.18205354288323521983, 0.00000000000000000000), Inertia(0.03973032389964631372, 0.00402033039460706742, 0.04028213395380807194, 0., 0., 0.));
	model->addBody(tibia_r);

	OpenSim::Body* talus_r;
	talus_r = new OpenSim::Body("talus_r", 0.08156671971912461683, Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), Inertia(0.00076720249394416165, 0.00076720249394416165, 0.00076720249394416165, 0., 0., 0.));
	model->addBody(talus_r);

	OpenSim::Body* calcn_r;
	calcn_r = new OpenSim::Body("calcn_r", 1.01958399648905762014, Vec3(0.09698364543529712500, 0.02909509363058913542, 0.00000000000000000000), Inertia(0.00107408349152182604, 0.00299208972638222983, 0.00314553022517106290, 0., 0., 0.));
	model->addBody(calcn_r);

	OpenSim::Body* toes_r;
	toes_r = new OpenSim::Body("toes_r", 0.17667351491162391897, Vec3(0.03355634132061280361, 0.00581901872611782691, -0.01697213795117699653), Inertia(0.00007672024939441617, 0.00015344049878883233, 0.00076720249394416165, 0., 0., 0.));
	model->addBody(toes_r);

	OpenSim::Body* femur_l;
	femur_l = new OpenSim::Body("femur_l", 8.05003614489175234326, Vec3(0.00000000000000000000, -0.16816138016093473695, 0.00000000000000000000), Inertia(0.11339261465182806643, 0.02972427762717822983, 0.11957458692186798133, 0., 0., 0.));
	model->addBody(femur_l);

	OpenSim::Body* tibia_l;
	tibia_l = new OpenSim::Body("tibia_l", 3.07371144883765357037, Vec3(0.00000000000000000000, -0.18205354288323521983, 0.00000000000000000000), Inertia(0.03973032389964631372, 0.00402033039460706742, 0.04028213395380807194, 0., 0., 0.));
	model->addBody(tibia_l);

	OpenSim::Body* talus_l;
	talus_l = new OpenSim::Body("talus_l", 0.08156671971912461683, Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), Inertia(0.00076720249394416165, 0.00076720249394416165, 0.00076720249394416165, 0., 0., 0.));
	model->addBody(talus_l);

	OpenSim::Body* calcn_l;
	calcn_l = new OpenSim::Body("calcn_l", 1.01958399648905762014, Vec3(0.09698364543529712500, 0.02909509363058913542, 0.00000000000000000000), Inertia(0.00107408349152182604, 0.00299208972638222983, 0.00314553022517106290, 0., 0., 0.));
	model->addBody(calcn_l);

	OpenSim::Body* toes_l;
	toes_l = new OpenSim::Body("toes_l", 0.17667351491162391897, Vec3(0.03355634132061280361, 0.00581901872611782691, 0.01697213795117699653), Inertia(0.00007672024939441617, 0.00015344049878883233, 0.00076720249394416165, 0., 0., 0.));
	model->addBody(toes_l);

	// Definition of joints.
	SpatialTransform st_ground_pelvis;
	st_ground_pelvis[0].setCoordinateNames(OpenSim::Array<std::string>("pelvis_tilt", 1, 1));
	st_ground_pelvis[0].setFunction(new LinearFunction(1.0000, 0.0000));
	st_ground_pelvis[0].setAxis(Vec3(0.00000000000000000000, 0.00000000000000000000, 1.00000000000000000000));
	st_ground_pelvis[1].setCoordinateNames(OpenSim::Array<std::string>("pelvis_list", 1, 1));
	st_ground_pelvis[1].setFunction(new LinearFunction(1.0000, 0.0000));
	st_ground_pelvis[1].setAxis(Vec3(1.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));
	st_ground_pelvis[2].setCoordinateNames(OpenSim::Array<std::string>("pelvis_rotation", 1, 1));
	st_ground_pelvis[2].setFunction(new LinearFunction(1.0000, 0.0000));
	st_ground_pelvis[2].setAxis(Vec3(0.00000000000000000000, 1.00000000000000000000, 0.00000000000000000000));
	st_ground_pelvis[3].setFunction(new Constant(-0.16418680684476588683));
	st_ground_pelvis[3].setAxis(Vec3(1.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));
	st_ground_pelvis[4].setFunction(new Constant(0.79243081323333364185));
	st_ground_pelvis[4].setAxis(Vec3(0.00000000000000000000, 1.00000000000000000000, 0.00000000000000000000));
	st_ground_pelvis[5].setFunction(new Constant(0.00000000000000000000));
	st_ground_pelvis[5].setAxis(Vec3(0.00000000000000000000, 0.00000000000000000000, 1.00000000000000000000));
	OpenSim::CustomJoint* ground_pelvis;
	ground_pelvis = new OpenSim::CustomJoint("ground_pelvis", model->getGround(), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), *pelvis, Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), st_ground_pelvis);

	SpatialTransform st_hip_r;
	st_hip_r[0].setCoordinateNames(OpenSim::Array<std::string>("hip_flexion_r", 1, 1));
	st_hip_r[0].setFunction(new LinearFunction(1.0000, 0.0000));
	st_hip_r[0].setAxis(Vec3(0.00000000000000000000, 0.00000000000000000000, 1.00000000000000000000));
	st_hip_r[1].setCoordinateNames(OpenSim::Array<std::string>("hip_adduction_r", 1, 1));
	st_hip_r[1].setFunction(new LinearFunction(1.0000, 0.0000));
	st_hip_r[1].setAxis(Vec3(1.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));
	st_hip_r[2].setCoordinateNames(OpenSim::Array<std::string>("hip_rotation_r", 1, 1));
	st_hip_r[2].setFunction(new LinearFunction(1.0000, 0.0000));
	st_hip_r[2].setAxis(Vec3(0.00000000000000000000, 1.00000000000000000000, 0.00000000000000000000));
	st_hip_r[3].setFunction(new MultiplierFunction(new Constant(0.00000000000000000000), 1.00000246881220999917));
	st_hip_r[3].setAxis(Vec3(1.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));
	st_hip_r[4].setFunction(new MultiplierFunction(new Constant(0.00000000000000000000), 1.00000246881220999917));
	st_hip_r[4].setAxis(Vec3(0.00000000000000000000, 1.00000000000000000000, 0.00000000000000000000));
	st_hip_r[5].setFunction(new MultiplierFunction(new Constant(0.00000000000000000000), 1.00000246881220999917));
	st_hip_r[5].setAxis(Vec3(0.00000000000000000000, 0.00000000000000000000, 1.00000000000000000000));
	OpenSim::CustomJoint* hip_r;
	hip_r = new OpenSim::CustomJoint("hip_r", *pelvis, Vec3(-0.05627613893487593000, -0.07849019377707036615, 0.07726019074043133372), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), *femur_r, Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), st_hip_r);

	SpatialTransform st_knee_r;
	st_knee_r[0].setCoordinateNames(OpenSim::Array<std::string>("knee_angle_r", 1, 1));
	st_knee_r[0].setFunction(new LinearFunction(1.0000, 0.0000));
	st_knee_r[0].setAxis(Vec3(1.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));
	st_knee_r[1].setCoordinateNames(OpenSim::Array<std::string>("knee_angle_r", 1, 1));
	osim_double_adouble st_knee_r_1_coeffs[5] = {0.00842757634497734789, -0.01572010903931060632, -0.04459471668396831900, 0.08388252170605378644, -0.00037447878753134366}; 
	Vector st_knee_r_1_coeffs_vec(5); 
	for (int i = 0; i < 5; ++i) st_knee_r_1_coeffs_vec[i] = st_knee_r_1_coeffs[i]; 
	st_knee_r[1].setFunction(new PolynomialFunction(st_knee_r_1_coeffs_vec));
	st_knee_r[1].setAxis(Vec3(0.00000000000000000000, 0.00000000000000000000, 1.00000000000000000000));
	st_knee_r[2].setCoordinateNames(OpenSim::Array<std::string>("knee_angle_r", 1, 1));
	osim_double_adouble st_knee_r_2_coeffs[5] = {0.00014284849657449241, 0.02460491089280504931, -0.16879029504402268125, 0.36921976672147305276, 0.00002186407554714539}; 
	Vector st_knee_r_2_coeffs_vec(5); 
	for (int i = 0; i < 5; ++i) st_knee_r_2_coeffs_vec[i] = st_knee_r_2_coeffs[i]; 
	st_knee_r[2].setFunction(new PolynomialFunction(st_knee_r_2_coeffs_vec));
	st_knee_r[2].setAxis(Vec3(0.00000000000000000000, 1.00000000000000000000, 0.00000000000000000000));
	st_knee_r[3].setCoordinateNames(OpenSim::Array<std::string>("knee_angle_r", 1, 1));
	osim_double_adouble st_knee_r_3_coeffs[5] = {0.00015803856484491590, -0.00100644153573099717, 0.00179943851112354149, 0.00002658709108871718, -0.00000113235441923643}; 
	Vector st_knee_r_3_coeffs_vec(5); 
	for (int i = 0; i < 5; ++i) st_knee_r_3_coeffs_vec[i] = st_knee_r_3_coeffs[i]; 
	st_knee_r[3].setFunction(new PolynomialFunction(st_knee_r_3_coeffs_vec));
	st_knee_r[3].setAxis(Vec3(1.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));
	st_knee_r[4].setCoordinateNames(OpenSim::Array<std::string>("knee_angle_r", 1, 1));
	osim_double_adouble st_knee_r_4_coeffs[5] = {-0.00079387949836120771, 0.00591995082528921912, -0.01248032446903473563, 0.00441721030263595923, -0.00008092206665916439}; 
	Vector st_knee_r_4_coeffs_vec(5); 
	for (int i = 0; i < 5; ++i) st_knee_r_4_coeffs_vec[i] = st_knee_r_4_coeffs[i]; 
	st_knee_r[4].setFunction(new PolynomialFunction(st_knee_r_4_coeffs_vec));
	st_knee_r[4].setAxis(Vec3(0.00000000000000000000, 1.00000000000000000000, 0.00000000000000000000));
	st_knee_r[5].setCoordinateNames(OpenSim::Array<std::string>("knee_angle_r", 1, 1));
	osim_double_adouble st_knee_r_5_coeffs[5] = {0.00097816246130743786, -0.00353648552100712180, -0.00049772968246897102, 0.00667245656764641794, -0.00005946764123302973}; 
	Vector st_knee_r_5_coeffs_vec(5); 
	for (int i = 0; i < 5; ++i) st_knee_r_5_coeffs_vec[i] = st_knee_r_5_coeffs[i]; 
	st_knee_r[5].setFunction(new PolynomialFunction(st_knee_r_5_coeffs_vec));
	st_knee_r[5].setAxis(Vec3(0.00000000000000000000, 0.00000000000000000000, 1.00000000000000000000));
	OpenSim::CustomJoint* knee_r;
	knee_r = new OpenSim::CustomJoint("knee_r", *femur_r, Vec3(-0.00445133065131885983, -0.40517000772893452254, -0.00173107303106844563), Vec3(-1.64156999999999997364, 1.44618000000000002103, 1.57079999999999997407), *tibia_r, Vec3(-0.00788821326885723838, -0.00344682840591140766, -0.00144778884445878218), Vec3(-1.64156999999999997364, 1.44618000000000002103, 1.57079999999999997407), st_knee_r);

	OpenSim::PinJoint* ankle_r;
	ankle_r = new OpenSim::PinJoint("ankle_r", *tibia_r, Vec3(-0.00975112709604902123, -0.39004508384196084902, 0.00000000000000000000), Vec3(0.17589499999999999580, -0.10520799999999999597, 0.01866220000000000032), *talus_r, Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), Vec3(0.17589499999999999580, -0.10520799999999999597, 0.01866220000000000032));

	OpenSim::PinJoint* subtalar_r;
	subtalar_r = new OpenSim::PinJoint("subtalar_r", *talus_r, Vec3(-0.04729892387879440496, -0.04068463926010713883, 0.00768110471847553190), Vec3(-1.76818999999999992845, 0.90622300000000000075, 1.81960000000000010623), *calcn_r, Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), Vec3(-1.76818999999999992845, 0.90622300000000000075, 1.81960000000000010623));

	OpenSim::WeldJoint* mtp_r;
	mtp_r = new OpenSim::WeldJoint("mtp_r", *calcn_r, Vec3(0.17340675803831123725, -0.00193967290870594245, 0.00104742337070120890), Vec3(-3.14158999999999988262, 0.61990100000000003533, 0.00000000000000000000), *toes_r, Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), Vec3(-3.14158999999999988262, 0.61990100000000003533, 0.00000000000000000000));

	SpatialTransform st_hip_l;
	st_hip_l[0].setCoordinateNames(OpenSim::Array<std::string>("hip_flexion_l", 1, 1));
	st_hip_l[0].setFunction(new LinearFunction(1.0000, 0.0000));
	st_hip_l[0].setAxis(Vec3(0.00000000000000000000, 0.00000000000000000000, 1.00000000000000000000));
	st_hip_l[1].setCoordinateNames(OpenSim::Array<std::string>("hip_adduction_l", 1, 1));
	st_hip_l[1].setFunction(new LinearFunction(-1.0000, 0.0000));
	st_hip_l[1].setAxis(Vec3(1.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));
	st_hip_l[2].setCoordinateNames(OpenSim::Array<std::string>("hip_rotation_l", 1, 1));
	st_hip_l[2].setFunction(new LinearFunction(-1.0000, 0.0000));
	st_hip_l[2].setAxis(Vec3(0.00000000000000000000, 1.00000000000000000000, 0.00000000000000000000));
	st_hip_l[3].setFunction(new MultiplierFunction(new Constant(0.00000000000000000000), 1.00000246881220999917));
	st_hip_l[3].setAxis(Vec3(1.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));
	st_hip_l[4].setFunction(new MultiplierFunction(new Constant(0.00000000000000000000), 1.00000246881220999917));
	st_hip_l[4].setAxis(Vec3(0.00000000000000000000, 1.00000000000000000000, 0.00000000000000000000));
	st_hip_l[5].setFunction(new MultiplierFunction(new Constant(0.00000000000000000000), 1.00000246881220999917));
	st_hip_l[5].setAxis(Vec3(0.00000000000000000000, 0.00000000000000000000, 1.00000000000000000000));
	OpenSim::CustomJoint* hip_l;
	hip_l = new OpenSim::CustomJoint("hip_l", *pelvis, Vec3(-0.05627613893487593000, -0.07849019377707036615, -0.07726019074043133372), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), *femur_l, Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), st_hip_l);

	SpatialTransform st_knee_l;
	st_knee_l[0].setCoordinateNames(OpenSim::Array<std::string>("knee_angle_l", 1, 1));
	st_knee_l[0].setFunction(new LinearFunction(1.0000, 0.0000));
	st_knee_l[0].setAxis(Vec3(-1.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));
	st_knee_l[1].setCoordinateNames(OpenSim::Array<std::string>("knee_angle_l", 1, 1));
	osim_double_adouble st_knee_l_1_coeffs[5] = {0.00842757634497734789, -0.01572010903931060632, -0.04459471668396831900, 0.08388252170605378644, -0.00037447878753134366}; 
	Vector st_knee_l_1_coeffs_vec(5); 
	for (int i = 0; i < 5; ++i) st_knee_l_1_coeffs_vec[i] = st_knee_l_1_coeffs[i]; 
	st_knee_l[1].setFunction(new PolynomialFunction(st_knee_l_1_coeffs_vec));
	st_knee_l[1].setAxis(Vec3(0.00000000000000000000, 0.00000000000000000000, 1.00000000000000000000));
	st_knee_l[2].setCoordinateNames(OpenSim::Array<std::string>("knee_angle_l", 1, 1));
	osim_double_adouble st_knee_l_2_coeffs[5] = {-0.00014284849657449241, -0.02460491089280504931, 0.16879029504402268125, -0.36921976672147305276, -0.00002186407554714539}; 
	Vector st_knee_l_2_coeffs_vec(5); 
	for (int i = 0; i < 5; ++i) st_knee_l_2_coeffs_vec[i] = st_knee_l_2_coeffs[i]; 
	st_knee_l[2].setFunction(new PolynomialFunction(st_knee_l_2_coeffs_vec));
	st_knee_l[2].setAxis(Vec3(0.00000000000000000000, 1.00000000000000000000, 0.00000000000000000000));
	st_knee_l[3].setCoordinateNames(OpenSim::Array<std::string>("knee_angle_l", 1, 1));
	osim_double_adouble st_knee_l_3_coeffs[5] = {0.00015803856484491590, -0.00100644153573099717, 0.00179943851112354149, 0.00002658709108871718, -0.00000113235441923643}; 
	Vector st_knee_l_3_coeffs_vec(5); 
	for (int i = 0; i < 5; ++i) st_knee_l_3_coeffs_vec[i] = st_knee_l_3_coeffs[i]; 
	st_knee_l[3].setFunction(new PolynomialFunction(st_knee_l_3_coeffs_vec));
	st_knee_l[3].setAxis(Vec3(1.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));
	st_knee_l[4].setCoordinateNames(OpenSim::Array<std::string>("knee_angle_l", 1, 1));
	osim_double_adouble st_knee_l_4_coeffs[5] = {-0.00079387949836120771, 0.00591995082528921912, -0.01248032446903473563, 0.00441721030263595923, -0.00008092206665916439}; 
	Vector st_knee_l_4_coeffs_vec(5); 
	for (int i = 0; i < 5; ++i) st_knee_l_4_coeffs_vec[i] = st_knee_l_4_coeffs[i]; 
	st_knee_l[4].setFunction(new PolynomialFunction(st_knee_l_4_coeffs_vec));
	st_knee_l[4].setAxis(Vec3(0.00000000000000000000, 1.00000000000000000000, 0.00000000000000000000));
	st_knee_l[5].setCoordinateNames(OpenSim::Array<std::string>("knee_angle_l", 1, 1));
	osim_double_adouble st_knee_l_5_coeffs[5] = {-0.00097816246130743786, 0.00353648552100712180, 0.00049772968246897102, -0.00667245656764641794, 0.00005946764123302973}; 
	Vector st_knee_l_5_coeffs_vec(5); 
	for (int i = 0; i < 5; ++i) st_knee_l_5_coeffs_vec[i] = st_knee_l_5_coeffs[i]; 
	st_knee_l[5].setFunction(new PolynomialFunction(st_knee_l_5_coeffs_vec));
	st_knee_l[5].setAxis(Vec3(0.00000000000000000000, 0.00000000000000000000, 1.00000000000000000000));
	OpenSim::CustomJoint* knee_l;
	knee_l = new OpenSim::CustomJoint("knee_l", *femur_l, Vec3(-0.00445133065131885983, -0.40517000772893452254, 0.00173107303106844563), Vec3(1.64156999999999997364, -1.44618000000000002103, 1.57079999999999997407), *tibia_l, Vec3(-0.00788821326885723838, -0.00344682840591140766, 0.00144778884445878218), Vec3(1.64156999999999997364, -1.44618000000000002103, 1.57079999999999997407), st_knee_l);

	OpenSim::PinJoint* ankle_l;
	ankle_l = new OpenSim::PinJoint("ankle_l", *tibia_l, Vec3(-0.00975112709604902123, -0.39004508384196084902, 0.00000000000000000000), Vec3(-0.17589499999999999580, 0.10520799999999999597, 0.01866220000000000032), *talus_l, Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), Vec3(-0.17589499999999999580, 0.10520799999999999597, 0.01866220000000000032));

	OpenSim::PinJoint* subtalar_l;
	subtalar_l = new OpenSim::PinJoint("subtalar_l", *talus_l, Vec3(-0.04729892387879440496, -0.04068463926010713883, -0.00768110471847553190), Vec3(1.76818999999999992845, -0.90622300000000000075, 1.81960000000000010623), *calcn_l, Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), Vec3(1.76818999999999992845, -0.90622300000000000075, 1.81960000000000010623));

	OpenSim::WeldJoint* mtp_l;
	mtp_l = new OpenSim::WeldJoint("mtp_l", *calcn_l, Vec3(0.17340675803831123725, -0.00193967290870594245, -0.00104742337070120890), Vec3(-3.14158999999999988262, -0.61990100000000003533, 0.00000000000000000000), *toes_l, Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), Vec3(-3.14158999999999988262, -0.61990100000000003533, 0.00000000000000000000));

	model->addJoint(ground_pelvis);
	model->addJoint(hip_r);
	model->addJoint(knee_r);
	model->addJoint(ankle_r);
	model->addJoint(subtalar_r);
	model->addJoint(mtp_r);
	model->addJoint(hip_l);
	model->addJoint(knee_l);
	model->addJoint(ankle_l);
	model->addJoint(subtalar_l);
	model->addJoint(mtp_l);

	// Initialize system.
	SimTK::State* state;
	state = new State(model->initSystem());

	// Read inputs.
	std::vector<T> x(arg[0], arg[0] + NX);
	std::vector<T> u(arg[1], arg[1] + NU);

	// States and controls.
	T ua[nCoordinates];
	Vector QsUs(NX);
	/// States
	for (int i = 0; i < NX; ++i) QsUs[i] = x[i];
	/// Controls
	/// OpenSim and Simbody have different state orders.
	auto indicesOSInSimbody = getIndicesOSInSimbody(*model);
	for (int i = 0; i < nCoordinates; ++i) ua[i] = u[indicesOSInSimbody[i]];

	// Set state variables and realize.
	model->setStateVariableValues(*state, QsUs);
	model->realizeVelocity(*state);

	// Compute residual forces.
	/// Set appliedMobilityForces (# mobilities).
	Vector appliedMobilityForces(nCoordinates);
	appliedMobilityForces.setToZero();
	/// Set appliedBodyForces (# bodies + ground).
	Vector_<SpatialVec> appliedBodyForces;
	int nbodies = model->getBodySet().getSize() + 1;
	appliedBodyForces.resize(nbodies);
	appliedBodyForces.setToZero();
	/// Set gravity.
	Vec3 gravity(0);
	gravity[0] = 0.00000000000000000000;
	gravity[1] = -9.80664999999999942304;
	gravity[2] = 0.00000000000000000000;
	/// Add weights to appliedBodyForces.
	for (int i = 0; i < model->getBodySet().getSize(); ++i) {
		model->getMatterSubsystem().addInStationForce(*state,
		model->getBodySet().get(i).getMobilizedBodyIndex(),
		model->getBodySet().get(i).getMassCenter(),
		model->getBodySet().get(i).getMass()*gravity, appliedBodyForces);
	}
	/// Add contact forces to appliedBodyForces.
	/// knownUdot.
	Vector knownUdot(nCoordinates);
	knownUdot.setToZero();
	for (int i = 0; i < nCoordinates; ++i) knownUdot[i] = ua[i];

	/// forces acting on bodies
	Vec3 Point_pedal_force_r = Vec3(0.15809175685227902508, -0.05445792902269121910, 0.00000000000000000000);
	Vec3 Force_pedal_force_r;	for (int i = 0; i < 3; ++i) Force_pedal_force_r[i] = u[15+i];
	Vec3 Force_pedal_force_r_inG = Force_pedal_force_r;
	model->getMatterSubsystem().addInStationForce(*state, calcn_r->getMobilizedBodyIndex(), Point_pedal_force_r, Force_pedal_force_r_inG, appliedBodyForces);

	Vec3 Point_pedal_force_l = Vec3(0.15809175685227902508, -0.05445792902269121910, 0.00000000000000000000);
	Vec3 Force_pedal_force_l;	for (int i = 0; i < 3; ++i) Force_pedal_force_l[i] = u[18+i];
	Vec3 Force_pedal_force_l_inG = Force_pedal_force_l;
	model->getMatterSubsystem().addInStationForce(*state, calcn_l->getMobilizedBodyIndex(), Point_pedal_force_l, Force_pedal_force_l_inG, appliedBodyForces);

	/// Calculate residual forces.
	Vector residualMobilityForces(nCoordinates);
	residualMobilityForces.setToZero();
	model->getMatterSubsystem().calcResidualForceIgnoringConstraints(*state,
			appliedMobilityForces, appliedBodyForces, knownUdot, residualMobilityForces);

	/// Station locations.
	Vec3 pedal_r_posInGround = calcn_r->findStationLocationInGround(*state, Vec3(0.15809175685227902508, -0.05445792902269121910, 0.00000000000000000000));
	Vec3 pedal_l_posInGround = calcn_l->findStationLocationInGround(*state, Vec3(0.15809175685227902508, -0.05445792902269121910, 0.00000000000000000000));
	Vec3 lat_pedal_r_posInGround = calcn_r->findStationLocationInGround(*state, Vec3(-0.02835640909808854679, -0.05445792902269121910, 0.00000000000000000000));
	Vec3 lat_pedal_l_posInGround = calcn_l->findStationLocationInGround(*state, Vec3(-0.02835640909808854679, -0.05445792902269121910, 0.00000000000000000000));
	Vec3 tibia_r_posInGround = tibia_r->findStationLocationInGround(*state, Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));
	Vec3 tibia_l_posInGround = tibia_l->findStationLocationInGround(*state, Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));

	/// Station velocities.
	Vec3 pedal_r_velInGround = calcn_r->findStationVelocityInGround(*state, Vec3(0.15809175685227902508, -0.05445792902269121910, 0.00000000000000000000));
	Vec3 pedal_l_velInGround = calcn_l->findStationVelocityInGround(*state, Vec3(0.15809175685227902508, -0.05445792902269121910, 0.00000000000000000000));

	/// Outputs.
	/// Residual forces (OpenSim and Simbody have different state orders).
	auto indicesSimbodyInOS = getIndicesSimbodyInOS(*model);
	for (int i = 0; i < nCoordinates; ++i) res[0][i] =
			value<T>(residualMobilityForces[indicesSimbodyInOS[i]]);
	for (int i = 0; i < 3; ++i) res[0][i + nCoordinates + 0] = value<T>(pedal_r_posInGround[i]);
	for (int i = 0; i < 3; ++i) res[0][i + nCoordinates + 3] = value<T>(pedal_l_posInGround[i]);
	for (int i = 0; i < 3; ++i) res[0][i + nCoordinates + 6] = value<T>(lat_pedal_r_posInGround[i]);
	for (int i = 0; i < 3; ++i) res[0][i + nCoordinates + 9] = value<T>(lat_pedal_l_posInGround[i]);
	for (int i = 0; i < 3; ++i) res[0][i + nCoordinates + 12] = value<T>(tibia_r_posInGround[i]);
	for (int i = 0; i < 3; ++i) res[0][i + nCoordinates + 15] = value<T>(tibia_l_posInGround[i]);
	for (int i = 0; i < 3; ++i) res[0][i + nCoordinates + 18] = value<T>(pedal_r_velInGround[i]);
	for (int i = 0; i < 3; ++i) res[0][i + nCoordinates + 21] = value<T>(pedal_l_velInGround[i]);

	return 0;
}

int main() {
	Recorder x[NX];
	Recorder u[NU];
	Recorder tau[NR];
	for (int i = 0; i < NX; ++i) x[i] <<= 0;
	for (int i = 0; i < NU; ++i) u[i] <<= 0;
	const Recorder* Recorder_arg[n_in] = { x,u };
	Recorder* Recorder_res[n_out] = { tau };
	F_generic<Recorder>(Recorder_arg, Recorder_res);
	double res[NR];
	for (int i = 0; i < NR; ++i) Recorder_res[0][i] >>= res[i];
	Recorder::stop_recording();
	return 0;
}
