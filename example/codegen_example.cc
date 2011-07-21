/*
 *
 * g++ codegen_example.cc -L../DebugGinac/ -lrbdl -I../DebugGinac/include/rbdl -I../src -I/usr/include/ginac -lginac -o codegen_example
 */

#include <iostream>
#include <ginac/ginac.h>
#include "rbdl.h"
#include "mathutils.h"
#include <vector>
#include <sstream>

using namespace std;
using namespace GiNaC;
using namespace RigidBodyDynamics;
using namespace SpatialAlgebra;

void q_print_func(const ex& t1, const ex& t2, const print_context &pc) {
	stringstream out;
	out << t2;	
	int index;
	out >> index;

	cout << "q[" << index << "]";
}

void qdot_print_func(const ex& t1, const ex& t2, const print_context &pc) {
	stringstream out;
	out << t2;	
	int index;
	out >> index;

	cout << "qdot[" << index << "]";
}

void tau_print_func(const ex& t1, const ex& t2, const print_context &pc) {
	stringstream out;
	out << t2;	
	int index;
	out >> index;

	cout << "tau[" << index << "]";
}

DECLARE_FUNCTION_2P(qdotfunc);
static ex qdotfunc_eval (const ex &t, const ex &index) {
	return qdotfunc(t, index).hold();
}
REGISTER_FUNCTION(qdotfunc,eval_func(qdotfunc_eval).
		print_func<print_csrc>(qdot_print_func));

DECLARE_FUNCTION_2P(qfunc);
static ex qfunc_eval (const ex &t, const ex &index) {
	return qfunc(t, index).hold();
}
static ex qfunc_deriv (const ex &t, const ex &index, unsigned deriv_param) {
	GINAC_ASSERT(deriv_param == 0);
	return qdotfunc(t, index);
}

DECLARE_FUNCTION_2P(taufunc);
static ex taufunc_eval (const ex &t, const ex &index) {
	return taufunc(t, index).hold();
}

REGISTER_FUNCTION(qfunc, 
		eval_func(qfunc_eval).
		derivative_func(qfunc_deriv).
		print_func<print_csrc>(q_print_func));

REGISTER_FUNCTION(taufunc, 
		eval_func(taufunc_eval).
		print_func<print_csrc>(tau_print_func));

VectorNd CreateQFunctions (size_t num, symbol t) {
	VectorNd result (num);

	unsigned int i;
	for (i = 0; i < num; i++) {
		result[i] = qfunc(t, i);
	}

	return result;
}

VectorNd CreateQDotFunctions (size_t num, symbol t) {
	VectorNd result (num);

	unsigned int i;
	for (i = 0; i < num; i++) {
		result[i] = qdotfunc(t, i);
	}

	return result;
}

VectorNd CreateTauFunctions (size_t num, symbol t) {
	VectorNd result (num);

	unsigned int i;
	for (i = 0; i < num; i++) {
		result[i] = taufunc(t, i);
	}

	return result;
}


int main (int argc, char* argv[]) {
	Model* model = NULL;

	unsigned int body_a_id, body_b_id, body_c_id;
	Body body_a, body_b, body_c;
	Joint joint_a, joint_b, joint_c;

	model = new Model();
	model->Init();

	model->gravity = Vector3d (0., -9.81, 0.);

	body_a = Body (1., Vector3d (0.5, 0., 0.0), Vector3d (1., 1., 1.));
		joint_a = Joint(
		JointTypeRevolute,
		Vector3d (0., 0., 1.)
	);
	
	body_a_id = model->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_a, body_a);
	
	body_b = Body (1., Vector3d (0., 0.5, 0.), Vector3d (1., 1., 1.));
		joint_b = Joint (
		JointTypeRevolute,
		Vector3d (0., 0., 1.)
	);
	
	body_b_id = model->AddBody(body_a_id, Xtrans(Vector3d(1., 0., 0.)), joint_b, body_b);
	
	body_c = Body (1., Vector3d (0.5, 0., 0.), Vector3d (1., 1., 1.));
		joint_c = Joint (
		JointTypeRevolute,
		Vector3d (0., 0., 1.)
	);
	
	body_c_id = model->AddBody(body_b_id, Xtrans(Vector3d(0., 1., 0.)), joint_c, body_c);

	cout << csrc;

	symbol t("t");
	VectorNd q_sym = CreateQFunctions (model->dof_count, t);
	VectorNd qdot_sym = CreateQDotFunctions (model->dof_count, t);
	VectorNd qddot_sym = VectorNd::Zero (model->dof_count);
	VectorNd tau_sym = CreateTauFunctions (model->dof_count, t);

	cout << q_sym << endl;
	q_sym[0] = t;

	MatrixNd mass_matrix_sym = MatrixNd::Zero (model->dof_count, model->dof_count);
	ForwardKinematics (*model, q_sym, qdot_sym, qddot_sym);

	CompositeRigidBodyAlgorithm (*model, q_sym, mass_matrix_sym);
	cout << "### JOINT SPACE INERTIA M(q) ###" << endl;
	cout << mass_matrix_sym << endl;

	cout << endl;

	VectorNd coriolis_sym = VectorNd::Zero (model->dof_count);
	InverseDynamics (*model, q_sym, qdot_sym, qddot_sym, tau_sym);

	cout << "### CORIOLIS FORCES N(q, qdot) ###" << endl;
	cout << tau_sym << endl;

	cout << endl;

 	ForwardDynamics (*model, q_sym, qdot_sym, tau_sym, qddot_sym);

	cout << "### FORWARD DYNAMICS qddot = FD (q, qdot, tau) ###" << endl;
	cout << qddot_sym << endl;

	delete model;

 	return 0;
}

