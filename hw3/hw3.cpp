#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>
#include <fstream>

#define QUESTION_1   1
#define QUESTION_2   2
#define QUESTION_3   3
#define QUESTION_4   4

// handle ctrl-c nicely
#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/panda_arm.urdf";
const string robot_name = "PANDA";

// redis keys:
// - read:
const std::string JOINT_ANGLES_KEY = "sai2::cs225a::panda_robot::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::cs225a::panda_robot::sensors::dq";
// - write
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::panda_robot::actuators::fgc";
const string CONTROLLER_RUNING_KEY = "sai2::cs225a::controller_running";

unsigned long long controller_counter = 0;

double sat(double x) {
	if (abs(x) <= 1.0)
		return x;
	else if (x > 0) 
		return 1.0;
	else
		return -1.0;
}

int main() {

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	VectorXd initial_q = robot->_q;
	robot->updateModel();

	// prepare controller
	int dof = robot->dof();
	const string link_name = "link7";
	const Vector3d pos_in_link = Vector3d(0, 0, 0.15);
	VectorXd command_torques = VectorXd::Zero(dof);

	// model quantities for operational space control
	MatrixXd Jv = MatrixXd::Zero(3,dof);
	MatrixXd Lambda = MatrixXd::Zero(3,3);
	MatrixXd J_bar = MatrixXd::Zero(dof,3);
	MatrixXd N = MatrixXd::Zero(dof,dof);

	robot->Jv(Jv, link_name, pos_in_link);
	robot->taskInertiaMatrix(Lambda, Jv);
	robot->dynConsistentInverseJacobian(J_bar, Jv);
	robot->nullspaceMatrix(N, Jv);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	// Initialization
	Vector3d x;
	Vector3d xd;
	Vector3d x0; // Initial position
	robot->positionInWorld(x0, link_name, pos_in_link);
	cout << "Initial position: " << x0.transpose() << endl;

	Vector3d xdes(0.3, 0.1, 0.5);
	Vector3d xdes0(0.3, 0.1, 0.5);
	Vector3d xd_des;
	Vector3d xdd_des;

	VectorXd g(dof); // Gravity vector
	VectorXd h(dof); // Coriolis plus gravity

	Matrix3d R = Matrix3d::Zero();
	Matrix3d Rdes = Matrix3d::Zero();

	Vector3d omega = Vector3d::Zero();

	MatrixXd J_0 = MatrixXd::Zero(6,dof);
	MatrixXd Lambda_0 = MatrixXd::Zero(6,6);

	// Joint Limits
	VectorXd q_lo(dof);
	q_lo << -165., -100., -165., -170., -165., 0.,  -165.0;
	VectorXd q_up(dof);
	q_up << 165.,   100.,  165.,  -30.,  165., 210., 165.0;
	q_lo = q_lo*M_PI/180.0;
	q_up = q_up*M_PI/180.0;
	

	// Set up output file
	ofstream outfile("prob4b.txt");

	redis_client.set(CONTROLLER_RUNING_KEY, "1");
	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
		robot->updateModel();
		robot->coriolisPlusGravity(h);
		robot->gravityVector(g);

		robot->Jv(Jv, link_name, pos_in_link);
		robot->J_0(J_0, link_name, pos_in_link);
		robot->taskInertiaMatrix(Lambda, Jv);
		robot->taskInertiaMatrix(Lambda_0, J_0);
		robot->positionInWorld(x, link_name, pos_in_link);
		robot->rotationInWorld(R, link_name);
		robot->linearVelocityInWorld(xd, link_name, pos_in_link);
		robot->angularVelocityInWorld(omega, link_name);
		robot->nullspaceMatrix(N, Jv);
		robot->dynConsistentInverseJacobian(J_bar, Jv);


		// **********************
		// WRITE YOUR CODE AFTER
		// **********************
		int controller_number = QUESTION_4;  

		// ---------------------------  question 1 ---------------------------------------
		if(controller_number == QUESTION_1)
		{

			xdes << 0.1*sin(M_PI*time), 0.1*cos(M_PI*time), 0;
			xdes = xdes + xdes0;
			double kp = 100;
			double kv = 20.0;
			double kp_q = 50;
			double kv_q = 14;
			VectorXd q_desired = VectorXd::Zero(dof);
			VectorXd Kv = VectorXd::Constant(dof,10);
			Vector3d p = J_bar.transpose()*g; 
			MatrixXd F;

			xd_des << 0.1*M_PI*cos(M_PI*time), -0.1*M_PI*sin(M_PI*time), 0;
			xdd_des << -0.1*M_PI*M_PI*sin(M_PI*time), -0.1*M_PI*M_PI*cos(M_PI*time),0;

			// Part (a)
			F = Lambda*(-kp*(x - xdes) - kv*xd);
			command_torques = Jv.transpose()*F + N.transpose()*(-kp_q*(robot->_q - q_desired) - kv_q*robot->_dq) + g; 

			// Part (c)
			F = Lambda*(xdd_des - kp*(x - xdes) - kv*(xd - xd_des));
			command_torques = Jv.transpose()*F + N.transpose()*(-kp_q*(robot->_q - q_desired) - kv_q*robot->_dq) + g; 

			// Write to file
			for (int i = 0; i <= 2; i++)
				outfile << xdes(i) << ",";
			for (int i = 0; i <= 2; i++)
				outfile << x(i) << ",";
			for (int i = 0; i < dof - 1; i++)
				outfile << robot->_q(i) << ", ";
			outfile << robot->_q(dof-1) << endl;
		}

		// ---------------------------  question 2 ---------------------------------------
		if(controller_number == QUESTION_2)
		{

			xdes << -0.1, 0.15, 0.2; 
			double kp = 100;
			double kv = 20.0;
			double kp_q = 50;
			double kv_q = 14;
			double k_mid = 25;
			double k_damp = 14;
			VectorXd q_desired = VectorXd::Zero(dof);
			VectorXd Kv = VectorXd::Constant(dof,10);
			Vector3d p = J_bar.transpose()*g; 
			MatrixXd F;

			// Part (d)
			F = Lambda*(-kp*(x - xdes) - kv*xd);
			VectorXd Gamma_damp = -k_damp*robot->_dq;
			command_torques = Jv.transpose()*F + N.transpose()*Gamma_damp + g;

			// Part (e)
			F = Lambda*(-kp*(x - xdes) - kv*xd);
			VectorXd Gamma_mid = -2*k_mid*(robot->_q - (q_up+q_lo)/2);
			command_torques = Jv.transpose()*F + N.transpose()*(Gamma_damp + Gamma_mid) + g;

			// Part (f)
			xdes << -0.65, -0.45, 0.7;
			F = Lambda*(-kp*(x - xdes) - kv*xd);
			command_torques = Jv.transpose()*F + N.transpose()*(Gamma_damp + Gamma_mid) + g;

			// Part (g)
			xdes << -0.65, -0.45, 0.7;
			F = Lambda*(-kp*(x - xdes) - kv*xd);
			command_torques = Jv.transpose()*F + N.transpose()*Gamma_damp + Gamma_mid + g;

			// Write to file
			for (int i = 0; i <= 2; i++)
				outfile << xdes(i) << ",";
			for (int i = 0; i <= 2; i++)
				outfile << x(i) << ",";
			for (int i = 0; i < dof - 1; i++)
				outfile << robot->_q(i) << ", ";
			outfile << robot->_q(dof-1) << endl;
		}

		// ---------------------------  question 3 ---------------------------------------
		if(controller_number == QUESTION_3)
		{
			// Params
			double kp = 100;
			double kv = 20.0;
			double kp_q = 50;
			double kv_q = 14;
			
			// Desired pose
			xdes << 0.6, 0.3, 0.5;
			Rdes(0,0) = cos(M_PI/3.0);
			Rdes(2,0) = -sin(M_PI/3.0);
			Rdes(1,1) = 1.0;
			Rdes(0,2) = sin(M_PI/3.0);
			Rdes(2,2) = cos(M_PI/3.0);

			// Calculate delta_phi 
			VectorXd delta_phi = VectorXd::Zero(3);
			for (int i = 0; i < 3; i++) {
				delta_phi += R.col(i).cross(Rdes.col(i));
			}
			delta_phi *= -0.5;

			// Calculate torques
			VectorXd op_goal(6);
			op_goal << kp*(xdes - x) - kv*xd, -kp*delta_phi - kv*omega;
			MatrixXd F = Lambda_0 * op_goal;
			command_torques = J_0.transpose()*F - N.transpose()*kv_q*robot->_dq + g;	
			
			// Write to file
			for (int i = 0; i <= 2; i++)
				outfile << xdes(i) << ",";
			for (int i = 0; i <= 2; i++)
				outfile << x(i) << ",";
			for (int i = 0; i < dof; i++)
				outfile << robot->_q(i) << ", ";
			for (int i = 0; i < 2; i++)
				outfile << delta_phi(i) << ", ";
			outfile << delta_phi(2) << endl;
		}

		// ---------------------------  question 4 ---------------------------------------
		if(controller_number == QUESTION_4)
		{
			// Params
			double kp = 200;
			double kv = 28.28;
			double kp_q = 50;
			double kv_q = 14;
			double Vmax = 0.1;
			
			// Desired position
			xdes << 0.6, 0.3, 0.4;
			VectorXd q_desired = VectorXd::Zero(dof);

			// Part (a) 
			MatrixXd F = Lambda*(-kp*(x - xdes) - kv*xd);
			command_torques = Jv.transpose()*F + N.transpose()*(-kp_q*(robot->_q - q_desired) - kv_q*robot->_dq) + g; 

			// Part (b) 
			xd_des = kp/kv*(xdes - x);
			double nu = sat(Vmax / xd_des.lpNorm<Infinity>());

			MatrixXd F2 = Lambda*(-kv*(xd - nu*xd_des));
			command_torques = Jv.transpose()*F2 + N.transpose()*(-kp_q*(robot->_q - q_desired) - kv_q*robot->_dq) + g; 

			// Write to file
			for (int i = 0; i <= 2; i++)
				outfile << xdes(i) << ",";
			for (int i = 0; i <= 2; i++)
				outfile << x(i) << ",";
			for (int i = 0; i < dof; i++)
				outfile << robot->_q(i) << ", ";
			for (int i = 0; i < 2; i++)
				outfile << xd(i) << ", ";
			outfile << xd(2) << endl;
		}

		// **********************
		// WRITE YOUR CODE BEFORE
		// **********************

		// send to redis
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;

	}

	command_torques.setZero();
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
	redis_client.set(CONTROLLER_RUNING_KEY, "0");

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


	return 0;
}
