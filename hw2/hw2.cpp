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
	const Vector3d pos_in_link = Vector3d(0, 0, 0.1);
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
	VectorXd Kp = VectorXd::Constant(dof,400);
	Kp(6) = 50;
	cout << Kp.transpose() << endl;

	VectorXd Kv = VectorXd::Constant(dof,50);

	VectorXd qdes = initial_q;
	qdes(6) = 0.1;

	Vector3d x;
	Vector3d xd;
	Vector3d xdes(0.3, 0.1, 0.5);
	Vector3d xdes0(0.3, 0.1, 0.5);

	VectorXd g(dof); // Gravity vector
	VectorXd h(dof); // Coriolis plus gravity

	// Get info from user
	// cout << "Kp: " << endl;
	double kp7 = 0.3;
	// cin >> kp7;
	
	// Set up output file
	ofstream outfile("prob4d.txt");

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
		robot->taskInertiaMatrix(Lambda, Jv);
		robot->positionInWorld(x, link_name, pos_in_link);
		robot->linearVelocityInWorld(xd, link_name, pos_in_link);
		robot->nullspaceMatrix(N, Jv);
		robot->dynConsistentInverseJacobian(J_bar, Jv);

		// **********************
		// WRITE YOUR CODE AFTER
		// **********************
		int controller_number = QUESTION_4;  // change to the controller of the question you want : QUESTION_1, QUESTION_2, QUESTION_3, QUESTION_4, QUESTION_5


		// ---------------------------  question 1 ---------------------------------------
		if(controller_number == QUESTION_1)
		{
			// qdes(0) = initial_q(0) + 5*M_PI/180.0;			  
			Kv(6) = -kp7;
			// command_torques = -Kp*(robot->_q - qdes) - Kv*robot->_dq + h;
			// cout << robot->_q(1)*180.0/M_PI << endl;

			double kp = 400.0;      // chose your p gain
                        double kv = 40.0;      // chose your d gain

                        // VectorXd q_desired = initial_q;   // change to the desired robot joint angles for the question
                        // q_desired(0) = M_PI/2.0;

                        command_torques = (-Kp).asDiagonal()*(robot->_q - qdes) - Kv.asDiagonal()*robot->_dq + h;
			outfile << robot->_q(6) << endl;
		}

		// ---------------------------  question 2 ---------------------------------------
		if(controller_number == QUESTION_2)
		{
			double kp = 200;
			double kv = 28.28;
			VectorXd Kv = VectorXd::Constant(dof,10);

			MatrixXd F = Lambda*(-kp*(x - xdes) - kv*xd);
			command_torques = Jv.transpose()*F + g + N.transpose()*robot->_M*((-Kv).asDiagonal()*robot->_dq);
			// command_torques = Jv.transpose()*F + g - Kv.asDiagonal()*robot->_dq; 
			for (int i = 0; i <= 2; i++)
				outfile << x(i) << ",";
			for (int i = 0; i < dof - 1; i++)
				outfile << robot->_q(i) << ", ";
			outfile << robot->_q(dof-1) << endl;
		}

		// ---------------------------  question 3 ---------------------------------------
		if(controller_number == QUESTION_3)
		{

			double kp = 200;
			double kv = 28.28;
			VectorXd Kv = VectorXd::Constant(dof,10);

			Vector3d p = J_bar.transpose()*g;
			MatrixXd F = Lambda*(-kp*(x - xdes) - kv*xd) + p;
			command_torques = Jv.transpose()*F - N.transpose()*robot->_M*(Kv.asDiagonal()*robot->_dq);

			// Write to file
			for (int i = 0; i <= 2; i++)
				outfile << x(i) << ",";
			for (int i = 0; i < dof - 1; i++)
				outfile << robot->_q(i) << ", ";
			outfile << robot->_q(dof-1) << endl;
		}

		// ---------------------------  question 4 ---------------------------------------
		if(controller_number == QUESTION_4)
		{
			xdes << 0.1*sin(M_PI*time), 0.1*cos(M_PI*time), 0;
			xdes = xdes + xdes0;
			double kp = 200;
			double kv = 28.28;
			double kp_q = 400;
			double kv_q = 40;
			VectorXd Kv = VectorXd::Constant(dof,10);
			Vector3d p = J_bar.transpose()*g; 
			MatrixXd F;

			// Part (i)
			F = Lambda*(-kp*(x - xdes) - kv*xd) + p;
			command_torques = Jv.transpose()*F - N.transpose()*robot->_M*(Kv.asDiagonal()*robot->_dq);

			// Part (ii)
			F = (-kp*(x - xdes) - kv*xd) + p;
			command_torques = Jv.transpose()*F - N.transpose()*robot->_M*(Kv.asDiagonal()*robot->_dq);

			// Part (iii)
			VectorXd q_desired = VectorXd::Zero(dof);
			F = Lambda*(-kp*(x - xdes) - kv*xd) + p;
			command_torques = Jv.transpose()*F + N.transpose()*robot->_M*(-kp_q*(robot->_q - q_desired) - kv_q*robot->_dq); 

			// Part (iv)
			command_torques = Jv.transpose()*F + N.transpose()*robot->_M*(-kp_q*(robot->_q - q_desired) - kv_q*robot->_dq) + N.transpose()*g; 

			// Write to file
			for (int i = 0; i <= 2; i++)
				outfile << xdes(i) << ",";
			for (int i = 0; i <= 2; i++)
				outfile << x(i) << ",";
			for (int i = 0; i < dof - 1; i++)
				outfile << robot->_q(i) << ", ";
			outfile << robot->_q(dof-1) << endl;
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
