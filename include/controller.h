#pragma once
#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <vector>

#include "robotmodel.h"
#include "trajectory.h"
#include "custommath.h"

using namespace std;
using namespace Eigen;

#define NECS2SEC 1000000000

class CController
{

public:
    CController(int JDOF);
    virtual ~CController();	

    void read(double t, double* q, double* qdot, double timestep);
    void read(double t, double* q, double* qdot, double timestep, double* pos, double* quat);
    void control_mujoco();
    void write(double* torque);
    

    
    tuple<std::vector<double>, double> write_pybind();
    float gripper_goal;
	
    
    void Initialize();
    


private:
    
    void ModelUpdate();
    void motionPlan();
    void motionPlan_taskonly();
    void motionPlan_Heuristic(const char* object, double init_theta, double goal_theta);
    
    
    struct Robot{
		int id;
		Vector3d pos;
		double zrot; //frame rotation according to Z-axis
		double ee_align; //gripper and body align angle

	};

	struct Objects{
        const char* name;
		int id;
		Vector3d o_margin; // (x,y,z)residual from frame origin to rotation plane
		Vector3d r_margin; // (x,y,z)radius of the object (where we first grab)
		Vector3d grab_dir; // grabbing direction. r_margin vs o_margin x r_margin
        Vector3d pos;
	};

    struct Target{
        double x;
        double y;
        double z;
        double roll;
        double pitch;
        double yaw;
        double gripper;
        double time;
        Vector3d target_velocity;
        string state;
    };


    void reset_target(double motion_time, VectorXd target_joint_position);
    void reset_target(double motion_time, Vector3d target_pos, Vector3d target_ori);
    void reset_target(double motion_time, Vector3d target_pos, Vector3d target_ori, Vector3d target_velocity);
    void reset_target(double motion_time, string state);
    VectorXd _q; // joint angle
	VectorXd _qdot; // joint velocity
    VectorXd _torque; // joint torque

    double _gripper; // gripper joint angle
	double _gripperdot; // gripper joint velocity
    double _grippertorque; // gripper joint torque

    VectorXd _valve; // position of the valve
    VectorXd _handle_valve; // positon of the handle valve
    VectorXd _robot_base; // postion of the robot base .... we consider the rotation of the robot and the objects are fixed (temporarily) 
    MatrixXd _rotation_obj; // rotation from environment randomization 
    int _k; // DOF

    bool _bool_init;
    double _t;
    double _dt;
	double _init_t;
	double _pre_t;

    double du;

    
    //controller
	double _kpj, _kdj; //joint P,D gain
    double _kpj_gripper, _kdj_gripper; //gripper P,D gain
    double _x_kp; // task control P gain

    void JointControl();
    void GripperControl();
    void CLIK();
    void OperationalSpaceControl();

    // robotmodel
    CModel Model;

    int _cnt_plan;
	VectorXd _time_plan;
	VectorXi _bool_plan;

    int _control_mode; //1: joint space, 2: operational space
    bool _init_mp;
    VectorXd _q_home; // joint home position

    //motion trajectory
	double _start_time, _end_time, _motion_time;

    CTrajectory JointTrajectory; // joint space trajectory
    HTrajectory HandTrajectory; // task space trajectory
    RTrajectory CircularTrajectory; 

    bool _bool_joint_motion, _bool_ee_motion; // motion check

    VectorXd _q_des, _qdot_des, _q_pre, _qdot_pre; 
    VectorXd _q_goal, _qdot_goal;
    VectorXd _x_des_hand, _xdot_des_hand;
    VectorXd _x_goal_hand, _xdot_goal_hand;
    Vector3d _pos_goal_hand, _rpy_goal_hand;
    double _gripper_des, _gripper_goal, _gripperdot_goal, _init_gripper;

    MatrixXd _J_hands; // jacobian matrix
    MatrixXd _J_bar_hands; // pseudo invere jacobian matrix

    VectorXd _x_hand, _xdot_hand; // End-effector


    VectorXd _x_err_hand, _xdot_err_hand;
    Matrix3d _R_des_hand, _R_hand;
    Matrix3d _Rdot_des_hand, _Rdot_hand;
    MatrixXd _lambda;
    VectorXd _force;


    vector<Target> _target_plan;

    CController::Target TargetTransformMatrix(Objects obj, Robot robot, double angle);
    CController::Target rpyTransformMatrix(Objects obj, Robot robot, double angle);
    Matrix3d R3D(Objects obj, Vector3d unitVec, double angle);
    void TargetPlanHeuristic(Objects obj, Robot robot, double init_theta, double goal_theta);
    // Vector3d AddTorque();

    VectorXd _grab_vector, _normal_vector, _origin;
    Matrix4d _Tvr, _Tvb, _Tbu, _Tur;
    double _radius, _init_theta, _goal_theta;
    Objects _obj;
    Robot _robot;

    int Ccount;

	vector<double> _x_plan;
	vector<double> _y_plan;
	vector<double> _z_plan;
    VectorXd _x_force;
    
    double _theta_des;
    string _object_name;
    VectorXd _accum_err_x, _accum_err_q;

    double _gripper_close;
    double _print_time, _print_interval;
    MatrixXd _rotation_handle, _rotation_valve;


};

#endif

