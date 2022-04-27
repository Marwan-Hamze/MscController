#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <iostream>
#include <mc_tasks/TrajectoryTaskGeneric.h>
#include <mc_rbdyn/Robots.h>

#pragma once

namespace mc_tasks{

namespace msc_stabilizer{

using Eigen::Matrix;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3d;

struct Stabilizer {

public:

// Constructor to set the robots, realRobots and the robotIndex of the stabilizer as the ones of the controller

Stabilizer(mc_rbdyn::Robots &robots, mc_rbdyn::Robots &realRobots, unsigned int robotIndex);

/**
 * @brief Computes the LQR gain matrix (usually denoted K) for a discrete time
 * infinite horizon problem.
 *
 * @param A State matrix of the underlying system
 * @param B Input matrix of the underlying system
 * @param Q Weight matrix penalizing the state
 * @param R Weight matrix penalizing the controls
 * @param N Weight matrix penalizing state / control pairs
 * @return K, the Generated Gain matrix (has to be a double/dynamic size
 * matrix!). 
 * @param numIT Number of Iterations to compute K
 */

MatrixXd lqrGain(MatrixXd A, MatrixXd B, MatrixXd Q, MatrixXd R,
                    MatrixXd N, int numIT = 9);


// Defining a 36-element vector, to group the state error and force error

typedef Matrix <double,36,1> error;

// Defining a 12-element command vector

typedef Matrix <double,12,1> command;

// Struct to group the com variables

struct COM{

        Vector3d pos;
        Matrix3d R;
        Vector3d vel;
        Vector3d angvel;
        
    };

// Struct to group a contact's variables

struct contact{

        Vector3d pos;
        Matrix3d R;
        Vector3d vel;
        Vector3d angvel;
        Vector3d fc;
        Vector3d tc;

    };

// struct to group the reference and current state values

struct state{

COM CoM;

contact rightFoot;
contact leftFoot;

};

// This struct groups the current state and other variables from the feedback of the Robot

struct feedback{

state x;
Matrix3d R;
Vector3d pc_1, pc_d_1, oc_d_1, pc_2, pc_d_2, oc_d_2;

};

/*  This struct groups the accelerations generated by the stabilizer, including com/base accelerations to be used in the 
 transformations from the world to the com/Base Frame, and as CoMTasks and BaseTasks to prevent the QP from giving them
 undesired values when computing the alphaD vector. */

 struct accelerations{

Vector3d RF_linAcc;
Vector3d RF_angAcc;

Vector3d LF_linAcc;
Vector3d LF_angAcc;

Vector3d ddcom;
Vector3d dwb;

Matrix<double, 6, 1> acc;

 };

// This function computes the skew symmetric matrix of a given 3D vector

Matrix3d S(Vector3d v);

// This function transforms a rotation matrix to a 3d Vector
// The transformation is based on the matrix/ axis-angle transformation

Vector3d Mat2Ang(Matrix3d M);

/** This structure contains the stabilizer's configuration: constant values, the robot's mass  
 * and Inertia, the stiffness/damping matrices of the contacts and of the com/base accelerations, and the 
 * Weight matrices for the LQR and for the kinematics/force tradeoff 
*/

struct configuration {

    Vector3d ex {1, 0, 0};
    Vector3d ey {0, 1, 0};
    Vector3d ez {0, 0, 1};

    Matrix3d Id;
    Matrix<double, 36, 36> Id36;
    Matrix3d Zero3_3;
    Matrix<double, 12, 6> Zero12_6;
    Matrix<double, 12, 12> Zero12_12;

    int m;
    Matrix3d I;

    Matrix<double, 36, 36> Q;
    Matrix<double, 12, 12> R;
    Matrix<double, 36, 36> W;

    Matrix<double, 36, 12> N_xu;

    Vector3d wf {1, 1, 0};
    Vector3d wt {0, 0, 1};

    Matrix3d KFP_RF, KFP_LF;
    Matrix3d KFD_RF, KFD_LF;
    Matrix3d KTP_RF, KTP_LF;
    Matrix3d KTD_RF, KTD_LF;

    Matrix3d Rsc_RF, Rsc_LF;

    Matrix3d Kp;
    Matrix3d Kd;

};

// Run function to run the stabilizer

void run();

// Reset function

void reset();

// This function sets the stabilizer's configuration defined in the struct configuration

configuration configure (mc_rbdyn::Robots &robots);

// This function set the reference values 

state reference(mc_rbdyn::Robots &robots);

// Get the current feedback: the state needed for the stabilizer, and the variables needed to transform the
// accelerations of the contacts from the base frame to the world frame

feedback getFeedback(mc_rbdyn::Robots &realRobots);

// This function computes the control gain from reference values

MatrixXd computeGain(state &x_ref, configuration &config);

// This function computes the state and force errors (scaled), and returns a trade-off between the 2 errors in a different error vector

error computeError(state x_ref, feedback feedback, configuration config);

// This function generates the accelerations written in the world frame for the contact tasks

accelerations computeAccelerations(const MatrixXd K, feedback feedback, state x_ref, configuration config, error &error, mc_rbdyn::Robots &robots);

// Definition of the Control, Weight and Gain Matrices

Matrix<double, 36, 36> A;
Matrix<double, 36, 12> B;

Matrix<double, 36, 36> M;
Matrix<double, 36, 36> N;

MatrixXd K_;

// Definition of sub-Matrices to simplify the expression of the Control Matrices 

Matrix3d A31, A32, A33, A34, A41, A42, A43, A44;
Matrix3d F1_21, F1_23, F1_41, F1_42, F1_43, F1_44;
Matrix3d F2_21, F2_23, F2_41, F2_42, F2_43, F2_44;
Matrix3d T1_11, T1_12, T1_13, T1_14, T1_22, T1_24;
Matrix3d T2_11, T2_12, T2_13, T2_14, T2_22, T2_24;
Matrix3d V1_11, V1_13, V1_22, V1_24;
Matrix3d V2_11, V2_13, V2_22, V2_24;

Matrix<double, 12, 12> F0, F1, F2, D1, D2, T1, T2, V1, V2;
Matrix<double, 12, 6> G1, G2;

// Definition of Parameters to simplify the computation's expressions

int m;
Matrix3d I;
Matrix3d KFP_1;
Matrix3d KFD_1;
Matrix3d KTP_1;
Matrix3d KTD_1;
Matrix3d KFP_2;
Matrix3d KFD_2;
Matrix3d KTP_2;
Matrix3d KTD_2;

Matrix3d Rsc_1;
Matrix3d Rsc_2;

Vector3d com;
Matrix3d Rb;
Vector3d com_d;
Vector3d o_d;

Vector3d pc_1;
Matrix3d Rc_1;
Vector3d pc_d_1;
Vector3d oc_d_1;
Vector3d fc_1;
Vector3d tc_1;

Vector3d pc_2;
Matrix3d Rc_2;
Vector3d pc_d_2;
Vector3d oc_d_2;
Vector3d fc_2;
Vector3d tc_2;

Matrix3d Rint_1;
Matrix3d Cb_1;
Matrix3d Rint_2;
Matrix3d Cb_2;

state x_ref_;

feedback feedback_;

configuration config_;

error x_delta_;
error f_delta_;

error error_;

accelerations accelerations_;

mc_rbdyn::Robots robots_;
mc_rbdyn::Robots realRobots_;

unsigned int robotIndex_;

}; // Struct Stabilizer
} // Namespace msc_stabilizer
} // Namespace mc_Tasks