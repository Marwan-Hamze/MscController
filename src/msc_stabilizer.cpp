#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <iostream>
#include <math.h>
#include "msc_stabilizer.h"

namespace mc_tasks{

namespace msc_stabilizer {

using Eigen::Matrix;
using Eigen::Vector3d;
using Eigen::MatrixXd;
using Eigen::Matrix3d;

Stabilizer::Stabilizer(mc_rbdyn::Robots &robots, mc_rbdyn::Robots &realRobots, unsigned int robotIndex)
{
      robots_ = robots; 
      realRobots_ = realRobots;
      robotIndex_ = robotIndex;
}

// LQR function

MatrixXd Stabilizer::lqrGain(MatrixXd A, MatrixXd B, MatrixXd Q, MatrixXd R,
                    MatrixXd N, int numIT) {

  MatrixXd K;
  // check if dimensions are compatible
  if (A.rows() != A.cols() || B.rows() != A.rows() || Q.rows() != Q.cols() ||
      Q.rows() != A.rows() || R.rows() != R.cols() || R.rows() != B.cols() ||
      N.rows() != A.rows() || N.cols() != B.cols()) {
    std::cout << "One or more matrices have incompatible dimensions. Aborting."
              << std::endl;
        return K;
      }
  
  // precompute as much as possible
  MatrixXd B_T = B.transpose();
  MatrixXd Acal = A - B * R.inverse() * N.transpose();
  MatrixXd Acal_T = Acal.transpose();
  MatrixXd Qcal = Q - N * R.inverse() * N.transpose();

  // initialize P with Q
  MatrixXd P = Q;

  // iterate for eps number of iterations
  unsigned int numIterations = 0;
  MatrixXd Pold = P;
  while (true) {
    numIterations++;

    // compute new P
    P = Acal_T * P * Acal -
        Acal_T * P * B * (R + B_T * P * B).inverse() * B_T * P * Acal + Qcal;

    // Check the number of Iterations and finish
    if (fabs(numIterations) > numIT) {
      break;
    }
    Pold = P;
  }

  // compute the lqr gain from P
  K = (R + B_T * P * B).inverse() * (B_T * P * A + N.transpose());

  return K;
}

// Skew-Symmetric function

Matrix3d Stabilizer::S(Vector3d v){

Matrix3d M;

M << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;

return M;

}

// Rotation matrix to 3d Vector function

Vector3d Stabilizer::Mat2Ang( Matrix3d M){

Vector3d v;

Matrix3d eye;

eye << 1, 0, 0, 0, 1, 0, 0, 0, 1;

if (M == eye)
{
     v << 0,0,0;
     return v;
}
else 
{
     double angle = acos((M(0,0) + M(1,1) + M(2,2)-1)/2);
     double x = (M(2,1)-M(1,2))/sqrt( pow((M(2,1)-M(1,2)),2) + pow((M(0,2)-M(2,0)),2) + pow((M(1,0)-M(0,1)),2));
     double y = (M(0,2)-M(2,0))/sqrt( pow((M(2,1)-M(1,2)),2) + pow((M(0,2)-M(2,0)),2) + pow((M(1,0)-M(0,1)),2));
     double z = (M(1,0)-M(0,1))/sqrt( pow((M(2,1)-M(1,2)),2) + pow((M(0,2)-M(2,0)),2) + pow((M(1,0)-M(0,1)),2));

     v << angle * x, angle * y, angle * z;
     return v;
}

}

// configuring the stabilizer

Stabilizer::configuration Stabilizer::configure(mc_rbdyn::Robots &robots){

configuration config;

config.Id = Matrix3d::Identity();
config.Id36 = Matrix<double, 36, 36>::Identity();

config.Zero3_3 << config.Zero3_3.Zero();
config.Zero12_6 << config.Zero12_6.Zero();
config.Zero12_12 << config.Zero12_12.Zero();

config.m = robots.robot().mass();
config.I << 217, 0, 0, 0, 212, 0, 0, 0, 20;

config.Q << config.Q.Zero();
config.R << config.R.Zero();
config.N_xu << config.N_xu.Zero();
config.W << config.W.Zero();

config.Q.block(0,0,3,3) = 10000 * config.Id;
config.Q.block(3,3,3,3) = config.Id;
config.Q.block(6,6,3,3) = 10000 * config.Id;
config.Q.block(9,9,3,3) = config.Id;
config.Q.block(12,12,3,3) = 30000 * config.Id;
config.Q.block(15,15,3,3) = config.Id;
config.Q.block(18,18,3,3) = 30000 * config.Id;
config.Q.block(21,21,3,3) = config.Id;
config.Q.block(24,24,3,3) = 30000 * config.Id;
config.Q.block(27,27,3,3) = config.Id;
config.Q.block(30,30,3,3) = 30000 * config.Id;
config.Q.block(33,33,3,3) = config.Id;

config.R.block(0,0,3,3) = config.Id;
config.R.block(3,3,3,3) = config.Id;
config.R.block(6,6,3,3) = config.Id;
config.R.block(9,9,3,3) = config.Id;

config.W.block(12,12,3,3) = 0.5 * config.wf;
config.W.block(15,15,3,3) = 0.5 * config.wt;
config.W.block(24,24,3,3) = 0.5 * config.wf;
config.W.block(27,27,3,3) = 0.5 * config.wt;

config.KFP_RF << 20000, 0, 0, 0, 20000, 0, 0, 0, 20000;
config.KFD_RF << 1000, 0, 0, 0, 1000, 0, 0, 0, 1000;
config.KTP_RF << 500, 0, 0, 0, 500, 0, 0, 0, 500;
config.KTD_RF << 50, 0, 0, 0, 50, 0, 0, 0, 50;

config.Rsc_RF = robots.robot().X_b1_b2("base_link","R_ANKLE_R_S").rotation().transpose();

config.KFP_LF << 20000, 0, 0, 0, 20000, 0, 0, 0, 20000;
config.KFD_LF << 1000, 0, 0, 0, 1000, 0, 0, 0, 1000;
config.KTP_LF << 500, 0, 0, 0, 500, 0, 0, 0, 500;
config.KTD_LF << 50, 0, 0, 0, 50, 0, 0, 0, 50;

config.Rsc_LF = robots.robot().X_b1_b2("base_link","L_ANKLE_R_S").rotation().transpose();

config.Kp << 1, 0, 0, 0, 1, 0, 0, 0, 1;
config.Kd << 0.5, 0, 0, 0, 0.5, 0, 0, 0, 0.5;

return config;

}

// Adding reference values, in addition to contact information

Stabilizer::state Stabilizer::reference(mc_rbdyn::Robots &robots){

state x_ref; 

x_ref.CoM.pos = robots.robot().com();
x_ref.CoM.R = robots.robot().posW().rotation().transpose();
x_ref.CoM.vel = robots.robot().comVelocity();
x_ref.CoM.angvel = robots.robot().bodyVelW("base_link").angular();

x_ref.rightFoot.pos = robots.robot().X_b1_b2("base_link","R_ANKLE_R_S").translation();
x_ref.rightFoot.R = robots.robot().X_b1_b2("base_link","R_ANKLE_R_S").rotation().transpose();
x_ref.rightFoot.vel = robots.robot().bodyVelB("R_ANKLE_R_S").linear();
x_ref.rightFoot.angvel = robots.robot().bodyVelB("R_ANKLE_R_S").angular();

x_ref.rightFoot.fc << 0, 0, 9.81 * robots.robot().mass()/2;
x_ref.rightFoot.tc << 0, 0, 0;

x_ref.leftFoot.pos = robots.robot().X_b1_b2("base_link","L_ANKLE_R_S").translation();
x_ref.leftFoot.R = robots.robot().X_b1_b2("base_link","L_ANKLE_R_S").rotation().transpose();
x_ref.leftFoot.vel = robots.robot().bodyVelB("L_ANKLE_R_S").linear();
x_ref.leftFoot.angvel = robots.robot().bodyVelB("L_ANKLE_R_S").angular(); 

x_ref.leftFoot.fc << 0, 0, 9.81 * robots.robot().mass()/2;
x_ref.leftFoot.tc << 0, 0, 0;

return x_ref;

}

Stabilizer::feedback Stabilizer::getFeedback(mc_rbdyn::Robots &realRobots){

feedback feedback;

feedback.x.CoM.pos = realRobots.robot().com();
feedback.x.CoM.R = realRobots.robot().posW().rotation().transpose();
feedback.x.CoM.vel = realRobots.robot().comVelocity();    
feedback.x.CoM.angvel = realRobots.robot().bodyVelW("base_link").angular();

feedback.x.rightFoot.pos = realRobots.robot().X_b1_b2("base_link","R_ANKLE_R_S").translation();
feedback.x.rightFoot.R = realRobots.robot().X_b1_b2("base_link","R_ANKLE_R_S").rotation().transpose();
feedback.x.rightFoot.vel = realRobots.robot().bodyVelB("R_ANKLE_R_S").linear();
feedback.x.rightFoot.angvel = realRobots.robot().bodyVelB("R_ANKLE_R_S").angular();
feedback.x.rightFoot.fc = realRobots.robot().surfaceWrench("RightFoot").force();
feedback.x.rightFoot.tc = realRobots.robot().surfaceWrench("RightFoot").moment();

feedback.x.leftFoot.pos = realRobots.robot().X_b1_b2("base_link","L_ANKLE_R_S").translation();
feedback.x.leftFoot.R = realRobots.robot().X_b1_b2("base_link","L_ANKLE_R_S").rotation().transpose();
feedback.x.leftFoot.vel = realRobots.robot().bodyVelB("L_ANKLE_R_S").linear();
feedback.x.leftFoot.angvel = realRobots.robot().bodyVelB("L_ANKLE_R_S").angular();
feedback.x.leftFoot.fc = realRobots.robot().surfaceWrench("LeftFoot").force();
feedback.x.leftFoot.tc = realRobots.robot().surfaceWrench("LeftFoot").moment();

feedback.R = realRobots.robot().posW().rotation().transpose();

feedback.pc_1 = realRobots.robot().bodyPosW("R_ANKLE_R_S").translation();
feedback.pc_d_1 = realRobots.robot().bodyVelW("R_ANKLE_R_S").angular();
feedback.oc_d_1 = realRobots.robot().bodyVelW("R_ANKLE_R_S").angular();

feedback.pc_2 = realRobots.robot().bodyPosW("L_ANKLE_R_S").translation();
feedback.pc_d_2 = realRobots.robot().bodyVelW("L_ANKLE_R_S").angular();
feedback.oc_d_2 = realRobots.robot().bodyVelW("L_ANKLE_R_S").angular();

return feedback;

}

// Compute function

MatrixXd Stabilizer::computeGain(state &x_ref, configuration &config) {

// Casting to variables to make the formulation easier to read

// Robot and Environment Parameters

    // Robot's Mass and Inertia 

    m = config.m;
    I = config.I;
    
    // Flexibility at the right foot
    
    KFP_1 = config.KFP_RF;
    KFD_1 = config.KFD_RF;
    KTP_1 = config.KTP_RF;
    KTD_1 = config.KTD_RF;

    // Flexibility at the left foot
    
    KFP_2 = config.KFP_LF;
    KFD_2 = config.KFD_LF;
    KTP_2 = config.KTP_LF;
    KTD_2 = config.KTD_LF;

    // Rest Orientation of the contact springs

    Rsc_1 = config.Rsc_RF;
    Rsc_2 = config.Rsc_LF;

    // Base of the Robot's reference

    com = x_ref.CoM.pos;
    Rb =  x_ref.CoM.R;
    com_d = x_ref.CoM.vel;
    o_d = x_ref.CoM.angvel;
    
    // Right foot reference

    pc_1 = x_ref.rightFoot.pos;
    Rc_1 = x_ref.rightFoot.R;
    pc_d_1 = x_ref.rightFoot.vel;
    oc_d_1 = x_ref.rightFoot.angvel;
    fc_1 = x_ref.rightFoot.fc;
    tc_1 = x_ref.rightFoot.tc;

    // Left foot reference

    pc_2 = x_ref.leftFoot.pos;
    Rc_2 = x_ref.leftFoot.R;
    pc_d_2 = x_ref.leftFoot.vel;
    oc_d_2 = x_ref.leftFoot.angvel;
    fc_2 = x_ref.leftFoot.fc;
    tc_2 = x_ref.leftFoot.tc;

// Matrices to simplify the control matrices' expressions

Rint_1 = Rsc_1 * Rc_1.transpose() * Rb.transpose();
Cb_1 = 0.5 * (S(config.ex) * Rint_1 * S(config.ex) + S(config.ey) * Rint_1 * S(config.ey) + S(config.ez) * Rint_1 * S(config.ez));

Rint_2 = Rsc_2 * Rc_2.transpose() * Rb.transpose();
Cb_2 = 0.5 * (S(config.ex) * Rint_2 * S(config.ex) + S(config.ey) * Rint_2 * S(config.ey) + S(config.ez) * Rint_2 * S(config.ez));

// Filling the Control Matrices

A31 = -1/m * (KFP_1 + KFP_2);
A32 = 1/m * (KFP_1 * S(Rb * pc_1) + KFD_1 * (S(Rb * pc_d_1) + S(o_d)) * S(Rb * pc_1) 
    + KFP_2 * S(Rb * pc_2) + KFD_2 * (S(Rb * pc_d_2) + S(o_d)) * S(Rb * pc_2));
A33 = -1/m * (KFD_1 + KFD_2);
A34 = 1/m * (KFD_1 * S(Rb * pc_1) + KFD_2 * S(Rb * pc_2));
A41 = - Rb * I.inverse() * Rb.transpose() * (S(Rb * pc_1) * KFP_1 + S(Rb * pc_2) * KFP_2);
A42 = Rb * I.inverse() * Rb.transpose() * (S(o_d) * Rb * I * Rb.transpose() * S(o_d) 
    + S(fc_1) * S(Rb * pc_1) + S(Rb * pc_1) * (KFP_1 * S(Rb * pc_1) + KFD_1 * (S(Rb * pc_d_1) + S(o_d) * S(Rb * pc_1))) + KTP_1 * Cb_1 + KTD_1 * S(Rb * oc_d_1) 
    + S(fc_2) * S(Rb * pc_2) + S(Rb * pc_2) * (KFP_2 * S(Rb * pc_2) + KFD_2 * (S(Rb * pc_d_2) + S(o_d) * S(Rb * pc_2))) + KTP_2 * Cb_2 + KTD_2 * S(Rb * oc_d_2));
A43 = - Rb * I.inverse() * Rb.transpose() * (S(Rb * pc_1) * KFD_1 + S(Rb * pc_2) * KFD_2);
A44 = Rb * I.inverse() * Rb.transpose() * (S(Rb * I * Rb.transpose() * o_d) - S(o_d) * Rb * I * Rb.transpose() 
    - KTD_1 + S(Rb * pc_1) * KFD_1 * S(Rb * pc_1) - KTD_2 + S(Rb * pc_2) * KFD_2 * S(Rb * pc_2));

F1_21 = -1/m * (KFP_1 + KFD_1 * S(o_d)) * Rb;
F1_23 = -1/m * KFD_1 * Rb; 
F1_41 = - Rb * I.inverse() * Rb.transpose() * (S(fc_1) + S(Rb * pc_1) * (KFD_1 * S(o_d) + KFP_1)) * Rb;
F1_42 = Rb * I.inverse() * Rb.transpose() * KTP_1 * Cb_1 * Rb;
F1_43 = - Rb * I.inverse() * Rb.transpose() * S(Rb * pc_1) * KFD_1 * Rb;
F1_44 = - Rb * I.inverse() * Rb.transpose() * KTD_1 * Rb;

F2_21 = -1/m * (KFP_2 + KFD_2 * S(o_d)) * Rb;
F2_23 = -1/m * KFD_2 * Rb; 
F2_41 = - Rb * I.inverse() * Rb.transpose() * (S(fc_2) + S(Rb * pc_2) * (KFD_2 * S(o_d) + KFP_2)) * Rb;
F2_42 = Rb * I.inverse() * Rb.transpose() * KTP_2 * Cb_2 * Rb;
F2_43 = - Rb * I.inverse() * Rb.transpose() * S(Rb * pc_2) * KFD_2 * Rb;
F2_44 = - Rb * I.inverse() * Rb.transpose() * KTD_2 * Rb;

D1 << config.Zero3_3, config.Zero3_3, config.Id, config.Zero3_3, 
      config.Zero3_3, config.Zero3_3, config.Zero3_3, config.Id, 
      config.Zero3_3, config.Zero3_3, config.Zero3_3, config.Zero3_3, 
      config.Zero3_3, config.Zero3_3, config.Zero3_3, config.Zero3_3; 
D2 = D1;

G1 << config.Zero3_3, config.Zero3_3, 
      config.Zero3_3, config.Zero3_3, 
      config.Id, config.Zero3_3, 
      config.Zero3_3, config.Id;
G2 = G1;

F0 << config.Zero3_3, config.Id, config.Zero3_3, config.Zero3_3,  
      config.Zero3_3, config.Zero3_3, config.Zero3_3, config.Id,
      A31, A32, A33, A34,
      A41, A42, A43, A44;

F1 << config.Zero3_3, config.Zero3_3, config.Zero3_3, config.Zero3_3, 
      F1_21, config.Zero3_3, F1_23, config.Zero3_3, 
      config.Zero3_3, config.Zero3_3, config.Zero3_3, config.Zero3_3, 
      F1_41, F1_42, F1_43, F1_44;

F2 << config.Zero3_3, config.Zero3_3, config.Zero3_3, config.Zero3_3, 
      F2_21, config.Zero3_3, F2_23, config.Zero3_3, 
      config.Zero3_3, config.Zero3_3, config.Zero3_3, config.Zero3_3, 
      F2_41, F2_42, F2_43, F2_44;

A << F0, F1, F2, 
     config.Zero12_12, D1, config.Zero12_12, 
     config.Zero12_12, config.Zero12_12, D2;

B << config.Zero12_6, config.Zero12_6, 
     G1, config.Zero12_6, 
     config.Zero12_6, G2;


T1_11 = Rb.transpose();
T1_13 = Rb.transpose() * KFP_1.inverse() * KFD_1;
T1_12 = - Rb.transpose() * (S(Rb * pc_1) + KFP_1.inverse() * KFD_1 * (S(Rb * pc_d_1) + S(o_d) * S(Rb * pc_1)));
T1_14 = - Rb.transpose() * KFP_1.inverse() * KFD_1 * S(Rb * pc_1);
T1_22 = - Rb.transpose() * (Cb_1 + KTP_1.inverse() * KTD_1 * S(Rb * oc_d_1));
T1_24 = Rb.transpose() * KTP_1.inverse() * KTD_1;

T2_11 = Rb.transpose();
T2_13 = Rb.transpose() * KFP_2.inverse() * KFD_2;
T2_12 = - Rb.transpose() * (S(Rb * pc_2) + KFP_2.inverse() * KFD_2 * (S(Rb * pc_d_2) + S(o_d) * S(Rb * pc_2)));
T2_14 = - Rb.transpose() * KFP_2.inverse() * KFD_2 * S(Rb * pc_2);
T2_22 = - Rb.transpose() * (Cb_2 + KTP_2.inverse() * KTD_2 * S(Rb * oc_d_2));
T2_24 = Rb.transpose() * KTP_2.inverse() * KTD_2;

V1_11 = Rb.transpose() * (config.Id + KFP_1.inverse() * KFD_1 * S(o_d)) * Rb;
V1_13 = Rb.transpose() * KFP_1.inverse() * KFD_1 * Rb;
V1_22 = - Rb.transpose() * Cb_1 * Rb;
V1_24 = Rb.transpose() * KTP_1.inverse() * KTD_1 * Rb;

V2_11 = Rb.transpose() * (config.Id + KFP_2.inverse() * KFD_2 * S(o_d)) * Rb;
V2_13 = Rb.transpose() * KFP_2.inverse() * KFD_2 * Rb;
V2_22 = - Rb.transpose() * Cb_2 * Rb;
V2_24 = Rb.transpose() * KTP_2.inverse() * KTD_2 * Rb;

T1 << T1_11, T1_12, T1_13, T1_14, 
      config.Zero3_3, T1_22, config.Zero3_3, T1_24, 
      config.Zero3_3, config.Zero3_3, config.Zero3_3, config.Zero3_3, 
      config.Zero3_3, config.Zero3_3, config.Zero3_3, config.Zero3_3;

T2 << T2_11, T2_12, T2_13, T2_14, 
      config.Zero3_3, T2_22, config.Zero3_3, T2_24, 
      config.Zero3_3, config.Zero3_3, config.Zero3_3, config.Zero3_3, 
      config.Zero3_3, config.Zero3_3, config.Zero3_3, config.Zero3_3;

V1 << V1_11, config.Zero3_3, V1_13, config.Zero3_3, 
      config.Zero3_3, V1_22, config.Zero3_3, V1_24, 
      config.Zero3_3, config.Zero3_3, config.Zero3_3, config.Zero3_3, 
      config.Zero3_3, config.Zero3_3, config.Zero3_3, config.Zero3_3;

V2 << V2_11, config.Zero3_3, V2_13, config.Zero3_3, 
      config.Zero3_3, V2_22, config.Zero3_3, V2_24, 
      config.Zero3_3, config.Zero3_3, config.Zero3_3, config.Zero3_3, 
      config.Zero3_3, config.Zero3_3, config.Zero3_3, config.Zero3_3;

M << config.Zero12_12, config.Zero12_12, config.Zero12_12, 
T1, V1, config.Zero12_12, 
T2, config.Zero12_12, V2;

N = config.Id36 - config_.W + config_.W * M;

A = N * A * N.inverse();
B = N * B;

Matrix<double, 36, 36> Qy;
Qy = N.inverse().transpose() * config_.Q * N.inverse();

// Computing the Control Gain

MatrixXd K;

K = Stabilizer::lqrGain(A, B, Qy, config_.R, config_.N_xu);

return K;

}

Stabilizer::error Stabilizer::computeError(state x_ref, feedback feedback, configuration config){

error error;

x_delta_ = x_delta_.Zero();
f_delta_ = f_delta_.Zero();

x_delta_.block(0,0,3,1) = feedback.x.CoM.pos - x_ref.CoM.pos;
x_delta_.block(3,0,3,1) = Mat2Ang(feedback.x.CoM.R * x_ref.CoM.R.transpose());
x_delta_.block(6,0,3,1) = feedback.x.CoM.vel - x_ref.CoM.vel;
x_delta_.block(9,0,3,1) = feedback.x.CoM.angvel - x_ref.CoM.angvel;

x_delta_.block(12,0,3,1) = feedback.x.rightFoot.pos - x_ref.rightFoot.pos;
x_delta_.block(15,0,3,1) = Mat2Ang(feedback.x.rightFoot.R * x_ref.rightFoot.R.transpose());
x_delta_.block(18,0,3,1) = feedback.x.rightFoot.vel - x_ref.rightFoot.vel;
x_delta_.block(21,0,3,1) = feedback.x.rightFoot.angvel - x_ref.rightFoot.angvel;

x_delta_.block(24,0,3,1) = feedback.x.leftFoot.pos - x_ref.leftFoot.pos;
x_delta_.block(27,0,3,1) = Mat2Ang(feedback.x.leftFoot.R * x_ref.leftFoot.R.transpose());
x_delta_.block(30,0,3,1) = feedback.x.leftFoot.vel - x_ref.leftFoot.vel;
x_delta_.block(33,0,3,1) = feedback.x.leftFoot.angvel - x_ref.leftFoot.angvel;

feedback.R.transpose() * config.KFD_RF.inverse();

f_delta_.block(12,0,3,1) = - feedback.R.transpose() * config.KFP_RF.inverse() * (feedback.x.rightFoot.fc - x_ref.rightFoot.fc);
f_delta_.block(15,0,3,1) = - feedback.R.transpose() * config.KTP_RF.inverse() * (feedback.x.rightFoot.tc - x_ref.rightFoot.tc);

f_delta_.block(24,0,3,1) = - feedback.R.transpose() * config.KFP_LF.inverse() * (feedback.x.leftFoot.fc - x_ref.leftFoot.fc);
f_delta_.block(27,0,3,1) = - feedback.R.transpose() * config.KTP_LF.inverse() * (feedback.x.leftFoot.tc - x_ref.leftFoot.tc);

error = (config.Id36 - config.W) * x_delta_ + config.W * f_delta_;

return error;

}

// Feet tasks generation

Stabilizer::accelerations Stabilizer::computeAccelerations(const MatrixXd K, feedback fd, state x_ref, configuration config, error &error, mc_rbdyn::Robots &robots){

accelerations accelerations;

Stabilizer::command u, ub;
Vector3d pc_dd_1, oc_dd_1, pc_dd_2, oc_dd_2;

u = u.Zero();

ub = - K * error;

pc_dd_1 << ub(0), ub(1), ub(2);
oc_dd_1 << ub(3), ub(4), ub(5);

pc_dd_2 << ub(6), ub(7), ub(8);
oc_dd_2 << ub(9), ub(10), ub(11);

/*  Generating com and base accelerations, using:

 ddcom = Kp(com - com_ref) + Kd(dcom - dcom_ref) + ddcom_ref
 dwb = Kp* Mat2Ang(R * R_ref') + Kd(wb - wb_ref) + dwb_ref */

accelerations.ddcom = - config.Kp * (robots.robot().com() - x_ref.CoM.pos) - config.Kd * (robots.robot().comVelocity() - x_ref.CoM.vel);
accelerations.dwb = - config.Kp * Mat2Ang(robots.robot().posW().rotation().transpose() * x_ref.CoM.R.transpose()) - config.Kd * (robots.robot().bodyVelW("base_link").angular() - x_ref.CoM.angvel);

accelerations.acc << accelerations.ddcom, accelerations.dwb;
 
/* Transforming the accelerations from the base frame to the world frame, using:

  pc_dd = Rb*pc_dd_b - S^2(wb)*(pc-com) + S(dwb)*(pc - com) + 2S(wb)(pc_d-dcom) + ddcom;
  oc_dd = Rb*oc_dd_b + S(wb)*(oc_d - wb) + dwb; */

u.block(0,0,3,1) = fd.R * pc_dd_1 - S(fd.x.CoM.angvel) * S(fd.x.CoM.angvel) * (fd.pc_1 - fd.x.CoM.pos) - S(accelerations.dwb) * (fd.pc_1 - fd.x.CoM.pos)
+ 2 * S(fd.x.CoM.angvel) * (fd.pc_d_1 - fd.x.CoM.vel) + accelerations.ddcom;
u.block(3,0,3,1) = fd.R * oc_dd_1 + S(fd.x.CoM.angvel) * (fd.oc_d_1 - fd.x.CoM.angvel) + accelerations.dwb;
u.block(6,0,3,1) = fd.R * pc_dd_2 - S(fd.x.CoM.angvel) * S(fd.x.CoM.angvel) * (fd.pc_2 - fd.x.CoM.pos) - S(accelerations.dwb) * (fd.pc_2 - fd.x.CoM.pos)
+ 2 * S(fd.x.CoM.angvel) * (fd.pc_d_2 - fd.x.CoM.vel) + accelerations.ddcom;
u.block(9,0,3,1) = fd.R * oc_dd_2 + S(fd.x.CoM.angvel) * (fd.oc_d_2 - fd.x.CoM.angvel) + accelerations.dwb;

accelerations.RF_linAcc = u.block(0,0,3,1);
accelerations.RF_angAcc = u.block(3,0,3,1);
accelerations.LF_linAcc = u.block(6,0,3,1);
accelerations.LF_angAcc = u.block(9,0,3,1);

return accelerations;

}

// Running the stabilizer

void Stabilizer::run(){

config_ = configure(robots_);
x_ref_ = reference(robots_);
feedback_ = getFeedback(realRobots_);
K_ = computeGain(x_ref_,config_);
error_ = computeError(x_ref_, feedback_, config_);
accelerations_ = computeAccelerations(K_, feedback_, x_ref_, config_, error_, robots_);

}

} // Namespace msc_stabilizer
} // Namespace mc_Tasks
