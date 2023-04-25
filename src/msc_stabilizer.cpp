#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <iostream>
#include <math.h>
#include "msc_stabilizer.h"

namespace msc_stabilizer {

using Eigen::Matrix;
using Eigen::Vector3d;
using Eigen::MatrixXd;
using Eigen::Matrix3d;

Stabilizer::Stabilizer(mc_rbdyn::Robots &robots, mc_rbdyn::Robots &realRobots, unsigned int robotIndex)
    : robots_(robots), realRobots_(realRobots_), robotIndex_(robotIndex)
{
}

// LQR Gain calculation method

MatrixXd Stabilizer::lqrGain(MatrixXd A, MatrixXd B, MatrixXd Q, MatrixXd R,
                    MatrixXd N, double eps) {

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
  MatrixXd P;
  MatrixXd Pold = Q;

  // iterate until P converges
 
  while (true) {

    // compute new P
    P = Acal_T * Pold * Acal -
        Acal_T * Pold * B * (R + B_T * Pold * B).inverse() * B_T * Pold * Acal + Qcal;

    // update delta
    MatrixXd delta = P - Pold;

    // Check the number of Iterations and finish
    if (fabs(delta.maxCoeff()) < eps) {
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

Vector3d Stabilizer::Mat2Ang(Matrix3d M){

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

Vector3d Stabilizer::finiteDifferences(Vector3d &vel, Vector3d &vel_old, double dt){

Vector3d acc;

acc = (vel - vel_old)/dt;
vel_old = vel;

return acc;

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

config.KFP_RF << config.KFP_RF.Zero(); 
config.KFD_RF << config.KFD_RF.Zero();
config.KTP_RF << config.KTP_RF.Zero();
config.KTD_RF << config.KTD_RF.Zero();

config.KFP_LF << config.KFP_LF.Zero();
config.KFD_LF << config.KFD_LF.Zero();
config.KTP_LF << config.KTP_LF.Zero();
config.KTD_LF << config.KTD_LF.Zero();

config.Q.block(0,0,3,3).diagonal() = config.qcom_p;
config.Q.block(3,3,3,3).diagonal() = config.qcom_R;
config.Q.block(6,6,3,3).diagonal() = config.qcom_vel;
config.Q.block(9,9,3,3).diagonal() = config.qcom_angvel;
config.Q.block(12,12,3,3).diagonal() = config.qRF_p;
config.Q.block(15,15,3,3).diagonal() = config.qRF_R;
config.Q.block(18,18,3,3).diagonal() = config.qRF_vel;
config.Q.block(21,21,3,3).diagonal() = config.qRF_angvel;
config.Q.block(24,24,3,3).diagonal() = config.qLF_p;
config.Q.block(27,27,3,3).diagonal() = config.qLF_R;
config.Q.block(30,30,3,3).diagonal() = config.qLF_vel;
config.Q.block(33,33,3,3).diagonal() = config.qLF_angvel;

config.R.block(0,0,3,3).diagonal() = config.rRF_lacc;
config.R.block(3,3,3,3).diagonal() = config.rRF_aacc;
config.R.block(6,6,3,3).diagonal() = config.rLF_lacc;
config.R.block(9,9,3,3).diagonal() = config.rLF_aacc;

config.W.block(12,12,3,3).diagonal() = config.wf_RF;
config.W.block(15,15,3,3).diagonal() = config.wt_RF;
config.W.block(24,24,3,3).diagonal() = config.wf_LF;
config.W.block(27,27,3,3).diagonal() = config.wt_LF;

config.KFP_RF.diagonal() = config.kfp_rf; 
config.KFD_RF.diagonal() = config.kfd_rf; 
config.KTP_RF.diagonal() = config.ktp_rf; 
config.KTD_RF.diagonal() = config.ktd_rf; 

config.Rsc_RF = robots.robot().bodyPosW("RLEG_LINK5").rotation().transpose();

config.KFP_LF.diagonal() = config.kfp_lf; 
config.KFD_LF.diagonal() = config.kfd_lf; 
config.KTP_LF.diagonal() = config.ktp_lf; 
config.KTD_LF.diagonal() = config.ktd_lf; 

config.Rsc_LF = robots.robot().bodyPosW("LLEG_LINK5").rotation().transpose();


// kp and kd are used for the CoM and Base tasks. They can be changed only from here
config.Kp << 100, 0, 0, 0, 100, 0, 0, 0, 100;
config.Kd << 30, 0, 0, 0, 30, 0, 0, 0, 30;

// kf and kt are used for the addmittance gains. DON'T change them as their implementation might not be correct
config.Kf << 0, 0, 0, 0, 0, 0, 0, 0, 0;
config.Kt << 0, 0, 0, 0, 0, 0, 0, 0, 0;

return config;

}

// Reconfigure

void Stabilizer::reconfigure(Stabilizer::configuration &config){

config.Q.block(0,0,3,3).diagonal() = config.qcom_p;
config.Q.block(3,3,3,3).diagonal() = config.qcom_R;
config.Q.block(6,6,3,3).diagonal() = config.qcom_vel;
config.Q.block(9,9,3,3).diagonal() = config.qcom_angvel;
config.Q.block(12,12,3,3).diagonal() = config.qRF_p;
config.Q.block(15,15,3,3).diagonal() = config.qRF_R;
config.Q.block(18,18,3,3).diagonal() = config.qRF_vel;
config.Q.block(21,21,3,3).diagonal() = config.qRF_angvel;
config.Q.block(24,24,3,3).diagonal() = config.qLF_p;
config.Q.block(27,27,3,3).diagonal() = config.qLF_R;
config.Q.block(30,30,3,3).diagonal() = config.qLF_vel;
config.Q.block(33,33,3,3).diagonal() = config.qLF_angvel;

config.R.block(0,0,3,3).diagonal() = config.rRF_lacc;
config.R.block(3,3,3,3).diagonal() = config.rRF_aacc;
config.R.block(6,6,3,3).diagonal() = config.rLF_lacc;
config.R.block(9,9,3,3).diagonal() = config.rLF_aacc;

config.W.block(12,12,3,3).diagonal() = config.wf_RF;
config.W.block(15,15,3,3).diagonal() = config.wt_RF;
config.W.block(24,24,3,3).diagonal() = config.wf_LF;
config.W.block(27,27,3,3).diagonal() = config.wt_LF;

config.KFP_RF.diagonal() = config.kfp_rf; 
config.KFD_RF.diagonal() = config.kfd_rf; 
config.KTP_RF.diagonal() = config.ktp_rf; 
config.KTD_RF.diagonal() = config.ktd_rf; 

config.KFP_LF.diagonal() = config.kfp_lf; 
config.KFD_LF.diagonal() = config.kfd_lf; 
config.KTP_LF.diagonal() = config.ktp_lf; 
config.KTD_LF.diagonal() = config.ktd_lf; 

}

// Getting reference values

Stabilizer::state Stabilizer::reference(mc_rbdyn::Robots &robots){

state x_ref; 

// The below variables are used to make the frame transformations
Matrix3d R, Rc_1,Rc_2;
Vector3d pc_1, pc_d_1, oc_d_1, pc_2, pc_d_2, oc_d_2;

R = robots.robot().posW().rotation().transpose();

pc_1 = robots.robot().bodyPosW("RLEG_LINK5").translation();
Rc_1 = robots.robot().bodyPosW("RLEG_LINK5").rotation().transpose();
pc_d_1 = robots.robot().bodyVelW("RLEG_LINK5").linear();
oc_d_1 = robots.robot().bodyVelW("RLEG_LINK5").angular();

pc_2 = robots.robot().bodyPosW("LLEG_LINK5").translation();
Rc_2 = robots.robot().bodyPosW("LLEG_LINK5").rotation().transpose();
pc_d_2 = robots.robot().bodyVelW("LLEG_LINK5").linear();
oc_d_2 = robots.robot().bodyVelW("LLEG_LINK5").angular();

x_ref.CoM.pos = robots.robot().com();
x_ref.CoM.R = robots.robot().posW().rotation().transpose();
x_ref.CoM.vel = robots.robot().comVelocity();    
x_ref.CoM.angvel = robots.robot().bodyVelW("BODY").angular();

x_ref.rightFoot.pos = R.transpose() * (pc_1 - x_ref.CoM.pos);
x_ref.rightFoot.R = R.transpose() * Rc_1;
x_ref.rightFoot.vel = R.transpose() * (pc_d_1 - x_ref.CoM.vel - S(x_ref.CoM.angvel) * (pc_1 - x_ref.CoM.pos));
x_ref.rightFoot.angvel = R.transpose() * (oc_d_1 - x_ref.CoM.angvel);
x_ref.rightFoot.fc = Rc_1 * robots.robot().forceSensor("RightFootForceSensor").wrenchWithoutGravity(robots.robot()).force();
x_ref.rightFoot.tc = Rc_1 * robots.robot().forceSensor("RightFootForceSensor").wrenchWithoutGravity(robots.robot()).moment();

x_ref.leftFoot.pos = R.transpose() * (pc_2 - x_ref.CoM.pos);
x_ref.leftFoot.R = R.transpose() * Rc_2;
x_ref.leftFoot.vel = R.transpose() * (pc_d_2 - x_ref.CoM.vel - S(x_ref.CoM.angvel) * (pc_2 - x_ref.CoM.pos));
x_ref.leftFoot.angvel = R.transpose() * (oc_d_2 - x_ref.CoM.angvel);
x_ref.leftFoot.fc = Rc_2 * robots.robot().forceSensor("LeftFootForceSensor").wrenchWithoutGravity(robots.robot()).force();
x_ref.leftFoot.tc = Rc_2 * robots.robot().forceSensor("LeftFootForceSensor").wrenchWithoutGravity(robots.robot()).moment();

return x_ref;

}

// Getting the feedback from robot and realRobot

Stabilizer::feedback Stabilizer::getFeedback(mc_rbdyn::Robots &robots, mc_rbdyn::Robots &realRobots){

feedback feedback;

Matrix3d Rc_1_real, Rc_2_real;

Rc_1_real = realRobots.robot().bodyPosW("RLEG_LINK5").rotation().transpose();
Rc_2_real = realRobots.robot().bodyPosW("LLEG_LINK5").rotation().transpose();

feedback.CoM.pos = robots.robot().com();
feedback.CoM.R = robots.robot().posW().rotation().transpose();
feedback.CoM.vel = robots.robot().comVelocity();    
feedback.CoM.angvel = robots.robot().bodyVelW("BODY").angular();

feedback.pc_1 = robots.robot().bodyPosW("RLEG_LINK5").translation();
feedback.Rc_1 = robots.robot().bodyPosW("RLEG_LINK5").rotation().transpose();
feedback.pc_d_1 = robots.robot().bodyVelW("RLEG_LINK5").linear();
feedback.oc_d_1 = robots.robot().bodyVelW("RLEG_LINK5").angular();

feedback.pc_2 = robots.robot().bodyPosW("LLEG_LINK5").translation();
feedback.Rc_2 = robots.robot().bodyPosW("LLEG_LINK5").rotation().transpose();
feedback.pc_d_2 = robots.robot().bodyVelW("LLEG_LINK5").linear();
feedback.oc_d_2 = robots.robot().bodyVelW("LLEG_LINK5").angular();

feedback.x.CoM.pos = realRobots.robot().com();
feedback.x.CoM.R = realRobots.robot().posW().rotation().transpose();
feedback.x.CoM.vel = realRobots.robot().comVelocity();    
feedback.x.CoM.angvel = realRobots.robot().bodyVelW("BODY").angular();

feedback.x.rightFoot.pos = feedback.CoM.R.transpose() * (feedback.pc_1 - feedback.CoM.pos);
feedback.x.rightFoot.R = feedback.CoM.R.transpose() * feedback.Rc_1;
feedback.x.rightFoot.vel = feedback.CoM.R.transpose() * (feedback.pc_d_1 - feedback.CoM.vel - S(feedback.CoM.angvel) * (feedback.pc_1 - feedback.CoM.pos));
feedback.x.rightFoot.angvel = feedback.CoM.R.transpose() * (feedback.oc_d_1 - feedback.CoM.angvel);
feedback.x.rightFoot.fc = Rc_1_real * realRobots.robot().forceSensor("RightFootForceSensor").wrenchWithoutGravity(realRobots.robot()).force();
feedback.x.rightFoot.tc = Rc_1_real * realRobots.robot().forceSensor("RightFootForceSensor").wrenchWithoutGravity(realRobots.robot()).moment();

feedback.x.leftFoot.pos = feedback.CoM.R.transpose() * (feedback.pc_2 - feedback.CoM.pos);
feedback.x.leftFoot.R = feedback.CoM.R.transpose() * feedback.Rc_2;
feedback.x.leftFoot.vel = feedback.CoM.R.transpose() * (feedback.pc_d_2 - feedback.CoM.vel - S(feedback.CoM.angvel) * (feedback.pc_2 - feedback.CoM.pos));
feedback.x.leftFoot.angvel = feedback.CoM.R.transpose() * (feedback.oc_d_2 - feedback.CoM.angvel);
feedback.x.leftFoot.fc = Rc_2_real * realRobots.robot().forceSensor("LeftFootForceSensor").wrenchWithoutGravity(realRobots.robot()).force();
feedback.x.leftFoot.tc = Rc_2_real * realRobots.robot().forceSensor("LeftFootForceSensor").wrenchWithoutGravity(realRobots.robot()).moment();


return feedback;

}

// Linearized Matrices Computation (A, B and M)

Stabilizer::linearMatrix Stabilizer::computeMatrix(state &x_ref, configuration &config) {

Stabilizer::linearMatrix linearMatrix;

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
A32 = 1/m * (KFP_1 * S(Rb * pc_1) + KFD_1 * (S(Rb * pc_d_1) + S(o_d) * S(Rb * pc_1)) 
    + KFP_2 * S(Rb * pc_2) + KFD_2 * (S(Rb * pc_d_2) + S(o_d) * S(Rb * pc_2)));
A33 = -1/m * (KFD_1 + KFD_2);
A34 = 1/m * (KFD_1 * S(Rb * pc_1) + KFD_2 * S(Rb * pc_2));
A41 = - Rb * I.inverse() * Rb.transpose() * (S(Rb * pc_1) * KFP_1 + S(Rb * pc_2) * KFP_2);
A42 = Rb * I.inverse() * Rb.transpose() * (S(o_d) * Rb * I * Rb.transpose() * S(o_d) 
    + S(fc_1) * S(Rb * pc_1) + S(Rb * pc_1) * (KFP_1 * S(Rb * pc_1) + KFD_1 * (S(Rb * pc_d_1) + S(o_d) * S(Rb * pc_1))) + KTP_1 * Cb_1 + KTD_1 * S(Rb * oc_d_1) 
    + S(fc_2) * S(Rb * pc_2) + S(Rb * pc_2) * (KFP_2 * S(Rb * pc_2) + KFD_2 * (S(Rb * pc_d_2) + S(o_d) * S(Rb * pc_2))) + KTP_2 * Cb_2 + KTD_2 * S(Rb * oc_d_2));
A43 = - Rb * I.inverse() * Rb.transpose() * (S(Rb * pc_1) * KFD_1 + S(Rb * pc_2) * KFD_2);
A44 = Rb * I.inverse() * Rb.transpose() * (S(Rb * I * Rb.transpose() * o_d) - S(o_d) * Rb * I * Rb.transpose() 
    - KTD_1 + S(Rb * pc_1) * KFD_1 * S(Rb * pc_1) - KTD_2 + S(Rb * pc_2) * KFD_2 * S(Rb * pc_2));

F1_31 = -1/m * (KFP_1 + KFD_1 * S(o_d)) * Rb;
F1_33 = -1/m * KFD_1 * Rb; 
F1_41 = - Rb * I.inverse() * Rb.transpose() * (S(fc_1) + S(Rb * pc_1) * (KFD_1 * S(o_d) + KFP_1)) * Rb;
F1_42 = Rb * I.inverse() * Rb.transpose() * KTP_1 * Cb_1 * Rb;
F1_43 = - Rb * I.inverse() * Rb.transpose() * S(Rb * pc_1) * KFD_1 * Rb;
F1_44 = - Rb * I.inverse() * Rb.transpose() * KTD_1 * Rb;

F2_31 = -1/m * (KFP_2 + KFD_2 * S(o_d)) * Rb;
F2_33 = -1/m * KFD_2 * Rb; 
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

F0 << config.Zero3_3, config.Zero3_3, config.Id, config.Zero3_3,  
      config.Zero3_3, config.Zero3_3, config.Zero3_3, config.Id,
      A31, A32, A33, A34,
      A41, A42, A43, A44;

F1 << config.Zero3_3, config.Zero3_3, config.Zero3_3, config.Zero3_3,  
      config.Zero3_3, config.Zero3_3, config.Zero3_3, config.Zero3_3,
      F1_31, config.Zero3_3, F1_33, config.Zero3_3, 
      F1_41, F1_42, F1_43, F1_44;

F2 << config.Zero3_3, config.Zero3_3, config.Zero3_3, config.Zero3_3,  
      config.Zero3_3, config.Zero3_3, config.Zero3_3, config.Zero3_3,
      F2_31, config.Zero3_3, F2_33, config.Zero3_3, 
      F2_41, F2_42, F2_43, F2_44;

linearMatrix.A << F0, F1, F2, 
                  config.Zero12_12, D1, config.Zero12_12, 
                  config.Zero12_12, config.Zero12_12, D2;

linearMatrix.B << config.Zero12_6, config.Zero12_6, 
                  G1, config.Zero12_6, 
                  config.Zero12_6, G2;


T1_11 = Rb.transpose();
T1_12 = - Rb.transpose() * (S(Rb * pc_1) + KFP_1.inverse() * KFD_1 * (S(Rb * pc_d_1) + S(o_d) * S(Rb * pc_1)));
T1_13 = Rb.transpose() * KFP_1.inverse() * KFD_1;
T1_14 = - Rb.transpose() * KFP_1.inverse() * KFD_1 * S(Rb * pc_1);
T1_22 = - Rb.transpose() * (Cb_1 + KTP_1.inverse() * KTD_1 * S(Rb * oc_d_1));
T1_24 = Rb.transpose() * KTP_1.inverse() * KTD_1;

T2_11 = Rb.transpose();
T2_12 = - Rb.transpose() * (S(Rb * pc_2) + KFP_2.inverse() * KFD_2 * (S(Rb * pc_d_2) + S(o_d) * S(Rb * pc_2)));
T2_13 = Rb.transpose() * KFP_2.inverse() * KFD_2;
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

linearMatrix.M << config.Zero12_12, config.Zero12_12, config.Zero12_12, 
                  T1, V1, config.Zero12_12, 
                  T2, config.Zero12_12, V2;


return linearMatrix;

}

// Computing the LQR Gain

MatrixXd Stabilizer::computeGain(linearMatrix &linearMatrix, configuration &config){

N = config.Id36 - config.W + config.W * linearMatrix.M;

Ay = N * linearMatrix.A * N.inverse();
By = N * linearMatrix.B;

Qy = N.inverse().transpose() * config.Q * N.inverse();

// As the LQR calculation method is discrete for an infinite horizon, we should use the matrices Ay and By in their discrete form 

MatrixXd K;

K = Stabilizer::lqrGain(config.dt * Ay + config.Id36, config.dt * By, Qy, config.R, config.N_xu);

return K;

}

// Error vectors computation, and addition of the admittance gain (commented the gain is set to zero in the configuration as it didn't show improvement)

Stabilizer::error Stabilizer::computeError(state &x_ref, feedback &feedback, linearMatrix &linearMatrix, configuration &config){

error error;

x_delta_ = x_delta_.Zero();
z_delta_ = z_delta_.Zero();
f_delta_ = f_delta_.Zero();
v_delta_ = v_delta_.Zero();

f_delta_.block(0,0,3,1) = feedback.x.rightFoot.fc - x_ref.rightFoot.fc;
f_delta_.block(3,0,3,1) = feedback.x.rightFoot.tc - x_ref.rightFoot.tc;
f_delta_.block(6,0,3,1) = feedback.x.leftFoot.fc - x_ref.leftFoot.fc;
f_delta_.block(9,0,3,1) = feedback.x.leftFoot.tc - x_ref.leftFoot.tc;

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

z_delta_.block(12,0,3,1) = - feedback.x.CoM.R.transpose() * config.KFP_RF.inverse() * f_delta_.block(0,0,3,1);
z_delta_.block(15,0,3,1) = - feedback.x.CoM.R.transpose() * config.KTP_RF.inverse() * f_delta_.block(3,0,3,1);

z_delta_.block(24,0,3,1) = - feedback.x.CoM.R.transpose() * config.KFP_LF.inverse() * f_delta_.block(6,0,3,1);
z_delta_.block(27,0,3,1) = - feedback.x.CoM.R.transpose() * config.KTP_LF.inverse() * f_delta_.block(9,0,3,1);

// Admittance gains

v_delta_.block(18,0,3,1) =  config.Kf * feedback.x.CoM.R.transpose() * (f_delta_.block(0,0,3,1) - (- config.KFP_RF * feedback.x.CoM.R * linearMatrix.M.block(12,0,3,36) * x_delta_));
v_delta_.block(21,0,3,1) =  config.Kt * feedback.x.CoM.R.transpose() * (f_delta_.block(3,0,3,1) - (- config.KTP_RF * feedback.x.CoM.R * linearMatrix.M.block(15,0,3,36) * x_delta_));

v_delta_.block(30,0,3,1) =  config.Kf * feedback.x.CoM.R.transpose() * (f_delta_.block(6,0,3,1) - (- config.KFP_LF * feedback.x.CoM.R * linearMatrix.M.block(24,0,3,36) * x_delta_));
v_delta_.block(33,0,3,1) =  config.Kt * feedback.x.CoM.R.transpose() * (f_delta_.block(9,0,3,1) - (- config.KTP_LF * feedback.x.CoM.R * linearMatrix.M.block(27,0,3,36) * x_delta_));

error = (config.Id36 - config.W) * x_delta_ + config.W * z_delta_ + v_delta_;

return error;

}

// Accelerations calculations

Stabilizer::accelerations Stabilizer::computeAccelerations(const MatrixXd &K, feedback &fd, state &x_ref, configuration &config, error &error){

accelerations accelerations;

Stabilizer::command u, ub;
Vector3d gamma, pc_1_w, pc_2_w;
double xsi;

u = u.Zero();
ub = ub.Zero();
gamma = gamma.Zero();

/*  Generating com and base accelerations, using:

 ddcom = - Kp(com - com_ref) - Kd(dcom - dcom_ref) + ddcom_ref
 dwb = - Kp* Mat2Ang(R * R_ref') - Kd(wb - wb_ref) + dwb_ref 
 
 Since ddcom_ref = dwb_ref = 0 as the reference state is a static equilibrium state, I did not add them in the calculations below*/

accelerations.ddcom = - config.Kp * (fd.CoM.pos - x_ref.CoM.pos) - config.Kd * (fd.CoM.vel - x_ref.CoM.vel);
accelerations.dwb = - config.Kp * Mat2Ang(fd.CoM.R  * x_ref.CoM.R.transpose()) - config.Kd * (fd.CoM.angvel - x_ref.CoM.angvel);

// Calculating the accelerations in the CoM/base frame

ub = - K * error;

pc_dd_1 << ub(0), ub(1), ub(2);
oc_dd_1 << ub(3), ub(4), ub(5);

pc_dd_2 << ub(6), ub(7), ub(8);
oc_dd_2 << ub(9), ub(10), ub(11);

/* Transforming the accelerations from the CoM/base frame to the world frame, using:

  pc_dd = Rb*pc_dd_b - S^2(wb)*(pc-com) + S(dwb)*(pc - com) + 2S(wb)(pc_d-dcom) + ddcom;
  oc_dd = Rb*oc_dd_b + S(wb)*(oc_d - wb) + dwb; */

u.block(0,0,3,1) = fd.CoM.R * pc_dd_1 - S(fd.CoM.angvel) * S(fd.CoM.angvel) * (fd.pc_1 - fd.CoM.pos) + S(accelerations.dwb) * (fd.pc_1 - fd.CoM.pos)
+ 2 * S(fd.CoM.angvel) * (fd.pc_d_1 - fd.CoM.vel) + accelerations.ddcom;
u.block(3,0,3,1) = fd.CoM.R * oc_dd_1 + S(fd.CoM.angvel) * (fd.oc_d_1 - fd.CoM.angvel) + accelerations.dwb;
u.block(6,0,3,1) = fd.CoM.R * pc_dd_2 - S(fd.CoM.angvel) * S(fd.CoM.angvel) * (fd.pc_2 - fd.CoM.pos) + S(accelerations.dwb) * (fd.pc_2 - fd.CoM.pos)
+ 2 * S(fd.CoM.angvel) * (fd.pc_d_2 - fd.CoM.vel) + accelerations.ddcom;
u.block(9,0,3,1) = fd.CoM.R* oc_dd_2 + S(fd.CoM.angvel) * (fd.oc_d_2 - fd.CoM.angvel) + accelerations.dwb;

accelerations.RF_linAcc = u.block(0,0,3,1);
accelerations.RF_angAcc = u.block(3,0,3,1);
accelerations.LF_linAcc = u.block(6,0,3,1);
accelerations.LF_angAcc = u.block(9,0,3,1);

return accelerations;

}

} // Namespace msc_stabilizer
