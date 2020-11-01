/*
 * gem2_state_publisher - a ros package to generate necessary data for gem2
 *
 * Copyright 2020-2021 Stylianos Piperakis, Foundation for Research and Technology Hellas (FORTH)
 * License: BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Foundation for Research and Technology Hellas (FORTH) 
 *	 nor the names of its contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __GEM2_SP_H__
#define __GEM2_SP_H__

// ROS Headers
#include <gem2_state_publisher/robotDyn.h>
#include <gem2_state_publisher/Madgwick.h>
#include <gem2_state_publisher/Mahony.h>
#include <gem2_state_publisher/JointDF.h>
#include <gem2_state_publisher/differentiator.h>
#include <gem2_state_publisher/butterworthLPF.h>
#include <gem2_state_publisher/butterworthHPF.h>
#include <gem2_state_publisher/MovingAverageFilter.h>
#include <gem2_state_publisher/Queue.h>


#include <ros/ros.h>

#include <eigen3/Eigen/Dense>
// ROS Messages
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/UInt8.h>
using namespace Eigen;
using namespace std;
//using namespace message_filters;
class gem2_state_publisher
{
private:
	///ROS Standard Variables
	ros::NodeHandle n;
	ros::Publisher rel_LLeg_wrench_pub, rel_RLeg_wrench_pub, rel_bodyIMU_pub, rel_bodyIMU_pub2, rel_bodyAcceleration_pub, rel_LFootPose_pub, rel_RFootPose_pub, rel_LFootTwist_pub, rel_RFootTwist_pub,
		rel_CoM_pub, rel_LFootIMU_pub, rel_RFootIMU_pub, rel_vCoM_pub, rel_ddxbb_r_pub, rel_ddxbb_l_pub, gtGaitPhase_pub;
	ros::Subscriber imu_sub, himu_sub, limu_sub, rimu_sub, lfsr_sub, rfsr_sub, joint_state_sub, gtGaitPhase_sub;

	Queue<sensor_msgs::JointState> joint_data;
	Queue<sensor_msgs::Imu> base_imu_data, head_imu_data, LLeg_imu_data, RLeg_imu_data;
	Queue<geometry_msgs::WrenchStamped> LLeg_FT_data, RLeg_FT_data;
	std_msgs::UInt8 gtGaitPhase;
	bool gtGaitPhase_inc = true;
	std::thread output_thread, filtering_thread;
	bool firstJointStates;
	bool computeJointVelocity;
	bool data_inc;
	int buffer_size;
	Matrix3d Rwb, Rwl, Rwr, Rwh;
	string base_link_frame, head_link_frame, lfoot_frame, rfoot_frame;
	Eigen::VectorXd joint_state_pos, joint_state_vel;
	Eigen::Vector3d  all, wll, arr, wrr, abb, wbb,  ahh, whh, wbbf, wbbf_, dwbb, CoM, vCoM;
	Affine3d T_B_I,  T_H_I, T_FT_RL, T_FT_LL, T_L_I, T_R_I;
	Quaterniond q_B_I, q_H_I, q_L_I, q_R_I;
	Quaterniond qbr, qbl, qwb, qwh, qbh, qwl, qwr;
	Eigen::Vector3d wbl, wbr, vbl, vbr;
	Eigen::Vector3d ddxbb_l, ddxbb_r;
	Affine3d Tbl, Tbr, Tbh;
	robotDyn *rd;
	bool useMahony;
	bool useHeadIMU;
	bool useBaseOrientation, useHeadOrientation, useLLegOrientation, useRLegOrientation;
	Madgwick *mw, *mwl, *mwr, *mwh;
	Mahony *mh, *mhl, *mhr, *mhh;
	///Mahoney gains
	double Kp, Ki, lKp, lKi, rKp, rKi, hKp, hKi;
	///Base/LFoot/RFoot IMU biases
	Vector3d bias_g, bias_a, bias_hg, bias_ha, bias_lg, bias_la, bias_rg, bias_ra;
	///Madgwick gain
	double beta, lbeta, rbeta, hbeta;
	bool imuCalibrated, himuCalibrated, limuCalibrated, rimuCalibrated;
	int imuCalibrationCycles,  himuCalibrationCycles, limuCalibrationCycles, rimuCalibrationCycles, maxImuCalibrationCycles;
	int maxAttitudeEstimationConvergeCycles, attitudeCycle, rattitudeCycle, lattitudeCycle;
	double g;
	///Helper
	bool is_connected_;
	std::map<std::string, double> joint_state_pos_map, joint_state_vel_map;
	double freq, imu_freq, leg_imu_freq, joint_freq, fsr_freq;
	int number_of_joints;
	bool firstGyrodot;
	bool useGyroLPF;
	int maWindow;
	int medianWindow;
	geometry_msgs::WrenchStamped temp_wrench_msg;
	geometry_msgs::PointStamped temp_point_msg;
	geometry_msgs::PoseStamped temp_pose_msg;
	geometry_msgs::TwistStamped temp_twist_msg;
	sensor_msgs::Imu temp_imu_msg;
	butterworthLPF **gyroLPF;
	MovingAverageFilter **gyroMAF;
	//Cuttoff Freqs for LPF
	double gyro_fx, gyro_fy, gyro_fz;
	JointDF **JointVF;
	Vector3d LLegGRF, RLegGRF, LLegGRT, RLegGRT;
	/****/
	double joint_noise_density, joint_cutoff_freq;
	/** Real odometry Data **/
	string lfsr_topic, rfsr_topic;
	string imu_topic, himu_topic, limu_topic, rimu_topic, ground_truth_gait_phase_topic;
	string joint_state_topic;
	string modelname;
	//Odometry, from supportleg to inertial, transformation from support leg to other leg
	void subscribeToIMU();
	void subscribeToFSR();
	void subscribeToJointState();
	void imuCb(const sensor_msgs::Imu::ConstPtr &msg);
	void himuCb(const sensor_msgs::Imu::ConstPtr &msg);

	void limuCb(const sensor_msgs::Imu::ConstPtr &msg);
	void rimuCb(const sensor_msgs::Imu::ConstPtr &msg);
	void joint_stateCb(const sensor_msgs::JointState::ConstPtr &msg);
	void lfsrCb(const geometry_msgs::WrenchStamped::ConstPtr &msg);
	void rfsrCb(const geometry_msgs::WrenchStamped::ConstPtr &msg);
	//void callback(const sensor_msgs::JointState::ConstPtr &msg, const sensor_msgs::Imu::ConstPtr &imu_msg,  const sensor_msgs::Imu::ConstPtr &limu_msg,
	//const sensor_msgs::Imu::ConstPtr &rimu_msg,const geometry_msgs::WrenchStamped::ConstPtr &lf_msg, const geometry_msgs::WrenchStamped::ConstPtr &rf_msg);
	void filterGyrodot();
	void init();
	void computeKinTFs();
	// Advertise to ROS Topics
	void advertise();
	void subscribe();
	void outputPublishThread();
	void filteringThread();
	void joints(const sensor_msgs::JointState &msg);
	void baseIMU(const sensor_msgs::Imu &msg);
	void headIMU(const sensor_msgs::Imu &msg);

	void LLegIMU(const sensor_msgs::Imu &msg);
	void RLegIMU(const sensor_msgs::Imu &msg);
	void LLeg_FT(const geometry_msgs::WrenchStamped &msg);
	void RLeg_FT(const geometry_msgs::WrenchStamped &msg);
	void contactStateCb(const std_msgs::UInt8::ConstPtr &msg);
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	bool connect(const ros::NodeHandle& nh);
	void disconnect();
	// Parameter Server
	void loadparams();
	void run();
	bool connected();
};
#endif
