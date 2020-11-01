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

#include <iostream>
#include <algorithm>
#include <gem2_state_publisher/gem2_state_publisher.h>

void gem2_state_publisher::loadparams()
{
    ros::NodeHandle n_p("~");
    // Load Server Parameters
    n_p.param<std::string>("modelname", modelname, "nao.urdf");
    rd = new robotDyn(modelname, false);
    n_p.param<std::string>("base_link", base_link_frame, "base_link");
    n_p.param<std::string>("head_link", head_link_frame, "head_link");
    n_p.param<std::string>("lfoot", lfoot_frame, "l_ankle");
    n_p.param<std::string>("rfoot", rfoot_frame, "r_ankle");
    n_p.param<double>("imu_topic_freq", imu_freq, 100.0);
    n_p.param<bool>("useBaseOrientation", useBaseOrientation, true);
    n_p.param<double>("leg_imu_topic_freq", leg_imu_freq, 100.0);
    n_p.param<bool>("useLLegOrientation", useLLegOrientation, true);
    n_p.param<bool>("useRLegOrientation", useRLegOrientation, true);
    n_p.param<double>("fsr_topic_freq", fsr_freq, 100.0);
    n_p.param<double>("joint_topic_freq", joint_freq, 100.0);
    n_p.param<double>("joint_cutoff_freq", joint_cutoff_freq, 10.0);
    n_p.param<double>("joint_noise_density", joint_noise_density, 0.0);
    n_p.param<bool>("computeJointVelocity", computeJointVelocity, true);
    gtGaitPhase_inc = false;
    freq = fmin(fmin(imu_freq, fsr_freq), fmin(leg_imu_freq,joint_freq));
    //buffer_size = buffer_size;
    buffer_size = 10;
    std::vector<double> affine_list;
    T_B_I.setIdentity();
    T_H_I.setIdentity();
    T_L_I.setIdentity();
    T_R_I.setIdentity();
    T_FT_RL.setIdentity();
    T_FT_LL.setIdentity();
    n_p.getParam("T_B_I", affine_list);
    if (affine_list.size() == 16)
    {
        T_B_I(0, 0) = affine_list[0];
        T_B_I(0, 1) = affine_list[1];
        T_B_I(0, 2) = affine_list[2];
        T_B_I(0, 3) = affine_list[3];
        T_B_I(1, 0) = affine_list[4];
        T_B_I(1, 1) = affine_list[5];
        T_B_I(1, 2) = affine_list[6];
        T_B_I(1, 3) = affine_list[7];
        T_B_I(2, 0) = affine_list[8];
        T_B_I(2, 1) = affine_list[9];
        T_B_I(2, 2) = affine_list[10];
        T_B_I(2, 3) = affine_list[11];
        T_B_I(3, 0) = affine_list[12];
        T_B_I(3, 1) = affine_list[13];
        T_B_I(3, 2) = affine_list[14];
        T_B_I(3, 3) = affine_list[15];
    }
    q_B_I = Quaterniond(T_B_I.linear()); 
   
    n_p.getParam("T_H_I", affine_list);
    if (affine_list.size() == 16)
    {
        T_H_I(0, 0) = affine_list[0];
        T_H_I(0, 1) = affine_list[1];
        T_H_I(0, 2) = affine_list[2];
        T_H_I(0, 3) = affine_list[3];
        T_H_I(1, 0) = affine_list[4];
        T_H_I(1, 1) = affine_list[5];
        T_H_I(1, 2) = affine_list[6];
        T_H_I(1, 3) = affine_list[7];
        T_H_I(2, 0) = affine_list[8];
        T_H_I(2, 1) = affine_list[9];
        T_H_I(2, 2) = affine_list[10];
        T_H_I(2, 3) = affine_list[11];
        T_H_I(3, 0) = affine_list[12];
        T_H_I(3, 1) = affine_list[13];
        T_H_I(3, 2) = affine_list[14];
        T_H_I(3, 3) = affine_list[15];
    }
    q_H_I = Quaterniond(T_H_I.linear()); 

   
    n_p.getParam("T_L_I", affine_list);
    if (affine_list.size() == 16)
    {
        T_L_I(0, 0) = affine_list[0];
        T_L_I(0, 1) = affine_list[1];
        T_L_I(0, 2) = affine_list[2];
        T_L_I(0, 3) = affine_list[3];
        T_L_I(1, 0) = affine_list[4];
        T_L_I(1, 1) = affine_list[5];
        T_L_I(1, 2) = affine_list[6];
        T_L_I(1, 3) = affine_list[7];
        T_L_I(2, 0) = affine_list[8];
        T_L_I(2, 1) = affine_list[9];
        T_L_I(2, 2) = affine_list[10];
        T_L_I(2, 3) = affine_list[11];
        T_L_I(3, 0) = affine_list[12];
        T_L_I(3, 1) = affine_list[13];
        T_L_I(3, 2) = affine_list[14];
        T_L_I(3, 3) = affine_list[15];
    }
    q_L_I = Quaterniond(T_L_I.linear()); 

    n_p.getParam("T_R_I", affine_list);
    if (affine_list.size() == 16)
    {
        T_R_I(0, 0) = affine_list[0];
        T_R_I(0, 1) = affine_list[1];
        T_R_I(0, 2) = affine_list[2];
        T_R_I(0, 3) = affine_list[3];
        T_R_I(1, 0) = affine_list[4];
        T_R_I(1, 1) = affine_list[5];
        T_R_I(1, 2) = affine_list[6];
        T_R_I(1, 3) = affine_list[7];
        T_R_I(2, 0) = affine_list[8];
        T_R_I(2, 1) = affine_list[9];
        T_R_I(2, 2) = affine_list[10];
        T_R_I(2, 3) = affine_list[11];
        T_R_I(3, 0) = affine_list[12];
        T_R_I(3, 1) = affine_list[13];
        T_R_I(3, 2) = affine_list[14];
        T_R_I(3, 3) = affine_list[15];
    }
    q_R_I = Quaterniond(T_R_I.linear()); 

   
    n_p.param<std::string>("imu_topic", imu_topic, "/imu");
    n_p.param<bool>("useHeadIMU", useHeadIMU, false);
    n_p.param<bool>("useHeadOrientation", useHeadOrientation, true);

    n_p.param<std::string>("himu_topic", himu_topic, "/himu");
    n_p.param<std::string>("lleg_imu_topic", limu_topic, "/limu");
    n_p.param<std::string>("rleg_imu_topic", rimu_topic, "/rimu");
    n_p.param<std::string>("joint_state_topic", joint_state_topic, "/joint_states");
    n_p.param<std::string>("lfoot_force_torque_topic", lfsr_topic, "/force_torque/left");
    n_p.param<std::string>("rfoot_force_torque_topic", rfsr_topic, "/force_torque/right");
    n_p.param<std::string>("ground_truth/gait_phase", ground_truth_gait_phase_topic, "/ground_truth/gait_phase");

    n_p.getParam("T_FT_LL", affine_list);
    if (affine_list.size() == 16)
    {
        T_FT_LL(0, 0) = affine_list[0];
        T_FT_LL(0, 1) = affine_list[1];
        T_FT_LL(0, 2) = affine_list[2];
        T_FT_LL(0, 3) = affine_list[3];
        T_FT_LL(1, 0) = affine_list[4];
        T_FT_LL(1, 1) = affine_list[5];
        T_FT_LL(1, 2) = affine_list[6];
        T_FT_LL(1, 3) = affine_list[7];
        T_FT_LL(2, 0) = affine_list[8];
        T_FT_LL(2, 1) = affine_list[9];
        T_FT_LL(2, 2) = affine_list[10];
        T_FT_LL(2, 3) = affine_list[11];
        T_FT_LL(3, 0) = affine_list[12];
        T_FT_LL(3, 1) = affine_list[13];
        T_FT_LL(3, 2) = affine_list[14];
        T_FT_LL(3, 3) = affine_list[15];
    }
    n_p.getParam("T_FT_RL", affine_list);
    if (affine_list.size() == 16)
    {
        T_FT_RL(0, 0) = affine_list[0];
        T_FT_RL(0, 1) = affine_list[1];
        T_FT_RL(0, 2) = affine_list[2];
        T_FT_RL(0, 3) = affine_list[3];
        T_FT_RL(1, 0) = affine_list[4];
        T_FT_RL(1, 1) = affine_list[5];
        T_FT_RL(1, 2) = affine_list[6];
        T_FT_RL(1, 3) = affine_list[7];
        T_FT_RL(2, 0) = affine_list[8];
        T_FT_RL(2, 1) = affine_list[9];
        T_FT_RL(2, 2) = affine_list[10];
        T_FT_RL(2, 3) = affine_list[11];
        T_FT_RL(3, 0) = affine_list[12];
        T_FT_RL(3, 1) = affine_list[13];
        T_FT_RL(3, 2) = affine_list[14];
        T_FT_RL(3, 3) = affine_list[15];
    }
    //Attitude Estimation for Leg Odometry
    n_p.param<bool>("useMahony", useMahony, true);
    if (useMahony)
    {
        //Mahony Filter for Attitude Estimation
        n_p.param<double>("Mahony_Kp_base_imu", Kp, 0.25);
        n_p.param<double>("Mahony_Ki_base_imu", Ki, 0.0);
        n_p.param<double>("Mahony_Kp_head_imu", hKp, 0.25);
        n_p.param<double>("Mahony_Ki_head_imu", hKi, 0.0);
        n_p.param<double>("Mahony_Kp_lleg_imu", lKp, 0.25);
        n_p.param<double>("Mahony_Ki_lleg_imu", lKi, 0.0);
        n_p.param<double>("Mahony_Kp_rleg_imu", rKp, 0.25);
        n_p.param<double>("Mahony_Ki_rleg_imu", rKi, 0.0);
        mh = new Mahony(freq, Kp, Ki);
        mhl = new Mahony(freq, lKp, lKi);
        mhr = new Mahony(freq, rKp, rKi);
        mhh = new Mahony(freq, hKp, hKi);
    }
    else
    {
        //Madgwick Filter for Attitude Estimation
        n_p.param<double>("Madgwick_gain_base_imu", beta, 0.012f);
        n_p.param<double>("Madgwick_gain_head_imu", hbeta, 0.012f);
        n_p.param<double>("Madgwick_gain_lleg_imu", lbeta, 0.012f);
        n_p.param<double>("Madgwick_gain_rleg_imu", rbeta, 0.012f);
        mw = new Madgwick(freq, beta);
        mwl = new Madgwick(freq, lbeta);
        mwr = new Madgwick(freq, rbeta);
        mwh = new Madgwick(freq, hbeta);
    }
    n_p.param<double>("gravity", g, 9.81);
    n_p.param<bool>("imuCalibrated", imuCalibrated, true);
    n_p.param<bool>("imuCalibrated", himuCalibrated, true);
    n_p.param<bool>("imuCalibrated", limuCalibrated, true);
    n_p.param<bool>("imuCalibrated", rimuCalibrated, true);
    n_p.param<int>("maxImuCalibrationCycles", maxImuCalibrationCycles, 1000);
    n_p.param<int>("maxAttitudeEstimationConvergeCycles", maxAttitudeEstimationConvergeCycles, 1000);
    n_p.param<bool>("useGyroLPF", useGyroLPF, false);
    n_p.param<double>("gyro_cut_off_freq", gyro_fx, 7.0);
    n_p.param<double>("gyro_cut_off_freq", gyro_fy, 7.0);
    n_p.param<double>("gyro_cut_off_freq", gyro_fz, 7.0);
    n_p.param<int>("maWindow", maWindow, 10);
    if (useGyroLPF)
    {
        gyroLPF = new butterworthLPF *[3];
        for (unsigned int i = 0; i < 3; i++)
        {
            gyroLPF[i] = new butterworthLPF();
        }
        gyroLPF[0]->init("gyro X LPF", freq, gyro_fx);
        gyroLPF[1]->init("gyro Y LPF", freq, gyro_fy);
        gyroLPF[2]->init("gyro Z LPF", freq, gyro_fz);
    }
    else
    {
        gyroMAF = new MovingAverageFilter *[3];
        for (unsigned int i = 0; i < 3; i++)
            gyroMAF[i] = new MovingAverageFilter();
        for (unsigned int i = 0; i < 3; i++)
            gyroMAF[i]->setParams(maWindow);
    }
}
bool gem2_state_publisher::connect(const ros::NodeHandle& nh)
{
    ROS_INFO_STREAM("Gait-Phase Estimation Module State Publisher Initializing...");
    // Initialize ROS nodes
    n = nh;
    // Load ROS Parameters
    loadparams();
    //Initialization
    init();
    //Subscribe/Publish ROS Topics/Services
    subscribe();
    advertise();
    is_connected_ = true;
    ros::Duration(0.5).sleep();
    ROS_INFO_STREAM("Gait-Phase Estimation Module State Publisher Initialized");
    return true;
}
bool gem2_state_publisher::connected()
{
    return is_connected_;
}
void gem2_state_publisher::subscribe()
{
    subscribeToIMU();
    subscribeToFSR();
    subscribeToJointState();
    gtGaitPhase_sub = n.subscribe(ground_truth_gait_phase_topic, 1000, &gem2_state_publisher::contactStateCb, this, ros::TransportHints().tcpNoDelay());
}
void gem2_state_publisher::init()
{
    /** Initialize Variables **/
    //Kinematic TFs
    LLegGRF = Vector3d::Zero();
    RLegGRF = Vector3d::Zero();
    LLegGRT = Vector3d::Zero();
    RLegGRT = Vector3d::Zero();
    CoM = Vector3d::Zero();
    vCoM = Vector3d::Zero();
    bias_g = Vector3d::Zero();
    bias_a = Vector3d::Zero();
    bias_hg = Vector3d::Zero();
    bias_ha = Vector3d::Zero();
    bias_rg = Vector3d::Zero();
    bias_ra = Vector3d::Zero();
    bias_lg = Vector3d::Zero();
    bias_la = Vector3d::Zero();
    abb = Vector3d::Zero();
    wbb = Vector3d::Zero();
    ahh = Vector3d::Zero();
    whh = Vector3d::Zero();
    all = Vector3d::Zero();
    wll = Vector3d::Zero();
    arr = Vector3d::Zero();
    wrr = Vector3d::Zero();
    wbbf = Vector3d::Zero();
    dwbb = Vector3d::Zero();
    wbbf_ = Vector3d::Zero();
    
    wbl = Vector3d::Zero();
    wbr = Vector3d::Zero();
    vbl = Vector3d::Zero();
    vbr = Vector3d::Zero();
    
    Tbl = Affine3d::Identity();
    Tbr = Affine3d::Identity();
    firstJointStates = true;
    firstGyrodot = true;
    data_inc = false;
    imuCalibrationCycles = 0;
    himuCalibrationCycles = 0;
    limuCalibrationCycles = 0;
    rimuCalibrationCycles = 0;
    attitudeCycle = 0;
    lattitudeCycle = 0;
    rattitudeCycle = 0;
}
/** Main Loop **/
void gem2_state_publisher::run()
{

    output_thread = std::thread([this]{this->outputPublishThread();});
    filtering_thread = std::thread([this]{this->filteringThread();});
    ros::spin();
}
void gem2_state_publisher::filteringThread()
{
    ros::Rate rate(freq);
    //ROS Node Loop Rate
    while (ros::ok())
    {
        if(useHeadIMU)
        {
            if(joint_data.size()>0 && base_imu_data.size()>0 &&  head_imu_data.size()>0 && LLeg_imu_data.size()>0 && RLeg_imu_data.size()>0 && LLeg_FT_data.size()>0 && RLeg_FT_data.size()>0)
            {
                joints(joint_data.pop());
                baseIMU(base_imu_data.pop());
                headIMU(head_imu_data.pop());
                LLegIMU(LLeg_imu_data.pop());
                RLegIMU(RLeg_imu_data.pop());
                LLeg_FT(LLeg_FT_data.pop());
                RLeg_FT(RLeg_FT_data.pop());
                computeKinTFs();
                data_inc = true;
            }
        }
        else
        {
            if (joint_data.size()>0 && base_imu_data.size()>0  && LLeg_imu_data.size()>0 && RLeg_imu_data.size()>0 && LLeg_FT_data.size()>0 && RLeg_FT_data.size()>0)
            {
                joints(joint_data.pop());
                baseIMU(base_imu_data.pop());
                LLegIMU(LLeg_imu_data.pop());
                RLegIMU(RLeg_imu_data.pop());
                LLeg_FT(LLeg_FT_data.pop());
                RLeg_FT(RLeg_FT_data.pop());
                computeKinTFs();
                data_inc = true;
            }
        }
        rate.sleep();
    }
}
void gem2_state_publisher::computeKinTFs()
{
    //Update the Kinematic Structure
    rd->updateJointConfig(joint_state_pos_map, joint_state_vel_map, joint_noise_density);
    //Get the CoM w.r.t Body Frame
    CoM = rd->comPosition();
    vCoM = rd->comVelocity();
    Tbl.translation() = rd->linkPosition(lfoot_frame);
    qbl = rd->linkOrientation(lfoot_frame);
    Tbl.linear() = qbl.toRotationMatrix();
    Tbr.translation() = rd->linkPosition(rfoot_frame);
    qbr = rd->linkOrientation(rfoot_frame);
    Tbr.linear() = qbr.toRotationMatrix();

    Tbh.translation() = rd->linkPosition(head_link_frame);
    qbh = rd->linkOrientation(head_link_frame);
    Tbh.linear() = qbh.toRotationMatrix();

    //Differential Kinematics with Pinnochio
    wbl = rd->getAngularVelocity(lfoot_frame);
    vbl = rd->getLinearVelocity(lfoot_frame);

    wbr = rd->getAngularVelocity(rfoot_frame);
    vbr = rd->getLinearVelocity(rfoot_frame);
  
    LLegGRF = Tbl.linear() * LLegGRF;
    LLegGRT = Tbl.linear() * LLegGRT;
    RLegGRF = Tbr.linear() * RLegGRF;
    RLegGRT = Tbr.linear() * RLegGRT;
}
void gem2_state_publisher::filterGyrodot()
{
    //Compute numerical derivative
    if (useGyroLPF)
    {
        wbbf(0) = gyroLPF[0]->filter(wbb(0));
        wbbf(1) = gyroLPF[1]->filter(wbb(1));
        wbbf(2) = gyroLPF[2]->filter(wbb(2));
    }
    else
    {
        gyroMAF[0]->filter(wbb(0));
        gyroMAF[1]->filter(wbb(1));
        gyroMAF[2]->filter(wbb(2));
        wbbf(0) = gyroMAF[0]->x;
        wbbf(1) = gyroMAF[1]->x;
        wbbf(2) = gyroMAF[2]->x;
    }
    if (!firstGyrodot)
    {
        dwbb = (wbbf - wbbf_) * freq;
    }
    else
    {
        dwbb = Vector3d::Zero();
        firstGyrodot = false;
    }
    wbbf_ = wbbf;
}
void gem2_state_publisher::advertise()
{
    rel_bodyIMU_pub = n.advertise<sensor_msgs::Imu>("gem2/rel_base_imu", 1000);
    rel_bodyIMU_pub2 = n.advertise<sensor_msgs::Imu>("gem2/rel_head_imu", 1000);
    rel_bodyAcceleration_pub = n.advertise<geometry_msgs::TwistStamped>("gem2/rel_base_acceleration", 1000);
    rel_LFootPose_pub = n.advertise<geometry_msgs::PoseStamped>("gem2/rel_LLeg_pose", 1000);
    rel_RFootPose_pub = n.advertise<geometry_msgs::PoseStamped>("gem2/rel_RLeg_pose", 1000);
    rel_LFootTwist_pub = n.advertise<geometry_msgs::TwistStamped>("gem2/rel_LLeg_velocity", 1000);
    rel_RFootTwist_pub = n.advertise<geometry_msgs::TwistStamped>("gem2/rel_RLeg_velocity", 1000);
    rel_LFootIMU_pub = n.advertise<sensor_msgs::Imu>("gem2/rel_LLeg_imu", 1000);
    rel_RFootIMU_pub = n.advertise<sensor_msgs::Imu>("gem2/rel_RLeg_imu", 1000);
    rel_CoM_pub = n.advertise<geometry_msgs::PointStamped>("gem2/rel_CoM_position", 1000);
    rel_vCoM_pub = n.advertise<geometry_msgs::TwistStamped>("gem2/rel_CoM_velocity", 1000);
    rel_RLeg_wrench_pub = n.advertise<geometry_msgs::WrenchStamped>("gem2/rel_RLeg_wrench", 1000);
    rel_LLeg_wrench_pub = n.advertise<geometry_msgs::WrenchStamped>("gem2/rel_LLeg_wrench", 1000);
    //rel_ddxbb_r_pub = n.advertise<geometry_msgs::TwistStamped>("gem2/rel_base_acceleration_RLeg", 1000);
    //rel_ddxbb_l_pub = n.advertise<geometry_msgs::TwistStamped>("gem2/rel_base_acceleration_LLeg", 1000);
    gtGaitPhase_pub = n.advertise<std_msgs::UInt8>("gem2/ground_truth/gait_phase", 1000);
}
void gem2_state_publisher::subscribeToJointState()
{
    joint_state_sub = n.subscribe(joint_state_topic, 1000, &gem2_state_publisher::joint_stateCb, this, ros::TransportHints().tcpNoDelay());
    firstJointStates = true;
}
void gem2_state_publisher::joint_stateCb(const sensor_msgs::JointState::ConstPtr &msg)
{
    joint_data.push(*msg);
    if (joint_data.size() > (int) buffer_size)
        joint_data.pop();
}
void gem2_state_publisher::joints(const sensor_msgs::JointState &msg)
{
    if (firstJointStates)
    {
        number_of_joints = msg.name.size();
        joint_state_vel.resize(number_of_joints);
        joint_state_pos.resize(number_of_joints);
        JointVF = new JointDF *[number_of_joints];

        if (computeJointVelocity)
        {
            JointVF = new JointDF *[number_of_joints];
            for (unsigned int i = 0; i < number_of_joints; i++)
            {
                JointVF[i] = new JointDF();
                JointVF[i]->init(msg.name[i], freq, joint_cutoff_freq);
            }
        }
        firstJointStates = false;
    }

    if (computeJointVelocity)
    {
        for (unsigned int i = 0; i < msg.name.size(); i++)
        {
            joint_state_pos[i] = msg.position[i];
            joint_state_vel[i] = JointVF[i]->filter(msg.position[i]);
            joint_state_pos_map[msg.name[i]] = joint_state_pos[i];
            joint_state_vel_map[msg.name[i]] = joint_state_vel[i];
        }
    }
    else
    {
        for (unsigned int i = 0; i < msg.name.size(); i++)
        {
            joint_state_pos[i] = msg.position[i];
            joint_state_vel[i] = msg.velocity[i];
            joint_state_pos_map[msg.name[i]] = joint_state_pos[i];
            joint_state_vel_map[msg.name[i]] = joint_state_vel[i];
        }
    }
}
void gem2_state_publisher::subscribeToIMU()
{
    imu_sub = n.subscribe(imu_topic, 1000, &gem2_state_publisher::imuCb, this, ros::TransportHints().tcpNoDelay());
    himu_sub = n.subscribe(himu_topic, 1000, &gem2_state_publisher::himuCb, this, ros::TransportHints().tcpNoDelay());
    limu_sub = n.subscribe(limu_topic, 1000, &gem2_state_publisher::limuCb, this, ros::TransportHints().tcpNoDelay());
    rimu_sub = n.subscribe(rimu_topic, 1000, &gem2_state_publisher::rimuCb, this, ros::TransportHints().tcpNoDelay());
}
void gem2_state_publisher::baseIMU(const sensor_msgs::Imu &msg)
{
    //wbb -= Vector3d(0.00,-0.016, 0.00245);
    wbb = T_B_I.linear() * (Vector3d(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z));
    abb = T_B_I.linear() * (Vector3d(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z));
    if(useBaseOrientation)
    {
        qwb = q_B_I * (Quaterniond(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)) * q_B_I.inverse();
        Rwb = qwb.toRotationMatrix();
    }
    else
    {
        if (useMahony)
        {
            mh->updateIMU(wbb, abb);
            Rwb = mh->getR();
            qwb = Quaterniond(Rwb);
        }
        else
        {
            mw->updateIMU(wbb, abb);
            Rwb = mw->getR();
            qwb = Quaterniond(Rwb);
        }

        if(attitudeCycle<maxAttitudeEstimationConvergeCycles)
        {
            attitudeCycle++;
            return;
        }
    }

    if (imuCalibrationCycles < maxImuCalibrationCycles && imuCalibrated)
    {
        bias_g += wbb;
        bias_a += (abb -  Rwl.transpose() * Vector3d(0,0,g));
        imuCalibrationCycles++;
        return;
    }
    else if (imuCalibrated)
    {
        bias_a /= imuCalibrationCycles;
        bias_g /= imuCalibrationCycles;

        imuCalibrated = false;
        std::cout << "Calibration finished at " << imuCalibrationCycles << std::endl;
        std::cout << "Body Gyro biases " << bias_g(0) << " " << bias_g(1) << " " << bias_g(2) << std::endl;
        std::cout << "Body Acc biases " << bias_a(0) << " " << bias_a(1) << " " << bias_a(2) << std::endl;
    }
    abb -= (bias_a +  Rwb.transpose() * Vector3d(0,0,g));
    wbb -= bias_g;
    filterGyrodot();
}

void gem2_state_publisher::headIMU(const sensor_msgs::Imu &msg)
{
    whh = T_H_I.linear() * (Vector3d(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z));
    ahh = T_H_I.linear() * (Vector3d(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z));

    if(useHeadOrientation)
    {
        qwh = q_H_I * (Quaterniond(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)) * q_H_I.inverse();
        Rwh =  qwh.toRotationMatrix();
    }
    else
    {
        if (useMahony)
        {
            mhh->updateIMU(whh, ahh);
            Rwh = mhh->getR();
            qwh = Quaterniond(Rwh);
        }
        else
        {
            mwh->updateIMU(whh, ahh);
            Rwh = mwh->getR();
            qwh = Quaterniond(Rwh);
        }
    }
    
    if (himuCalibrationCycles < maxImuCalibrationCycles && himuCalibrated)
    {
        bias_hg += whh;
        bias_ha += (ahh -  Rwh.transpose() * Vector3d(0,0,g));
        himuCalibrationCycles++;
        return;
    }
    else if (himuCalibrated)
    {
        bias_ha /= himuCalibrationCycles;
        bias_hg /= himuCalibrationCycles;

        himuCalibrated = false;
        std::cout << "Calibration finished at " << himuCalibrationCycles << std::endl;
        std::cout << "Head Gyro biases " << bias_hg(0) << " " << bias_hg(1) << " " << bias_hg(2) << std::endl;
        std::cout << "Head Acc biases " << bias_ha(0) << " " << bias_ha(1) << " " << bias_ha(2) << std::endl;
    }
    ahh -= (bias_ha +  Rwh.transpose() * Vector3d(0,0,g));
    whh -= bias_hg;
}

void gem2_state_publisher::imuCb(const sensor_msgs::Imu::ConstPtr &msg)
{
    base_imu_data.push(*msg);
    if(base_imu_data.size()>(int) buffer_size)
        base_imu_data.pop();
}

void gem2_state_publisher::himuCb(const sensor_msgs::Imu::ConstPtr &msg)
{
    head_imu_data.push(*msg);
    if(head_imu_data.size()>(int) buffer_size)
        head_imu_data.pop();
}

void gem2_state_publisher::LLegIMU(const sensor_msgs::Imu &msg)
{
    //wll -=Vector3d(-0.0053,-0.0029, -0.0026);
    wll = T_L_I.linear() * (Vector3d(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z));
    all = T_L_I.linear() * (Vector3d(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z));
    //cout<<"LLeg "<<all<<endl;
    if(useLLegOrientation)
    {
        qwl = q_L_I * (Quaterniond(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)) * q_L_I.inverse();
        Rwl = qwl.toRotationMatrix();
    }
    else
    {
        if (useMahony)
        {
            mhl->updateIMU(wll, all);
            Rwl = mhl->getR();
            qwl = Quaterniond(Rwl);
            //cout<<"World Left LEG"<<endl<<Rwl<<endl;
        }
        else
        {
            mwl->updateIMU(wll, all);
            Rwl = mwl->getR();
            qwl = Quaterniond(Rwl);
        }

        if(lattitudeCycle<maxAttitudeEstimationConvergeCycles)
        {
            lattitudeCycle++;
            return;
        }
    }
    if (limuCalibrationCycles < maxImuCalibrationCycles && limuCalibrated)
    {

        bias_lg += wll;
        bias_la += (all  -  Rwl.transpose() * Vector3d(0,0,g));
        limuCalibrationCycles++;
        return;
    }
    else if (limuCalibrated)
    {

        bias_la /= limuCalibrationCycles;
        bias_lg /= limuCalibrationCycles;

        limuCalibrated = false;
        std::cout << "Calibration finished at " << limuCalibrationCycles << std::endl;
        std::cout << "Left Foot Gyro biases " << bias_lg(0) << " " << bias_lg(1) << " " << bias_lg(2) << std::endl;
        std::cout << "Left Foot Acc biases " << bias_la(0) << " " << bias_la(1) << " " << bias_la(2) << std::endl;
    }
    all -= (bias_la +  Rwl.transpose() * Vector3d(0,0,g));
        wll -= bias_lg;
}
void gem2_state_publisher::limuCb(const sensor_msgs::Imu::ConstPtr &msg)
{
    LLeg_imu_data.push(*msg);
    if(LLeg_imu_data.size()>(int) buffer_size)
        LLeg_imu_data.pop();
}
void gem2_state_publisher::RLegIMU(const sensor_msgs::Imu &msg)
{
    //wrr -= Vector3d(-0.0053,-0.0029, -0.0026);
    wrr = T_R_I.linear() * (Vector3d(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z));
    arr = T_R_I.linear() * (Vector3d(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z));
    //cout<<"RLeg "<<arr<<endl;


   if(useRLegOrientation)
    {
        qwr = q_R_I * (Quaterniond(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)) * q_R_I.inverse();
        Rwr = qwr.toRotationMatrix();
    }
    else
    {
        if (useMahony)
        {
            mhr->updateIMU(wrr, arr);
            Rwr = mhr->getR();
            qwr = Quaterniond(Rwr);
            //cout<<"World RIGHT LEG"<<endl<<Rwr<<endl;
        }
        else
        {
            mwr->updateIMU(wrr, arr);
            Rwr = mwr->getR();
            qwr = Quaterniond(Rwr);
        }

        if(rattitudeCycle<maxAttitudeEstimationConvergeCycles)
        {
            rattitudeCycle++;
            return;
        }
    }

    if (rimuCalibrationCycles < maxImuCalibrationCycles && rimuCalibrated)
    {

        bias_rg += wrr;
        bias_ra += (arr -  Rwr.transpose() * Vector3d(0,0,g)); 
        rimuCalibrationCycles++;
        return;
    }
    else if (rimuCalibrated)
    {

        bias_ra /= rimuCalibrationCycles;
        bias_rg /= rimuCalibrationCycles;

        rimuCalibrated = false;
        std::cout << "Calibration finished at " << rimuCalibrationCycles << std::endl;
        std::cout << "Right Foot Gyro biases " << bias_rg(0) << " " << bias_rg(1) << " " << bias_rg(2) << std::endl;
        std::cout << "Right Foot Acc biases " << bias_ra(0) << " " << bias_ra(1) << " " << bias_ra(2) << std::endl;
    }
    arr -= (bias_ra +  Rwr.transpose() * Vector3d(0,0,g));
    wrr -= bias_rg;

}
void gem2_state_publisher::rimuCb(const sensor_msgs::Imu::ConstPtr &msg)
{
    RLeg_imu_data.push(*msg);
    if(RLeg_imu_data.size()>(int) buffer_size)
        RLeg_imu_data.pop();
}

void gem2_state_publisher::subscribeToFSR()
{
    //Left Foot Wrench
    lfsr_sub = n.subscribe(lfsr_topic, 1000, &gem2_state_publisher::lfsrCb, this, ros::TransportHints().tcpNoDelay());
    //Right Foot Wrench
    rfsr_sub = n.subscribe(rfsr_topic, 1000, &gem2_state_publisher::rfsrCb, this, ros::TransportHints().tcpNoDelay());
}
void gem2_state_publisher::lfsrCb(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
    LLeg_FT_data.push(*msg);
    if(LLeg_FT_data.size()>(int) buffer_size)
        LLeg_FT_data.pop();
}


void gem2_state_publisher::contactStateCb(const std_msgs::UInt8::ConstPtr &msg)
{
    gtGaitPhase.data = msg->data;
    gtGaitPhase_inc = true;
}
void gem2_state_publisher::rfsrCb(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
    RLeg_FT_data.push(*msg);
    if(RLeg_FT_data.size()>(int) buffer_size)
        RLeg_FT_data.pop();
}
void gem2_state_publisher::LLeg_FT(const geometry_msgs::WrenchStamped &msg)
{
    LLegGRF(0) = msg.wrench.force.x;
    LLegGRF(1) = msg.wrench.force.y;
    LLegGRF(2) = msg.wrench.force.z;
    LLegGRT(0) = msg.wrench.torque.x;
    LLegGRT(1) = msg.wrench.torque.y;
    LLegGRT(2) = msg.wrench.torque.z;
    LLegGRF = T_FT_LL.linear() * LLegGRF;
    LLegGRT = T_FT_LL.linear() * LLegGRT;
}
void gem2_state_publisher::RLeg_FT(const geometry_msgs::WrenchStamped &msg)
{
    RLegGRF(0) = msg.wrench.force.x;
    RLegGRF(1) = msg.wrench.force.y;
    RLegGRF(2) = msg.wrench.force.z;
    RLegGRT(0) = msg.wrench.torque.x;
    RLegGRT(1) = msg.wrench.torque.y;
    RLegGRT(2) = msg.wrench.torque.z;
    RLegGRF = T_FT_RL.linear() * RLegGRF;
    RLegGRT = T_FT_RL.linear() * RLegGRT;
}

void gem2_state_publisher::outputPublishThread()
{

    ros::Rate rate(2.0*freq);
    while (ros::ok())
    {
        if(rimuCalibrated && rimuCalibrated && imuCalibrated && himuCalibrated)
            continue;

        if(!data_inc)
            continue;
        temp_imu_msg.header.stamp = ros::Time::now();
        temp_imu_msg.header.frame_id = base_link_frame;
        temp_imu_msg.linear_acceleration.x = abb(0);
        temp_imu_msg.linear_acceleration.y = abb(1);
        temp_imu_msg.linear_acceleration.z = abb(2);
        temp_imu_msg.angular_velocity.x = wbb(0);
        temp_imu_msg.angular_velocity.y = wbb(1);
        temp_imu_msg.angular_velocity.z = wbb(2);
        temp_imu_msg.orientation.x = qwb.x();
        temp_imu_msg.orientation.y = qwb.y();
        temp_imu_msg.orientation.z = qwb.z();
        temp_imu_msg.orientation.w = qwb.w();
        rel_bodyIMU_pub.publish(temp_imu_msg);

        if(gtGaitPhase_inc)
        {
            gtGaitPhase_pub.publish(gtGaitPhase);
        }

        temp_imu_msg.header.stamp = ros::Time::now();
        temp_imu_msg.header.frame_id = head_link_frame;
        temp_imu_msg.linear_acceleration.x = ahh(0);
        temp_imu_msg.linear_acceleration.y = ahh(1);
        temp_imu_msg.linear_acceleration.z = ahh(2);
        temp_imu_msg.angular_velocity.x = whh(0);
        temp_imu_msg.angular_velocity.y = whh(1);
        temp_imu_msg.angular_velocity.z = whh(2);
        temp_imu_msg.orientation.x = qwh.x();
        temp_imu_msg.orientation.y = qwh.y();
        temp_imu_msg.orientation.z = qwh.z();
        temp_imu_msg.orientation.w = qwh.w();
        rel_bodyIMU_pub2.publish(temp_imu_msg);


        temp_imu_msg.header.stamp = ros::Time::now();
        temp_imu_msg.header.frame_id = lfoot_frame;
        temp_imu_msg.linear_acceleration.x = all(0);
        temp_imu_msg.linear_acceleration.y = all(1);
        temp_imu_msg.linear_acceleration.z = all(2);
        temp_imu_msg.angular_velocity.x = wll(0);
        temp_imu_msg.angular_velocity.y = wll(1);
        temp_imu_msg.angular_velocity.z = wll(2);
        temp_imu_msg.orientation.x = qwl.x();
        temp_imu_msg.orientation.y = qwl.y();
        temp_imu_msg.orientation.z = qwl.z();
        temp_imu_msg.orientation.w = qwl.w();

        rel_LFootIMU_pub.publish(temp_imu_msg);

        temp_imu_msg.header.stamp = ros::Time::now();
        temp_imu_msg.header.frame_id = rfoot_frame;
        temp_imu_msg.linear_acceleration.x = arr(0);
        temp_imu_msg.linear_acceleration.y = arr(1);
        temp_imu_msg.linear_acceleration.z = arr(2);
        temp_imu_msg.angular_velocity.x = wrr(0);
        temp_imu_msg.angular_velocity.y = wrr(1);
        temp_imu_msg.angular_velocity.z = wrr(2);
        temp_imu_msg.orientation.x = qwr.x();
        temp_imu_msg.orientation.y = qwr.y();
        temp_imu_msg.orientation.z = qwr.z();
        temp_imu_msg.orientation.w = qwr.w();
        rel_RFootIMU_pub.publish(temp_imu_msg);



        temp_pose_msg.pose.position.x = Tbl.translation()(0);
        temp_pose_msg.pose.position.y = Tbl.translation()(1);
        temp_pose_msg.pose.position.z = Tbl.translation()(2);
        temp_pose_msg.pose.orientation.x = qbl.x();
        temp_pose_msg.pose.orientation.y = qbl.y();
        temp_pose_msg.pose.orientation.z = qbl.z();
        temp_pose_msg.pose.orientation.w = qbl.w();
        temp_pose_msg.header.stamp = ros::Time::now();
        temp_pose_msg.header.frame_id = base_link_frame;
        rel_LFootPose_pub.publish(temp_pose_msg);

        temp_pose_msg.pose.position.x = Tbr.translation()(0);
        temp_pose_msg.pose.position.y = Tbr.translation()(1);
        temp_pose_msg.pose.position.z = Tbr.translation()(2);
        temp_pose_msg.pose.orientation.x = qbr.x();
        temp_pose_msg.pose.orientation.y = qbr.y();
        temp_pose_msg.pose.orientation.z = qbr.z();
        temp_pose_msg.pose.orientation.w = qbr.w();
        temp_pose_msg.header.stamp = ros::Time::now();
        temp_pose_msg.header.frame_id = base_link_frame;
        rel_RFootPose_pub.publish(temp_pose_msg);


        // cout<<"Left Leg"<<endl;
        // cout<<Rwb * Tbl.linear()<<endl;
        // cout<<Rwl<<endl;

        // cout<<"Right Leg"<<endl;
        // cout<<Rwb * Tbr.linear()<<endl;
        // cout<<Rwr<<endl;
        temp_twist_msg.header.stamp = ros::Time::now();
        temp_twist_msg.header.frame_id = base_link_frame;
        temp_twist_msg.twist.linear.x = abb(0);
        temp_twist_msg.twist.linear.y = abb(1);
        temp_twist_msg.twist.linear.z = abb(2);
        temp_twist_msg.twist.angular.x = dwbb(0);
        temp_twist_msg.twist.angular.y = dwbb(1);
        temp_twist_msg.twist.angular.z = dwbb(2);
        rel_bodyAcceleration_pub.publish(temp_twist_msg);


        temp_twist_msg.header.stamp = ros::Time::now();
        temp_twist_msg.header.frame_id = base_link_frame;
        temp_twist_msg.twist.linear.x = vCoM(0);
        temp_twist_msg.twist.linear.y = vCoM(1);
        temp_twist_msg.twist.linear.z = vCoM(2);
        temp_twist_msg.twist.angular.x = 0;
        temp_twist_msg.twist.angular.y = 0;
        temp_twist_msg.twist.angular.z = 0;
        rel_vCoM_pub.publish(temp_twist_msg);
        temp_twist_msg.header.stamp = ros::Time::now();
        temp_twist_msg.header.frame_id = base_link_frame;
        temp_twist_msg.twist.linear.x = vbl(0);
        temp_twist_msg.twist.linear.y = vbl(1);
        temp_twist_msg.twist.linear.z = vbl(2);
        temp_twist_msg.twist.angular.x = wbl(0);
        temp_twist_msg.twist.angular.y = wbl(1);
        temp_twist_msg.twist.angular.z = wbl(2);
        rel_LFootTwist_pub.publish(temp_twist_msg);

        temp_twist_msg.header.stamp = ros::Time::now();
        temp_twist_msg.header.frame_id = base_link_frame;
        temp_twist_msg.twist.linear.x = vbr(0);
        temp_twist_msg.twist.linear.y = vbr(1);
        temp_twist_msg.twist.linear.z = vbr(2);
        temp_twist_msg.twist.angular.x = wbr(0);
        temp_twist_msg.twist.angular.y = wbr(1);
        temp_twist_msg.twist.angular.z = wbr(2);
        rel_RFootTwist_pub.publish(temp_twist_msg);

        temp_point_msg.header.stamp = ros::Time::now();
        temp_point_msg.header.frame_id = base_link_frame;
        temp_point_msg.point.x = CoM(0);
        temp_point_msg.point.y = CoM(1);
        temp_point_msg.point.z = CoM(2);
        rel_CoM_pub.publish(temp_point_msg);

        temp_wrench_msg.wrench.force.x = LLegGRF(0);
        temp_wrench_msg.wrench.force.y = LLegGRF(1);
        temp_wrench_msg.wrench.force.z = LLegGRF(2);
        temp_wrench_msg.wrench.torque.x = LLegGRT(0);
        temp_wrench_msg.wrench.torque.y = LLegGRT(1);
        temp_wrench_msg.wrench.torque.z = LLegGRT(2);
        temp_wrench_msg.header.frame_id = base_link_frame;
        temp_wrench_msg.header.stamp = ros::Time::now();
        rel_LLeg_wrench_pub.publish(temp_wrench_msg);

        temp_wrench_msg.wrench.force.x = RLegGRF(0);
        temp_wrench_msg.wrench.force.y = RLegGRF(1);
        temp_wrench_msg.wrench.force.z = RLegGRF(2);
        temp_wrench_msg.wrench.torque.x = RLegGRT(0);
        temp_wrench_msg.wrench.torque.y = RLegGRT(1);
        temp_wrench_msg.wrench.torque.z = RLegGRT(2);
        temp_wrench_msg.header.frame_id = base_link_frame;
        temp_wrench_msg.header.stamp = ros::Time::now();
        rel_RLeg_wrench_pub.publish(temp_wrench_msg);


        // ddxbb_l = -abl - dwbb.cross(Tbl.translation()) - wbb.cross(vbl);
        // ddxbb_r = -abr - dwbb.cross(Tbr.translation()) - wbb.cross(vbr);
        // //ddxbb_l = -awl - Rwb*(dwbb.cross(Tbl.translation()) - wbb.cross(vbl));
        // //ddxbb_r = -awr - Rwb*(dwbb.cross(Tbr.translation()) - wbb.cross(vbr));

        // temp_twist_msg.header.stamp = ros::Time::now();
        // temp_twist_msg.header.frame_id = base_link_frame;
        // temp_twist_msg.twist.linear.x = ddxbb_l(0);
        // temp_twist_msg.twist.linear.y = ddxbb_l(1);
        // temp_twist_msg.twist.linear.z = ddxbb_l(2);
        // temp_twist_msg.twist.angular.x = 0;
        // temp_twist_msg.twist.angular.y = 0;
        // temp_twist_msg.twist.angular.z = 0;
        // rel_ddxbb_l_pub.publish(temp_twist_msg);


        // temp_twist_msg.header.stamp = ros::Time::now();
        // temp_twist_msg.header.frame_id = base_link_frame;
        // temp_twist_msg.twist.linear.x = ddxbb_r(0);
        // temp_twist_msg.twist.linear.y = ddxbb_r(1);
        // temp_twist_msg.twist.linear.z = ddxbb_r(2);
        // temp_twist_msg.twist.angular.x = 0;
        // temp_twist_msg.twist.angular.y = 0;
        // temp_twist_msg.twist.angular.z = 0;
        // rel_ddxbb_r_pub.publish(temp_twist_msg);

        data_inc = false;
        rate.sleep();
    }
}
