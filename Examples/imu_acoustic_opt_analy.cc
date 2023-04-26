
#include<Optimizer.h>

using std::cout;
using std::endl; 
void readDistances(const string & filename, vector<double>& ts, vector<double>& distances);
void readDistances(const string & filename, vector<double>& ts, vector<vector<double>>& distances);
void readPose(const string& filename, Sophus::SE3d &pose);
void readTraj(const string& filename, vector<double>& tss, vector<g2o::SE3Quat> &traj);
void readTrajNoMark(const string& filename, vector<double>& tss, vector<Sophus::SE3d> &traj);
void readIMU(const string& filename, vector<double>& tss, vector<Eigen::Vector3d>& acces, vector<Eigen::Vector3d>& gyros);
void readVel(const string& filename, vector<double>& tss, vector<Eigen::Vector3d>& vel);
void writeData(const string& filename, vector<double>& tss, vector<Eigen::Vector3d>& pos);

// perform offline analysis for the optimization of acoustic ranging and imu tracking
int main(int argc, char **argv)
{
    if(argc != 6 && argc != 8 && argc!=3 && argc!=4)
    {
        cerr << endl << "Usage: ./opt_analy path_to_distances_measurements path_to_traj_file path_to_imu_file path_to_store_output smartphone_setup" << endl;
        cerr << endl << "Or: ./opt_analy path_to_distances_measurements path_to_traj_file path_to_imu_file path_to_velocity path_to_velocity_filtered path_to_store_output smartphone_setup" << endl;
        cerr << endl << "Or: ./opt_analy path_to_distances path_to_store_output" << endl;
        cerr << endl << "Or: ./opt_analy path_to_distances path_to_slam_trajectory path_to_store_output" << endl;
        return 1;
    }

    if (argc==4){
        // calibration, correct the transformation and scale
        Eigen::Vector3d t_mc(-0.1377,-0.01991,0.00765);
        double s = 0.58;
        vector<vector<double>> all_distances; // all distances measurement from user0 to other users, four values form a group, to u1,2,3,4
        vector<double> dist_ts; // timestamp for distance measurement 
        string p_name = argv[1];
        string f_name = p_name + "distances.csv";
        readDistances(f_name, dist_ts, all_distances);
        vector<Sophus::SE3d> poses_u0;
        vector<double> t_u0;
        f_name = p_name+argv[2];
        readTrajNoMark(f_name,t_u0,poses_u0);
        /*for(int i=0;i<10;i++){
            cout<<poses_u0[i].translation().transpose()<<endl;
            cout<<(poses_u0[i].rotationMatrix()*(-s*t_mc)+poses_u0[i].translation()).transpose()<<endl;
            //cout<<poses_u0[i].rotationMatrix()<<endl;
        }*/
        vector<Sophus::SE3d> poses_others;
        vector<Eigen::Quaterniond> q_others;
        vector<Eigen::Vector3d> t_others;
        /*q_others.push_back(Eigen::Quaterniond(-0.0447343000000000,-0.00690550000000000,0.731727500000000,-0.680092600000000)); // eigen format quaternion: w,x,y,z
        t_others.push_back(Eigen::Vector3d(0.568148400000000,-0.504411800000000,-0.228592800000000));
        q_others.push_back(Eigen::Quaterniond(0.000392000000000000,-0.0248527000000000,0.731722300000000,-0.681149500000000));
        t_others.push_back(Eigen::Vector3d(0.672983000000000,-0.385356300000000,-0.0704225000000000));
        q_others.push_back(Eigen::Quaterniond(0.0415944000000000,-0.00769230000000000,0.741368600000000,-0.669763700000000));
        t_others.push_back(Eigen::Vector3d(0.772508600000000,-0.479500600000000,-0.140420300000000));
        q_others.push_back(Eigen::Quaterniond(-0.0194217000000000,0.0523481000000000,0.744451500000000,-0.665337900000000));
        t_others.push_back(Eigen::Vector3d(0.842432800000000,-0.343560700000000,0.0620087000000000));*/
        for(int i=0;i<4;i++){
            //Sophus::SE3d pos0_(q_others[i],t_others[i]);
            Sophus::SE3d pos0_;
            string t_name = argv[1];
            t_name = t_name +"allTrajectory"+std::to_string(i+1)+".txt";
            readPose(t_name,pos0_);
            poses_others.push_back(pos0_);
            //cout<<(pos0_.rotationMatrix()*(-s*t_mc)+pos0_.translation()).transpose()<<endl;
        }
        /*Eigen::Quaterniond q0(-0.0447343000000000,-0.00690550000000000,0.731727500000000,-0.680092600000000); // eigen format quaternion: w,x,y,z
        Eigen::Vector3d t0(0.568148400000000,-0.504411800000000,-0.228592800000000);
        Eigen::Quaterniond q1(0.000392000000000000,-0.0248527000000000,0.731722300000000,-0.681149500000000);
        Eigen::Vector3d t1(0.672983000000000,-0.385356300000000,-0.0704225000000000);
        Sophus::SE3d pos0_(q0,t0);
        Sophus::SE3d pos1_(q1,t1);
        //Eigen::Vector3d t_wm_0=pos0_.rotationMatrix()*(-s*t_mc)+pos0_.translation();
        //Eigen::Vector3d t_wm_1=pos1_.rotationMatrix()*(-s*t_mc)+pos1_.translation();
        //cout << t_wm_0.transpose() << endl;
        //cout << t_wm_1.transpose() << endl;*/

        // optimze the scale and transformation
        ORB_SLAM3::Optimizer::CalibOptimization(t_mc, &s, poses_u0, poses_others, all_distances);
        cout << t_mc.transpose() << endl;
        cout << s << endl;
    }
    else if (argc==3){
        vector<Eigen::Vector3d> poses_other_users_m; // the poses of other users of microphone
        vector<Sophus::SE3d> poses_others;
        //Eigen::Vector3d t_mc(-0.180431, -0.0136339,  0.0612767);
        //double s = 0.537344;
        Eigen::Vector3d t_mc(-0.1377,-0.01991,0.00765);
        //Eigen::Vector3d t_mc(-0.140562, -0.107471, 0.0575822);
        double s = 0.585996;
        /*vector<Eigen::Quaterniond> q_others;
        vector<Eigen::Vector3d> t_others;
        q_others.push_back(Eigen::Quaterniond(-0.0447343000000000,-0.00690550000000000,0.731727500000000,-0.680092600000000)); // eigen format quaternion: w,x,y,z
        t_others.push_back(Eigen::Vector3d(0.568148400000000,-0.504411800000000,-0.228592800000000));
        q_others.push_back(Eigen::Quaterniond(0.000392000000000000,-0.0248527000000000,0.731722300000000,-0.681149500000000));
        t_others.push_back(Eigen::Vector3d(0.672983000000000,-0.385356300000000,-0.0704225000000000));
        q_others.push_back(Eigen::Quaterniond(0.0415944000000000,-0.00769230000000000,0.741368600000000,-0.669763700000000));
        t_others.push_back(Eigen::Vector3d(0.772508600000000,-0.479500600000000,-0.140420300000000));
        q_others.push_back(Eigen::Quaterniond(-0.0194217000000000,0.0523481000000000,0.744451500000000,-0.665337900000000));
        t_others.push_back(Eigen::Vector3d(0.842432800000000,-0.343560700000000,0.0620087000000000));*/
        for(int i=0;i<4;i++){
            //Sophus::SE3d pos0_(q_others[i],t_others[i]);
            Sophus::SE3d pos0_;
            string f_name = argv[1];
            f_name = f_name +"allTrajectory"+std::to_string(i+1)+".txt";
            readPose(f_name,pos0_);
            poses_others.push_back(pos0_);
            poses_other_users_m.push_back(pos0_.rotationMatrix()*(-s*t_mc)+pos0_.translation());
            cout << poses_other_users_m[i].transpose() << endl;
        }

        /*poses_other_users_m.push_back(Eigen::Vector3d(0.488039116653866,-0.495076111005820,-0.233785510526573));
        poses_other_users_m.push_back(Eigen::Vector3d(0.5926490829769997, -0.3830629034884827, -0.07895588842562916));
        poses_other_users_m.push_back(Eigen::Vector3d(0.6931207349079481, -0.4792718017649787, -0.15555737822975801));
        poses_other_users_m.push_back(Eigen::Vector3d(0.7641037561887519, -0.3296247373684681, 0.04779720989557681));*/

        vector<vector<double>> all_distances; // all distances measurement from user0 to other users, four values form a group, to u1,2,3,4
        vector<double> dist_ts; // timestamp for distance measurement 
        string p_name = argv[1];
        string f_name = p_name + "distances.csv";
        readDistances(f_name, dist_ts, all_distances);

        vector<Eigen::Vector3d> est_poses;
        // iterate over distances and conduct optimization
        Eigen::Vector3d est(0.14453414, -0.45255567, -0.06184022); // initial microphone pos
        for(int ac_cnt=0; ac_cnt<all_distances.size(); ++ac_cnt){
            ORB_SLAM3::Optimizer::PoseOptimizationDistanceGivenScale(est,1/0.58,poses_other_users_m,all_distances[ac_cnt]); 
            // the output is microphone's pose, convert to imu body pos in other scripts
            est_poses.push_back(est);
        }
        writeData(p_name+argv[2],dist_ts,est_poses);
    }
    else{
    /*scale: 1.116997
    rot:
    [[ 0.999  0.011 -0.031]
    [-0.01   0.999  0.044]
    [ 0.032 -0.043  0.999]]
    tran:
    [[0.226]
    [1.753]
    [1.348]]
    0.045328,1.116997,0.031723
    */
        Eigen::Vector3d est_trans;
        vector<Eigen::Vector3d> other_trans; 
        other_trans.push_back(Eigen::Vector3d(-0.1262477, -0.3380130, 0.0708579)); // u1
        other_trans.push_back(Eigen::Vector3d(-0.2596412, -0.1879242, -0.1486742)); // u2
        other_trans.push_back(Eigen::Vector3d(-0.3530733, -0.2923105, 0.1471222)); // u3
        other_trans.push_back(Eigen::Vector3d(-0.5457934, -0.2865348, -0.1410383)); // u4
        vector<double> all_distances; // all distances measurement from user0 to other users, four values form a group, to u1,2,3,4
        vector<double> dist_ts; // timestamp for distance measurement, four values in a group, the same 
        readDistances(argv[1], dist_ts, all_distances);
        vector<double> ts_traj;
        vector<g2o::SE3Quat> slam_traj;
        readTraj(argv[2],ts_traj,slam_traj);
        //cout << setprecision(12) << ts_traj[0] << " " << ts_traj[1] << endl; 
        vector<vector<double>> round_distances; // each item contain four distances, from user 0 to u1,2,3,4
        for(int i=0;i<all_distances.size()/4;i++){
            vector<double> dist;
            for(int j=0;j<4;j++){
                dist.push_back(all_distances[i*4+j]);
                //cout << all_distances[i*4+j] << " " ; 
            }
            //cout << endl; 
            round_distances.push_back(dist);
        }
        vector<Eigen::Vector3d> imu_acce;
        vector<Eigen::Vector3d> imu_gyro;
        vector<double> ts_imu;
        readIMU(argv[3],ts_imu,imu_acce,imu_gyro);
        // test whether IMU has been successfully extracted
        //for(int i=0;i<3;i++){
            //cout << ts_imu[i] << " " << imu_acce[i].transpose() << " " << imu_gyro[i].transpose() << endl;  
            //cout << ts_imu[i] << " " << imu_acce[i][0] << " " << imu_acce[i][1] << " " << imu_acce[i][2] << " " << 
            //    imu_gyro[i][0] << " " << imu_gyro[i][1] << " " << imu_gyro[i][2] << endl;  
        //}

        if (argc==6){
            // track IMU only
            // find initial position and orientation
            double init_time = 102475;
            double end_time = 102596;
            Eigen::Vector3d pos(-0.0161928, -0.1929543, 0.0376005);
            Eigen::Vector3d prev_ac_pose = pos; 
            Eigen::Quaterniond init_q(0.6127005, -0.7877569, -0.0623650, 0.0121559); // w,x,y,z
            Eigen::Matrix3d rot = init_q.toRotationMatrix();
            Eigen::Vector3d velocity; 
            Eigen::Vector3d cal_pos_x; // displacement in each axis
            Eigen::Vector3d g_w(0, 0, 9.8);
            vector<Eigen::Vector3d> imu_only_pose;
            vector<double> cal_ts;
            int ac_cnt = 0;
            int i = 0;
            while (i<ts_imu.size()&&ts_imu[i]<ts_traj[0]) i++; 
            //cout << i << " " << ts_imu[i] << endl;

            // load imu parameters
            ORB_SLAM3::Settings* settings = new ORB_SLAM3::Settings(argv[5],ORB_SLAM3::System::eSensor::IMU_MONOCULAR);
            ORB_SLAM3::IMU::Calib* mImuCalib;
            // parse imu calibration
            Sophus::SE3f Tbc = settings->Tbc();
            double mImuFreq = settings->imuFrequency();
            float Ng = settings->noiseGyro();
            float Na = settings->noiseAcc();
            float Ngw = settings->gyroWalk();
            float Naw = settings->accWalk();

            const float sf = sqrt(mImuFreq);
            mImuCalib = new ORB_SLAM3::IMU::Calib(Tbc,Ng*sf,Na*sf,Ngw/sf,Naw/sf);
            //cout<<mImuCalib->mTcb.translation() << endl; 
            ORB_SLAM3::IMU::Bias mBias = ORB_SLAM3::IMU::Bias(); 

            // assign calculated scale using RANSAC
            vector<double> ransac_scale { 1.2413229473591059, 1.1314382806649064, 1.1309707195544323, 1.1345153842814706, 1.137540360947597, 1.137013995411847, 1.1386840089531796, 1.1345153842814706, 1.1360782908963805, 1.1371614084081503, 1.1353872216950096, 1.1334964433951018, 1.1368516933598143, 1.136851693359814, 1.1339495909761326, 1.1391032675625068, 1.1384381720298644, 1.1387902457607488, 1.1391032675625066, 1.132651817441362, 1.1330533142302541, 1.1390742746833842, 1.1365262521003545, 1.1397132572314361, 1.1376123849741195};

            for (size_t id_traj=1;id_traj<ts_traj.size();id_traj++){
                vector<ORB_SLAM3::IMU::Point> mvImuFromLastFrame;
                while (i<ts_imu.size()&&ts_imu[i]<ts_traj[id_traj]) {
                    ORB_SLAM3::IMU::Point p(imu_acce[i][0],imu_acce[i][1],imu_acce[i][2],imu_gyro[i][0],imu_gyro[i][1],imu_gyro[i][2],ts_imu[i]);
                    mvImuFromLastFrame.push_back(p);
                    // self defined double integration
                    /*Eigen::Vector3d g_b = rot.transpose()*g_w - rot.transpose()*pos;
                    velocity = velocity + (imu_acce[i]-g_b)*(ts_imu[i+1]-ts_imu[i]);
                    Eigen::Vector3d delta = velocity*(ts_imu[i+1]-ts_imu[i]);
                    pos = rot*delta + pos; 
                    Eigen::Vector3d delta_ang = imu_gyro[i]*(ts_imu[i+1]-ts_imu[i]);
                    Eigen::Quaterniond delta_quat = Eigen::AngleAxisd(delta_ang[0], Eigen::Vector3d::UnitX())
                                            * Eigen::AngleAxisd(delta_ang[1], Eigen::Vector3d::UnitY())
                                            * Eigen::AngleAxisd(delta_ang[2], Eigen::Vector3d::UnitZ());
                    rot = rot*delta_quat.toRotationMatrix();*/
                    i++; 
                }
                //cout << i << " " << mvImuFromLastFrame.size() << endl; 
                // use ORB SLAM method
                ORB_SLAM3::IMU::Preintegrated* pImuPreintegratedFromLastFrame = new ORB_SLAM3::IMU::Preintegrated(mBias,*mImuCalib);
                for(int j=0;j<mvImuFromLastFrame.size()-1;j++){
                    Eigen::Vector3f acc = (mvImuFromLastFrame[j].a+mvImuFromLastFrame[j+1].a)*0.5f;
                    Eigen::Vector3f angVel = (mvImuFromLastFrame[j].w+mvImuFromLastFrame[j+1].w)*0.5f;
                    float tstep = mvImuFromLastFrame[j+1].t-mvImuFromLastFrame[j].t;
                    pImuPreintegratedFromLastFrame->IntegrateNewMeasurement(acc,angVel,tstep);
                }
                const Eigen::Vector3f twb1 = pos.cast<float>();
                const Eigen::Matrix3f Rwb1 = rot.cast<float>();
                const Eigen::Vector3f Vwb1 = velocity.cast<float>();
                const Eigen::Vector3f Gz(0, 0, -ORB_SLAM3::IMU::GRAVITY_VALUE);
                const float t12 = ts_traj[id_traj] - ts_traj[id_traj-1];

                Eigen::Matrix3f Rwb2 = ORB_SLAM3::IMU::NormalizeRotation(Rwb1 * pImuPreintegratedFromLastFrame->GetDeltaRotation(mBias));
                Eigen::Vector3f twb2 = twb1 + Vwb1*t12 + 0.5f*t12*t12*Gz+ Rwb1 * pImuPreintegratedFromLastFrame->GetDeltaPosition(mBias);
                Eigen::Vector3f Vwb2 = Vwb1 + t12*Gz + Rwb1 * pImuPreintegratedFromLastFrame->GetDeltaVelocity(mBias);

                
                // check static case
                double ave_acc_mag = 0;
                for(int j=0;j<mvImuFromLastFrame.size();j++){
                    ave_acc_mag += mvImuFromLastFrame[j].a.norm();
                }
                ave_acc_mag /= mvImuFromLastFrame.size();
                if (abs(ave_acc_mag-ORB_SLAM3::IMU::GRAVITY_VALUE)<0.1 ){
                    // pos, rot, not change, set velocity to 0
                    velocity = Eigen::Vector3d(0,0,0); 
                }
                else {
                    pos = twb2.cast<double>();
                    rot = Rwb2.cast<double>();
                    //rot = slam_traj[id_traj].rotation().toRotationMatrix();
                    velocity = Vwb2.cast<double>();
                }

                mBias = pImuPreintegratedFromLastFrame->b;

                // reset imu using acoustic 
                if (ac_cnt < dist_ts.size()/4 && ts_imu[i] > dist_ts[ac_cnt*4]){
                    Eigen::Vector3d est;
                    //ORB_SLAM3::Optimizer::PoseOptimizationDistanceGivenScale(est,ransac_scale[ac_cnt],other_trans,round_distances[ac_cnt]); 
                    ORB_SLAM3::Optimizer::PoseOptimizationDistanceGivenScale(est,1.116997,other_trans,round_distances[ac_cnt]); 
                    //ORB_SLAM3::Optimizer::PoseOptimizationDistanceRegu(est,prev_ac_pose,1.116997,other_trans,round_distances[ac_cnt]); 
                    velocity = Eigen::Vector3d(0,0,0); 
                    pos = est;
                    //rot = slam_traj[id_traj].rotation().toRotationMatrix();
                    prev_ac_pose = est; 
                    ac_cnt++;
                    //cout << "time: "<<dist_ts[ac_cnt*4] << "estimated pose: "<<pos.transpose() << "reset velocity" << velocity.transpose() << endl; 
                }
                cal_ts.push_back(ts_traj[id_traj]);
                imu_only_pose.push_back(pos);
            }
            //cout << mBias.bax << " " << mBias.bay << " " << mBias.baz << endl; 
            writeData(argv[4],cal_ts,imu_only_pose);
            /*for(size_t i=0;i<round_distances.size();i++){
                ORB_SLAM3::Optimizer::PoseOptimizationDistanceGivenScale(est_trans,1.116997,other_trans,round_distances[i]); 
                cout << "estimate pose: " << est_trans.transpose() << endl;
            }*/
        }
        else if (argc==8){
            // precalculated velocity with/without filtering
            vector<double> ts_vel_naive;
            vector<Eigen::Vector3d> vel_naive;
            vector<double> ts_vel_filter;
            vector<Eigen::Vector3d> vel_filter;
            readVel(argv[4],ts_vel_naive,vel_naive);
            readVel(argv[5],ts_vel_filter,vel_filter);

            vector<Eigen::Vector3d> opt_pos(ts_traj.size()); // store optimized pose
            vector<Eigen::Vector3d> opt_vel(ts_traj.size()); // store optimized velocity
            vector<Eigen::Vector3d> delta_vel(ts_traj.size()); // store relative velocity between i and i-1
            vector<Eigen::Vector3d> delta_pos(ts_traj.size()); // store relative position between i and i-1
            delta_vel[0] = Eigen::Vector3d(0,0,0);
            delta_pos[0] = Eigen::Vector3d(0,0,0);
            Eigen::Vector3d pos(-0.0161928, -0.1929543, 0.0376005); // init pose estimated by slam
            opt_pos[0] = pos; 
            opt_vel[0] = Eigen::Vector3d(0,0,0);

            int last_id_traj = 0; // the id of the trajectory when last acoustic ranging happens
            int id_ac = 0;
            int id_imu = 0;
            while (ts_imu[id_imu]<ts_traj[0])
                id_imu++;

            // optimize all frames from last acoustic ranging to current acoustic ranging
            /*for (int id_traj=1;id_traj<(int)ts_traj.size();id_traj++){
                Eigen::Vector3d delta_v(0,0,0);
                Eigen::Vector3d delta_p(0,0,0);
                int start_id = id_imu;
                while (ts_imu[id_imu]<ts_traj[id_traj])
                    id_imu++;
                for(int temp_id_imu=start_id;temp_id_imu<=id_imu;temp_id_imu++) {
                    double imu_gap = ts_imu[temp_id_imu]-ts_imu[temp_id_imu-1];
                    if (ts_imu[temp_id_imu-1]<ts_traj[id_traj-1] ){
                        // the first imu measurement
                        double true_gap = ts_imu[temp_id_imu]-ts_traj[id_traj-1];
                        Eigen::Vector3d temp_v = (vel_filter[temp_id_imu]-vel_filter[temp_id_imu-1])*true_gap/imu_gap;
                        delta_v += temp_v;
                        delta_p += (vel_filter[temp_id_imu] - temp_v*0.5)*true_gap;
                    }
                    else if (ts_imu[temp_id_imu]>ts_traj[id_traj]){
                        // the last imu measurement
                        double true_gap = ts_traj[id_traj]-ts_imu[temp_id_imu-1];
                        Eigen::Vector3d temp_v = (vel_filter[temp_id_imu]-vel_filter[temp_id_imu-1])*true_gap/imu_gap;
                        delta_v += temp_v;
                        delta_p += (vel_filter[temp_id_imu-1] + temp_v*0.5)*true_gap;
                    }
                    else{
                        Eigen::Vector3d temp_v = (vel_filter[temp_id_imu]-vel_filter[temp_id_imu-1]);
                        delta_v += temp_v;
                        delta_p += (vel_filter[temp_id_imu-1] + temp_v*0.5)*imu_gap;
                    }
                }
                delta_vel[id_traj] = delta_v/1.116997;
                delta_pos[id_traj] = delta_p/1.116997; 
                opt_pos[id_traj] = opt_pos[id_traj-1] + delta_pos[id_traj]; 
                opt_vel[id_traj] = opt_vel[id_traj-1] + delta_vel[id_traj];
                
                // conduct imu-acoustic optimization 
                if (id_ac*4 < (int)dist_ts.size() && ts_imu[id_imu] > dist_ts[id_ac*4]){
                    // [last_id_traj,id_traj], total size: id_traj-last_id_traj+1, TODO:  the first one will not be optimized, just for reference 
                    vector<Eigen::Vector3d> period_pos(id_traj-last_id_traj+1);
                    copy(opt_pos.begin()+last_id_traj, opt_pos.begin()+id_traj+1, period_pos.begin());
                    vector<Eigen::Vector3d> period_vel(id_traj-last_id_traj+1);
                    copy(opt_vel.begin()+last_id_traj, opt_vel.begin()+id_traj+1, period_vel.begin());
                    vector<Eigen::Vector3d> period_delta_vel(id_traj-last_id_traj+1);
                    copy(delta_vel.begin()+last_id_traj, delta_vel.begin()+id_traj+1, period_delta_vel.begin());
                    vector<Eigen::Vector3d> period_delta_pos(id_traj-last_id_traj+1);
                    copy(delta_pos.begin()+last_id_traj, delta_pos.begin()+id_traj+1, period_delta_pos.begin());

                    ORB_SLAM3::Optimizer::IMUAcousticOptimization(period_pos,period_vel,period_delta_pos,period_delta_vel,1.116997,other_trans,round_distances[id_ac]); 

                    // update the estimation
                    for(int i=0;i<id_traj-last_id_traj+1;i++){
                        opt_pos[last_id_traj+i] = period_pos[i];
                        opt_vel[last_id_traj+1] = period_vel[i]; 
                    }

                    last_id_traj = id_traj; 
                    id_ac++; 
                }
                
            }*/
        
            // optimize only frames of acoustic ranging time
            Eigen::Vector3d ac_delta_p = Eigen::Vector3d(0,0,0);
            // establish vectors to store acoustic related data
            vector<Eigen::Vector3d> ac_poses;
            ac_poses.push_back(pos); // include the first time slam losts
            vector<Eigen::Vector3d> ac_delta_poses;
            vector<vector<double>> ac_dists; 
            for (int id_traj=1;id_traj<(int)ts_traj.size();id_traj++){
                Eigen::Vector3d delta_p(0,0,0);
                int start_id = id_imu;
                while (ts_imu[id_imu]<ts_traj[id_traj])
                    id_imu++;
                for(int temp_id_imu=start_id;temp_id_imu<=id_imu;temp_id_imu++) {
                    double imu_gap = ts_imu[temp_id_imu]-ts_imu[temp_id_imu-1];
                    if (ts_imu[temp_id_imu-1]<ts_traj[id_traj-1] ){
                        // the first imu measurement
                        double true_gap = ts_imu[temp_id_imu]-ts_traj[id_traj-1];
                        Eigen::Vector3d temp_v = (vel_filter[temp_id_imu]-vel_filter[temp_id_imu-1])*true_gap/imu_gap;
                        delta_p += (vel_filter[temp_id_imu] - temp_v*0.5)*true_gap;
                    }
                    else if (ts_imu[temp_id_imu]>ts_traj[id_traj]){
                        // the last imu measurement
                        double true_gap = ts_traj[id_traj]-ts_imu[temp_id_imu-1];
                        Eigen::Vector3d temp_v = (vel_filter[temp_id_imu]-vel_filter[temp_id_imu-1])*true_gap/imu_gap;
                        delta_p += (vel_filter[temp_id_imu-1] + temp_v*0.5)*true_gap;
                    }
                    else{
                        Eigen::Vector3d temp_v = (vel_filter[temp_id_imu]-vel_filter[temp_id_imu-1]);
                        delta_p += (vel_filter[temp_id_imu-1] + temp_v*0.5)*imu_gap;
                    }
                }
                delta_pos[id_traj] = delta_p/1.116997; 
                opt_pos[id_traj] = opt_pos[id_traj-1] + delta_pos[id_traj]; 
                ac_delta_p += delta_pos[id_traj];

                // conduct imu-acoustic optimization 
                if (id_ac*4 < (int)dist_ts.size() && ts_imu[id_imu] > dist_ts[id_ac*4]){
                    ac_poses.push_back(opt_pos[id_traj]);
                    ac_delta_poses.push_back(ac_delta_p);
                    ac_dists.push_back(round_distances[id_ac]);

                    ORB_SLAM3::Optimizer::IMUAcousticKeyOptimization(ac_poses,ac_delta_poses,ac_dists,1.116997,other_trans); 

                    // only update the lastest pose, since previous cannot be changed
                    opt_pos[id_traj] = ac_poses[ac_poses.size()-1];

                    last_id_traj = id_traj; 
                    id_ac++; 
                    ac_delta_p = Eigen::Vector3d(0,0,0);
                }
                
            }

            writeData(argv[6],ts_traj,opt_pos);
            //writeData("./evaluation/lost_sim_1/opt_v.csv",ts_traj,opt_vel);
            //writeData("./evaluation/lost_sim_1/delta_v.csv",ts_traj,delta_vel);
            //writeData("./evaluation/lost_sim_1/delta_p.csv",ts_traj,delta_pos);
        }
    }

}


void readDistances(const string & filename, vector<double>& ts, vector<double>& distances){
    ifstream fin(filename);
    if (!fin) {
        cout << "cannot find " << filename << "!" << endl;
        return;
    }
    int cnt = 0;
    string line;
    std::getline(fin, line);
    cout << line << endl; 
    while (!fin.eof())
    {
        string ts1, u1id, tx1, ty1, tz1, ts2, u2id, tx2, ty2, tz2, dist;
        fin >> ts1 >> u1id >> tx1 >> ty1 >> tz1 >> ts2 >> u2id >> tx2 >> ty2 >> tz2 >> dist;
        if (cnt < 0)
            cout << " ts1: " << ts1 << " tx1: " << tx1 << " ty1: " << ty1 << " tz1: " << tz1
            << " ts2: " << ts2 << " tx2: " << tx2 << " ty2: " << ty2 << " tz2: " << tz2 << " dist: " << dist << endl;
        cnt++;
        if (fin.good() == false)
            break;
        int u1id_ = (int)atof(u1id.c_str());
        if (u1id_ != 0) continue;
        distances.push_back(atof(dist.c_str()));
        ts.push_back(atof(ts1.c_str()));
    }
    fin.close();
}

void readDistances(const string & filename, vector<double>& ts, vector<vector<double>>& distances){
    ifstream fin(filename);
    if (!fin) {
        cout << "cannot find " << filename << "!" << endl;
        return;
    }
    int cnt = 0;
    string line;
    //std::getline(fin, line);
    //cout << line << endl; 
    vector<double> prev;
    while (!fin.eof())
    {
        std::getline(fin, line);
        string ts_d, dist;
        stringstream ss(line); 
        char delim = ',';
        std::getline(ss,ts_d,delim);
        vector<double> dist_temp;
        for(int i=0;i<4;i++){
            std::getline(ss,dist,delim);
            double d = atof(dist.c_str());
            if (cnt<1 || (d>0 && abs(d-prev[i])<0.3))
                dist_temp.push_back(d);
            else
                dist_temp.push_back(prev[i]);
        }

        if (cnt < 0)
            cout << " ts: " << ts_d << " dist1: " << dist_temp[0] << " dist2: " << dist_temp[1] << " dist3: " << dist_temp[2] << " dist4: " << dist_temp[3] << endl;
        cnt++;
        if (fin.good() == false)
            break;

        prev = dist_temp;
        distances.push_back(dist_temp);
        ts.push_back(atof(ts_d.c_str()));
    }
    fin.close();
}

void readPose(const string& filename, Sophus::SE3d &pose){
    ifstream fin(filename);
    if (!fin) {
        cout << "cannot find " << filename << "!" << endl;
        return;
    }
    int cnt = 0;
    string line;
    //std::getline(fin, line);
    //cout << line << endl; 
    vector<string> lines;
    while (!fin.eof())
    {
        std::getline(fin, line);
        if (fin.good() == false)
            break;
        lines.push_back(line);
    }
    fin.close();
    //cout << lines[lines.size()-2] << endl; 
    stringstream ss(lines[lines.size()-2]); 
    char delim = ' ';
    string ts, tx, ty, tz, qx, qy, qz, qw;
    std::getline(ss,ts,delim);
    std::getline(ss,tx,delim);
    std::getline(ss,ty,delim);
    std::getline(ss,tz,delim);
    std::getline(ss,qx,delim);
    std::getline(ss,qy,delim);
    std::getline(ss,qz,delim);
    std::getline(ss,qw,delim);
    
    Eigen::Vector3d pos(atof(tx.c_str()), atof(ty.c_str()), atof(tz.c_str()));
    Eigen::Quaterniond q(atof(qw.c_str()), atof(qx.c_str()), atof(qy.c_str()), atof(qz.c_str()) );
    pose = Sophus::SE3d(q,pos);
}

void readTraj(const string& filename, vector<double>& tss, vector<g2o::SE3Quat> &traj){
    // read timestamps of the recorded trajectory 
    ifstream fin(filename);
    if (!fin) {
        cout << "cannot find " << filename << "!" << endl;
        return;
    }
    int cnt = 0; 
    string line;
    std::getline(fin, line);
    cout << line << endl; // skip header line
    while (!fin.eof())
    {
        if (fin.good() == false)
            break;
        string trajectory_time, tx, ty, tz, qx, qy, qz, qw, mark;
        fin >> trajectory_time >> tx >> ty >> tz >> qx >> qy >> qz >> qw >> mark;
        cnt++; 
        tss.push_back(atof(trajectory_time.c_str()));
        Eigen::Vector3d pos(atof(tx.c_str()), atof(ty.c_str()), atof(tz.c_str()));
        Eigen::Quaterniond q(atof(qw.c_str()), atof(qx.c_str()), atof(qy.c_str()), atof(qz.c_str()) );
        g2o::SE3Quat pose(q,pos);
        traj.push_back(pose);
    }
    
    fin.close();

    return;
}

void readTrajNoMark(const string& filename, vector<double>& tss, vector<Sophus::SE3d> &traj){
    // read timestamps of the recorded trajectory 
    ifstream fin(filename);
    if (!fin) {
        cout << "cannot find " << filename << "!" << endl;
        return;
    }

    int cnt = 0;
    string line;
    //std::getline(fin, line);
    //cout << line << endl; // skip header line
    while (!fin.eof())
    {
        std::getline(fin, line);
        if (fin.good() == false || line.size()<2)
            break;
        //cout << line << endl;
        string trajectory_time, tx, ty, tz, qx, qy, qz, qw;
        stringstream ss(line);
        char delim = ',';
        std::getline(ss,trajectory_time,delim);
        std::getline(ss,tx,delim);
        std::getline(ss,ty,delim);
        std::getline(ss,tz,delim);
        std::getline(ss,qx,delim);
        std::getline(ss,qy,delim);
        std::getline(ss,qz,delim);
        std::getline(ss,qw,delim);
        cnt++; 
        tss.push_back(atof(trajectory_time.c_str()));
        Eigen::Vector3d pos(atof(tx.c_str()), atof(ty.c_str()), atof(tz.c_str()));
        Eigen::Quaterniond q(atof(qw.c_str()), atof(qx.c_str()), atof(qy.c_str()), atof(qz.c_str()) );
        Sophus::SE3d pose(q,pos);
        traj.push_back(pose);
    }

    fin.close();

    return;
}

void readIMU(const string& filename, vector<double>& tss, vector<Eigen::Vector3d>& acces, vector<Eigen::Vector3d>& gyros){
    ifstream fin(filename);
    if (!fin) {
        cout << "cannot find " << filename << "!" << endl;
        return;
    }
    int cnt = 0;
    while (!fin.eof())
    {
        string line; 
        string ts, gyro_x, gyro_y, gyro_z, acce_x, acce_y, acce_z;
        std::getline(fin, line);
        if (fin.good() == false)
            break;
        stringstream ss(line); 
        char delim = ',';
        std::getline(ss,ts,delim);
        std::getline(ss,gyro_x,delim);
        std::getline(ss,gyro_y,delim);
        std::getline(ss,gyro_z,delim);
        std::getline(ss,acce_x,delim);
        std::getline(ss,acce_y,delim);
        std::getline(ss,acce_z,delim);
        if (cnt < 0){
            cout << line << endl;
            cout << " ts: " << ts << " gyro_x: " << gyro_x << " gyro_y: " << gyro_y << " gyro_z: " << gyro_z
            << " acce_x: " << acce_x << " acce_y: " << acce_y << " acce_z: " << acce_z << endl;
            //cout << ts.c_str() << endl;
            //printf( "%.6f",atof(ts.c_str()));
        }
        cnt++;
        tss.push_back(atof(ts.c_str()));
        gyros.push_back(Eigen::Vector3d(atof(gyro_x.c_str()),atof(gyro_y.c_str()),atof(gyro_z.c_str())));
        acces.push_back(Eigen::Vector3d(atof(acce_x.c_str()),atof(acce_y.c_str()),atof(acce_z.c_str())));
    }
    fin.close();
}

void readVel(const string& filename, vector<double>& tss, vector<Eigen::Vector3d>& vel){
    ifstream fin(filename);
    if (!fin) {
        cout << "cannot find " << filename << "!" << endl;
        return;
    }
    int cnt = 0;
    while (!fin.eof())
    {
        string line; 
        string ts, vel_x, vel_y, vel_z;
        std::getline(fin, line);
        if (fin.good() == false)
            break;
        stringstream ss(line); 
        char delim = ',';
        std::getline(ss,ts,delim);
        std::getline(ss,vel_x,delim);
        std::getline(ss,vel_y,delim);
        std::getline(ss,vel_z,delim);
        if (cnt == 14800){
            cout << line << endl;
            cout << " ts: " << ts << " vel_x: " << vel_x << " vel_y: " << vel_y << " vel_z: " << vel_z << endl;
            //cout << ts.c_str() << endl;
            //printf( "%.6f",atof(ts.c_str()));
        }
        cnt++;
        tss.push_back(atof(ts.c_str()));
        vel.push_back(Eigen::Vector3d(atof(vel_x.c_str()),atof(vel_y.c_str()),atof(vel_z.c_str())));
    }
    fin.close();
}

void writeData(const string& filename, vector<double>& tss, vector<Eigen::Vector3d>& pos){
    ofstream f;
    f.open(filename.c_str());
    //f << "#timeStamp tx ty tz" << endl; 
    f << fixed;
    for (size_t i = 0; i < pos.size(); i++)
    {
        Eigen::Vector3d tp = pos[i]; 
        f << setprecision(6) << tss[i] << "," << tp[0] << "," << tp[1] << "," << tp[2] << endl;
    }

    f.close();
    std::cout << endl << filename << " saved!" << endl;
}

