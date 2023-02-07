
#include<Optimizer.h>

using std::cout;
using std::endl; 
void readDistances(const string & filename, vector<double>& ts, vector<double>& distances);
void readTraj(const string& filename, vector<double>& tss, vector<g2o::SE3Quat> &traj);
void readIMU(const string& filename, vector<double>& tss, vector<Eigen::Vector3d>& acces, vector<Eigen::Vector3d>& gyros);
void writeData(const string& filename, vector<double>& tss, vector<Eigen::Vector3d>& pos);

// perform offline analysis for the optimization of acoustic ranging and imu tracking
int main(int argc, char **argv)
{
    if(argc != 6)
    {
        cerr << endl << "Usage: ./opt_analy path_to_distances_measurements path_to_traj_file path_to_imu_file path_to_store_output smartphone_setup" << endl;
        return 1;
    }
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
    vector<double> all_distances; 
    vector<double> dist_ts; 
    readDistances(argv[1], dist_ts, all_distances);
    vector<double> ts_traj;
    vector<g2o::SE3Quat> slam_traj;
    readTraj(argv[2],ts_traj,slam_traj);
    //cout << setprecision(12) << ts_traj[0] << " " << ts_traj[1] << endl; 
    vector<vector<double>> round_distances;
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
            //pos = twb2.cast<double>();
            rot = Rwb2.cast<double>();
            //rot = slam_traj[id_traj].rotation().toRotationMatrix();
            velocity = Vwb2.cast<double>();
        }

        mBias = pImuPreintegratedFromLastFrame->b;

        // reset imu using acoustic 
        if (ac_cnt < dist_ts.size() && ts_imu[i] > dist_ts[ac_cnt*4]){
            Eigen::Vector3d est;
            //ORB_SLAM3::Optimizer::PoseOptimizationDistanceGivenScale(est,ransac_scale[ac_cnt],other_trans,round_distances[ac_cnt]); 
            //ORB_SLAM3::Optimizer::PoseOptimizationDistanceGivenScale(est,1.116997,other_trans,round_distances[ac_cnt]); 
            ORB_SLAM3::Optimizer::PoseOptimizationDistanceRegu(est,prev_ac_pose,1.116997,other_trans,round_distances[ac_cnt]); 
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

