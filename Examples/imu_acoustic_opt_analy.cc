
#include<Optimizer.h>

using std::cout;
using std::endl; 
void readDistances(const string & filename, vector<double>& ts, vector<double>& distances);
void readTrajTS(const string& filename, vector<double>& tss);
void readIMU(const string& filename, vector<double>& tss, vector<Eigen::Vector3d>& acces, vector<Eigen::Vector3d>& gyros);
void writeData(const string& filename, vector<double>& tss, vector<Eigen::Vector3d>& pos);

// perform offline analysis for the optimization of acoustic ranging and imu tracking
int main(int argc, char **argv)
{
    if(argc != 5)
    {
        cerr << endl << "Usage: ./opt_analy path_to_distances_measurements path_to_traj_file path_to_imu_file path_to_store_output" << endl;
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
    readTrajTS(argv[2],ts_traj);
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
    vector<double> imu_ts;
    readIMU(argv[3],imu_ts,imu_acce,imu_gyro);
    // test whether IMU has been successfully extracted
    //for(int i=0;i<3;i++){
        //cout << imu_ts[i] << " " << imu_acce[i].transpose() << " " << imu_gyro[i].transpose() << endl;  
        //cout << imu_ts[i] << " " << imu_acce[i][0] << " " << imu_acce[i][1] << " " << imu_acce[i][2] << " " << 
        //    imu_gyro[i][0] << " " << imu_gyro[i][1] << " " << imu_gyro[i][2] << endl;  
    //}

    // track IMU only
    // find initial position and orientation
    double init_time = 102475;
    double end_time = 102596;
    Eigen::Vector3d pos(-0.0161928, -0.1929543, 0.0376005);
    Eigen::Quaterniond init_q(0.6127005, -0.7877569, -0.0623650, 0.0121559); // w,x,y,z
    Eigen::Matrix3d rot = init_q.toRotationMatrix();
    Eigen::Vector3d velocity; 
    Eigen::Vector3d cal_pos_x; // displacement in each axis
    Eigen::Vector3d g_w(0, 0, 9.8);
    vector<Eigen::Vector3d> imu_only_pose;
    vector<double> cal_ts;
    int ac_cnt = 0;
    for (size_t i=0;i<imu_ts.size()-1;i++){
        if (imu_ts[i]<init_time) continue; 
        if (imu_ts[i]>end_time) break;
        Eigen::Vector3d g_b = rot.transpose()*g_w - rot.transpose()*pos;
        velocity = velocity + (imu_acce[i]-g_b)*(imu_ts[i+1]-imu_ts[i]);
        Eigen::Vector3d delta = velocity*(imu_ts[i+1]-imu_ts[i]);
        pos = rot*delta + pos; 
        Eigen::Vector3d delta_ang = imu_gyro[i]*(imu_ts[i+1]-imu_ts[i]);
        Eigen::Quaterniond delta_quat = Eigen::AngleAxisd(delta_ang[0], Eigen::Vector3d::UnitX())
                                * Eigen::AngleAxisd(delta_ang[1], Eigen::Vector3d::UnitY())
                                * Eigen::AngleAxisd(delta_ang[2], Eigen::Vector3d::UnitZ());
        rot = rot*delta_quat.toRotationMatrix();
        // reset imu using acoustic 
        if (ac_cnt < dist_ts.size() && imu_ts[i] > dist_ts[ac_cnt*4]){
            Eigen::Vector3d est;
            //ORB_SLAM3::Optimizer::PoseOptimizationDistanceGivenScale(est,1.116997,other_trans,round_distances[ac_cnt]); 
            velocity = Eigen::Vector3d(); 
            pos = est;
            ac_cnt++;
            //cout << "time: "<<dist_ts[ac_cnt*4] << "estimated pose: "<<pos.transpose() << endl; 
        }
        cal_ts.push_back(imu_ts[i]);
        imu_only_pose.push_back(pos);
    }
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

void readTrajTS(const string& filename, vector<double>& tss){
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
