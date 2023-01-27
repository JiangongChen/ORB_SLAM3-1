
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>
#include<Converter.h>
#include<Socket/server.h>
#include<Frame.h>
#include<Optimizer.h>

using namespace std;

void DrawTrajectory(const vector<Sophus::SE3f> & esti, const vector<Sophus::SE3f>& gt, vector<int> gt_points);
// for multiple users
void DrawTrajectory(const vector<vector<Sophus::SE3f>> & esti, vector<vector<int>> gt_points);
void SaveTrajectory(const string& filename, const vector<Sophus::SE3f>& trajectory, vector<double> timeStamps, vector<int> trajGTpts);
void SaveIMU(const string & filename, const vector<ORB_SLAM3::IMU::Point>& imuMeas);
void SaveDistanceMeasurement(const string & filename, const vector<double> ts1, const vector<int> users1, const vector<Eigen::Vector3d> pos1, const vector<double> ts2, const vector<int> users2, const vector<Eigen::Vector3d> pos2, vector<double> distances);


int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl << "Usage: ./mono_pixel path_to_vocabulary path_to_settings" << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System *SLAM = new ORB_SLAM3::System(argv[1],argv[2],ORB_SLAM3::System::IMU_MONOCULAR,true,0,"edge sequence");

    // Vector for tracking time statistics
    vector<vector<double>> vTimesTrack;
    vector<vector<Sophus::SE3f>> trajectory; 
    vector<vector<double>> vTimestamps; 
    vector<vector<int>> trajectory_gt_points; 

    std::cout << endl << "-------" << endl;

    // Main loop
    Server* server = new Server(argv[2], SLAM); 

    server->StartListening(); 

    usleep(1000000); 
    while(server->listenFlag){
        if (server->CheckAcoustic()){
            vector<double> distances = server->CalAcoustic(); 
            if (distances.size() < server->max_client_num - 1) continue; 

            // optimize user 0's pose using acoustic ranging results
            Sophus::SE3f pose; 
            int poseId = server->clients[0]->getLatestTraj(pose); 
            // get Twc
            Eigen::Vector3d est_trans = pose.inverse().translation().cast<double>(); 
            cout << "user 0 " << est_trans.transpose() << endl; 
            vector<Eigen::Vector3d> other_trans; 
            for (int i=1;i<server->max_client_num;i++){
                Sophus::SE3f o_pose; 
                int idx = server->clients[i]->getLatestTraj(o_pose); // the trajectory could be empty matrix, handling that
                if (idx == -1) //handle invalid pose, e.g., has not been initialized
                    continue; 
                Eigen::Vector3d pose_other = o_pose.inverse().translation().cast<double>();
                other_trans.push_back(pose_other); 
                cout << "user " << i << " " << other_trans[i-1].transpose() << endl; 
                server->hist_poses_1.push_back(est_trans);
                server->hist_poses_2.push_back(pose_other);
                server->hist_users_1.push_back(0);
                server->hist_users_2.push_back(i); 
                server->hist_TS_1.push_back(server->clients[0]->getLatestTS()); 
                server->hist_TS_2.push_back(server->clients[i]->getLatestTS()); 
                server->hist_distances.push_back(distances[i-1]); 
            }
            if (other_trans.size()!= server->max_client_num-1) continue; 
            ORB_SLAM3::Optimizer::PoseOptimizationDistanceGivenScale(est_trans,server->est_scale,other_trans,distances); 
            // rewrite the trajectory, convert back to Tcw
            Eigen::Matrix3f R = pose.rotationMatrix();
            Eigen::Vector3f est_trans_inv = -R * est_trans.cast<float>();
            Sophus::SE3f newpose = Sophus::SE3f(R,est_trans_inv); 
            cout << "estimate pose: " << est_trans.transpose() << " estimate scale: " << server->est_scale << endl; // << " mat: " << newmat; 
            server->clients[0]->rewriteTraj(poseId,newpose); 
            //server->hist_scales.push_back(server->est_scale); 
        }
        else
            usleep(30000); 
        usleep(1000000);
    }

    // request close of the server
    server->Close(); 

    std::cout << "request stop thread" << endl; 
    // Stop all threads
    SLAM->Shutdown();

    // get tracking time statistics from each client
    for (size_t id=0; id<server->clients.size();id++) {
        trajectory.push_back(server->clients[id]->trajectory); 
        vTimesTrack.push_back(server->clients[id]->vTimesTrack); 
        vTimestamps.push_back(server->clients[id]->vTimestamps);
        trajectory_gt_points.push_back(server->clients[id]->trajectory_gt_points);
    }

    std::cout << "-------" << endl << endl;
    for (size_t i=0; i< vTimesTrack.size();i++){
        // Tracking time statistics
        int nImages = vTimesTrack[i].size(); 
        sort(vTimesTrack[i].begin(),vTimesTrack[i].end());
        float totaltime = 0;
        for(int ni=0; ni<nImages; ni++)
        {
            totaltime+=vTimesTrack[i][ni];
        }
        std::cout << "median tracking time for client " << i << " : " << vTimesTrack[i][nImages/2] << endl;
        std::cout << "mean tracking time for client " << i << " : "  << totaltime/nImages << endl;
        std::ostringstream ss;
        ss << "allTrajectory" << i << ".txt";
        string trajFileName(ss.str()); 
        std::ostringstream ss2;
        ss2 << "imu" << i << ".csv";
        string imuFileName(ss2.str());
        SaveIMU(imuFileName,server->clients[i]->vImuMeas);
        // save the trajectory for each user
        SaveTrajectory(trajFileName,trajectory[i],vTimestamps[i],trajectory_gt_points[i]);
    }

    // save the measured distances
    std::ostringstream ss;
    ss << "allDistances.txt";
    string trajFileName(ss.str()); 
    SaveDistanceMeasurement(trajFileName, server->hist_TS_1, server->hist_users_1, server->hist_poses_1, server->hist_TS_2, server->hist_users_2, server->hist_poses_2, server->hist_distances); 

    //draw the trajectory for all users
    std::cout << "start drawing the trajectory." << endl; 
    DrawTrajectory(trajectory, trajectory_gt_points);

    // Save camera trajectory
    //SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

void DrawTrajectory(const vector<Sophus::SE3f>& esti, const vector<Sophus::SE3f>& gt, vector<int> gt_points) {
    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View& d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        if (!gt.empty()) {
            for (size_t i = 0; i < gt.size() - 1; i++) {// keep ground truth size the same as esti
                cv::Mat tcw = ORB_SLAM3::Converter::toCvMat(ORB_SLAM3::Converter::toSE3Quat(gt[i]));
                cv::Mat tcw2 = ORB_SLAM3::Converter::toCvMat(ORB_SLAM3::Converter::toSE3Quat(gt[i + 1]));
                if (tcw.empty() || tcw2.empty()) continue;
                glColor3f(0.0f, 0.0f, 1.0f);  // blue for ground truth
                glBegin(GL_LINES);
                cv::Mat t = -tcw.rowRange(0, 3).colRange(0, 3).t() * tcw.rowRange(0, 3).col(3);
                Eigen::Vector3d p1(t.at<float>(0), t.at<float>(1), t.at<float>(2));
                cv::Mat t2 = -tcw2.rowRange(0, 3).colRange(0, 3).t() * tcw2.rowRange(0, 3).col(3);
                Eigen::Vector3d p2(t2.at<float>(0), t2.at<float>(1), t2.at<float>(2));
                glVertex3d(p1(0, 0), p1(1, 0), p1(2, 0));
                glVertex3d(p2(0, 0), p2(1, 0), p2(2, 0));
                glEnd();
            }
        }

        for (size_t i = 0; i < esti.size() - 1; i++) {
            cv::Mat tcw = ORB_SLAM3::Converter::toCvMat(ORB_SLAM3::Converter::toSE3Quat(esti[i]));
            cv::Mat tcw2 = ORB_SLAM3::Converter::toCvMat(ORB_SLAM3::Converter::toSE3Quat(esti[i + 1]));
            if (tcw.empty()||tcw2.empty()) continue;
            if (gt_points[i] == 0){
                glLineWidth(2);
                glColor3f(1.0f, 0.0f, 0.0f);  // red for estimated
            }
            else {
                glLineWidth(6);
                glColor3f(0.0f, 0.0f, 1.0f); // blue for annotated ground truth measurement points
            }
            glBegin(GL_LINES);
            cv::Mat t = -tcw.rowRange(0, 3).colRange(0, 3).t() * tcw.rowRange(0, 3).col(3);
            Eigen::Vector3d p1(t.at<float>(0), t.at<float>(1), t.at<float>(2));
            cv::Mat t2 = -tcw2.rowRange(0, 3).colRange(0, 3).t() * tcw2.rowRange(0, 3).col(3);
            Eigen::Vector3d p2(t2.at<float>(0), t2.at<float>(1), t2.at<float>(2));
            glVertex3d(p1(0, 0), p1(1, 0), p1(2, 0));
            glVertex3d(p2(0, 0), p2(1, 0), p2(2, 0));
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}

void DrawTrajectory(const vector<vector<Sophus::SE3f>> & esti, vector<vector<int>> gt_points){
    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View& d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);

        for (size_t i = 0; i < esti.size(); i++) {
            for (size_t j = 0; j < esti[i].size() -1 ; j++){
                cv::Mat tcw = ORB_SLAM3::Converter::toCvMat(ORB_SLAM3::Converter::toSE3Quat(esti[i][j]));
                cv::Mat tcw2 = ORB_SLAM3::Converter::toCvMat(ORB_SLAM3::Converter::toSE3Quat(esti[i][j + 1]));
                if (tcw.empty()||tcw2.empty()) continue;
                if (gt_points[i][j] == 0){
                    glLineWidth(2);
                    if (i==0)
                        glColor3f(0.0f,1.0f,0.0f); // green for the first client
                    else if (i==1)
                        glColor3f(1.0f,0.0f,0.0f); // red for the second client
                    else if (i==2)
                        glColor3f(1.0f,1.0f,0.0f); // yellow for the third client
                    else if (i==3)
                        glColor3f(1.0f,0.647f,1.0f); // orange for the fourth client
                    else if (i==4)
                        glColor3f(0.0f,1.0f,1.0f); // cyan for the fifth client
                    else 
                        glColor3f(0.0,0.0f,1.0f); // blue for the other clients
                }
                else {
                    glLineWidth(6);
                    glColor3f(0.0f, 0.0f, 0.0f); // black for annotated ground truth measurement points
                }
                glBegin(GL_LINES);
                cv::Mat t = -tcw.rowRange(0, 3).colRange(0, 3).t() * tcw.rowRange(0, 3).col(3);
                Eigen::Vector3d p1(t.at<float>(0), t.at<float>(1), t.at<float>(2));
                cv::Mat t2 = -tcw2.rowRange(0, 3).colRange(0, 3).t() * tcw2.rowRange(0, 3).col(3);
                Eigen::Vector3d p2(t2.at<float>(0), t2.at<float>(1), t2.at<float>(2));
                glVertex3d(p1(0, 0), p1(1, 0), p1(2, 0));
                glVertex3d(p2(0, 0), p2(1, 0), p2(2, 0));
                glEnd();
            }
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}

void SaveTrajectory(const string & filename, const vector<Sophus::SE3f>&trajectory, vector<double> timeStamps, vector<int> trajGTpts) {
    ofstream f;
    f.open(filename.c_str());
    f << "#timeStamp tx ty tz qx qy qz qw groundTruthPoints" << endl; 
    f << fixed;
    for (size_t i = 0; i < trajectory.size(); i++)
    {
        cv::Mat tcw = ORB_SLAM3::Converter::toCvMat(ORB_SLAM3::Converter::toSE3Quat(trajectory[i])); 
        if (tcw.empty()) continue; 
        cv::Mat R = tcw.rowRange(0, 3).colRange(0, 3).t();
        vector<float> q = ORB_SLAM3::Converter::toQuaternion(R);
        cv::Mat t = -R * tcw.rowRange(0, 3).col(3);
        f << setprecision(6) << timeStamps[i] << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
            << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << " " << trajGTpts[i] << endl;
    }

    f.close();
    std::cout << endl << filename << " saved!" << endl;
}

void SaveIMU(const string & filename, const vector<ORB_SLAM3::IMU::Point>& imuMeas){
    ofstream f;
    f.open(filename.c_str());
    //f << "#timeStamp gx gy gz ax ay az" << endl; 
    f << fixed;
    for (size_t i = 0; i < imuMeas.size(); i++)
    {
        ORB_SLAM3::IMU::Point imu = imuMeas[i]; 
        f << setprecision(6) << imu.t << "," << imu.w[0] << "," << imu.w[1] << "," << imu.w[2]
            << "," << imu.a[0] << "," << imu.a[1] << "," << imu.a[2] << endl;
    }

    f.close();
    std::cout << endl << filename << " saved!" << endl;
}

void SaveDistanceMeasurement(const string & filename, const vector<double> ts1, const vector<int> users1, const vector<Eigen::Vector3d> pos1, const vector<double> ts2, const vector<int> users2, const vector<Eigen::Vector3d> pos2, vector<double> distances){
    ofstream f;
    f.open(filename.c_str());
    f << "# timeStamp1 user1 tx1 ty1 tz1 timeStamp2 user2 tx2 ty2 tz2 distance" << endl; 
    f << fixed;
    for (size_t i = 0; i < distances.size(); i++)
    {
        f << setprecision(3) << ts1[i] << " " << setprecision(0) << users1[i] << setprecision(5) << " " << pos1[i][0] << " " << pos1[i][1] << " " << pos1[i][2] << " " 
            << setprecision(3) << ts2[i] << " " << setprecision(0) << users2[i] << setprecision(5) << " " << pos2[i][0] << " " << pos2[i][1] << " " << pos2[i][2]
            << " " << distances[i] << endl;
    }

    f.close();
    cout << endl << filename << " saved!" << endl;
}
