#ifndef CLIENT_H
#define CLIENT_H

#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/tcp.h> // TCP_NO_DELAY
#include <unistd.h>
#include <thread>
#include <vector>
#include <queue>
#include <iostream>
#include "slampkt_vi.h"
#include "cmdpkt.h"
#include "Frame.h"
#include "Tracking.h"
#include "server.h"
#include "ORBextractor.h"
#include "ORBVocabulary.h"
using namespace std; 

class Server; 
class Client{
public:
    Client(int id, int connf, const string settingFile, Server* server, int total_c_num);
    void Close(); 
    void receiveLoop();
    void acousticLoop();
    void trackLoop(); 
    void AddAcoustic(int conn); 
    void InsertFrame(ORB_SLAM3::Frame *pF);
    bool CheckNewFrame();
    // send a number to the client
    bool sendMsg(int num);
    void sendMsgAcoustic(char* msg); 
    void updateTraj(Sophus::SE3f tcw, double ttrack, double timeStamp, int gt_id);
    int getLatestTraj(Sophus::SE3f &mat); 
    double getLatestTS(); 
    void rewriteTraj(int poseId, Sophus::SE3f mat); 
    ORB_SLAM3::Frame* GetNewFrame();
    vector<string> split (const string &s, char delim) {
        vector<string> result;
        stringstream ss (s);
        string item;

        while (getline (ss, item, delim)) {
            result.push_back (item);
        }

        return result;
    }
public:
    Server* server_;
    // client id starts from 0 
    int id_;
    int connfd_,connfd_ac_;
    int nFeaturesInit;
    int nFeatures; 
    int k_track_; 
    std::thread client_thread_; 
    std::thread acoustic_thread_; 
    std::thread tracking_thread_; 
    bool recvFlag, recvFlagAcoustic; 
    bool initFlag; 
    ORB_SLAM3::ORBextractor* extractor_;
    ORB_SLAM3::Tracking* mpTracker;

    std::mutex mMutexClient;

    std::list<ORB_SLAM3::Frame*> mlNewFrames;
    ORB_SLAM3::Frame* lastFrame;

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;
    // not used for monocular slam
    float mbf;
    float mThDepth; 

    // for acoustic ranging
    int num_users; 
    vector<queue<int>> intervals;

    int first_imu = 0;
    // Vector for tracking time statistics
    vector<double> vTimesTrack;
    vector<Sophus::SE3f> trajectory; 
    vector<double> vTimestamps; 
    vector<int> trajectory_gt_points; 
    vector<ORB_SLAM3::IMU::Point> vImuMeas;
}; 

#endif