# include "Socket/client.h"

Client::Client(int id, int connfd, const string settingFile, Server* server, int total_c_num):
id_(id), connfd_(connfd), nFeaturesInit(1000), nFeatures(500), k_track_(5), recvFlag(true), recvFlagAcoustic(true), initFlag(true), num_users(total_c_num) {
    server_ = server; 
    cv::FileStorage fSettings(settingFile, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];
    mThDepth = mbf*(float)fSettings["ThDepth"]/fx;

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;


    float rows = fSettings["Camera.rows"];
    float cols = fSettings["Camera.cols"];

    // Load ORB parameters

    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    extractor_ = new ORB_SLAM3::ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    // add new tracking thread in SLAM system
    server_->system->AddClient(id_); 

    mpTracker = server_->system->GetTracker(id_); 

    // send the number of feature points for initialization
    sendMsg(nFeaturesInit); 

    client_thread_ = std::thread(&Client::receiveLoop,this); 
    //client_thread_.detach();
    tracking_thread_ = std::thread(&Client::trackLoop,this);
    //tracking_thread_.detach();
}

void Client::AddAcoustic(int conn){
    connfd_ac_ = conn; 
    int yes = 1; 
    int result = setsockopt(connfd_ac_,
                            IPPROTO_TCP,
                            TCP_NODELAY,
                            (char *) &yes, 
                            sizeof(int));    // 1 - on, 0 - off
    if (result < 0)
          cout << "set TCP_NODELAY failed. " << endl; 
    for (int i=0;i<num_users;i++){
      queue<int> queue; 
      intervals.push_back(queue); 
    }
    acoustic_thread_ = std::thread(&Client::acousticLoop,this); 
}

void Client::Close(){
    recvFlag = false; 
    recvFlagAcoustic = false;
    close(connfd_);
    close(connfd_ac_); 
    //client_thread_.join();
    tracking_thread_.join();
}

// fd: file descriptor
void Client::receiveLoop() {
    unsigned char header[2] = { 0 };
    char buffer[1024] = { 0 };
    /*char* msg = "Hello from server";
    // test simple connection
    int valread = read(connfd_, buffer, 10024);
    printf("%s\n", buffer);
    send(connfd_, msg, strlen(msg), 0);
    printf("Hello message sent\n");
    printf("client %d closed \n",id_);*/
    int valread;
    while (valread!=-1&&recvFlag){
        valread = read(connfd_, header, 2);
        if (valread!=2) {
            cout << "client " << id_ << " disconnected!" << endl; 
            break;
        }
        std::chrono::steady_clock::time_point tt1 = std::chrono::steady_clock::now();
        //sendPoseDelay(0,vector<float>(3));
        // get the packet size
        unsigned short size = ((unsigned short) header[0])*256 + (unsigned short)header[1]; 
        //cout << "current packet size from client: " << size << endl; 
        unsigned char payload[size];
        int recv = 0;
        while (recv < size){
        if (recv + 1024 < size)
            valread = read(connfd_,buffer,1024);
        else
            valread = read(connfd_,buffer,size-recv);
        // copy to target payload
        for (int i=recv;i<recv+valread;i++)
            payload[i] = buffer[i-recv];
        recv = recv + valread; 
        }
        //sendPoseDelay(0,vector<float>(3));
        std::chrono::steady_clock::time_point tt = std::chrono::steady_clock::now();
        //cout << "time to receive slam packet" << std::chrono::duration_cast<std::chrono::duration<double> >(tt-tt1).count() << endl;
        recv_times.push_back(tt);
        SlamPktVI* pkt = new SlamPktVI(payload,size); 
        vector<cv::KeyPoint> keypoints_ = pkt->getKeyPoints(); 
        cv::Mat descriptors_ = pkt->getDescriptors();
        int frameID = pkt->getFrameId()+100000*id_; 
        long stamp = pkt->getTimeStamp(); 
        // add imu measurements
        for (IMUData data : pkt->imus_){
            vImuMeas.push_back(ORB_SLAM3::IMU::Point(data.acce_[0],data.acce_[1],data.acce_[2],
                                                data.gyro_[0],data.gyro_[1],data.gyro_[2],
                                                data.ts_/1e9));
        }
        ORB_SLAM3::Frame* frame = new ORB_SLAM3::Frame(keypoints_, descriptors_, frameID, id_, stamp/1e9, extractor_, mpTracker->GetVocab(), mpTracker->GetCamera(), mDistCoef, mbf, mThDepth, nullptr, *mpTracker->GetIMUCalib());
        //lastFrame = frame; 
        InsertFrame(frame); // comment this line to test acoustic only 

        //cout << "frame id " << frameID << "number of IMU:" << pkt->imus_.size() << endl;
        //cout << "image stamp:" << stamp << " first stamp of IMU:" << pkt->imus_[0].ts_ << " last stamp of IMU:"<<pkt->imus_[pkt->imus_.size()-1].ts_<<endl; 
        //cout << "keypoints " << keypoints_[10].pt << " " << keypoints_[71].pt << endl;
        //cout << "descriptors " << descriptors_.row(10) << endl;
        //cout << fixed << "time stamp " << frame->mTimeStamp << endl;
    }
}


void Client::trackLoop(){
    while(recvFlag){
        if (CheckNewFrame()){
            #ifdef COMPILEDWITHC11
                    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
            #else
                    std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
            #endif
            ORB_SLAM3::Frame* im= GetNewFrame();  
            // for the users (except user 0) which tracks normally, only track frames every k_track frames
            if (id_!=0 && !initFlag && im->mnId%k_track_!=0)
                continue;
            //int clientID = im->clientId; 
            //cout << "client: " << id_ << " frame id: " << im->mnId << " number of feature points: " << im->mvKeys.size() << endl;  

            // grab imu data
            while(first_imu<vImuMeas.size()&&vImuMeas[first_imu].t<=im->mTimeStamp){
                if (im->mTimeStamp - vImuMeas[first_imu].t<0.1)
                    mpTracker->GrabImuData(vImuMeas[first_imu]);
                first_imu++;
            }

            Sophus::SE3f tcw = server_->system->TrackEdge(im);

            Eigen::Vector3f twc = -tcw.rotationMatrix().transpose() * tcw.translation();
            vector<float> est_t{twc[0],twc[1],twc[2]};

            // detect the state of tracking to change the number of feature points
            if (!initFlag && server_->system->GetTrackingState(id_)!=2) {
                sendMsg(nFeaturesInit);
                initFlag = true; 
            }
            if (initFlag && server_->system->GetTrackingState(id_)==2) {
                sendMsg(nFeatures); 
                initFlag = false; 
            }
            #ifdef COMPILEDWITHC11
                    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
            #else
                    std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
            #endif
            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
            updateTraj(tcw,ttrack,im->mTimeStamp,0); 
            t2 = std::chrono::steady_clock::now();
            send_times.push_back(t2);
            double tprocess= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - recv_times[send_times.size()-1]).count();
            // send estimated pose and delay to clients
            sendPoseDelay((float)tprocess,est_t);
        }
        else {
            //sendTrivialMsg();
            usleep(3000); 
        }
    }
}


void Client::acousticLoop(){
    char buffer[1024] = { 0 };
    std::ostringstream ss;
    ss << id_ << "," << server_->max_client_num << "\n"; 
    const char* start_msg = ss.str().c_str(); 
    send(connfd_ac_, start_msg, strlen(start_msg), 0); 
    std::cout << "client " << id_ << " acoustic established" << endl; 
    int valread = read(connfd_ac_, buffer, 1024);
    while(recvFlagAcoustic) {
        printf("received %s", buffer);
        string msg(buffer); 
        vector<string> tokens = split(msg,' ');
        int group_num = tokens.size()/2; 
        for (int i=0;i<group_num;i++) {
          cout << "client " << id_ << " calculated interval of client " << tokens[2*i+0] << ": " << tokens[2*i+1] << endl; 
          intervals[(int)atof(tokens[2*i+0].c_str())].push((int)atof(tokens[2*i+1].c_str())); 
        }
        cout << endl; 
        valread = read(connfd_ac_, buffer, 1024);
        if (valread<=0) break;
    }
    std::cout << "client " << id_ << " acoustic stopped" << endl; 
}

void Client::updateTraj(Sophus::SE3f tcw, double ttrack, double timeStamp, int gt_id){
    unique_lock<mutex> lock(mMutexClient);
    trajectory.push_back(tcw); 
    vTimesTrack.push_back(ttrack); 
    vTimestamps.push_back(timeStamp);
    trajectory_gt_points.push_back(gt_id);
}

int Client::getLatestTraj(Sophus::SE3f &mat){
    unique_lock<mutex> lock(mMutexClient);
    int size = trajectory.size()-1; 
    /*while(size>=0){
        mat = trajectory[size];
        if (!mat.empty()) break; // get last successfully tracked frame
        size--; 
    }*/
    mat = trajectory[size];
    return size;  
}

double Client::getLatestTS(){
    unique_lock<mutex> lock(mMutexClient);
    int size = vTimestamps.size()-1; 
    if (size>=0){
        return vTimestamps[size];  
    }
    return -1;  
}

void Client::rewriteTraj(int poseId, Sophus::SE3f mat){
    unique_lock<mutex> lock(mMutexClient);
    if (poseId >= trajectory.size()) return;
    trajectory[poseId] = mat; 
    trajectory_gt_points[poseId] = -1; 
}

bool Client::sendMsg(int num){
    CmdPkt* pkt = new CmdPkt(0,num);  
    unsigned char* head = pkt->getHead();
    if (head == nullptr) return false;
    send(connfd_, head, 2, 0);
    int size = pkt->getTotalLength();
    unsigned char* payload = pkt->getPayload();
    send(connfd_, payload, size, 0);
    return true; 
}

bool Client::sendPoseDelay(float d, vector<float> p){
    CmdPkt* pkt = new CmdPkt(1,d,p);  
    unsigned char* head = pkt->getHead();
    if (head == nullptr) return false;
    //cout << "server sent packet size: " << pkt->getTotalLength() << endl;
    send(connfd_, head, 2, 0);
    int size = pkt->getTotalLength();
    unsigned char* payload = pkt->getPayload();
    send(connfd_, payload, size, 0);
    return true; 
}

bool Client::sendTrivialMsg(){
    CmdPkt* pkt = new CmdPkt(20,100);  
    unsigned char* head = pkt->getHead();
    if (head == nullptr) return false;
    send(connfd_, head, 2, 0);
    int size = pkt->getTotalLength();
    unsigned char* payload = pkt->getPayload();
    send(connfd_, payload, size, 0);
    return true; 
}

void Client::sendMsgAcoustic(char *msg) {
    send(connfd_ac_, msg, strlen(msg), 0); 
    //cout << "send msg " << msg << " to client " << id_ << endl; 
}

void Client::InsertFrame(ORB_SLAM3::Frame *pF)
{
    unique_lock<mutex> lock(mMutexClient);
    mlNewFrames.push_back(pF);
}

bool Client::CheckNewFrame()
{
    unique_lock<mutex> lock(mMutexClient);
    return(!mlNewFrames.empty());
}

ORB_SLAM3::Frame* Client::GetNewFrame()
{
    unique_lock<mutex> lock(mMutexClient);
    ORB_SLAM3::Frame* mpCurrentFrame = mlNewFrames.front();
    mlNewFrames.pop_front();
    return mpCurrentFrame; 
}
