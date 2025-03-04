// server, responsible for listening to clients (smartphones)
// support multiple clients, generate a new thread for each client

#include "Socket/server.h"

Server::Server():
    client_num(0), max_client_num(1), opt(1), listenFlag(true){
    // initialize the server
    //hello = "Hello from server";

    // Creating socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0))
        == 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    // Forcefully attaching socket to the port 8080
    if (setsockopt(server_fd, SOL_SOCKET,
                SO_REUSEADDR | SO_REUSEPORT, &opt,
                sizeof(opt))) {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    // Forcefully attaching socket to the port 8080
    if (bind(server_fd, (struct sockaddr*)&address,
            sizeof(address))
        < 0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

}

Server::Server(const string &strSettingsFile, ORB_SLAM3::System *sys):
    client_num(0), client_num_ac(0), max_client_num(2), opt(1), listenFlag(true), listenFlagAcoustic(true), est_scale(1.0){
    // initialize the server
    //hello = "Hello from server";
    settingFile = strSettingsFile; 

    system = sys; 

    speedOfSound = (331.3 + 0.606*27.1); 
    sample_rate = 48000; 
    kdistance = 0.0272; 
    
    // socket for visual part
    // Creating socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0))
        == 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    // Forcefully attaching socket to the port 8080
    if (setsockopt(server_fd, SOL_SOCKET,
                SO_REUSEADDR | SO_REUSEPORT, &opt,
                sizeof(opt))) {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    // Forcefully attaching socket to the port 8080
    if (bind(server_fd, (struct sockaddr*)&address,
            sizeof(address))
        < 0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }


    // socket for acoustic part
    if ((server_fd_ac = socket(AF_INET, SOCK_STREAM, 0))
        == 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    // opt 1
    if (setsockopt(server_fd_ac, SOL_SOCKET,
                SO_REUSEADDR | SO_REUSEPORT, &opt,
                sizeof(opt))) {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
    address_ac.sin_family = AF_INET;
    address_ac.sin_addr.s_addr = INADDR_ANY;
    address_ac.sin_port = htons(PORT_AC);

    if (bind(server_fd_ac, (struct sockaddr*)&address_ac,
            sizeof(address_ac))
        < 0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
}

void Server::StartListening(){
    std::cout << "server starts listening to new clients." << std::endl; 

    // start listening  
    listen_thread_ = std::thread(&Server::Listening, this);

    // start listening for acoustic
    listen_thread_acoustic_ = std::thread(&Server::ListeningAcoustic, this); 
}

void Server::Listening(){
    while(listenFlag){
        if (listen(server_fd, 3) < 0) {
            perror("listen");
            exit(EXIT_FAILURE);
        }
        new_socket
            = accept(server_fd, (struct sockaddr*)&address,
                    (socklen_t*)&addrlen);
        if(client_num>=max_client_num) {
            // closing the connected socket
            close(new_socket);
            break;
        }
        // client id starts from 0 
        // add new tracking thread in SLAM system
        //system->AddClient(client_num); 
        Client* client = new Client(client_num,new_socket,settingFile,this,max_client_num);
        clients.push_back(client);
        std::cout << "client " << client_num << " has been accepted" << std::endl; 
        client_num++;
    }
    // closing the listening socket
    shutdown(server_fd, SHUT_RDWR);
    std::cout << "server listening thread has been stopped." << std::endl; 
    listenFlag = false; 
}

void Server::ListeningAcoustic(){
    while(listenFlagAcoustic && client_num_ac<max_client_num){
        if (listen(server_fd_ac, 3) < 0) {
            perror("listen");
            exit(EXIT_FAILURE);
        }
        new_socket_ac
            = accept(server_fd_ac, (struct sockaddr*)&address_ac,
                    (socklen_t*)&addrlen_ac);
        // attach the acoustic socket to existing client
        while (client_num_ac>=(int)clients.size()) usleep(3000); 
        Client* client = clients[client_num_ac];
        client->AddAcoustic(new_socket_ac); 
        std::cout << "client " << client_num_ac << " acoustic communication has been accepted" << std::endl; 
        client_num_ac++;
    }
    // wait for all clients have been initialized 
    bool flag = true; 
    while(flag) {
        flag = false; 
        for (Client* client : clients)
            if (client->initFlag) flag = true; 
        usleep(30000); 
    }

    //periodically let client emit acoustic signal
    chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    chrono::steady_clock::time_point prev = std::chrono::steady_clock::now();
    double gap = std::chrono::duration_cast<std::chrono::duration<double> >(now - start_time).count();
    while(gap < 1.2){
        if (std::chrono::duration_cast<std::chrono::duration<double> >(now - prev).count()<1){
            usleep(10000); 
            now = std::chrono::steady_clock::now();
            continue; 
        }
        for (Client* client : clients){
            char* msg = "emit\n"; 
            client->sendMsgAcoustic(msg); 
            //if (client->id_ == 2)
            //    usleep(1000000); // emit signal in an interval of 1 second 
        }
        usleep(300000); // gap between each release of acoustic signals, in nanoseconds
        now = std::chrono::steady_clock::now();
        prev = std::chrono::steady_clock::now();
        gap = std::chrono::duration_cast<std::chrono::duration<double> >(now - start_time).count();
    }
    // closing the listening socket
    //shutdown(server_fd_ac, SHUT_RDWR);
    std::cout << "server listening thread for acoustic has been stopped." << std::endl; 
    listenFlagAcoustic = false; 
}

void Server::Close(){
    std::cout << "request for stopping the server." << std::endl; 
    listenFlag = false; 
    listen_thread_.join();

    //wait for all clients to be closed
    for (Client* client : clients)
        client->Close(); 
    std::cout << "server has been successfully closed." << std::endl; 
}

void Server::InsertFrame(ORB_SLAM3::Frame *pF)
{
    unique_lock<mutex> lock(mMutexServer);
    mlNewFrames.push_back(pF);
}

bool Server::CheckNewFrame()
{
    unique_lock<mutex> lock(mMutexServer);
    return(!mlNewFrames.empty());
}

bool Server::CheckAcoustic(){
    if (clients.size() == 0) return false; 
    for (Client* client : clients) {
        for(int i=0;i<max_client_num;i++){
            if (i==client->id_) continue; 
            if (client->intervals[i].empty() ) {
                return false;
            }
        }
    }
    return true; 
}

vector<double> Server::CalAcoustic(){
    vector<double> distances; 
    for (int i=0;i<max_client_num;i++){
        Sophus::SE3f o_pose; 
        int idx = clients[i]->getLatestTraj(o_pose); // the trajectory could be empty matrix, handling that
        //if (idx == -1) //handle invalid pose, e.g., has not been initialized
        //    continue;
        double ts1 = clients[i]->getLatestTS(); 
        if (i==0) cout << "client" << i << " timestamp:" << ts1 << endl; 
        Eigen::Vector3d pose1 = o_pose.inverse().translation().cast<double>(); // get the position in the world coordinates, i.e., t in Twc
        for (int j=i+1;j<max_client_num;j++){
            int n1 = clients[i]->intervals[j].front();
            clients[i]->intervals[j].pop();
            int n2 = clients[j]->intervals[i].front();
            clients[j]->intervals[i].pop();
            double distance = (speedOfSound * (n1+n2)) / (2 * sample_rate) + kdistance;
            cout << "sample client " << i << ": " << n1 << " and client " << j << ": " << n2 << ", distance is " << distance << endl;  
            if (distance > 0 && distance < 4) { // ignore obviously wrong distances 
                if (i == 0)
                    distances.push_back(distance); 
                else {
                    int idx = clients[j]->getLatestTraj(o_pose); // the trajectory could be empty matrix, handling that
                    if (idx == -1) //handle invalid pose, e.g., has not been initialized
                        continue;
                    double ts2 = clients[j]->getLatestTS(); 
                    Eigen::Vector3d pose2 = o_pose.inverse().translation().cast<double>(); 
                    hist_poses_1.push_back(pose1);
                    hist_poses_2.push_back(pose2);
                    hist_users_1.push_back(i);
                    hist_users_2.push_back(j); 
                    hist_TS_1.push_back(ts1); 
                    hist_TS_2.push_back(ts2); 
                    hist_distances.push_back(distance); 
                }
            }
        } 
    }
    return distances; 
}

ORB_SLAM3::Frame* Server::GetNewFrame()
{
    unique_lock<mutex> lock(mMutexServer);
    ORB_SLAM3::Frame* mpCurrentFrame = mlNewFrames.front();
    mlNewFrames.pop_front();
    return mpCurrentFrame; 
}
