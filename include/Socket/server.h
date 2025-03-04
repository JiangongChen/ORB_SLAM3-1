#ifndef SERVER_H
#define SERVER_H

#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <thread>
#include <vector>
#include <iostream>
#include "client.h"
#include "Frame.h"
#include "System.h"
#include "Converter.h"
#define PORT 8080
#define PORT_AC 8848

class Client; 
class Server{
public:
    Server(const string &strSettingsFile, ORB_SLAM3::System *sys);
    Server(); 
    void StartListening(); 
    void StartListendingAcoustic(); 
    void Listening(); 
    void ListeningAcoustic(); 
    void Close(); 
    bool CheckNewFrame();
    bool CheckAcoustic(); 
    vector<double> CalAcoustic(); 
    ORB_SLAM3::Frame* GetNewFrame();
    void InsertFrame(ORB_SLAM3::Frame *pF);

public:
	struct sockaddr_in address;
    struct sockaddr_in address_ac; 
	int addrlen = sizeof(address);
    int addrlen_ac = sizeof(address_ac); 

    int client_num, client_num_ac, max_client_num;
	int opt;
    bool listenFlag,listenFlagAcoustic; 
	char* hello;
    
    int server_fd, new_socket, valread;
    int server_fd_ac, new_socket_ac, valread_ac; 

    std::thread listen_thread_; 
    std::thread listen_thread_acoustic_; 

    std::vector<Client*> clients; 

    std::mutex mMutexServer;

    std::list<ORB_SLAM3::Frame*> mlNewFrames;

    string settingFile; 

    ORB_SLAM3::System *system; 

    // parameters for acoustic ranging
    double speedOfSound;
    int sample_rate;
    double kdistance; 
    double est_scale; 

    vector<Eigen::Vector3d> hist_poses_1; 
    vector<Eigen::Vector3d> hist_poses_2; 
    vector<int> hist_users_1; 
    vector<int> hist_users_2; 
    vector<double> hist_TS_1; 
    vector<double> hist_TS_2; 
    vector<double> hist_distances; 
    vector<double> hist_scales; 
}; 

#endif