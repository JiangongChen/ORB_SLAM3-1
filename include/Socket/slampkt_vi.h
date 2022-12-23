#pragma once
// communication packet between server and client for the edge-SLAM
// each slam packet corresponds to a frame, plus the measurement of IMU since last frame
#ifndef SLAMPKT_VI_H
#define SLAMPKT_VI_H
#include <opencv2/core/core.hpp>
#include <iostream>
#include <cstring> // memcpy
#include "imudata.h"

using namespace cv; 
using namespace std; 

class SlamPktVI {
public:
	int num_pts_; // number of feature points for this frame, assume no distortion
	int num_imu_; // number of imu measurements
	int total_len_; // total length of current packet, unit: bytes
	int pt_len_=36; // length of a keypoint, each point use 36 bytes, 2 bytes x, 2 bytes y, plus 32 bytes descriptor
	int imu_len_=32; // length of an IMU measurement, each measurement includes 32 bytes, 8 bytes of timestamp, 3*4 bytes of gyroscope, 3*4 bytes of accelerometer
	int info_len_=16; // length of the info packet, currently, four bytes for frame id, eight bytes for the timestamp of the image, two bytes for the number of keypoints, two bytes for the number of imu measurements
	int descriptor_len_ = 32; // length of descriptor for a keypoint, 32 bytes
	int frame_id_; // the frame id
	long time_stamp_;
	unsigned char* payload; // store the payload
	vector<KeyPoint> kps_; // all keypoints, without undistortion
	Mat descriptors_; // descriptors for all keypoints
	vector<IMUData> imus_; // all imu measurements


	float byte2float(int start){
		unsigned char b[4];
		for(int i=0;i<4;i++){
			b[i] = payload[start+i];
		}
		float res = 0.0;
		memcpy(&res, b, 4);
		return res;
	}

	void float2byte(float f, int start){
		unsigned char b[4];
		memcpy(b, &f, 4);
		for(int i=0;i<4;i++){
			payload[start+i] = b[i];
		}
	}

	long byte2long(int start){
		unsigned char b[8];
		for(int i=0;i<8;i++){
			b[i] = payload[start+i];
		}
		long res = 0;
		memcpy(&res, b, 8);
		return res;
	}

	void long2byte(long value, int start){
		unsigned char b[8];
		memcpy(b, &value, 8);
		for(int i=0;i<8;i++){
			payload[start+i] = b[i];
		}
	}

	int byte2int(int start){
		unsigned char b[4];
		for(int i=0;i<4;i++){
			b[i] = payload[start+i];
		}
		int res = 0;
		memcpy(&res, b, 4);
		return res;
	}

	void int2byte(int value, int start){
		unsigned char b[4];
		memcpy(b, &value, 4);
		for(int i=0;i<4;i++){
			payload[start+i] = b[i];
		}
	}

	SlamPktVI(unsigned char* buffer, int packet_size) {
		total_len_ = packet_size;
		payload = buffer;

		// process the info packet
		frame_id_ = byte2int(0);

		time_stamp_ = byte2long(4);

		unsigned short num_pts_us = ((unsigned short) payload[12])*256 + (unsigned short)payload[13];
		num_pts_ = (int) num_pts_us;

		unsigned short num_imu_us = ((unsigned short) payload[14])*256 + (unsigned short)payload[15];
		num_imu_ = (int) num_imu_us;

		descriptors_ = Mat(num_pts_, descriptor_len_, CV_8UC1);
		for (int i = 0; i < num_pts_; i++) {
			unsigned short x = ((unsigned short) payload[i* pt_len_ + info_len_]) * 256 + (unsigned short)payload[i * pt_len_ + 1 + info_len_];
			unsigned short y = ((unsigned short) payload[i * pt_len_ + 2 + info_len_]) * 256 + (unsigned short)payload[i * pt_len_ + 3 + info_len_];
			kps_.push_back(KeyPoint(x,y,1));
			for (int j = 0; j < descriptor_len_; j++) {
				descriptors_.at<unsigned char>(i,j) = payload[i * pt_len_ + 4 + j + info_len_];
			} 
		}

		int imu_start_ = info_len_ + num_pts_*pt_len_;
		for(int i=0; i<num_imu_; i++){
			long ts = byte2long(i*imu_len_ + imu_start_);
			vector<float> gyro;
			for(int j=0;j<3;j++){
				gyro.push_back(byte2float(i*imu_len_ + imu_start_ + 8 + j*4));
			}
			vector<float> acce;
			for (int j=0;j<3;j++){
				acce.push_back(byte2float(i*imu_len_ + imu_start_ + 20 + j*4));
			}
			IMUData imu_data(ts,gyro,acce);
			imus_.push_back(imu_data);
		}
	}

	SlamPktVI(int id, long timestamp, vector<KeyPoint>& kps, Mat& descriptors, vector<IMUData>& imus) {
		num_pts_ = (int) kps.size();
		num_imu_ = (int) imus.size();
		total_len_ = info_len_ + num_pts_ * pt_len_ + num_imu_ * imu_len_;
		frame_id_ = id;
		time_stamp_ = timestamp;
		kps_ = kps; 
		descriptors_ = descriptors;
		imus_ = imus;
		payload = new unsigned char[total_len_];

		// add header
		int2byte(frame_id_,0);
		long2byte(time_stamp_,4);
		payload[12] = (unsigned short)num_pts_ >> 8 ;
		payload[13] = (unsigned short)num_pts_ & 0xff ;
		payload[14] = (unsigned short)num_imu_ >> 8 ;
		payload[15] = (unsigned short)num_imu_ & 0xff ;

		for (int i = 0; i < num_pts_; i++) {
			payload[i * pt_len_ + info_len_] = (unsigned short)kps_[i].pt.x >> 8 ;
			payload[i * pt_len_ + 1 + info_len_] = (unsigned short)kps_[i].pt.x & 0xff;
			payload[i * pt_len_ + 2 + info_len_] = (unsigned short)kps_[i].pt.y >> 8 ;
			payload[i * pt_len_ + 3 + info_len_] = (unsigned short)kps_[i].pt.y & 0xff;
			for (int j = 0; j < descriptor_len_; j++) {
				payload[i * pt_len_ + 4 + j + info_len_] = descriptors_.at<unsigned char>(i, j);
			}
		}

		int imu_start_ = info_len_ + num_pts_*pt_len_;
		for(int i=0; i<num_imu_; i++){
			IMUData data = imus_[i];
			long2byte(data.ts_,i * imu_len_ + imu_start_);
			for(int j=0;j<3;j++){
				float2byte(data.gyro_[j],i * imu_len_ + imu_start_ + 8 + j*4);
			}
			for(int j=0;j<3;j++){
				float2byte(data.acce_[j],i * imu_len_ + imu_start_ + 20 + j*4);
			}
		}
	}

	vector<KeyPoint> getKeyPoints() {
		return kps_; 
	}

	Mat getDescriptors() {
		return descriptors_; 
	}

	vector<IMUData> getIMUData() {
		return imus_;
	}

	unsigned char* getPayload() {
		return payload; 
	}

	unsigned char* getHead(){
		if (total_len_>65536) return nullptr;
		// currently, head only contains the packet size, assume the packet size is lower than 65536
		unsigned char* head = new unsigned char[2];
		head[0] = (unsigned short)total_len_ >> 8 ;
		head[1] = (unsigned short)total_len_ & 0xff;
		return head;
	}

	int getTotalLength(){
		return total_len_;
	}

	int getNumPoints(){
		return num_pts_;
	}

	int getNumIMUs(){
		return num_imu_;
	}

	int getFrameId(){
		return frame_id_;
	}

	long getTimeStamp(){
		return time_stamp_;
	}

}; 
#endif // SLAMPKT_VI_H
