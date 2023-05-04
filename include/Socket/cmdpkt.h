/* packet protocol of the information transmitted from server to client
 * including pose return, command to change number of feature points
 */
#ifndef CMDPKT_H
#define CMDPKT_H
using namespace std; 

class CmdPkt{
public:
    // total length of current packet, unit: bytes
    int total_len_;
    // a number used to differentiate different commands, 0 for number of feature points
    int cmd_code_;
    // number of feature points
    int num_fps_;
    // estimated pose from server
    vector<float> poses;
    // processing delay of SLAM
    float delay_slam_;
    unsigned char* payload; // store the payload

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

    CmdPkt(unsigned char* buffer, int packet_size) {
        total_len_ = packet_size;
        payload = buffer;

        // first get the code
        unsigned short code_us = (unsigned short)payload[0];
        cmd_code_ = (int) code_us;

        // different commands refers to different format
        if (cmd_code_==0){
            // 0 for number of feature points
            unsigned short fp_us = ((unsigned short) payload[1])*256 + (unsigned short)payload[2];
            num_fps_ = (int) fp_us;
        }
        else if (cmd_code_==1){
            // 1 for estimated pose and slam processing delay
            delay_slam_ = byte2float(1);
            for(int i=0;i<3;++i){
                poses.push_back(byte2float(5+i*4));
            }
        }
    }

    // determine the number of feature points in client side
    CmdPkt(int code, int num){
        total_len_ = 3;
        payload = new unsigned char[total_len_];
        payload[0] = (unsigned short)code & 0xff ;
        payload[1] = (unsigned short)num >> 8 ;
        payload[2] = (unsigned short)num & 0xff ;
    }

    // send the estimated poses and slam processing delay
    CmdPkt(int code, float d, vector<float> p){
        total_len_ = 1+4+4*3;
        payload = new unsigned char[total_len_];
        payload[0] = (unsigned short)code & 0xff ;
        float2byte(d,1);
        for(int i=0;i<3;++i){
            float2byte(p[i],5+4*i);
        }
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

    int getCommandCode(){
        return cmd_code_;
    }

    // get the number of feature points
    int getNumFPs(){
        return num_fps_;
    }
};

#endif