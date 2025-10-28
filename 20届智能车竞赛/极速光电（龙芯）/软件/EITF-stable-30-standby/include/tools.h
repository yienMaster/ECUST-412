/*
工具函数
programmed by yaoze
*/





#pragma once

#include <unistd.h>
#include <sys/stat.h>
#include <stdbool.h>
#include <sstream>
#include <iomanip>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstring>
#include <opencv2/opencv.hpp>


#define MAX_PACKET_SIZE 65507  // UDP最大有效载荷


/*
@brief 判断目录是否存在
@param const char* 目录路径
@return bool 成功 ->true
*/
bool is_directory_exist(const char *path);

/*
@brief 将double类型的数转换为字符串 并保留precision位小数
@param const double num 要转换的数
@param const int precision 保留的小数位数
@return std::string 转换后的字符串

*/
std::string to_string_p(const double num,const int precision) ;

/*
@brief UDP连接上位机
@param int* sockfd 套接字文件描述符
@param const int port 端口号
@return struct sockaddr_in 远程地址结构体
*/
struct sockaddr_in udp_connect(int *sockfd,const int port);


/*
@brief UDP发送视频帧
@param const Mat& pic 要发送的图片
@param int sockfd 套接字文件描述符
@param const struct sockaddr_in remote_addr 远程地址结构体
@param const int zip_rate 压缩率
@return void
*/
void udp_send_frame(const cv::Mat pic,int sockfd,const struct sockaddr_in remote_addr,const int zip_rate);
