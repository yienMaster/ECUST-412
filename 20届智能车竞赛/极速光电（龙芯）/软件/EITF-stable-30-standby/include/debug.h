/*
调试用的头文件
*/
#pragma once
#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <signal.h>


extern int sock;        //套接字

/*
@brief TCP 连接Vofa
@param 服务器ip地址
@param 端口号

*/
void tcp_debug(const char *server_ip,const int port);

