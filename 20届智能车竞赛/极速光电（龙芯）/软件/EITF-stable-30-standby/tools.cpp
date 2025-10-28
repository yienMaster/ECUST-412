/*
工具库
programmed by yaoze
*/
#include "tools.h"


bool is_directory_exist(const char *path)
{
    struct stat st;
    if (stat(path, &st) == 0)
    {
        if (S_ISDIR(st.st_mode))
            return true; // 目录存在
        else
            return false; // 不是目录
    }
    else
        return false; // stat()调用失败，目录不存在
}


std::string to_string_p(const double num,const int precision) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << num;  // 固定小数点，保留两位
    return oss.str();  // 返回格式化后的字符串
}


struct sockaddr_in udp_connect(int *sockfd,const int port)
{
    // 2. 创建UDP套接字（POSIX API替换Boost）[6,7](@ref)
    *sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        std::cerr << "套接字创建失败: " << strerror(errno) << std::endl;
        system("pause");
    }

    struct sockaddr_in remote_addr;
    memset(&remote_addr, 0, sizeof(remote_addr));
    remote_addr.sin_family = AF_INET;
    remote_addr.sin_port = htons(port);
    inet_pton(AF_INET, "192.168.137.1", &remote_addr.sin_addr);  // 上位机IP
    return remote_addr;
}



void udp_send_frame(const cv::Mat pic,int sockfd,const struct sockaddr_in remote_addr,const int zip_rate)
{
    
        // JPEG压缩（与原代码相同）

        std::vector<uchar> jpeg_buffer;
        cv::imencode(".jpg", pic, jpeg_buffer, {cv::IMWRITE_JPEG_QUALITY, zip_rate});         //90  压缩率 数值需调整
        // 分包发送（改用sendto）
        size_t offset = 0;
        while (offset < jpeg_buffer.size()) {
            size_t chunk_size = std::min<size_t>(MAX_PACKET_SIZE, jpeg_buffer.size() - offset);
            ssize_t sent = sendto(sockfd, &jpeg_buffer[offset], chunk_size, 0,
                                 (struct sockaddr*)&remote_addr, sizeof(remote_addr));
            if (sent < 0) {
                std::cerr << "发送失败: " << strerror(errno) << std::endl;
                break;
            }
            offset += sent;
        }
}

