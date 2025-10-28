/* gpio.c
*    gpio控制相关
programmed by yaoze
*/
#include "gpio.h"
#include "tools.h"

// /sys/class/gpio/export
bool gpio_register(const int num)
{
    const char export_path[]="/sys/class/gpio/export";
    int fd=open(export_path,O_WRONLY|O_TRUNC);      //打开export并只写,截断
    if(fd==-1) 
    {
        perror("gpio_register():gpio export failed.");
        strerror(errno);
        printf("\n");
        return false;
    }
    char num_str[4]="";
    if(sprintf(num_str,"%d",num)==1)    
    {
        perror("gpio_register():sprintf err.");
        strerror(errno);
        printf("\n");
        return false;
    }
    int write_bytes=write(fd,num_str,sizeof(num_str));              //写入export
    if(write_bytes==-1)
    {
        perror("gpio_register():write failed.");
        strerror(errno);
        printf("\n");
        return false;
    }
    if(close(fd))  strerror(errno);

    //验证是否成功导出
    char gpio_dir[30]="/sys/class/gpio/gpio";
    strcat(gpio_dir,num_str);

    if (!is_directory_exist(gpio_dir))  return false;
    else return true;
}


bool gpio_unregister(const int num)
{
    const char export_path[]="/sys/class/gpio/unexport";
    int fd=open(export_path,O_WRONLY|O_TRUNC);      //打开export并只写,截断
    if(fd==-1) 
    {
        perror("gpio_unregister():gpio export failed.");
        strerror(errno);
        printf("\n");
        return false;
    }
    char num_str[4]="";
    if(sprintf(num_str,"%d",num)==1)    
    {
        perror("gpio_unregister():sprintf err.");
        strerror(errno);
        printf("\n");
        return false;
    }
    int write_bytes=write(fd,num_str,sizeof(num_str));              //写入export
    if(write_bytes==-1)
    {
        perror("gpio_unregister():write failed.");
        strerror(errno);
        printf("\n");
        return false;
    }
    if(close(fd))  strerror(errno);

    //验证是否成功unexport
    char gpio_dir[30]="/sys/class/gpio/gpio";
    strcat(gpio_dir,num_str);
    if (is_directory_exist(gpio_dir))  return false;
    else return true;
}


bool gpio_direction(const int num,const bool isout)
{
    char direction[40]="/sys/class/gpio/gpio";
    char num_str[4]="";
    if(sprintf(num_str,"%d",num)==1)    
    {
        perror("gpio_direction():sprintf err.\n");
        strerror(errno);
        printf("\n");
        return false;
    }
    strcat(direction,num_str);
    strcat(direction,"/direction");
    int fd=open(direction,O_WRONLY|O_TRUNC);
    if(fd==-1)
    {
        perror("gpio_direction() open failed.");
        strerror(errno);
        printf("\n");
        return false;
    }
    int write_bytes=0;
    if(isout)   write_bytes=write(fd,"out",sizeof("out"));            //写入export
    else    write_bytes=write(fd,"in",sizeof("in")); 
    if(write_bytes==-1)
    {
        perror("gpio_direction():write failed.");
        strerror(errno);
        printf("\n");
        return false;
    }
    if(close(fd))  
    {
        strerror(errno);
        return false;
    }
    else    return true;
}

bool gpio_set(const int num,const bool ishigh)
{
    char gpio_path[40]="/sys/class/gpio/gpio";
    char num_str[4]="";
    sprintf(num_str,"%d",num);
    strcat(gpio_path,num_str);
    strcat(gpio_path,"/direction");
    int fd=open(gpio_path,O_WRONLY|O_TRUNC);
    if(fd==-1)
    {
        perror("gpio_set_high open failed.");       //还未注册
        strerror(errno);
        printf("\n");
        return false;
    }
    int write_bytes=0;
    if(ishigh)  write_bytes=write(fd,"high",sizeof("high"));
    else    write_bytes=write(fd,"low",sizeof("low"));
    if(write_bytes==-1)
    {
        perror("gpio_set:write failed.");
        strerror(errno);
        printf("\n");
        return false;
    }
    if(close(fd))  
    {
        strerror(errno);
        return false;
    }
    else    return true;
}

bool gpio_get(const int num)
{
    char gpio_path[40]="/sys/class/gpio/gpio";
    char num_str[4]="";
    sprintf(num_str,"%d",num);
    strcat(gpio_path,num_str);
    strcat(gpio_path,"/value");
    int fd=open(gpio_path,O_RDONLY);
    if(fd==-1)
    {
        perror("gpio_get open failed.");       //还未注册
        strerror(errno);
        printf("\n");
        return false;
    }
    char level[2]="";
    int read_bytes=read(fd,level,1);
    if(read_bytes==-1)
    {
        perror("gpio_get:read failed.");
        strerror(errno);
        printf("\n");
        return false;
    }
    if(close(fd))  perror("gpio_get close falied.") ;
    if(strcmp(level,"1")==0)   return true;
    else    return false;
}


bool gpio_toggle(const int num)
{
    if(gpio_get(num))    gpio_set(num,false);       //原来是1 翻转为0
    else    gpio_set(num,true);
    return true;
}



