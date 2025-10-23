#include "wifi.h"
#include "image.h"
#include "control.h"
//WiFi��ʼ��
void wifi_init(void)
{

    while(wifi_spi_init(WIFI_SSID_TEST, WIFI_PASSWORD_TEST))
    {
        printf("\r\n connect wifi failed. \r\n");
        system_delay_ms(100);                                                   // ��ʼ��ʧ�� �ȴ� 100ms
    }

    printf("\r\n module version:%s",wifi_spi_version);                          // ģ��̼��汾
    printf("\r\n module mac    :%s",wifi_spi_mac_addr);                         // ģ�� MAC ��Ϣ
    printf("\r\n module ip     :%s",wifi_spi_ip_addr_port);                     // ģ�� IP ��ַ

}
//��λ������TCP����
void TCP_client_init(void)
{
    // zf_device_wifi_spi.h �ļ��ڵĺ궨����Ը���ģ������(����) WIFI ֮���Ƿ��Զ����� TCP ������������ UDP ����
    if(0 == WIFI_SPI_AUTO_CONNECT)                                              // ���û�п����Զ����� ����Ҫ�ֶ�����Ŀ�� IP
    {
        while(wifi_spi_socket_connect(                                          // ��ָ��Ŀ�� IP �Ķ˿ڽ��� TCP ����
            "TCP",                                                              // ָ��ʹ��TCP��ʽͨѶ
            WIFI_SPI_TARGET_IP,                                                 // ָ��Զ�˵�IP��ַ����д��λ����IP��ַ
            WIFI_SPI_TARGET_PORT,                                               // ָ��Զ�˵Ķ˿ںţ���д��λ���Ķ˿ںţ�ͨ����λ��Ĭ����8080
            WIFI_SPI_LOCAL_PORT))                                               // ָ�������Ķ˿ں�
        {
            // ���һֱ����ʧ�� ����һ���ǲ���û�н�Ӳ����λ
            printf("\r\n Connect TCP Servers error, try again.");
            system_delay_ms(100);                                               // ��������ʧ�� �ȴ� 100ms
        }
    }
}
//�����λ��TCP����
void TCP_seekfree_init(void)
{
    // zf_device_wifi_spi.h �ļ��ڵĺ궨����Ը���ģ������(����) WIFI ֮���Ƿ��Զ����� TCP ������������ UDP ����
        if(1 != WIFI_SPI_AUTO_CONNECT)                                              // ���û�п����Զ����� ����Ҫ�ֶ�����Ŀ�� IP
        {
            while(wifi_spi_socket_connect(                                          // ��ָ��Ŀ�� IP �Ķ˿ڽ��� TCP ����
                "TCP",                                                              // ָ��ʹ��TCP��ʽͨѶ
                WIFI_TARGET_IP,                                                 // ָ��Զ�˵�IP��ַ����д��λ����IP��ַ
                WIFI_TARGET_PORT,                                               // ָ��Զ�˵Ķ˿ںţ���д��λ���Ķ˿ںţ�ͨ����λ��Ĭ����8080
                WIFI_LOCAL_PORT))                                               // ָ�������Ķ˿ں�
            {
                // ���һֱ����ʧ�� ����һ���ǲ���û�н�Ӳ����λ
                printf("\r\n Connect TCP Servers error, try again.");
                system_delay_ms(100);                                               // ��������ʧ�� �ȴ� 100ms
            }
        }
}

//wifi���ڷ�������
void wifi_send_int(int a)
{
    int len;
    uint8 wifi_data_buffer[256];
    len = snprintf((char*)wifi_data_buffer, sizeof(wifi_data_buffer),"%d",a);
    wifi_spi_send_buffer(wifi_data_buffer, sizeof(wifi_data_buffer));
}

//wifi���ڷ��͸�����
void wifi_send_float(float a)
{
    int len;
    uint8 wifi_data_buffer[256];
    len = snprintf((char*)wifi_data_buffer, sizeof(wifi_data_buffer),"%f",a);
    wifi_spi_send_buffer(wifi_data_buffer, sizeof(wifi_data_buffer));
}

//wifi���ڷ����ַ�
void wifi_send_char(char a[])
{
    uint8 *wifi_data_buffer = (uint8 *)a;
    wifi_spi_send_buffer(wifi_data_buffer, sizeof(wifi_data_buffer));
}

//wifi���ڽ���
void wifi_get(uint8 wifi_spi_get_data_buffer[])
{
    wifi_spi_read_buffer(wifi_spi_get_data_buffer, sizeof(wifi_spi_get_data_buffer));
}

// ������ֳ�ʼ��
void seekfree_init(void)
{
    // ������ֳ�ʼ�� ���ݴ���ʹ�ø���WIFI SPI
   seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);

   // ���Ҫ����ͼ����Ϣ������ص���seekfree_assistant_camera_information_config�������б�Ҫ�Ĳ�������
   // �����Ҫ���ͱ����������seekfree_assistant_camera_boundary_config�������ñ��ߵ���Ϣ
}

//������ֲ�������
void seekfree_oscilloscope_send(void)
{
    // д����Ҫ���͵����ݣ��м���ͨ����д���ٸ�����
    // �����д��4��ͨ������
    seekfree_assistant_oscilloscope_data.data[0] = imu963ra_gyro_z;
    seekfree_assistant_oscilloscope_data.data[1] = target_velocity;
    seekfree_assistant_oscilloscope_data.data[2] = Turn_Pwm;
    seekfree_assistant_oscilloscope_data.data[3] = 2;
//        detector_oscilloscope_data.data[4] = 10;
//        detector_oscilloscope_data.data[5] = 100;
//        detector_oscilloscope_data.data[6] = 1000;
//        detector_oscilloscope_data.data[7] = 10000;

    // ���ñ�����Ҫ���ͼ���ͨ��������
    seekfree_assistant_oscilloscope_data.channel_num = 4;

    // �����������4��ͨ�������ݣ����֧��8ͨ��
    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);

    system_delay_ms(20);
    // �п��ܻ��������������Ͽ������θ��²���������������Ϊʹ��WIFI�в�ȷ�����ӳٵ��µ�

    // ������λ�����͹����Ĳ��������������ݻ�����seekfree_assistant_oscilloscope_data�����У�����ͨ�����ߵ��Եķ�ʽ�鿴����
    // ����Ϊ�˷������д������ѭ����ʵ��ʹ�����Ƽ��ŵ������жϵ�λ�ã���Ҫȷ�������ܹ���ʱ�ı����ã��������ڲ�����20ms
    seekfree_assistant_data_analysis();

}


//�����������ͷ����ͼ��
void seekfree_camera_send(void)
{
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, bin_image[0], MT9V03X_W, MT9V03X_H);
    seekfree_assistant_camera_boundary_config(X_BOUNDARY, MT9V03X_H, x1_boundary, x2_boundary, x3_boundary, NULL, NULL ,NULL);
    // ����ͼ��
    seekfree_assistant_camera_send();
     // ���ʹ��UDPЭ�鴫���������Ƽ�������ȫ�����͵�ģ��֮����������wifi_spi_udp_send_now()�������Ը�֪ģ���������յ������ݷ��͵�������
     // ���û������������ģ����ڳ���2����δ�յ����ݺ󣬽����ݷ��͵�������
     // ����wifi_spi_udp_send_now()ǰ�����ģ��������������鲻Ҫ����40960�ֽ�
     // wifi_spi_udp_send_now();
}
