#ifndef CODE_WIFIS_H_
#define CODE_WIFI_H_

#include "zf_common_headfile.h"


#define WIFI_SSID_TEST          "xxx"        //����
#define WIFI_PASSWORD_TEST      "118837235"  //����
// �����Ҫ���ӵ�WIFI û����������Ҫ�� ���� �滻Ϊ NULL

#define WIFI_TARGET_IP      "192.168.1.180"       // ����Ŀ��� IP
#define WIFI_TARGET_PORT    "8086"                 // ����Ŀ��Ķ˿�
#define WIFI_LOCAL_PORT     "6666"                 // �����Ķ˿� 0�����  �����÷�Χ2048-65535  Ĭ�� 6666

//WiFi��ʼ��
void wifi_init(void);

//��λ������TCP����
void TCP_client_init(void);
//�����λ��TCP����
void TCP_seekfree_init(void);

//wifi���ڷ���
//wifi���ڷ�������
void wifi_send_int(int a);
//wifi���ڷ��͸�����
void wifi_send_float(float a);
//wifi���ڷ����ַ�
void wifi_send_char(char wifi_data_buffer[]);
//wifi���ڽ���
void wifi_get(uint8 wifi_spi_get_data_buffer[]);


// ������ֳ�ʼ��
void seekfree_init(void);
//������ֲ�������
void seekfree_oscilloscope_send(void);
//�����������ͷ����ͼ��
void seekfree_camera_send(void);

#endif /* CODE_PID_H_ */
