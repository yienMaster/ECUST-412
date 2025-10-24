#ifndef _OPENART_H_
#define _OPENART_H_

extern char* labels[];
extern int openart_info;
extern int ort_line;
extern int ort_label[20];

extern fifo_struct openart_fifo;
extern uint8 fifo_ort_data[64];

uint8 openart_judge(uint8 label);
void ort_show(void);


#endif
