#include "zf_common_headfile.h"
#include "my_code.h"
int openart_info;
int ort_label[20] ={0};//用于存贮ort传输数据的数组
char* labels[] = { "0", "1", "2", "3", "4", "5", "6", "7", "8", "9", 
								 "10", "11", "12", "13", "14", "15", "16", "17", "18", "19",
								 "20", "21", "22", "23", "24", "25", "26", "27", "28", "29",
								 "30", "31", "32", "33", "34", "35", "36", "37", "38", "39",
								 "40", "41", "42", "43", "44", "45", "46", "47", "48", "49",
								 "50", "51", "52", "53", "54", "55", "56", "57", "58", "59",
								 "60", "61", "62", "63", "64", "65", "66", "67", "68", "69",
								 "70", "71", "72", "73", "74", "75", "76", "77", "78", "79",
								 "80", "81", "82", "83", "84", "85", "86", "87", "88", "89",
								 "90", "91", "92", "93", "94", "95", "96", "97", "98", "99",
								 "wrench", "solderiron", "drill", "tapemeasure","speaker","screwdriver","plier","oscillgraph","monitor","multimeter","printer","keyboard","mobilephone",
								"mouse","headphone"};
int elect[7] = {104, 108, 110, 111, 112, 113, 114};		//if(label == 104 || label == 108 || (label >= 110 && label <= 114)) left
int tools[8] = {100, 101, 102, 103, 105, 106, 107, 109};//if(label >= 100 && label <= 109 && label != 104 && label != 108) right
//往左推 返回 1
//往右推 返回 0
//不是正常的 label 返回 2
//奇数推左，偶数推右，电子推左，常用推右
uint8 openart_judge(uint8 label)
{
		if(label >= 0 && label <= 99)
		{
			if(label % 2 == 1)
			{
				return 1;
			}
			else
				return 0;
		}
		else if(label == 104 || label == 108 || (label >= 110 && label <= 114))
		{
			return 1;
		}
		else if(label >= 100 && label <= 109 && label != 104 && label != 108)
		{
			return 0;
		}
		else
		{
			return 2;
		}
		
}
int ort_line=0;//分辨率160*128
void ort_show(void)
{
	tft180_show_string(80,0,"name");//名称
	tft180_show_string(0,0,"order");
	tft180_draw_line(30, 0, 30, 159,RGB565_RED);//分割线
	tft180_draw_line(0, 16, 127, 16,RGB565_RED);//分割线
	if(ort_line <= 7)
	{
		for(int i=0;i<ort_line;i++)
		{
			tft180_show_string(32,(i+1)*16,labels[ort_label[i]]);//名称
			//tft180_show_chinese(80, (i+1)*16, 15, labels[ort_label[i]], 3, RGB565_RED);
			tft180_draw_line(0, (i+1)*16+16, 127, (i+1)*16+16,RGB565_RED);//分割线
			tft180_show_string(0,(i+1)*16,labels[i+1]);//序号
		}	
	}
	else
	{
		if(key1_short == false)
		{
			for(int i=0;i<7;i++)
			{
				tft180_show_string(32,(i+1)*16,labels[ort_label[i]]);//名称
				//tft180_show_chinese(80, (i+1)*16, 15, labels[ort_label[i]], 3, RGB565_RED);
				tft180_draw_line(0, (i+1)*16+16, 127, (i+1)*16+16,RGB565_RED);//分割线
				tft180_show_string(0,(i+1)*16,labels[i+1]);//序号
			}
		}
		else
		{
			for(int i=7;i<ort_line;i++)
			{
				tft180_show_string(32,(i-7+1)*16,labels[ort_label[i]]);//名称
				//tft180_show_chinese(80, (i+1)*16, 15, labels[ort_label[i]], 3, RGB565_RED);
				tft180_draw_line(0, (i-7+1)*16+16, 127, (i-7+1)*16+16,RGB565_RED);//分割线
				tft180_show_string(0,(i-7+1)*16,labels[i+1]);//序号
			}
		}
		
	}
}

