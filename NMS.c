/*
 ============================================================================
 Name        : NMS.c
 Author      : yiqun
 Version     :
 Copyright   : Your copyright notice
 Description : NMS in C, Ansi-style
 ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define MAX(a,b)(a>b?a:b)
#define MIN(a,b)(a<=b?a:b)

//候选框
typedef struct{
    float lx, ly, rx, ry;	//左上角x  左上角y	 右下角x	 右下角y
    float cfd;	//置信度 Confidence
} box;

float overlap(float a_l, float a_r, float b_l, float b_r);

//传入两个候选框变量a、b，返回交并比值
//float box_iou(box a, box b) //传入指针是否更快？？
//{
//    return box_intersection(a, b)/box_union(a, b);	//dsp可进行除法优化！！
//}

//传入两个候选框变量某一维度坐标值，返回重叠部分的宽和高
float overlap(float a_l, float a_r, float b_l, float b_r){	//返回值是否可以优化？？//传入指针是否更快？？
	printf("a左边: %f,a右边: %f,b左边: %f,b右边: %f",a_l,a_r,b_r,b_r);
	float sum_sides = a_r - a_l + b_r - b_l;	//同一方向两个边长之和	//是否要绝对值
	float new_sides = MAX(a_r, b_r) - MIN(a_l , b_l);	//同一方向长度并集 //用max还是绝对值？？优化。
	return  sum_sides - new_sides;
}

//传入两个候选框变量a、b，返回相交面积值
float box_intersection(box a, box b)	//传入指针是否更快？？
{
    float w = overlap(a.lx, a.rx, b.lx, b.rx);
    float h = overlap(a.ry, a.ly, b.ry, b.ly);
    printf("w: %f, h: %f",w,h);
    if(w <= 0 || h <= 0) return 0;
    float area = w * h;
    return area;
}

int main(void) {
	box b1 = {200.0,50.0,400.0,0.0,0.8};
	box b2 = {0.0,100.0,100.0,0.0,0.9};
	float re = box_intersection(b1,b2);
	printf("re: %f",re);
	return EXIT_SUCCESS;
}
