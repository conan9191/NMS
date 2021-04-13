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
#include <time.h>

#define MAX(a,b)(a>b?a:b)
#define MIN(a,b)(a<=b?a:b)
#define BOX_SIZE 100
#define THRESH 0.3

 //候选框
typedef struct {
	float lx, ly, rx, ry;	//左上角x  左上角y	 右下角x	 右下角y
	float cfd;	//置信度 Confidence
	int supression; //是否被抑制
} box;

float overlap(float a_l, float a_r, float b_l, float b_r);

//传入两个候选框变量某一维度坐标值，返回重叠部分的宽和高
float overlap(float a_l, float a_r, float b_l, float b_r) {	//返回值是否可以优化？？//传入指针是否更快？？
	//printf("a左边: %f,a右边: %f,b左边: %f,b右边: %f", a_l, a_r, b_r, b_r);
	float sum_sides = a_r - a_l + b_r - b_l;	//同一方向两个边长之和	//是否要绝对值
	float new_sides = MAX(a_r, b_r) - MIN(a_l, b_l);	//同一方向长度并集 //用max还是绝对值？？优化。
	return  sum_sides - new_sides;
}

//传入两个候选框变量a、b，返回相交面积值
float box_intersection(box *a, box *b)	//传入指针是否更快？？
{
	float w = overlap(a->lx, a->rx, b->lx, b->rx);
	float h = overlap(a->ry, a->ly, b->ry, b->ly);
	//printf("w: %f, h: %f", w, h);
	if (w <= 0 || h <= 0) return 0;
	float area = w * h;	//直接返回 不创建变量??
	return w * h;
}

//传入两个候选框变量a、b，返回并集面积值
float box_union(box *a, box *b) {	//传入指针是否更快？？
	float insersection = box_intersection(a, b);	//减少函数调用还是开辟变量空间进行优化？？
	float areaA = (a->rx - a->lx) * (a->ly - a->ry);	//是否使用绝对值？？
	float areaB = (b->rx - b->lx) * (b->ly - b->ry);
	float area = areaA + areaB - insersection;	//直接返回 不创建变量??
	return area;
}

//传入两个候选框变量a、b，返回交并比值
float box_iou(box *a, box *b) //传入指针是否更快？？
{
    return box_intersection(a, b)/box_union(a, b);	//dsp可进行除法优化！！
}

//按照置信度从大到小排序
void sort_nms(box* B) { //排序算法用什么？？ 是否使用链表堆栈哈希表？？DSP呢？SoftNMS呢？（数组查找和链表删除哪个费时）C++ ERASE()源码改写

}

//NMS算法,传入出入候选框集合、输入候选框对应置信度集合、阈值
void do_nms(box* B, box*D, float Nt) {
	sort_nms(B);
	int count = 4;
	int max_index = 0;
	int current_index = 0;
	while (0<count--) {	//探究一轮循环的方法，与所以输出框比较，递归？？
		if (!B[current_index].supression) {
			D[current_index] = B[current_index];
		}
		current_index++;
	}
	return D;
}

//数据集分配处理
void data_clean(box* b) {	
	b[0].lx = 50.0;
	b[0].ly = 100.0;
	b[0].rx = 100.0;
	b[0].ry = 0.0;
	b[0].cfd = 0.8;
	b[0].supression = 0;

	b[1].lx = 0.0;
	b[1].ly = 100.0;
	b[1].rx = 100.0;
	b[1].ry = 0.0;
	b[1].cfd = 0.9;
	b[1].supression = 0;

	b[2].lx = 100.0;
	b[2].ly = 200.0;
	b[2].rx = 100.0;
	b[2].ry = 0.0;
	b[2].cfd = 0.7;
	b[2].supression = 0;

	b[3].lx = 0.0;
	b[3].ly = 200.0;
	b[3].rx = 0.0;
	b[3].ry = 100.0;
	b[3].cfd = 0.2;
	b[3].supression = 0;
}

int main(void) {
	box* b = (box*)malloc(sizeof(box) * BOX_SIZE);
//	float* s = (float*)malloc(sizeof(float) * BOX_SIZE);
	box* d = (box*)malloc(sizeof(box) * BOX_SIZE);
	data_clean(b);

	//计时器
	long i = 10000000L;
	clock_t start, finish;
	double duration;
	float iou;
	start = clock();
	/*******************/
	while (i--) {
		do_nms(b,d,THRESH);
		//iou = box_iou(b+0,b+1);
	}
	/*******************/
	finish = clock();
	duration = (double)(finish - start) / CLOCKS_PER_SEC;
	printf("%f seconds\n", duration);
	//printf("IoU: %f\n", iou);
	//printf("s1: %f, s2: %f",*(s+0), *(s+1));
	box* dd = d;

	for(i=0;i<4;i++)
		printf("d: %f\n",(dd+i)->cfd);
	free(b);
	free(d);
	system("pause");
	return EXIT_SUCCESS;
}
