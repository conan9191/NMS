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

 //��ѡ��
typedef struct {
	float lx, ly, rx, ry;	//���Ͻ�x  ���Ͻ�y	 ���½�x	 ���½�y
	float cfd;	//���Ŷ� Confidence
	int supression; //�Ƿ�����
} box;

float overlap(float a_l, float a_r, float b_l, float b_r);

//����������ѡ�����ĳһά������ֵ�������ص����ֵĿ�͸�
float overlap(float a_l, float a_r, float b_l, float b_r) {	//����ֵ�Ƿ�����Ż�����//����ָ���Ƿ���죿��
	//printf("a���: %f,a�ұ�: %f,b���: %f,b�ұ�: %f", a_l, a_r, b_r, b_r);
	float sum_sides = a_r - a_l + b_r - b_l;	//ͬһ���������߳�֮��	//�Ƿ�Ҫ����ֵ
	float new_sides = MAX(a_r, b_r) - MIN(a_l, b_l);	//ͬһ���򳤶Ȳ��� //��max���Ǿ���ֵ�����Ż���
	return  sum_sides - new_sides;
}

//����������ѡ�����a��b�������ཻ���ֵ
float box_intersection(box *a, box *b)	//����ָ���Ƿ���죿��
{
	float w = overlap(a->lx, a->rx, b->lx, b->rx);
	float h = overlap(a->ry, a->ly, b->ry, b->ly);
	//printf("w: %f, h: %f", w, h);
	if (w <= 0 || h <= 0) return 0;
	float area = w * h;	//ֱ�ӷ��� ����������??
	return w * h;
}

//����������ѡ�����a��b�����ز������ֵ
float box_union(box *a, box *b) {	//����ָ���Ƿ���죿��
	float insersection = box_intersection(a, b);	//���ٺ������û��ǿ��ٱ����ռ�����Ż�����
	float areaA = (a->rx - a->lx) * (a->ly - a->ry);	//�Ƿ�ʹ�þ���ֵ����
	float areaB = (b->rx - b->lx) * (b->ly - b->ry);
	float area = areaA + areaB - insersection;	//ֱ�ӷ��� ����������??
	return area;
}

//����������ѡ�����a��b�����ؽ�����ֵ
float box_iou(box *a, box *b) //����ָ���Ƿ���죿��
{
    return box_intersection(a, b)/box_union(a, b);	//dsp�ɽ��г����Ż�����
}

//�������ŶȴӴ�С����
void sort_nms(box* B) { //�����㷨��ʲô���� �Ƿ�ʹ�������ջ��ϣ����DSP�أ�SoftNMS�أ���������Һ�����ɾ���ĸ���ʱ��C++ ERASE()Դ���д

}

//NMS�㷨,��������ѡ�򼯺ϡ������ѡ���Ӧ���Ŷȼ��ϡ���ֵ
void do_nms(box* B, box*D, float Nt) {
	sort_nms(B);
	int count = 4;
	int max_index = 0;
	int current_index = 0;
	while (0<count--) {	//̽��һ��ѭ���ķ����������������Ƚϣ��ݹ飿��
		if (!B[current_index].supression) {
			D[current_index] = B[current_index];
		}
		current_index++;
	}
	return D;
}

//���ݼ����䴦��
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

	//��ʱ��
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
