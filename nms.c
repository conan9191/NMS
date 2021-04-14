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
#define BOX_SIZE 4
#define THRESH 0.3f

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

//�鲢����
void merge(box* B, box* temp, int l, int mid, int r) {
	int i = l, j = mid + 1, m = 1, k = 1;
	while (i <= mid && j <= r) {
		if ((B + i)->cfd < (B + j)->cfd)
			*(temp + m++) = *(B + j++);
		else 
			*(temp + m++) = *(B + i++);
	}
	while (i <= mid) 
		*(temp + m++) = *(B + i++);
	while (j <= r) 
		*(temp + m++) = *(B + j++);
	for (k = 1; k <= r - l + 1; k++)
		*(B + l + k - 1) = *(temp + k);
}

//�������ŶȴӴ�С����
void sort_nms(box* B, box* temp, int l, int r) {		//��������Һ�����ɾ���ĸ���ʱ��C++ ERASE()Դ���д
	if (l < r) {
		int mid = (l + r) / 2;
		sort_nms(B, temp, l, mid);
		sort_nms(B, temp, mid + 1, r);
		merge(B, temp, l, mid, r);
	}
}

//NMS�㷨,��������ѡ�򼯺ϡ������ѡ���Ӧ���Ŷȼ��ϡ���ֵ,���������ѡ�����
int do_nms(box* B, box*D, float Nt) {
	int i;
	for (i = 0; i < BOX_SIZE; i++)
		printf("d: %.2f\n", (B + i)->cfd);

	box* temp = (box*)malloc(sizeof(box) * (BOX_SIZE+1));
	sort_nms(B, temp, 0, (BOX_SIZE - 1));
	free(temp);

	for (i = 0; i < BOX_SIZE; i++)
		printf("d: %.2f\n", (B + i)->cfd);

	int max_index = 0, current_index = 0;
	int j;
	float iou;
	while (current_index< BOX_SIZE) {	//̽��һ��ѭ���ķ����������������Ƚϣ��ݹ飿��
		printf("current_index: %d\n", current_index);
		if (!((B+current_index)->supression)) {
			*(D + max_index) = *(B + current_index);
			(B + current_index)->supression = 1;
			for (j = current_index+1; j < BOX_SIZE; j++) {
				iou = box_iou(D + max_index, B + j);
				printf("iou: %f\n", iou);
				if (iou >= Nt)
					(B + j)->supression = 1;
			}
			max_index++;
		}
		current_index++;
	}
	return max_index;
}

//���ݼ����䴦��
void data_clean(box* b) {	
	b[0].lx = 50.0;
	b[0].ly = 100.0;
	b[0].rx = 100.0;
	b[0].ry = 0.0;
	b[0].cfd = 0.14f;
	b[0].supression = 0;

	b[1].lx = 0.0;
	b[1].ly = 100.0;
	b[1].rx = 100.0;
	b[1].ry = 0.0;
	b[1].cfd = 0.90f;
	b[1].supression = 0;

	b[2].lx = 100.0;
	b[2].ly = 200.0;
	b[2].rx = 200.0;
	b[2].ry = 0.0;
	b[2].cfd = 0.70f;
	b[2].supression = 0;

	b[3].lx = 150.0;
	b[3].ly = 200.0;
	b[3].rx = 300.0;
	b[3].ry = 100.0;
	b[3].cfd = 0.80f;
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
	int count = 0;
	start = clock();
	/*******************/
	//while (i--) {
		count = do_nms(b, d, THRESH);
		printf("count: %d\n", count);
		//iou = box_iou(b+0,b+1);
	//}
	/*******************/
	finish = clock();
	duration = (finish - start) / CLOCKS_PER_SEC;
	printf("%f seconds\n", duration);
	//printf("IoU: %f\n", iou);
	//printf("s1: %f, s2: %f",*(s+0), *(s+1));
	if (d) {
		for (i = 0; i < count; i++)
			printf("d: %.2f\n", (d + i)->cfd);
	}
	free(b);
	free(d);
	system("pause");
	return EXIT_SUCCESS;
}
