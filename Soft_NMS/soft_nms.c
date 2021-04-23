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
#include <string.h>
//#include <opencv2/opencv.hpp> 


#define MAX(a,b)(a>b?a:b)
#define MIN(a,b)(a<=b?a:b)
//#define BOX_SIZE 8
#define THRESH 0.40f
#pragma warning(disable:4996)

 //��ѡ��
typedef struct {
	float lx, ly, rx, ry;	//���Ͻ�x  ���Ͻ�y	 ���½�x	 ���½�y
	float cfd;	//���Ŷ� Confidence
	float class_cfd;
	float label;
	int supression; //�Ƿ�����
} box;

typedef struct {
	char name[50]; 
} namesList;

float overlap(float a_l, float a_r, float b_l, float b_r);

//����������ѡ�����ĳһά������ֵ�������ص����ֵĿ�͸�
float overlap(float a_l, float a_r, float b_l, float b_r) {	//����ֵ�Ƿ�����Ż�����//����ָ���Ƿ���죿��
	//printf("a���: %f,a�ұ�: %f,b���: %f,b�ұ�: %f", a_l, a_r, b_r, b_r);
	float sum_sides = a_r - a_l + b_r - b_l;	//ͬһ���������߳�֮��	//�Ƿ�Ҫ����ֵ
	float new_sides = MAX(a_r, b_r) - MIN(a_l, b_l);	//ͬһ���򳤶Ȳ��� //��max���Ǿ���ֵ�����Ż���
	return  sum_sides - new_sides;
}

//����������ѡ�����a��b�������ཻ���ֵ
float box_intersection(box* a, box* b)	//����ָ���Ƿ���죿��
{
	float w = overlap(a->lx, a->rx, b->lx, b->rx);
	float h = overlap(a->ly, a->ry, b->ly, b->ry);

	//printf("w: %f, h: %f", w, h);
	if (w <= 0 || h <= 0) return 0;
	float area = w * h;	//ֱ�ӷ��� ����������??
	return w * h;
}

//����������ѡ�����a��b�����ز������ֵ
float box_union(box* a, box* b) {	//����ָ���Ƿ���죿��
	float insersection = box_intersection(a, b);	//���ٺ������û��ǿ��ٱ����ռ�����Ż�����
	float areaA = (a->rx - a->lx) * (a->ry - a->ly);	//�Ƿ�ʹ�þ���ֵ����
	float areaB = (b->rx - b->lx) * (b->ry - b->ly);
	float area = areaA + areaB - insersection;	//ֱ�ӷ��� ����������??
	return area;
}

//����������ѡ�����a��b�����ؽ�����ֵ
float box_iou(box* a, box* b) //����ָ���Ƿ���죿��
{
	return box_intersection(a, b) / box_union(a, b);	//dsp�ɽ��г����Ż�����
}
/*
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
int do_nms(box* B, box* D, float Nt) {
	int i;
	printf("----------------����ǰ-----------------\n");
	for (i = 0; i < BOX_SIZE; i++)
		printf("d: %.2f\n", (B + i)->cfd);

	box* temp = (box*)malloc(sizeof(box) * (BOX_SIZE + 1));
	sort_nms(B, temp, 0, (BOX_SIZE - 1));
	free(temp);

	printf("----------------�����-----------------\n");
	for (i = 0; i < BOX_SIZE; i++)
		printf("d: %.2f\n", (B + i)->cfd);

	int max_index = 0, current_index = 0;
	int j;
	float iou;
	while (current_index < BOX_SIZE) {	//̽��һ��ѭ���ķ����������������Ƚϣ��ݹ飿��
		printf("----------current_index: %d----------\n", current_index);
		if (!((B + current_index)->supression)) {
			*(D + max_index) = *(B + current_index);
			(B + current_index)->supression = 1;
			for (j = current_index + 1; j < BOX_SIZE; j++) {
				iou = box_iou(D + max_index, B + j);
				printf("iou: %.2f\n", iou);
				if (iou >= Nt)
					(B + j)->supression = 1;
			}
			max_index++;
		}
		current_index++;
	}
	return max_index;
}
*/

//soft-nms �����ѡ�򼯺ϡ�������ϡ�nms������š�IoU��ֵ�����Ŷ���ֵ����˹��������
int do_soft_nms(box* B, unsigned int method, int max_size,
					float Nt, float threshold , float sigma) {
	//�Ƿ���ռ���??
	int i = 0, n = max_size, j, max_index, current_index;
	float max_cfd, weight,iou;
	box temp;
	while (i < n) {
		//printf("----------%d-----------\n", i);
		max_index = i;
		current_index = i + 1;
		max_cfd = (B + i)->cfd;
		temp = *(B + i);
		//������ֱ�������ֵ
		for (j = current_index; j < n; j++){
			//printf("max: %.6f\n", max_cfd);
			if (max_cfd < (B + j)->cfd) {
				max_cfd = (B + j)->cfd;
				max_index = j;
			}	
		}
		//������ѡ�������ǰ�棬���û��滻�ĺ�ѡ��
		*(B + i) = *(B + max_index);
		*(B + max_index) = temp;
		//IoU�Ƚ�
		while (current_index < n) {
			iou = box_iou(B + i, B + current_index);
			//printf("iou: %.6f\n", iou);
			if (iou <= 0 || (B + i)->label!=(B + current_index)->label){
				current_index++;
				continue;
			}
			if (method == 1){ // ��ѡ��������Ժ���
				if (iou > Nt) 
					weight = 1 - iou;
				else
					weight = 1;
			}
			else if (method == 2){	//��ѡ����Ǹ�˹����
				weight = exp(-(iou * iou) / sigma);
			}else {	 //��ѡ����Ǵ�ͳNMS
				if (iou > Nt)
					weight = 0;
				else
					weight = 1;
			}
			//printf("cfd: %.2f\n", (B + current_index)->cfd);
			(B + current_index)->cfd *= weight;
			//printf("New cfd: %.2f\n", (B + current_index)->cfd);
			if ((B + current_index)->cfd <= threshold) {
				//printf("����\n");
				*(B + current_index) = *(B + n - 1);
				(B + n - 1)->supression = 1;
				n--;
				current_index--;
			}
			current_index++;
		}
		i++;
	}
	return n;
}

int read_file(box* B,char* filePath) {
	//char data[100];
	int flag = 0, count = 0, i = 0;

	FILE* fp = fopen(filePath, "r");
	if (!fp)
	{
		printf("can't open file\n");
		return 0;
	}

	while (!feof(fp)){
		flag = fgetc(fp);
		if (flag == '\n')
			count++;
	}
	rewind(fp);

	char StrLine[1024];             //ÿ������ȡ���ַ���
	char text[] = " ";
	char* split = text;
	char* data;
	
	while (!feof(fp)&&i<count)
	{
		//fscanf(fp, "%s", &data);
		fgets(StrLine, 1024, fp);  //��ȡһ��
		data = strtok(StrLine, split);

		(B + i)->lx = atof(data);
		data = strtok(NULL, split);
		(B + i)->ly = atof(data);
		data = strtok(NULL, split);
		(B + i)->rx = atof(data);
		data = strtok(NULL, split);
		(B + i)->ry = atof(data);
		data = strtok(NULL, split);
		(B + i)->cfd = atof(data);
		data = strtok(NULL, split);
		(B + i)->class_cfd = atof(data);
		data = strtok(NULL, split);
		(B + i)->label = atof(data);
		data = strtok(NULL, split);
		(B + i)->supression = 0;

		i++;
	}
	printf("\n");
	fclose(fp);
	return count;
}

int read_file2(namesList* namelist, char* namePath) {
	//char data[100];
	int flag = 0, count = 0, i = 0, j = 0;
	FILE* fp = fopen(namePath, "r");
	if (!fp)
	{
		printf("can't open file\n");
		return 0;
	}

	char StrLine[50];             //ÿ������ȡ���ַ���
	char text[] = ".";
	char* split = text;

	while (!feof(fp))
	{
		//fscanf(fp, "%s", &data);
		fgets(StrLine, 50, fp);  //��ȡһ��
		strtok(StrLine, split);
		strcat(StrLine, ".txt");
		for (j = 0; j < 50; j++) {
			(namelist + i)->name[j] = StrLine[j+16];
		}
		printf("%s\n", (namelist + i)->name);

		i++;
	}
	fclose(fp);
	return i;
}

int writeFile(box* B,int count,char* fileName) {
	int i;
	char text[60] = "..\\Soft_NMS\\data\\coco\\output\\";
	strcat(text, fileName);
	char* filePath = text;
	printf("%s\n", filePath);

	FILE* fpWrite = fopen(filePath, "w");
	if (fpWrite == NULL)
	{
		return 0;

	}

	printf("----------�������----------\n");
	for (i = 0; i < count; i++){
		printf("box: %.3f\n", (B + i)->cfd);
		fprintf(fpWrite, "%f %f %f %f %f %f %f", (B + i)->lx, (B + i)->ly, 
			(B + i)->rx, (B + i)->ry, (B + i)->cfd, (B + i)->class_cfd, (B + i)->label);
		fprintf(fpWrite, "\n");
	}

	fclose(fpWrite);
}


int main(void) {

	int j,k;
	namesList* namelist = (namesList*)malloc(sizeof(namesList) * 1000);
	char text1[30] = "..\\Soft_NMS\\data\\5k.txt";
	char* namePath = text1;
	int names_size = read_file2(namelist,namePath);
	printf("name size: %d\n", names_size);

	//	//��ʱ��
	long i = 10000000L;
	clock_t start, finish;
	double duration;
	start = clock();

	for (j = 0; j < names_size; j++) {
		box* b = (box*)malloc(sizeof(box) * 50);
		char text[100] = "..\\Soft_NMS\\data\\coco\\";
		char* text2 = (namelist + j)->name;
		char* fileName = text2;
		strcat(text, text2);
		char* filePath = text;
		int max_size = read_file(b, filePath);
		int count = 0;
		count = do_soft_nms(b, 2, max_size, THRESH, 0.80, 0.6);
		int w = writeFile(b, count, fileName);
		free(b);
	}
	finish = clock();
	duration = (finish - start) / CLOCKS_PER_SEC;
	printf("Time: %f seconds\n", duration);
	free(namelist);
	system("pause");
	return EXIT_SUCCESS;
}

//	box* b = (box*)malloc(sizeof(box) * 20);
//	
//	char text[60] = "..\\Soft_NMS\\data\\coco\\";
//	char text2[30] = "COCO_val2014_000000000196.txt";
//	char* fileName = text2;
//	strcat(text, text2);
//	char* filePath = text;
//	
//	int max_size = read_file(b, filePath);
////
////	cv::Mat img = cv::imread("./data/COCO_val2014_000000000133.jpg", cv::IMREAD_UNCHANGED);
////
////	cv::Mat img_or = img.clone();
////
////	for (j = 0; j < max_size; j++) {
////		cv::Point p1 = cv::Point((b+j)->lx, (b + j)->ly);
////		cv::Point p2 = cv::Point((b + j)->rx, (b + j)->ry);
////		cv::rectangle(img, p1, p2, cv::Scalar(255, 0, 0), 3, 8);  //������--����һ
////	}
////	cv::imshow("img", img);
////
////
////	//��ʱ��
//	long i = 10000000L;
//	clock_t start, finish;
//	double duration;
//	int count = 0;
//	start = clock();
//	/*******************/
//	//while (i--) {
//	//count = do_nms(b, d, THRESH);
//	count = do_soft_nms(b,2, max_size,THRESH,0.80,0.6);
//	//printf("---------count: %d-------------\n", count);
//	//iou = box_iou(b+0,b+1);
////}
///*******************/
//	finish = clock();
//	duration = (finish - start) / CLOCKS_PER_SEC;
//	printf("----------%f seconds----------\n", duration);
//	//printf("IoU: %f\n", iou);
//	//printf("s1: %f, s2: %f",*(s+0), *(s+1));
//
////	for (j = 0; j < count; j++) {
////		cv::Point p1 = cv::Point((b + j)->lx, (b + j)->ly);
////		cv::Point p2 = cv::Point((b + j)->rx, (b + j)->ry);
////		cv::rectangle(img_or, p1, p2, cv::Scalar(255, 0, 0), 3, 8);  //������--����һ
////	}
////
////
////	cv::imshow("img2", img_or);
////
//	int w = writeFile(b, count, fileName);
//
//	free(b);
//
//	cv::waitKey();
