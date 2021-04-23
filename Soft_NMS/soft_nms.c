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

 //候选框
typedef struct {
	float lx, ly, rx, ry;	//左上角x  左上角y	 右下角x	 右下角y
	float cfd;	//置信度 Confidence
	float class_cfd;
	float label;
	int supression; //是否被抑制
} box;

typedef struct {
	char name[50]; 
} namesList;

float overlap(float a_l, float a_r, float b_l, float b_r);

//传入两个候选框变量某一维度坐标值，返回重叠部分的宽和高
float overlap(float a_l, float a_r, float b_l, float b_r) {	//返回值是否可以优化？？//传入指针是否更快？？
	//printf("a左边: %f,a右边: %f,b左边: %f,b右边: %f", a_l, a_r, b_r, b_r);
	float sum_sides = a_r - a_l + b_r - b_l;	//同一方向两个边长之和	//是否要绝对值
	float new_sides = MAX(a_r, b_r) - MIN(a_l, b_l);	//同一方向长度并集 //用max还是绝对值？？优化。
	return  sum_sides - new_sides;
}

//传入两个候选框变量a、b，返回相交面积值
float box_intersection(box* a, box* b)	//传入指针是否更快？？
{
	float w = overlap(a->lx, a->rx, b->lx, b->rx);
	float h = overlap(a->ly, a->ry, b->ly, b->ry);

	//printf("w: %f, h: %f", w, h);
	if (w <= 0 || h <= 0) return 0;
	float area = w * h;	//直接返回 不创建变量??
	return w * h;
}

//传入两个候选框变量a、b，返回并集面积值
float box_union(box* a, box* b) {	//传入指针是否更快？？
	float insersection = box_intersection(a, b);	//减少函数调用还是开辟变量空间进行优化？？
	float areaA = (a->rx - a->lx) * (a->ry - a->ly);	//是否使用绝对值？？
	float areaB = (b->rx - b->lx) * (b->ry - b->ly);
	float area = areaA + areaB - insersection;	//直接返回 不创建变量??
	return area;
}

//传入两个候选框变量a、b，返回交并比值
float box_iou(box* a, box* b) //传入指针是否更快？？
{
	return box_intersection(a, b) / box_union(a, b);	//dsp可进行除法优化！！
}
/*
//归并排序
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

//按照置信度从大到小排序
void sort_nms(box* B, box* temp, int l, int r) {		//（数组查找和链表删除哪个费时）C++ ERASE()源码改写
	if (l < r) {
		int mid = (l + r) / 2;
		sort_nms(B, temp, l, mid);
		sort_nms(B, temp, mid + 1, r);
		merge(B, temp, l, mid, r);
	}
}


//NMS算法,传入出入候选框集合、输入候选框对应置信度集合、阈值,返回输出候选框个数
int do_nms(box* B, box* D, float Nt) {
	int i;
	printf("----------------排序前-----------------\n");
	for (i = 0; i < BOX_SIZE; i++)
		printf("d: %.2f\n", (B + i)->cfd);

	box* temp = (box*)malloc(sizeof(box) * (BOX_SIZE + 1));
	sort_nms(B, temp, 0, (BOX_SIZE - 1));
	free(temp);

	printf("----------------排序后-----------------\n");
	for (i = 0; i < BOX_SIZE; i++)
		printf("d: %.2f\n", (B + i)->cfd);

	int max_index = 0, current_index = 0;
	int j;
	float iou;
	while (current_index < BOX_SIZE) {	//探究一轮循环的方法，与所以输出框比较，递归？？
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

//soft-nms 传入候选框集合、输出集合、nms方法编号、IoU阈值、置信度阈值、高斯函数参数
int do_soft_nms(box* B, unsigned int method, int max_size,
					float Nt, float threshold , float sigma) {
	//是否检测空集合??
	int i = 0, n = max_size, j, max_index, current_index;
	float max_cfd, weight,iou;
	box temp;
	while (i < n) {
		//printf("----------%d-----------\n", i);
		max_index = i;
		current_index = i + 1;
		max_cfd = (B + i)->cfd;
		temp = *(B + i);
		//不排序，直接找最大值
		for (j = current_index; j < n; j++){
			//printf("max: %.6f\n", max_cfd);
			if (max_cfd < (B + j)->cfd) {
				max_cfd = (B + j)->cfd;
				max_index = j;
			}	
		}
		//将最大候选框放入最前面，并置换替换的候选框
		*(B + i) = *(B + max_index);
		*(B + max_index) = temp;
		//IoU比较
		while (current_index < n) {
			iou = box_iou(B + i, B + current_index);
			//printf("iou: %.6f\n", iou);
			if (iou <= 0 || (B + i)->label!=(B + current_index)->label){
				current_index++;
				continue;
			}
			if (method == 1){ // 当选择的是线性函数
				if (iou > Nt) 
					weight = 1 - iou;
				else
					weight = 1;
			}
			else if (method == 2){	//当选择的是高斯函数
				weight = exp(-(iou * iou) / sigma);
			}else {	 //当选择的是传统NMS
				if (iou > Nt)
					weight = 0;
				else
					weight = 1;
			}
			//printf("cfd: %.2f\n", (B + current_index)->cfd);
			(B + current_index)->cfd *= weight;
			//printf("New cfd: %.2f\n", (B + current_index)->cfd);
			if ((B + current_index)->cfd <= threshold) {
				//printf("交换\n");
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

	char StrLine[1024];             //每行最大读取的字符数
	char text[] = " ";
	char* split = text;
	char* data;
	
	while (!feof(fp)&&i<count)
	{
		//fscanf(fp, "%s", &data);
		fgets(StrLine, 1024, fp);  //读取一行
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

	char StrLine[50];             //每行最大读取的字符数
	char text[] = ".";
	char* split = text;

	while (!feof(fp))
	{
		//fscanf(fp, "%s", &data);
		fgets(StrLine, 50, fp);  //读取一行
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

	printf("----------输出队列----------\n");
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

	//	//计时器
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
////		cv::rectangle(img, p1, p2, cv::Scalar(255, 0, 0), 3, 8);  //画矩形--方法一
////	}
////	cv::imshow("img", img);
////
////
////	//计时器
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
////		cv::rectangle(img_or, p1, p2, cv::Scalar(255, 0, 0), 3, 8);  //画矩形--方法一
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
