#ifndef bwlabel_h
#define bwlabel_h

#include <opencv2/opencv.hpp>

int bwlabel(IplImage* img, int n, int* labels);
int find( int set[], int x );
#endif