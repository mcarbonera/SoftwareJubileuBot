#ifndef LINKED_LIST_H
#define LINKED_LIST_H

#ifndef NULL_POINTER
#define NULL_POINTER (void*)0
#endif

#ifndef NULL_ELEMENT
#define NULL_ELEMENT -1
#endif

#define MAX_SIZE 9
#define LINE_CTTE 1
#define LINE_DESC 2
#define LINE_ASC 3
#define TRAPEZIUM_ASC 0
#define TRAPEZIUM_DESC 1
#define TYPE_INTERVAL_1 1
#define TYPE_INTERVAL_2 2
#define TYPE_INTERVAL_3 3

#include <math.h>

typedef struct Data {
    double base;
    double xCenter;
    double inferenceValue;
    double auxLeftX, auxRightX;
    int nextValue;
} Data;

typedef struct LinkedList {
    Data data[MAX_SIZE];
    int inicial;
    int size;
} LinkedList;

void initLinkedList(LinkedList *linkedList);
void writeNodeElement(Data *data, double base, double xCenter, double inferenceValue);
void updateNodeElement(Data *data, double inferenceValue, double auxLeft, double auxRight);
int addElementOrdered(LinkedList *linkedList, double base, double xCenter, double inferenceValue);

void assignLineParameters(Data *out, double *a, double *b, int type);
double xIntersectionCalc(double a1, double b1, double a2, double b2);
int verifyIntersectionOnInterval(Data *out1, Data *out2, double *xIntersection, int intersectionInterval, int *typeIntersection);
double evalIntegralNumerator(double a, double b, double xInf, double xSup);
double evalIntegralDenominator(double a, double b, double xInf, double xSup);
void areaCalculation(double *numerator, double *denominator, double a, double b, double xInf, double xSup);
double calculateTrapeziumIntersect(Data *data, int ascDesc);
void area1Calculation(Data *data, double *a, double *b, double *xInf, double *xSup, double *num, double *den);
void area2Calculation(Data *data, double *a, double *b, double *xInf, double *xSup, double *num, double *den);
void area3Calculation(Data *data, Data *proxData, double *a, double *b, double *xInf, double *xSup, double *num, double *den);
double deffuzify(LinkedList *linkedList);

#endif
