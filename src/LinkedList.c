#include "LinkedList.h"

void initLinkedList(LinkedList *linkedList) {
    linkedList->inicial = 0;
    linkedList->size = 0;
}

void writeNodeElement(Data *data, double base, double xCenter, double inferenceValue) {
    data->base = base;
    data->xCenter = xCenter;
    data->inferenceValue = inferenceValue;
    data->auxLeftX = calculateTrapeziumIntersect(data, TRAPEZIUM_ASC);
    data->auxRightX = calculateTrapeziumIntersect(data, TRAPEZIUM_DESC);
    data->nextValue = NULL_ELEMENT;
}

void updateNodeElement(Data *data, double inferenceValue, double auxLeft, double auxRight) {
    data->inferenceValue = inferenceValue;
    data->auxLeftX = auxLeft;
    data->auxRightX = auxRight;
}

int addElementOrdered(LinkedList *linkedList, double base, double xCenter, double inferenceValue) {
    int itIdx = linkedList->inicial;
    int proxIdx;
    Data tempData;

    if(linkedList->size == 0) { // inserir primeiro elemento
        writeNodeElement(&linkedList->data[linkedList->inicial], base, xCenter, inferenceValue);
        linkedList->size++;
        return 1;
    }
    if(linkedList->data[itIdx].xCenter > xCenter) { // Inserir no inicio:
        if(linkedList->size > MAX_SIZE) { // tamanho máximo atingido
            return 0;
        }
        //Tem espaço pra inserir
        writeNodeElement(&linkedList->data[linkedList->size], base, xCenter, inferenceValue);
        linkedList->data[linkedList->size].nextValue = linkedList->inicial;
        linkedList->inicial = linkedList->size;
        linkedList->size++;
        return 1;
    }
    // Determinar apontamentos
    while(linkedList->data[itIdx].nextValue != NULL_ELEMENT) { // tem espaço para inserir e elemento não será inserido ao final da lista
        proxIdx = linkedList->data[itIdx].nextValue;
        if(linkedList->data[itIdx].xCenter == xCenter) { // é a mesma recomendação! Cálculo da função de agregação utiliza max:
            if(inferenceValue > linkedList->data[itIdx].inferenceValue) {
                writeNodeElement(&tempData, base, xCenter, inferenceValue);
                updateNodeElement(&linkedList->data[itIdx], inferenceValue, tempData.auxLeftX, tempData.auxRightX);
            }
            return 1;
        }
        if(linkedList->data[proxIdx].xCenter <= xCenter) { // não encontrou posição de inserção
            itIdx = proxIdx;
        } else { // encontrou posição de inserção;
            if(linkedList->size > MAX_SIZE) { // tamanho máximo atingido
                return 0;
            }
            //Tem espaço pra inserir
            writeNodeElement(&linkedList->data[linkedList->size], base, xCenter, inferenceValue);

            linkedList->data[linkedList->size].nextValue = proxIdx; // recem inserido aponta para proximo.
            linkedList->data[itIdx].nextValue = linkedList->size; // nó anterior aponta para recem inserido.
            linkedList->size++;
            return 1;
        }
    }
    if(linkedList->data[itIdx].xCenter == xCenter) { // verificar se ultimo elemento é igual:
        if(inferenceValue > linkedList->data[itIdx].inferenceValue) {
            writeNodeElement(&tempData, base, xCenter, inferenceValue);
            updateNodeElement(&linkedList->data[itIdx], inferenceValue, tempData.auxLeftX, tempData.auxRightX);
        }
        return 1;
    }
    // inserir elemento no final da lista.
    if(linkedList->size > MAX_SIZE) { // tamanho máximo atingido
        return 0;
    }
    //Tem espaço pra inserir
    writeNodeElement(&linkedList->data[linkedList->size], base, xCenter, inferenceValue);
    linkedList->data[itIdx].nextValue = linkedList->size; // nó anterior aponta para recem inserido.
    linkedList->size++;
    return 1;
}

void assignLineParameters(Data *out, double *a, double *b, int type) {
    switch(type) {
        case LINE_CTTE:
            *a = 0;
            *b = out->inferenceValue;
            break;
        case LINE_DESC:
            *a = -2/out->base;
            *b = -(*a)*(out->xCenter + out->base/2);
            break;
        case LINE_ASC:
            *a = 2/out->base;
            *b = -(*a)*(out->xCenter - out->base/2);
            break;
    }
}

double xIntersectionCalc(double a1, double b1, double a2, double b2) {
    return (b2 - b1) / (a1 - a2);
}

int verifyIntersectionOnInterval(Data *out1, Data *out2, double *xIntersection, int intersectionInterval, int *typeIntersection) {
    double a1, a2, b1, b2; // parâmetros das retas a comparar
    double temp;
    /**
     * 1,4 - reta ctte
     * 2 - reta desc
     * 3 - reta asc
     * */
    if(intersectionInterval == TYPE_INTERVAL_1 && out1->inferenceValue <= out2->inferenceValue) {
        //1-3 1-4
        assignLineParameters(out1,&a1,&b1,LINE_CTTE);
        assignLineParameters(out2,&a2,&b2,LINE_ASC);
        temp = xIntersectionCalc(a1,b1,a2,b2);
        if(temp >= out1->auxLeftX && temp <= out1->auxRightX) {
            *xIntersection = temp;
            *typeIntersection = 1;
            return 1;
        }
        return 0;
    }
    //2-3
    if(intersectionInterval == TYPE_INTERVAL_2) {
        assignLineParameters(out1,&a1,&b1,LINE_DESC);
        assignLineParameters(out2,&a2,&b2,LINE_ASC);
        temp = xIntersectionCalc(a1,b1,a2,b2);
        if(temp >= out1->auxRightX && temp <= (out1->xCenter + out1->base/2) && (a1*temp+b1) < out2->inferenceValue) {
            *xIntersection = temp;
            *typeIntersection = 2;
            return 1;
        }
        return 0;
    }
    //2-4
    if(intersectionInterval == TYPE_INTERVAL_3) {
        assignLineParameters(out1,&a1,&b1,LINE_DESC);
        assignLineParameters(out2,&a2,&b2,LINE_CTTE);
        temp = xIntersectionCalc(a1,b1,a2,b2);
        if(temp >= out1->auxRightX && temp <= (out1->xCenter + out1->base/2)) {
            *xIntersection = temp;
            *typeIntersection = 3;
            return 1;
        }
    }
    return 0;
}

double evalIntegralNumerator(double a, double b, double xInf, double xSup) {
    // | (ax+b)x dx == | ax^2 + bx == ax^3 / 3 + bx^2 / 2
    return (a*xSup*xSup*xSup/3 + b*xSup*xSup/2) -
            (a*xInf*xInf*xInf/3 + b*xInf*xInf/2);
}

double evalIntegralDenominator(double a, double b, double xInf, double xSup) {
    // | (ax+b) dx == ax^2 / 2 + bx
    return (a*xSup*xSup/2 + b*xSup) -
            (a*xInf*xInf/2 + b*xInf);
}

void areaCalculation(double *numerator, double *denominator, double a, double b, double xInf, double xSup) {
    if(!((xInf < -1 && xSup < -1) || (xInf > 1 && xSup > 1))) {
        if(xInf < -1 && xSup > -1) {
            xInf = -1;
        }
        if(xSup > 1 && xInf < 1) {
            xSup = 1;
        }
        //if(xSup != xInf) {
        *numerator+=evalIntegralNumerator(a,b,xInf,xSup);
        *denominator+=evalIntegralDenominator(a,b,xInf,xSup);
        //}
    }
}

double calculateTrapeziumIntersect(Data *data, int ascDesc) {
    switch(ascDesc) {
        case TRAPEZIUM_ASC:
            return data->xCenter - (data->base/2)*(1-data->inferenceValue);
        case TRAPEZIUM_DESC:
            return data->xCenter + (data->base/2)*(1-data->inferenceValue);
    }
    return -1;
}

void area1Calculation(Data *data, double *a, double *b, double *xInf, double *xSup, double *num, double *den) {
    *xSup = data->auxLeftX;
    assignLineParameters(data,a,b,LINE_ASC);
    areaCalculation(num, den,*a,*b,*xInf,*xSup);
    *xInf = *xSup;
}

void area2Calculation(Data *data, double *a, double *b, double *xInf, double *xSup, double *num, double *den) {
    *xSup = data->auxRightX;
    assignLineParameters(data,a,b,LINE_CTTE);
    areaCalculation(num,den,*a,*b,*xInf,*xSup);
    *xInf = *xSup;
}

void area3Calculation(Data *data, Data *proxData, double *a, double *b, double *xInf, double *xSup, double *num, double *den) {
    *xSup = data->xCenter + data->base/2;
    assignLineParameters(data,a,b,LINE_DESC);
    areaCalculation(num,den,*a,*b,*xInf,*xSup);
    if(proxData != NULL_POINTER) {
        *xInf = proxData->xCenter - proxData->base/2;
    }
}

double deffuzify(LinkedList *linkedList) {
    int itIdx = linkedList->inicial;
    int proxIdx;
    double numerator = 0;
    double denominator = 0;
    double a, b;
    double xInf, xSup;
    int typeIntersection, lastTypeIntersection;

    if(linkedList->size > 0 && linkedList->size <= MAX_SIZE) {
        xInf = linkedList->data[itIdx].xCenter - linkedList->data[itIdx].base/2;
        lastTypeIntersection = -1;
        while(linkedList->data[itIdx].nextValue != NULL_ELEMENT) { // tem area != 0
            proxIdx = linkedList->data[itIdx].nextValue;

            // Área reta ascendente:
            if(lastTypeIntersection != 3) {
                area1Calculation(&linkedList->data[itIdx],&a,&b,&xInf,&xSup,&numerator,&denominator);
            }

            // Verificar presença de intersecção:
            if(0 < verifyIntersectionOnInterval(&linkedList->data[itIdx], &linkedList->data[proxIdx], &xSup, TYPE_INTERVAL_1, &typeIntersection)) {
                // tem intersecção - tipo 1 - (AREA RETA CONSTANTE)
                assignLineParameters(&linkedList->data[itIdx],&a,&b,LINE_CTTE);
                areaCalculation(&numerator,&denominator,a,b,xInf,xSup);
                xInf = xSup;
            } else {
                // não tem - calcula área parcial - (ÁREA RETA CONSTANTE)
                area2Calculation(&linkedList->data[itIdx],&a,&b,&xInf,&xSup,&numerator,&denominator);
                if(0 < verifyIntersectionOnInterval(&linkedList->data[itIdx], &linkedList->data[proxIdx], &xSup, TYPE_INTERVAL_2, &typeIntersection) ||
                        0 < verifyIntersectionOnInterval(&linkedList->data[itIdx], &linkedList->data[proxIdx], &xSup, TYPE_INTERVAL_3, &typeIntersection)) {
                    // tem intersecção - tipo 2 OU 3
                    assignLineParameters(&linkedList->data[itIdx],&a,&b,LINE_DESC);
                    areaCalculation(&numerator,&denominator,a,b,xInf,xSup);
                    xInf = xSup;
                } else {
                    // não tem intersecção - (AREA RETA DESCENDENTE):
                    area3Calculation(&linkedList->data[itIdx], &linkedList->data[proxIdx],&a,&b,&xInf,&xSup,&numerator,&denominator);
                }
            }

            // proxima função:
            itIdx = proxIdx;
            lastTypeIntersection = typeIntersection;
        }
        // área do último item da lista:
        if(lastTypeIntersection != 3) {
            area1Calculation(&linkedList->data[itIdx],&a,&b,&xInf,&xSup,&numerator,&denominator);
        }
        area2Calculation(&linkedList->data[itIdx],&a,&b,&xInf,&xSup,&numerator,&denominator);
        area3Calculation(&linkedList->data[itIdx],NULL_POINTER,&a,&b,&xInf,&xSup,&numerator,&denominator);

        // Centroide:
        return numerator/denominator;
    }
    return 0;
}
