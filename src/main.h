#ifndef MAIN_H
#define MAIN_H

/**
 * Define se Uart0 será usada para debug
 * Se não definido, apenas bluetooth comunica.
 * */
//#define DEBUG_ACTIVE// Uart0 para debug.

/**
 * Se Definido, o motor é ativado com uma saída
 * triangular:
 *
 *.\      /\      /\
 *._\____/__\____/__\__
 *.  \  /    \  /    \
 *.   \/      \/      \
 * Este modo de operação foi usado para obter as
 * curvas dos motores.
 */
// #define TESTE_MOTOR_X_ENCODER // Para verificar resposta do encoder

/*
 * Deve ser definido junto com TESTE_MOTOR_X_ENCODER.
 * Se definido, saída dos motores será constante.
 *
 * Usado para determinar se o cálculo de rps está
 * correto.
 */
// #define TESTE_MOTOR_RPS

#include "StateMachineComm.h"

#define CALCULOS_ODOMETRIA 10
#define FREQUENCIA_TIMER 10 // 10 Hz
#define DT (1.0/FREQUENCIA_TIMER)
#define JUBILEU_R 0.034
#define JUBILEU_L 0.18

#define JUBILEU_MIN_RPM 60.0
#define JUBILEU_MAX_RPM 166.5
#define JUBILEU_MIN_VEL (JUBILEU_MIN_RPM*2.0*M_PI/60.0) // 6.2832 rad/s    ==  0.21363708953606095 m/s
#define JUBILEU_MAX_VEL (JUBILEU_MAX_RPM*2.0*M_PI/60.0) // 17.4358 rad/s   ==  0.593063493182867 m/s

typedef struct JDInputOutput {
    double ObjX, ObjY;
    double CoordX, CoordY, CoordTheta;
    double v, w;
    double vel_r, vel_l;
    double motorEsquerdo, motorDireito;
    volatile double DistanciaSensor[5]; // retorno calculado (cm)
    float sensorOffSet[2][5];
    t_sm_Comm *stateMachineComm; // para solicitar envio de dados durante execução de controlador (debug)
} JDInputOutput;

#endif
