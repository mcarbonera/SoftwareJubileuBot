#ifndef _SupervisorHibrido_h
#define _SupervisorHibrido_h

/*
 * Definições, protótipos e tipos:
 */
#include<stdint.h>
#include<math.h>
#include "main.h"
#include "StateMachineGeneric.h"
#include "StateMachineComm.h"

#define SENTIDO_COMPARACAO_ESQ (int) 1
#define SENTIDO_COMPARACAO_DIR (int) -1

#define D_STOP 0.15
#define D_AT_OBS 75.0
#define D_UNSAFE 25.0
#define D_FW 45.0
#define D_PROG_EPSILON 0.02
#define D_SENSOR_SAT 80.0
#define SLIDING_RIGHT 1
#define SLIDING_LEFT -1

#define GTG_Kp 4
#define GTG_Ki 0.01
#define GTG_Kd 0.01
#define AO_Kp 4
#define AO_Ki 0.01
#define AO_Kd 0.01
#define AO_AND_GTG_Kp 4
#define AO_AND_GTG_Ki 0.01
#define AO_AND_GTG_Kd 0.04
#define FW_Kp 2

#define SENSOR_GANHO_1 0.7
#define SENSOR_GANHO_2 2
#define SENSOR_GANHO_3 1.2
#define SENSOR_GANHO_4 2
#define SENSOR_GANHO_5 0.7

typedef enum{
    ST_STOP=0,
    ST_GTG,
    ST_AO,
    ST_AO_AND_GTG,
    ST_FOLLOW_WALL,
    ST_SIZE_CONTROLADORES
} t_controladores;

typedef struct _sm_sup_Hibrido{
    t_controladores     controladorAtual;
    t_action_Supervisor transicao[ST_SIZE_CONTROLADORES];
    t_action_Supervisor controlador[ST_SIZE_CONTROLADORES];

    JDInputOutput *jubileuInputOutput;

    /* Parametros importantes: */
    float ganhosSensores[5];
    double d_prog;
    short fw_direction;
    short isWallPresent;

    /* Parâmetros reutilizáveis */
    double distanciaObjetivo;
    double E_k;
    double e_k_1;
    double u_gtg[2];
    double u_ao[2];
    double u_ao_gtg[2];
    double *u_fw[2]; // Aponta para o vetor u_fw_l ou u_fw_r (ponteiro de ponteiro)
    double u_fw_l[2];
    double u_fw_r[2];
} t_sm_ControladorHibrido;

void Init_SM_Controlador_Hibrido(t_sm_ControladorHibrido *sm, JDInputOutput *data);
void Exec_SM_ControladorHibrido_Transicao(t_sm_ControladorHibrido *sm);
void Exec_SM_ControladorHibrido_Controlador(t_sm_ControladorHibrido *sm);
void ExecutarSupervisorHibrido(t_sm_ControladorHibrido *sm);
uint8_t checkEventAtGoal();
uint8_t checkEventProgressMade();
uint8_t checkEventSlidingLeft();
uint8_t checkEventSlidingRight();
uint8_t checkEventObstacleCleared();
uint8_t checkEventAtObstacle();
uint8_t checkEventUnsafe();

void resetController(t_sm_ControladorHibrido *sm, uint8_t resetDProg);
double norm(double diffX, double diffY);
void normalizaVetor(double vetorEntrada[2], double vetorNormalizado[2]);
void calculaParametrosReutilizaveis(t_sm_ControladorHibrido *sm);
void encontraMenoresDistanciasSeguirParede(volatile double valores[], int sentidoComparacao, int *idx_min, int *idx_min2);
void calculaVetorGoToGoal(t_sm_ControladorHibrido *sm);
void calculaVetorAvoidObstacles(t_sm_ControladorHibrido *sm);
void calculaVetorEvitarObstaculoParcial(t_sm_ControladorHibrido *sm);
void verificaSeguirParedeEsquerdaDireita(t_sm_ControladorHibrido *sm);
void calculaVetorFollowWall(t_sm_ControladorHibrido *sm, int P1, int P2, double resultado[2]);
uint8_t obstaculoEstaPresente(t_sm_ControladorHibrido *sm);
void unicicloParaAcionamentoDiferencialPriorizandoOmega(t_sm_ControladorHibrido *sm);
void uniToDiff(t_sm_ControladorHibrido *sm, double v, double w);
void diffToUni(t_sm_ControladorHibrido *sm, double vel_l, double vel_r);
void converterRadPorSecParaPorcPWM(t_sm_ControladorHibrido *sm);

#endif
