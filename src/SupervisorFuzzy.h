#ifndef _SupervisorFuzzy_h
#define _SupervisorFuzzy_h

/*
 * Definições, protótipos e tipos:
 */
#include<stdint.h>
#include<math.h>
#include "main.h"
#include "StateMachineGeneric.h"
#include "StateMachineComm.h"
#include "LinkedList.h"

#define D_STOP 0.15
#define D_PROG_EPSILON 0.02
#define ATIVACAO_PASSOS 40
#define ATIVACAO_MARGEM 5

#define DIST_P_BIT 0x01
#define DIST_M_BIT 0x02
#define DIST_G_BIT 0x04
#define DIST_SAT_BIT 0x08

#define DIST_P_X 0.0
#define DIST_P_B2 40.0
#define DIST_M_X 40.0
#define DIST_M_B2 30.0
#define DIST_G_X 60.0
#define DIST_G_B2 20.0
#define DIST_SAT_X 80.0
#define DIST_SAT_B2 10.0

#define VET_NG_X -1.0
#define VET_NM_X (-2.0/3.0)
#define VET_NP_X (-1.0/3.0)
#define VET_Z_X 0.0
#define VET_PP_X (1.0/3.0)
#define VET_PM_X (2.0/3.0)
#define VET_PG_X 1.0
#define VET_B (2.0/3.0)

#define VEL_P_X (1.0/3.0)
#define VEL_M_X (2.0/3.0)
#define VEL_G_X (1.0)
#define VEL_B (2.0/3.0)

#define SV_VET_NGSAT_BIT 0x0001
#define SV_VET_NG_BIT 0x0002
#define SV_VET_NM_BIT 0x0004
#define SV_VET_NP_BIT 0x0008
#define SV_VET_Z_BIT 0x0010
#define SV_VET_PP_BIT 0x0020
#define SV_VET_PM_BIT 0x0040
#define SV_VET_PG_BIT 0x0080
#define SV_VET_PGSAT_BIT 0x0100

#define SV_VEL_VP_BIT 0x01
#define SV_VEL_VM_BIT 0x02
#define SV_VEL_VG_BIT 0x04

#define SV_VET_NGSAT_X -1.0
#define SV_VET_NG_X -0.75
#define SV_VET_NM_X -0.5
#define SV_VET_NP_X -0.25
#define SV_VET_Z_X 0.0
#define SV_VET_PP_X 0.25
#define SV_VET_PM_X 0.5
#define SV_VET_PG_X 0.75
#define SV_VET_PGSAT_X 1.0
#define SV_VET_B2_0 0.5
#define SV_VET_B2_X 0.25
#define SV_VET_B 0.5

#define SV_VEL_VP_X 0.0 // originalmente 0
#define SV_VEL_VM_X 0.3 //               0.5
#define SV_VEL_VG_X 0.6 //               1.0
#define SV_VEL_B2 0.3   //               0.5

typedef enum{
    DIST_P=0,
    DIST_M,
    DIST_G,
    DIST_SAT,
    DIST_SIZE
} EO_FIS_inputs;

typedef enum{
    VP=0,
    VM,
    VG,
    V_SIZE
} SV_FIS_inputs_V;

typedef enum{
    NGSAT=0,
    NG,
    NM,
    NP,
    Z,
    PP,
    PM,
    PG,
    PGSAT,
    W_SIZE
} SV_FIS_inputs_W;

typedef enum{
    ST_STOP_FUZZY=0,
    ST_FUZZY,
    ST_SIZE_FUZZY_STATES
} t_fuzzyStates;

typedef struct _fuzzy_EO_type {
    double grauDeInferenciaEO[5][DIST_SIZE];
    uint8_t ativacaoSensor[5];

    double recSensorVet[5][2];
    double recVel;

    double recSensorVetInfSoma[5];
    double recVelInfSoma;

    LinkedList eoFisOut[11];
} EO_FIS;

typedef struct _fuzzy_SP_type {
    double (*grauDeInferenciaEO)[5][DIST_SIZE];
    uint8_t *ativacaoSensor;

    double recSPVet[2];
    double recDistVet[2];

    double recSPSoma[2];
    double recDistSoma[2];

    LinkedList spFisOut[4];
} SP_FIS;

typedef struct _fuzzy_SV_type {
    double grauDeInferenciaV[V_SIZE];
    double grauDeInferenciaW[2][W_SIZE];

    uint8_t ativacaoV;
    uint16_t ativacaoW[2];

    double recW[2];
    double recSomaW;

    LinkedList svFisOut[2];
} SV_FIS;

typedef struct _sm_contr_Fuzzy{
    t_fuzzyStates     controladorAtual;
    t_action_Supervisor transicao[ST_SIZE_FUZZY_STATES];
    t_action_Supervisor controlador[ST_SIZE_FUZZY_STATES];

    JDInputOutput *jubileuInputOutput;
    EO_FIS eoFis;
    SP_FIS spFis;
    SV_FIS svFis;

    /* Parametros importantes: */
    // Direção - Seguir Parede;
    int SP_Dir;
    // Ativação IPO/SP
    float ativacaoSP;
    // Para verificar se houve progresso:
    float d_prog;
    double IPO_Vet[2];
    double EO_Vet[2];
    double SP_Vet[2];
    double recFinal[2];

    /* Parâmetros reutilizáveis */
    double distanciaObjetivo;
} t_sm_ControladorFuzzy;

void Init_SM_Controlador_Fuzzy(t_sm_ControladorFuzzy *sm, JDInputOutput *data);
void Exec_SM_ControladorFuzzy_Transicao(t_sm_ControladorFuzzy *sm);
void Exec_SM_ControladorFuzzy_Controlador(t_sm_ControladorFuzzy *sm);
void ExecutarSupervisorFuzzy(t_sm_ControladorFuzzy *sm);

uint8_t checkEventAtGoalFuzzy(t_sm_ControladorFuzzy *sm);
uint8_t checkEventProgressMadeFuzzy(t_sm_ControladorFuzzy *sm);

void resetControllerFuzzy(t_sm_ControladorFuzzy *sm);
double normFuzzy(double diffX, double diffY);
void calculaParametrosReutilizaveisFuzzy(t_sm_ControladorFuzzy *sm);

void calculaVetorGoToGoalFuzzy(t_sm_ControladorFuzzy *sm);
void calculaVetorAOFuzzy(t_sm_ControladorFuzzy *sm);
void calculaSPFuzzy(t_sm_ControladorFuzzy *sm);

void definirSentidoDeContorno(t_sm_ControladorFuzzy *sm, double compensaDistanciaSP);
void verificarPerdaDeReferencia(t_sm_ControladorFuzzy *sm);
void normalizarEntradasVetor(double vetor[2]);
void verificaAtivacaoSP(t_sm_ControladorFuzzy *sm);
void calculaRecomendacaoFinal(t_sm_ControladorFuzzy *sm);
void converterRadPorSecParaPorcPWMFuzzy(t_sm_ControladorFuzzy *sm);
double calculaInferencia(double x, double b2);
double calculaAreaTrianguloCortado(double base, double altura);
void calculaFISEO(t_sm_ControladorFuzzy *sm);
void fuzzyficacaoEOSP(t_sm_ControladorFuzzy *sm);
void aplicaRegrasEDefuzzificacaoEO(t_sm_ControladorFuzzy *sm);
void calculaFISSP(t_sm_ControladorFuzzy *sm);
void fuzzificacaoSP(t_sm_ControladorFuzzy *sm);
void aplicaRegrasEDefuzzificacaoSP(t_sm_ControladorFuzzy *sm);
void calculaFISSeguirVetor(t_sm_ControladorFuzzy *sm);
void fuzzificacaoSeguirVetor(t_sm_ControladorFuzzy *sm);
void aplicaRegrasEDefuzzificacaoSeguirVetor(t_sm_ControladorFuzzy *sm);
void calculaRegraFuzzySV(int saidaWlWr[2], int valorV, int valorW0, int valorW1);
double calculaGrauPertinenciaSV(t_sm_ControladorFuzzy *sm, int valorV, int valorW0, int valorW1);
int evaluateMax(int x, int y);
int evaluateMin(int x, int y);
void calculaSaidas(t_sm_ControladorFuzzy *sm);

#endif
