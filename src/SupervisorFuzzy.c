#include "SupervisorFuzzy.h"
//#include "StateMachineGeneric.h"

// FUNÇÕES DE TRANSICAO
static void fn_STOP(t_sm_ControladorFuzzy *sm) {
    if(!checkEventAtGoalFuzzy(sm)) {
        resetControllerFuzzy(sm);
        sm->controladorAtual = ST_FUZZY;
    }
}

static void fn_FUZZY(t_sm_ControladorFuzzy *sm) {
    if(checkEventAtGoalFuzzy(sm)) {
            sm->controladorAtual = ST_STOP_FUZZY;
    }
}

// FUNÇÕES DOS CONTROLADORES:
static void fn_controladorStop(t_sm_ControladorFuzzy *sm) {
    sm->jubileuInputOutput->vel_l = 0;
    sm->jubileuInputOutput->vel_r = 0;
}

static void fn_controladorFuzzy(t_sm_ControladorFuzzy *sm) {
    // Vetor IPO:
    calculaVetorGoToGoalFuzzy(sm);

    // Verificar ativação SP:
    verificaAtivacaoSP(sm);

    // Vetor EO:
    calculaFISEO(sm);
    calculaVetorAOFuzzy(sm);

    // Vetor SP:
    calculaSPFuzzy(sm);

    // Vetor recomendação final (IPO + EO + SP)
    calculaRecomendacaoFinal(sm);

    // Seguir Vetor:
    calculaFISSeguirVetor(sm);

    //regulariza
    calculaSaidas(sm);
}

//____________________________________________
//FUNÇÃO INICIALIZAÇÃO
void Init_SM_Controlador_Fuzzy(t_sm_ControladorFuzzy *sm, JDInputOutput *data)
{
    // Parâmetros inicializáveis:
    sm->jubileuInputOutput = data;

    // Estado inicial:
    sm->controladorAtual = ST_STOP_FUZZY;

    // Funções de transição de estado:
    sm->transicao[ST_STOP_FUZZY] = (t_action_Supervisor)fn_STOP;
    sm->transicao[ST_FUZZY] = (t_action_Supervisor)fn_FUZZY;

    // Funções para executar controlador atual:
    sm->controlador[ST_STOP_FUZZY] = (t_action_Supervisor)fn_controladorStop;
    sm->controlador[ST_FUZZY] = (t_action_Supervisor)fn_controladorFuzzy;

    sm->SP_Dir = 0;
    sm->ativacaoSP = 0;
    sm->d_prog = 100;

    sm->spFis.grauDeInferenciaEO = &sm->eoFis.grauDeInferenciaEO;
    sm->spFis.ativacaoSensor = sm->eoFis.ativacaoSensor;
}

// FUNÇÃO PARA EXECUTAR TRANSIÇÃO
void Exec_SM_ControladorFuzzy_Transicao(t_sm_ControladorFuzzy *sm)
{
    t_fuzzyStates controladorAnterior = sm->controladorAtual;
    sm->transicao[sm->controladorAtual](sm);
    if(controladorAnterior != sm->controladorAtual) {
        Exec_SM_Comm(sm->jubileuInputOutput->stateMachineComm,EV_INFORMARTRANSICAOCONTROLADOR);
        if(sm->controladorAtual == ST_STOP_FUZZY) {
            Exec_SM_Comm(sm->jubileuInputOutput->stateMachineComm,EV_EXIT_LOGGING);
        }
    }
}

// FUNÇÃO PARA EXECUTAR CONTROLADOR
void Exec_SM_ControladorFuzzy_Controlador(t_sm_ControladorFuzzy *sm)
{
    sm->controlador[sm->controladorAtual](sm);
}

// FUNÇÃO EXECUTAR SUPERVISOR HÍBRIDO
void ExecutarSupervisorFuzzy(t_sm_ControladorFuzzy *sm)
{
    /* Calcula parâmetros reutilizáveis */
    calculaParametrosReutilizaveisFuzzy(sm);

    /* Verifica transição de estado */
    Exec_SM_ControladorFuzzy_Transicao(sm);

    /* Executa Controlador: */
    Exec_SM_ControladorFuzzy_Controlador(sm);

    /* Converter rad/s para porcentagem de pwm - regra de 3 */
    converterRadPorSecParaPorcPWMFuzzy(sm);
}

// Funções Auxiliares:
/* Eventos: */
uint8_t checkEventAtGoalFuzzy(t_sm_ControladorFuzzy *sm) {
    return (sm->distanciaObjetivo < D_STOP);
}

uint8_t checkEventProgressMadeFuzzy(t_sm_ControladorFuzzy *sm) {
    if(sm->distanciaObjetivo < (sm->d_prog - D_PROG_EPSILON)) {
        sm->d_prog = fmin(sm->distanciaObjetivo, sm->d_prog);
        return 1;
    } else if(fabs(sm->distanciaObjetivo - sm->d_prog) <= D_PROG_EPSILON) {
        return 1;
    }

    return 0;
}

/* Funções utilitárias */
void resetControllerFuzzy(t_sm_ControladorFuzzy *sm) {
    sm->d_prog = 1000;
    sm->SP_Dir = 0;
}

double normFuzzy(double diffX, double diffY) {
    return sqrt(diffX*diffX + diffY*diffY);
}

void calculaParametrosReutilizaveisFuzzy(t_sm_ControladorFuzzy *sm) {
    sm->distanciaObjetivo = normFuzzy(sm->jubileuInputOutput->CoordX - sm->jubileuInputOutput->ObjX,
                                 sm->jubileuInputOutput->CoordY - sm->jubileuInputOutput->ObjY);
}

void calculaVetorGoToGoalFuzzy(t_sm_ControladorFuzzy *sm) {
    double angulo;
    double norma;

    sm->IPO_Vet[0] = sm->jubileuInputOutput->ObjX - sm->jubileuInputOutput->CoordX;
    sm->IPO_Vet[1] = sm->jubileuInputOutput->ObjY - sm->jubileuInputOutput->CoordY;

    norma = normFuzzy(sm->IPO_Vet[0], sm->IPO_Vet[1]);

    // Erro na orientação.
    angulo = atan2(sm->IPO_Vet[1], sm->IPO_Vet[0]);
    angulo -= sm->jubileuInputOutput->CoordTheta;

    if(norma >= 1) {
        sm->IPO_Vet[0] = cos(angulo);
        sm->IPO_Vet[1] = sin(angulo);
    } else {
        sm->IPO_Vet[0] = norma*cos(angulo);
        sm->IPO_Vet[1] = norma*sin(angulo);
    }
}

void calculaVetorAOFuzzy(t_sm_ControladorFuzzy *sm) {
    float norma;
    sm->EO_Vet[0] = sm->eoFis.recSensorVet[1][0] + sm->eoFis.recSensorVet[3][0] + 2*sm->eoFis.recSensorVet[2][0];
    sm->EO_Vet[1] = sm->eoFis.recSensorVet[0][1] + sm->eoFis.recSensorVet[4][1] + sm->eoFis.recSensorVet[1][1] + sm->eoFis.recSensorVet[3][1];

    //normalizar:
    norma = normFuzzy(sm->EO_Vet[0], sm->EO_Vet[1]);
    if(norma > 1) {
        sm->EO_Vet[0] /= norma;
        sm->EO_Vet[1] /= norma;
    }

    // vetor velocidade:
    sm->jubileuInputOutput->v = sm->eoFis.recVel;
}

void calculaSPFuzzy(t_sm_ControladorFuzzy *sm) {
    if(sm->ativacaoSP > ATIVACAO_MARGEM) { // sp não inibido
        double compensaDistanciaSP;

        calculaFISSP(sm);

        compensaDistanciaSP = sm->spFis.recDistVet[0] + sm->spFis.recDistVet[1] * 2.5;
        sm->SP_Vet[0] = sm->spFis.recSPVet[0] * 3;
        sm->SP_Vet[1] = compensaDistanciaSP + sm->spFis.recSPVet[1] * 3;

        sm->SP_Vet[0] += sm->EO_Vet[0];
        sm->SP_Vet[1] += sm->EO_Vet[1];

        definirSentidoDeContorno(sm, compensaDistanciaSP);
        verificarPerdaDeReferencia(sm);
        normalizarEntradasVetor(sm->SP_Vet);
    } else { // sp totalmente inibido
        sm->SP_Vet[0] = 0;
        sm->SP_Vet[1] = 0;
    }
}

void definirSentidoDeContorno(t_sm_ControladorFuzzy *sm, double compensaDistanciaSP) {
    if(sm->SP_Dir == 0) { // não estava seguindo parede e passou a seguir
        if(compensaDistanciaSP > 0.0001) {
            sm->SP_Dir = 1;
        }
        else if(compensaDistanciaSP < -0.0001) {
            sm->SP_Dir = -1;
        }
    }
}

void verificarPerdaDeReferencia(t_sm_ControladorFuzzy *sm) {
    if(normFuzzy(sm->SP_Vet[0], sm->SP_Vet[1]) < 0.05) {
        if(sm->SP_Dir == 1) {
            sm->SP_Vet[1] = 0.3;
        } else if(sm->SP_Dir == -1) {
            sm->SP_Vet[1] = -0.3;
        }
    }
}

void normalizarEntradasVetor(double vetor[2]) {
    double vetorAbs[2];
    vetorAbs[0] = fabs(vetor[0]);
    vetorAbs[1] = fabs(vetor[1]);
    if(vetorAbs[0] > 1) {
        vetor[0] /= vetorAbs[0];
        vetor[1] /= vetorAbs[0];
    }
    if(vetorAbs[1] > 1) {
        vetor[0] /= vetorAbs[1];
        vetor[1] /= vetorAbs[1];
    }
}

void verificaAtivacaoSP(t_sm_ControladorFuzzy *sm) {
    if(checkEventProgressMadeFuzzy(sm)) {
        if(sm->ativacaoSP > 0) {
            sm->ativacaoSP--;
        }
        if(sm->ativacaoSP < ATIVACAO_MARGEM) {
            sm->SP_Dir = 0;
        }
    } else {
        if(sm->ativacaoSP < ATIVACAO_PASSOS) {
            sm->ativacaoSP++;
        }
    }
}

void calculaRecomendacaoFinal(t_sm_ControladorFuzzy *sm) {
    int i;
    double alpha = 1.6;
    double coeffAtivacao = fmin(fmax(sm->ativacaoSP, ATIVACAO_MARGEM), ATIVACAO_PASSOS-ATIVACAO_MARGEM);

    coeffAtivacao /= ATIVACAO_PASSOS - 2*ATIVACAO_MARGEM;

    for(i=0; i<2; i++) {
        sm->recFinal[i] = sm->IPO_Vet[i] + alpha * sm->EO_Vet[i];
        sm->recFinal[i] = (1-coeffAtivacao)*sm->recFinal[i] + coeffAtivacao * sm->SP_Vet[i];
    }

    normalizarEntradasVetor(sm->recFinal);
}

void converterRadPorSecParaPorcPWMFuzzy(t_sm_ControladorFuzzy *sm) {
    sm->jubileuInputOutput->motorEsquerdo = sm->jubileuInputOutput->vel_l/JUBILEU_MAX_VEL;
    sm->jubileuInputOutput->motorDireito = sm->jubileuInputOutput->vel_r/JUBILEU_MAX_VEL;
}

void calculaFISEO(t_sm_ControladorFuzzy *sm) {
    fuzzyficacaoEOSP(sm);
    aplicaRegrasEDefuzzificacaoEO(sm);
}

void calculaFISSP(t_sm_ControladorFuzzy *sm) {
    fuzzificacaoSP(sm);
    aplicaRegrasEDefuzzificacaoSP(sm);
}

void calculaFISSeguirVetor(t_sm_ControladorFuzzy *sm) {
    fuzzificacaoSeguirVetor(sm);
    aplicaRegrasEDefuzzificacaoSeguirVetor(sm);
}

// x entre [0, b2]
double calculaInferencia(double x, double b2) {
    return -x/b2 + 1;
}

double calculaAreaTrianguloCortado(double base, double altura) {
    return base*altura*(1 - altura/2);
}

double calculaFuzzyNot(double grau) {
    return 1 - grau;
}

void fuzzyficacaoEOSP(t_sm_ControladorFuzzy *sm) {
    int i;

    sm->eoFis.recVel = 0;
    sm->eoFis.recVelInfSoma = 0;
    initLinkedList(&sm->eoFis.eoFisOut[10]);
    for(i=0; i<5; i++) {
        initLinkedList(&sm->eoFis.eoFisOut[2*i]);
        initLinkedList(&sm->eoFis.eoFisOut[2*i + 1]);
        sm->eoFis.ativacaoSensor[i] = 0x00;
        sm->eoFis.recSensorVet[i][0] = 0;
        sm->eoFis.recSensorVet[i][1] = 0;
        sm->eoFis.recSensorVetInfSoma[i] = 0;

        if(sm->jubileuInputOutput->DistanciaSensor[i] > (DIST_P_X - DIST_P_B2) && sm->jubileuInputOutput->DistanciaSensor[i] < (DIST_P_X + DIST_P_B2)) {
            sm->eoFis.ativacaoSensor[i] |= DIST_P_BIT;

            sm->eoFis.grauDeInferenciaEO[i][DIST_P] = sm->jubileuInputOutput->DistanciaSensor[i] - DIST_P_X;
            if(sm->eoFis.grauDeInferenciaEO[i][DIST_P] < 0) {
                sm->eoFis.grauDeInferenciaEO[i][DIST_P] *= -1;
            }
            sm->eoFis.grauDeInferenciaEO[i][DIST_P] = calculaInferencia(sm->eoFis.grauDeInferenciaEO[i][DIST_P], DIST_P_B2);
        }
        if(sm->jubileuInputOutput->DistanciaSensor[i] > (DIST_M_X - DIST_M_B2) && sm->jubileuInputOutput->DistanciaSensor[i] < (DIST_M_X + DIST_M_B2)) {
            sm->eoFis.ativacaoSensor[i] |= DIST_M_BIT;

            sm->eoFis.grauDeInferenciaEO[i][DIST_M] = sm->jubileuInputOutput->DistanciaSensor[i] - DIST_M_X;
            if(sm->eoFis.grauDeInferenciaEO[i][DIST_M] < 0) {
                sm->eoFis.grauDeInferenciaEO[i][DIST_M] *= -1;
            }
            sm->eoFis.grauDeInferenciaEO[i][DIST_M] = calculaInferencia(sm->eoFis.grauDeInferenciaEO[i][DIST_M], DIST_M_B2);
        }
        if(sm->jubileuInputOutput->DistanciaSensor[i] > (DIST_G_X - DIST_G_B2) && sm->jubileuInputOutput->DistanciaSensor[i] < (DIST_G_X + DIST_G_B2)) {
            sm->eoFis.ativacaoSensor[i] |= DIST_G_BIT;

            sm->eoFis.grauDeInferenciaEO[i][DIST_G] = sm->jubileuInputOutput->DistanciaSensor[i] - DIST_G_X;
            if(sm->eoFis.grauDeInferenciaEO[i][DIST_G] < 0) {
                sm->eoFis.grauDeInferenciaEO[i][DIST_G] *= -1;
            }
            sm->eoFis.grauDeInferenciaEO[i][DIST_G] = calculaInferencia(sm->eoFis.grauDeInferenciaEO[i][DIST_G], DIST_G_B2);
        }
        if(sm->jubileuInputOutput->DistanciaSensor[i] > (DIST_SAT_X - DIST_SAT_B2) && sm->jubileuInputOutput->DistanciaSensor[i] < (DIST_SAT_X + DIST_SAT_B2)) {
            sm->eoFis.ativacaoSensor[i] |= DIST_SAT_BIT;

            sm->eoFis.grauDeInferenciaEO[i][DIST_SAT] = sm->jubileuInputOutput->DistanciaSensor[i] - DIST_SAT_X;
            if(sm->eoFis.grauDeInferenciaEO[i][DIST_SAT] < 0) {
                sm->eoFis.grauDeInferenciaEO[i][DIST_SAT] *= -1;
            }
            sm->eoFis.grauDeInferenciaEO[i][DIST_SAT] = calculaInferencia(sm->eoFis.grauDeInferenciaEO[i][DIST_SAT], DIST_SAT_B2);
        }
    }
}

void aplicaRegrasEDefuzzificacaoEO(t_sm_ControladorFuzzy *sm) {
    double grau;
    int i;

    if(sm->eoFis.ativacaoSensor[0] & DIST_P_BIT) {
        grau = sm->eoFis.grauDeInferenciaEO[0][DIST_P];
        addElementOrdered(&sm->eoFis.eoFisOut[0],VET_B,VET_Z_X, grau);
        addElementOrdered(&sm->eoFis.eoFisOut[1],VET_B,VET_NG_X, grau);
    }
    if(sm->eoFis.ativacaoSensor[0] & DIST_M_BIT) {
        grau = sm->eoFis.grauDeInferenciaEO[0][DIST_M];
        addElementOrdered(&sm->eoFis.eoFisOut[0],VET_B,VET_Z_X, grau);
        addElementOrdered(&sm->eoFis.eoFisOut[1],VET_B,VET_NM_X, grau);
    }
    if(sm->eoFis.ativacaoSensor[0] & DIST_G_BIT) {
        grau = sm->eoFis.grauDeInferenciaEO[0][DIST_G];
        addElementOrdered(&sm->eoFis.eoFisOut[0],VET_B,VET_Z_X, grau);
        addElementOrdered(&sm->eoFis.eoFisOut[1],VET_B,VET_NP_X, grau);
    }
    if(sm->eoFis.ativacaoSensor[0] & DIST_SAT_BIT) {
        grau = sm->eoFis.grauDeInferenciaEO[0][DIST_SAT];
        addElementOrdered(&sm->eoFis.eoFisOut[0],VET_B,VET_Z_X, grau);
        addElementOrdered(&sm->eoFis.eoFisOut[1],VET_B,VET_Z_X, grau);
        addElementOrdered(&sm->eoFis.eoFisOut[10],VET_B,VEL_G_X, grau);
    }
    if(sm->eoFis.ativacaoSensor[1] & DIST_P_BIT) {
        grau = sm->eoFis.grauDeInferenciaEO[1][DIST_P];
        addElementOrdered(&sm->eoFis.eoFisOut[2],VET_B,VET_Z_X, grau);
        addElementOrdered(&sm->eoFis.eoFisOut[3],VET_B,VET_NG_X, grau);
        addElementOrdered(&sm->eoFis.eoFisOut[10],VET_B,VEL_P_X, grau);
    }
    if(sm->eoFis.ativacaoSensor[1] & DIST_M_BIT) {
        grau = sm->eoFis.grauDeInferenciaEO[1][DIST_M];
        addElementOrdered(&sm->eoFis.eoFisOut[2],VET_B,VET_Z_X, grau);
        addElementOrdered(&sm->eoFis.eoFisOut[3],VET_B,VET_NM_X, grau);
        addElementOrdered(&sm->eoFis.eoFisOut[10],VET_B,VEL_M_X, grau);
    }
    if(sm->eoFis.ativacaoSensor[1] & DIST_G_BIT) {
        grau = sm->eoFis.grauDeInferenciaEO[1][DIST_G];
        addElementOrdered(&sm->eoFis.eoFisOut[2],VET_B,VET_Z_X, grau);
        addElementOrdered(&sm->eoFis.eoFisOut[3],VET_B,VET_NP_X, grau);
        addElementOrdered(&sm->eoFis.eoFisOut[10],VET_B,VEL_M_X, grau);
    }
    if(sm->eoFis.ativacaoSensor[1] & DIST_SAT_BIT) {
        grau = sm->eoFis.grauDeInferenciaEO[1][DIST_SAT];
        addElementOrdered(&sm->eoFis.eoFisOut[2],VET_B,VET_Z_X, grau);
        addElementOrdered(&sm->eoFis.eoFisOut[3],VET_B,VET_Z_X, grau);
        addElementOrdered(&sm->eoFis.eoFisOut[10],VET_B,VEL_G_X, grau);
    }
    if(sm->eoFis.ativacaoSensor[2] & DIST_P_BIT) {
        grau = sm->eoFis.grauDeInferenciaEO[2][DIST_P];
        addElementOrdered(&sm->eoFis.eoFisOut[4],VET_B,VET_NG_X, grau);
        addElementOrdered(&sm->eoFis.eoFisOut[5],VET_B,VET_Z_X, grau);
        addElementOrdered(&sm->eoFis.eoFisOut[10],VET_B,VEL_P_X, grau);
    }
    if(sm->eoFis.ativacaoSensor[2] & DIST_M_BIT) {
        grau = sm->eoFis.grauDeInferenciaEO[2][DIST_M];
        addElementOrdered(&sm->eoFis.eoFisOut[4],VET_B,VET_NM_X, grau);
        addElementOrdered(&sm->eoFis.eoFisOut[5],VET_B,VET_Z_X, grau);
        addElementOrdered(&sm->eoFis.eoFisOut[10],VET_B,VEL_M_X, grau);
    }
    if(sm->eoFis.ativacaoSensor[2] & DIST_G_BIT) {
        grau = sm->eoFis.grauDeInferenciaEO[2][DIST_G];
        addElementOrdered(&sm->eoFis.eoFisOut[4],VET_B,VET_NP_X, grau);
        addElementOrdered(&sm->eoFis.eoFisOut[5],VET_B,VET_Z_X, grau);
        addElementOrdered(&sm->eoFis.eoFisOut[10],VET_B,VEL_M_X, grau);
    }
    if(sm->eoFis.ativacaoSensor[2] & DIST_SAT_BIT) {
        grau = sm->eoFis.grauDeInferenciaEO[2][DIST_SAT];
        addElementOrdered(&sm->eoFis.eoFisOut[4],VET_B,VET_Z_X, grau);
        addElementOrdered(&sm->eoFis.eoFisOut[5],VET_B,VET_Z_X, grau);
        addElementOrdered(&sm->eoFis.eoFisOut[10],VET_B,VEL_G_X, grau);
    }
    if(sm->eoFis.ativacaoSensor[3] & DIST_P_BIT) {
        grau = sm->eoFis.grauDeInferenciaEO[3][DIST_P];
        addElementOrdered(&sm->eoFis.eoFisOut[6],VET_B,VET_Z_X, grau);
        addElementOrdered(&sm->eoFis.eoFisOut[7],VET_B,VET_PG_X, grau);
        addElementOrdered(&sm->eoFis.eoFisOut[10],VET_B,VEL_P_X, grau);
    }
    if(sm->eoFis.ativacaoSensor[3] & DIST_M_BIT) {
        grau = sm->eoFis.grauDeInferenciaEO[3][DIST_M];
        addElementOrdered(&sm->eoFis.eoFisOut[6],VET_B,VET_Z_X, grau);
        addElementOrdered(&sm->eoFis.eoFisOut[7],VET_B,VET_PM_X, grau);
        addElementOrdered(&sm->eoFis.eoFisOut[10],VET_B,VEL_M_X, grau);
    }
    if(sm->eoFis.ativacaoSensor[3] & DIST_G_BIT) {
        grau = sm->eoFis.grauDeInferenciaEO[3][DIST_G];
        addElementOrdered(&sm->eoFis.eoFisOut[6],VET_B,VET_Z_X, grau);
        addElementOrdered(&sm->eoFis.eoFisOut[7],VET_B,VET_PP_X, grau);
        addElementOrdered(&sm->eoFis.eoFisOut[10],VET_B,VEL_M_X, grau);
    }
    if(sm->eoFis.ativacaoSensor[3] & DIST_SAT_BIT) {
        grau = sm->eoFis.grauDeInferenciaEO[3][DIST_SAT];
        addElementOrdered(&sm->eoFis.eoFisOut[6],VET_B,VET_Z_X, grau);
        addElementOrdered(&sm->eoFis.eoFisOut[7],VET_B,VET_Z_X, grau);
        addElementOrdered(&sm->eoFis.eoFisOut[10],VET_B,VEL_G_X, grau);
    }
    if(sm->eoFis.ativacaoSensor[4] & DIST_P_BIT) {
        grau = sm->eoFis.grauDeInferenciaEO[4][DIST_P];
        addElementOrdered(&sm->eoFis.eoFisOut[8],VET_B,VET_Z_X, grau);
        addElementOrdered(&sm->eoFis.eoFisOut[9],VET_B,VET_PG_X, grau);
    }
    if(sm->eoFis.ativacaoSensor[4] & DIST_M_BIT) {
        grau = sm->eoFis.grauDeInferenciaEO[4][DIST_M];
        addElementOrdered(&sm->eoFis.eoFisOut[8],VET_B,VET_Z_X, grau);
        addElementOrdered(&sm->eoFis.eoFisOut[9],VET_B,VET_PM_X, grau);
    }
    if(sm->eoFis.ativacaoSensor[4] & DIST_G_BIT) {
        grau = sm->eoFis.grauDeInferenciaEO[4][DIST_G];
        addElementOrdered(&sm->eoFis.eoFisOut[8],VET_B,VET_Z_X, grau);
        addElementOrdered(&sm->eoFis.eoFisOut[9],VET_B,VET_PP_X, grau);
    }
    if(sm->eoFis.ativacaoSensor[4] & DIST_SAT_BIT) {
        grau = sm->eoFis.grauDeInferenciaEO[4][DIST_SAT];
        addElementOrdered(&sm->eoFis.eoFisOut[8],VET_B,VET_Z_X, grau);
        addElementOrdered(&sm->eoFis.eoFisOut[9],VET_B,VET_Z_X, grau);
        addElementOrdered(&sm->eoFis.eoFisOut[10],VET_B,VEL_G_X, grau);
    }

    sm->eoFis.recVel = deffuzify(&sm->eoFis.eoFisOut[10]);
    for(i=0; i<5; i++) {
        sm->eoFis.recSensorVet[i][0] = deffuzify(&sm->eoFis.eoFisOut[2*i]);
        sm->eoFis.recSensorVet[i][1] = deffuzify(&sm->eoFis.eoFisOut[2*i + 1]);
    }
}

void fuzzificacaoSP(t_sm_ControladorFuzzy *sm) {
    /* não estão calculados os inputs ~dist_sat */
    int i;

    for(i=0; i<5; i++) {
        if(sm->jubileuInputOutput->DistanciaSensor[i] > (DIST_SAT_X - DIST_SAT_B2) && sm->jubileuInputOutput->DistanciaSensor[i] < (DIST_SAT_X + DIST_SAT_B2)) {
            *sm->spFis.grauDeInferenciaEO[i][DIST_SAT] = sm->jubileuInputOutput->DistanciaSensor[i] - DIST_SAT_X;
            if(*sm->spFis.grauDeInferenciaEO[i][DIST_SAT] < 0) {
                *sm->spFis.grauDeInferenciaEO[i][DIST_SAT] *= -1;
            }
            *sm->spFis.grauDeInferenciaEO[i][DIST_SAT] = calculaInferencia(*sm->spFis.grauDeInferenciaEO[i][DIST_SAT], DIST_SAT_B2);
        } else {
            *sm->spFis.grauDeInferenciaEO[i][DIST_SAT] = 0;
        }
    }
}

void aplicaRegrasEDefuzzificacaoSP(t_sm_ControladorFuzzy *sm) {
    double grau;
    int i;

    for(i=0; i<2; i++) {
        sm->spFis.recDistVet[i] = 0;
        sm->spFis.recDistSoma[i] = 0;
        sm->spFis.recSPVet[i] = 0;
        sm->spFis.recSPSoma[i] = 0;

        initLinkedList(&sm->spFis.spFisOut[i]);
        initLinkedList(&sm->spFis.spFisOut[i+2]);
    }

    if((sm->spFis.ativacaoSensor[0] & DIST_SAT_BIT) && (sm->spFis.ativacaoSensor[4] & DIST_SAT_BIT) &&
            (sm->spFis.ativacaoSensor[1] & DIST_SAT_BIT) && (sm->spFis.ativacaoSensor[3] & DIST_SAT_BIT)) {
        grau = fmin((*sm->spFis.grauDeInferenciaEO)[0][DIST_SAT], (*sm->spFis.grauDeInferenciaEO)[1][DIST_SAT]);
        grau = fmin(grau, (*sm->spFis.grauDeInferenciaEO)[3][DIST_SAT]);
        grau = fmin(grau, (*sm->spFis.grauDeInferenciaEO)[4][DIST_SAT]);
        addElementOrdered(&sm->spFis.spFisOut[0],VET_B,VET_Z_X, grau);
        addElementOrdered(&sm->spFis.spFisOut[1],VET_B,VET_Z_X, grau);
    }
    if((sm->spFis.ativacaoSensor[0] & ~DIST_SAT_BIT) || (sm->spFis.ativacaoSensor[4] & ~DIST_SAT_BIT) ||
            (sm->spFis.ativacaoSensor[1] & ~DIST_SAT_BIT) || (sm->spFis.ativacaoSensor[3] & ~DIST_SAT_BIT)) {
        grau = fmax(calculaFuzzyNot((*sm->spFis.grauDeInferenciaEO)[0][DIST_SAT]), calculaFuzzyNot((*sm->spFis.grauDeInferenciaEO)[1][DIST_SAT]));
        grau = fmax(grau, calculaFuzzyNot((*sm->spFis.grauDeInferenciaEO)[3][DIST_SAT]));
        grau = fmax(grau, calculaFuzzyNot((*sm->spFis.grauDeInferenciaEO)[4][DIST_SAT]));
        addElementOrdered(&sm->spFis.spFisOut[0],VET_B,VET_PP_X, grau);
    }
    if((sm->spFis.ativacaoSensor[0] & ~DIST_SAT_BIT) && (sm->spFis.ativacaoSensor[1] & DIST_SAT_BIT)) {
        grau = fmin(calculaFuzzyNot((*sm->spFis.grauDeInferenciaEO)[0][DIST_SAT]), (*sm->spFis.grauDeInferenciaEO)[1][DIST_SAT]);
        addElementOrdered(&sm->spFis.spFisOut[1],VET_B,VET_PP_X, grau);
        addElementOrdered(&sm->spFis.spFisOut[2],VET_B,VET_Z_X, grau);
    }
    if((sm->spFis.ativacaoSensor[4] & ~DIST_SAT_BIT) && (sm->spFis.ativacaoSensor[3] & DIST_SAT_BIT)) {
        grau = fmin(calculaFuzzyNot((*sm->spFis.grauDeInferenciaEO)[4][DIST_SAT]), (*sm->spFis.grauDeInferenciaEO)[3][DIST_SAT]);
        addElementOrdered(&sm->spFis.spFisOut[1],VET_B,VET_NP_X, grau);
        addElementOrdered(&sm->spFis.spFisOut[2],VET_B,VET_Z_X, grau);
    }
    if((sm->spFis.ativacaoSensor[0] & ~DIST_SAT_BIT) && (sm->spFis.ativacaoSensor[2] & ~DIST_SAT_BIT)) {
        grau = fmin(calculaFuzzyNot((*sm->spFis.grauDeInferenciaEO)[0][DIST_SAT]), calculaFuzzyNot((*sm->spFis.grauDeInferenciaEO)[2][DIST_SAT]));
        addElementOrdered(&sm->spFis.spFisOut[1],VET_B,VET_NP_X, grau);
        addElementOrdered(&sm->spFis.spFisOut[2],VET_B,VET_Z_X, grau);
    }
    if((sm->spFis.ativacaoSensor[4] & ~DIST_SAT_BIT) && (sm->spFis.ativacaoSensor[2] & ~DIST_SAT_BIT)) {
        grau = fmin(calculaFuzzyNot((*sm->spFis.grauDeInferenciaEO)[4][DIST_SAT]), calculaFuzzyNot((*sm->spFis.grauDeInferenciaEO)[2][DIST_SAT]));
        addElementOrdered(&sm->spFis.spFisOut[1],VET_B,VET_PP_X, grau);
        addElementOrdered(&sm->spFis.spFisOut[2],VET_B,VET_Z_X, grau);
    }
    if(sm->spFis.ativacaoSensor[0] & DIST_P_BIT) {
        grau = (*sm->spFis.grauDeInferenciaEO)[0][DIST_P];
        addElementOrdered(&sm->spFis.spFisOut[2],VET_B,VET_Z_X, grau);
    }
    if(sm->spFis.ativacaoSensor[0] & DIST_M_BIT) {
        grau = (*sm->spFis.grauDeInferenciaEO)[0][DIST_M];
        addElementOrdered(&sm->spFis.spFisOut[2],VET_B,VET_PP_X, grau);
    }
    if(sm->spFis.ativacaoSensor[0] & DIST_G_BIT) {
        grau = (*sm->spFis.grauDeInferenciaEO)[0][DIST_G];
        addElementOrdered(&sm->spFis.spFisOut[2],VET_B,VET_PP_X, grau);
    }
    if(sm->spFis.ativacaoSensor[4] & DIST_P_BIT) {
        grau = (*sm->spFis.grauDeInferenciaEO)[4][DIST_P];
        addElementOrdered(&sm->spFis.spFisOut[2],VET_B,VET_Z_X, grau);
    }
    if(sm->spFis.ativacaoSensor[4] & DIST_M_BIT) {
        grau = (*sm->spFis.grauDeInferenciaEO)[4][DIST_M];
        addElementOrdered(&sm->spFis.spFisOut[2],VET_B,VET_NP_X, grau);
    }
    if(sm->spFis.ativacaoSensor[4] & DIST_G_BIT) {
        grau = (*sm->spFis.grauDeInferenciaEO)[4][DIST_G];
        addElementOrdered(&sm->spFis.spFisOut[2],VET_B,VET_NP_X, grau);
    }
    if(sm->spFis.ativacaoSensor[1] & DIST_P_BIT) {
        grau = (*sm->spFis.grauDeInferenciaEO)[1][DIST_P];
        addElementOrdered(&sm->spFis.spFisOut[3],VET_B,VET_Z_X, grau);
    }
    if(sm->spFis.ativacaoSensor[1] & DIST_M_BIT) {
        grau = (*sm->spFis.grauDeInferenciaEO)[1][DIST_M];
        addElementOrdered(&sm->spFis.spFisOut[3],VET_B,VET_PP_X, grau);
    }
    if(sm->spFis.ativacaoSensor[1] & DIST_G_BIT) {
        grau = (*sm->spFis.grauDeInferenciaEO)[1][DIST_G];
        addElementOrdered(&sm->spFis.spFisOut[3],VET_B,VET_PP_X, grau);
    }
    if(sm->spFis.ativacaoSensor[3] & DIST_P_BIT) {
        grau = (*sm->spFis.grauDeInferenciaEO)[3][DIST_P];
        addElementOrdered(&sm->spFis.spFisOut[3],VET_B,VET_Z_X, grau);
    }
    if(sm->spFis.ativacaoSensor[3] & DIST_M_BIT) {
        grau = (*sm->spFis.grauDeInferenciaEO)[3][DIST_M];
        addElementOrdered(&sm->spFis.spFisOut[3],VET_B,VET_NP_X, grau);
    }
    if(sm->spFis.ativacaoSensor[3] & DIST_G_BIT) {
        grau = (*sm->spFis.grauDeInferenciaEO)[3][DIST_G];
        addElementOrdered(&sm->spFis.spFisOut[3],VET_B,VET_NP_X, grau);
    }
    if((sm->spFis.ativacaoSensor[0] & ~DIST_SAT_BIT) && (sm->spFis.ativacaoSensor[1] & ~DIST_SAT_BIT) &&
            (sm->spFis.ativacaoSensor[2] & ~DIST_SAT_BIT)) {
        grau = fmin(calculaFuzzyNot((*sm->spFis.grauDeInferenciaEO)[0][DIST_SAT]), calculaFuzzyNot((*sm->spFis.grauDeInferenciaEO)[1][DIST_SAT]));
        grau = fmin(grau, calculaFuzzyNot((*sm->spFis.grauDeInferenciaEO)[2][DIST_SAT]));

        addElementOrdered(&sm->spFis.spFisOut[1],VET_B,VET_NP_X, grau);
        addElementOrdered(&sm->spFis.spFisOut[2],VET_B,VET_Z_X, grau);
    }
    if((sm->spFis.ativacaoSensor[4] & ~DIST_SAT_BIT) && (sm->spFis.ativacaoSensor[3] & ~DIST_SAT_BIT) &&
            (sm->spFis.ativacaoSensor[2] & ~DIST_SAT_BIT)) {
        grau = fmin(calculaFuzzyNot((*sm->spFis.grauDeInferenciaEO)[4][DIST_SAT]), calculaFuzzyNot((*sm->spFis.grauDeInferenciaEO)[3][DIST_SAT]));
        grau = fmin(grau, calculaFuzzyNot((*sm->spFis.grauDeInferenciaEO)[2][DIST_SAT]));

        addElementOrdered(&sm->spFis.spFisOut[1],VET_B,VET_PP_X, grau);
        addElementOrdered(&sm->spFis.spFisOut[2],VET_B,VET_Z_X, grau);
    }

    for(i=0; i<2; i++) {
        sm->spFis.recSPVet[i] = deffuzify(&sm->spFis.spFisOut[i]);
        sm->spFis.recDistVet[i] = deffuzify(&sm->spFis.spFisOut[i+2]);
    }
}

void fuzzificacaoSeguirVetor(t_sm_ControladorFuzzy *sm) {
    int i;

    sm->svFis.ativacaoV = 0x00;
    sm->svFis.recSomaW = 0;
    for(i=0; i<2; i++) {
        initLinkedList(&sm->svFis.svFisOut[i]);
        sm->svFis.ativacaoW[i] = 0x00;
        sm->svFis.recW[i] = 0;
    }

    /* v: */
    if(sm->jubileuInputOutput->v > (SV_VEL_VP_X - SV_VEL_B2) && sm->jubileuInputOutput->v < (SV_VEL_VP_X + SV_VEL_B2)) {
        sm->svFis.ativacaoV |= SV_VEL_VP_BIT;
        sm->svFis.grauDeInferenciaV[VP] = sm->jubileuInputOutput->v - SV_VEL_VP_X;
        if(sm->svFis.grauDeInferenciaV[VP] < 0) {
            sm->svFis.grauDeInferenciaV[VP] *= -1;
        }
        sm->svFis.grauDeInferenciaV[VP] = calculaInferencia(sm->svFis.grauDeInferenciaV[VP],SV_VEL_B2);
    }
    if(sm->jubileuInputOutput->v > (SV_VEL_VM_X - SV_VEL_B2) && sm->jubileuInputOutput->v < (SV_VEL_VM_X + SV_VEL_B2)) {
        sm->svFis.ativacaoV |= SV_VEL_VM_BIT;
        sm->svFis.grauDeInferenciaV[VM] = sm->jubileuInputOutput->v - SV_VEL_VM_X;
        if(sm->svFis.grauDeInferenciaV[VM] < 0) {
            sm->svFis.grauDeInferenciaV[VM] *= -1;
        }
        sm->svFis.grauDeInferenciaV[VM] = calculaInferencia(sm->svFis.grauDeInferenciaV[VM],SV_VEL_B2);
    }
    if(sm->jubileuInputOutput->v > (SV_VEL_VG_X - SV_VEL_B2) && sm->jubileuInputOutput->v < (SV_VEL_VG_X + SV_VEL_B2)) {
        sm->svFis.ativacaoV |= SV_VEL_VG_BIT;
        sm->svFis.grauDeInferenciaV[VG] = sm->jubileuInputOutput->v - SV_VEL_VG_X;
        if(sm->svFis.grauDeInferenciaV[VG] < 0) {
            sm->svFis.grauDeInferenciaV[VG] *= -1;
        }
        sm->svFis.grauDeInferenciaV[VG] = calculaInferencia(sm->svFis.grauDeInferenciaV[VG],SV_VEL_B2);
    }

    /* w: */
    for(i=0; i<2; i++) {
        if(sm->recFinal[i] > (SV_VET_NGSAT_X - SV_VET_B2_X) && sm->recFinal[i] < (SV_VET_NGSAT_X + SV_VET_B2_X)) {
            sm->svFis.ativacaoW[i] |= SV_VET_NGSAT_BIT;
            sm->svFis.grauDeInferenciaW[i][NGSAT] = sm->recFinal[i] - SV_VET_NGSAT_X;
            if(sm->svFis.grauDeInferenciaW[i][NGSAT] < 0) {
                sm->svFis.grauDeInferenciaW[i][NGSAT] *= -1;
            }
            sm->svFis.grauDeInferenciaW[i][NGSAT] = calculaInferencia(sm->svFis.grauDeInferenciaW[i][NGSAT],SV_VET_B2_X);
        }
        if(sm->recFinal[i] > (SV_VET_NG_X - SV_VET_B2_X) && sm->recFinal[i] < (SV_VET_NG_X + SV_VET_B2_X)) {
            sm->svFis.ativacaoW[i] |= SV_VET_NG_BIT;
            sm->svFis.grauDeInferenciaW[i][NG] = sm->recFinal[i] - SV_VET_NG_X;
            if(sm->svFis.grauDeInferenciaW[i][NG] < 0) {
                sm->svFis.grauDeInferenciaW[i][NG] *= -1;
            }
            sm->svFis.grauDeInferenciaW[i][NG] = calculaInferencia(sm->svFis.grauDeInferenciaW[i][NG],SV_VET_B2_X);
        }
        if(sm->recFinal[i] > (SV_VET_NM_X - SV_VET_B2_X) && sm->recFinal[i] < (SV_VET_NM_X + SV_VET_B2_X)) {
            sm->svFis.ativacaoW[i] |= SV_VET_NM_BIT;
            sm->svFis.grauDeInferenciaW[i][NM] = sm->recFinal[i] - SV_VET_NM_X;
            if(sm->svFis.grauDeInferenciaW[i][NM] < 0) {
                sm->svFis.grauDeInferenciaW[i][NM] *= -1;
            }
            sm->svFis.grauDeInferenciaW[i][NM] = calculaInferencia(sm->svFis.grauDeInferenciaW[i][NM],SV_VET_B2_X);
        }
        if(sm->recFinal[i] > (SV_VET_NP_X - SV_VET_B2_X) && sm->recFinal[i] < (SV_VET_NP_X + SV_VET_B2_X)) {
            sm->svFis.ativacaoW[i] |= SV_VET_NP_BIT;
            sm->svFis.grauDeInferenciaW[i][NP] = sm->recFinal[i] - SV_VET_NP_X;
            if(sm->svFis.grauDeInferenciaW[i][NP] < 0) {
                sm->svFis.grauDeInferenciaW[i][NP] *= -1;
            }
            sm->svFis.grauDeInferenciaW[i][NP] = calculaInferencia(sm->svFis.grauDeInferenciaW[i][NP],SV_VET_B2_X);
        }
        if(sm->recFinal[i] > (SV_VET_Z_X - SV_VET_B2_0) && sm->recFinal[i] < (SV_VET_Z_X + SV_VET_B2_0)) {
            sm->svFis.ativacaoW[i] |= SV_VET_Z_BIT;
            sm->svFis.grauDeInferenciaW[i][Z] = sm->recFinal[i] - SV_VET_Z_X;
            if(sm->svFis.grauDeInferenciaW[i][Z] < 0) {
                sm->svFis.grauDeInferenciaW[i][Z] *= -1;
            }
            sm->svFis.grauDeInferenciaW[i][Z] = calculaInferencia(sm->svFis.grauDeInferenciaW[i][Z],SV_VET_B2_0);
        }
        if(sm->recFinal[i] > (SV_VET_PP_X - SV_VET_B2_X) && sm->recFinal[i] < (SV_VET_PP_X + SV_VET_B2_X)) {
            sm->svFis.ativacaoW[i] |= SV_VET_PP_BIT;
            sm->svFis.grauDeInferenciaW[i][PP] = sm->recFinal[i] - SV_VET_PP_X;
            if(sm->svFis.grauDeInferenciaW[i][PP] < 0) {
                sm->svFis.grauDeInferenciaW[i][PP] *= -1;
            }
            sm->svFis.grauDeInferenciaW[i][PP] = calculaInferencia(sm->svFis.grauDeInferenciaW[i][PP],SV_VET_B2_X);
        }
        if(sm->recFinal[i] > (SV_VET_PM_X - SV_VET_B2_X) && sm->recFinal[i] < (SV_VET_PM_X + SV_VET_B2_X)) {
            sm->svFis.ativacaoW[i] |= SV_VET_PM_BIT;
            sm->svFis.grauDeInferenciaW[i][PP] = sm->recFinal[i] - SV_VET_PM_X;
            if(sm->svFis.grauDeInferenciaW[i][PP] < 0) {
                sm->svFis.grauDeInferenciaW[i][PP] *= -1;
            }
            sm->svFis.grauDeInferenciaW[i][PP] = calculaInferencia(sm->svFis.grauDeInferenciaW[i][PP],SV_VET_B2_X);
        }
        if(sm->recFinal[i] > (SV_VET_PG_X - SV_VET_B2_X) && sm->recFinal[i] < (SV_VET_PG_X + SV_VET_B2_X)) {
            sm->svFis.ativacaoW[i] |= SV_VET_PG_BIT;
            sm->svFis.grauDeInferenciaW[i][PG] = sm->recFinal[i] - SV_VET_PG_X;
            if(sm->svFis.grauDeInferenciaW[i][PG] < 0) {
                sm->svFis.grauDeInferenciaW[i][PG] *= -1;
            }
            sm->svFis.grauDeInferenciaW[i][PG] = calculaInferencia(sm->svFis.grauDeInferenciaW[i][PG],SV_VET_B2_X);
        }
        if(sm->recFinal[i] > (SV_VET_PGSAT_X - SV_VET_B2_X) && sm->recFinal[i] < (SV_VET_PGSAT_X + SV_VET_B2_X)) {
            sm->svFis.ativacaoW[i] |= SV_VET_PGSAT_BIT;
            sm->svFis.grauDeInferenciaW[i][PGSAT] = sm->recFinal[i] - SV_VET_PGSAT_X;
            if(sm->svFis.grauDeInferenciaW[i][PGSAT] < 0) {
                sm->svFis.grauDeInferenciaW[i][PGSAT] *= -1;
            }
            sm->svFis.grauDeInferenciaW[i][PGSAT] = calculaInferencia(sm->svFis.grauDeInferenciaW[i][PGSAT],SV_VET_B2_X);
        }
    }
}

void aplicaRegrasEDefuzzificacaoSeguirVetor(t_sm_ControladorFuzzy *sm) {
    int i,j,k,popIdx;
    int idxV, idxW[2];
    int ativoV[2];
    int ativoW[2][3];
    int saidaWlWr[2];
    double grau;

    idxV = 0;
    if(sm->svFis.ativacaoV & SV_VEL_VP_BIT) {
        ativoV[idxV++] = 1;
    }
    if(sm->svFis.ativacaoV & SV_VEL_VM_BIT) {
        ativoV[idxV++] = 2;
    }
    if(sm->svFis.ativacaoV & SV_VEL_VG_BIT) {
        ativoV[idxV++] = 3;
    }

    for(i=0; i<2; i++) {
        idxW[i] = 0;
        if(sm->svFis.ativacaoW[i] & SV_VET_NGSAT_BIT) {
            ativoW[i][idxW[i]++] = -4;
        }
        if(sm->svFis.ativacaoW[i] & SV_VET_NG_BIT) {
            ativoW[i][idxW[i]++] = -3;
        }
        if(sm->svFis.ativacaoW[i] & SV_VET_NM_BIT) {
            ativoW[i][idxW[i]++] = -2;
        }
        if(sm->svFis.ativacaoW[i] & SV_VET_NP_BIT) {
            ativoW[i][idxW[i]++] = -1;
        }
        if(sm->svFis.ativacaoW[i] & SV_VET_Z_BIT) {
            ativoW[i][idxW[i]++] = 0;
        }
        if(sm->svFis.ativacaoW[i] & SV_VET_PP_BIT) {
            ativoW[i][idxW[i]++] = 1;
        }
        if(sm->svFis.ativacaoW[i] & SV_VET_PM_BIT) {
            ativoW[i][idxW[i]++] = 2;
        }
        if(sm->svFis.ativacaoW[i] & SV_VET_PG_BIT) {
            ativoW[i][idxW[i]++] = 3;
        }
        if(sm->svFis.ativacaoW[i] & SV_VET_PGSAT_BIT) {
            ativoW[i][idxW[i]++] = 4;
        }
    }

    for(i=0; i<idxV; i++) {
        for(j=0; j<idxW[0]; j++) {
            for(k=0; k<idxW[1]; k++) {
                calculaRegraFuzzySV(saidaWlWr,ativoV[i],ativoW[0][j],ativoW[1][k]);
                grau = calculaGrauPertinenciaSV(sm,ativoV[i],ativoW[0][j],ativoW[1][k]);
                for(popIdx=0; popIdx<2; popIdx++) {
                    switch(saidaWlWr[popIdx]) {
                        case -4:
                            addElementOrdered(&sm->svFis.svFisOut[popIdx],SV_VET_B,SV_VET_NGSAT_X, grau);
                            break;
                        case -3:
                            addElementOrdered(&sm->svFis.svFisOut[popIdx],SV_VET_B,SV_VET_NG_X, grau);
                            break;
                        case -2:
                            addElementOrdered(&sm->svFis.svFisOut[popIdx],SV_VET_B,SV_VET_NM_X, grau);
                            break;
                        case -1:
                            addElementOrdered(&sm->svFis.svFisOut[popIdx],SV_VET_B,SV_VET_NP_X, grau);
                            break;
                        case 0:
                            addElementOrdered(&sm->svFis.svFisOut[popIdx],SV_VET_B,SV_VET_Z_X, grau);
                            break;
                        case 1:
                            addElementOrdered(&sm->svFis.svFisOut[popIdx],SV_VET_B,SV_VET_PP_X, grau);
                            break;
                        case 2:
                            addElementOrdered(&sm->svFis.svFisOut[popIdx],SV_VET_B,SV_VET_PM_X, grau);
                            break;
                        case 3:
                            addElementOrdered(&sm->svFis.svFisOut[popIdx],SV_VET_B,SV_VET_PG_X, grau);
                            break;
                        case 4:
                            addElementOrdered(&sm->svFis.svFisOut[popIdx],SV_VET_B,SV_VET_PGSAT_X, grau);
                            break;
                    }
                }
            }
        }
    }

    sm->svFis.recW[0] = deffuzify(&sm->svFis.svFisOut[0]);
    sm->svFis.recW[1] = deffuzify(&sm->svFis.svFisOut[1]);
}

void calculaRegraFuzzySV(int saidaWlWr[2], int valorV, int valorW0, int valorW1) {
    int ptoRec, ptoRecVel;
    int sinal = 1;
    int absoluto = valorW1;

    if(absoluto < 0) {
        sinal = -1;
        absoluto *= -1;
    }

    /* recomendação para vetor */
    if(absoluto == 0) {
        ptoRec = 0;
    } else {
        ptoRec = 4;
        if(absoluto == 3 && valorW0 >= -1) {
            ptoRec--;
        }
        else if(absoluto == 2) {
            if(valorW0 >= -2) {
                ptoRec--;
            }
            if(valorW0 >= -1) {
                ptoRec--;
            }
        }
        else if(absoluto == 1) {
            if(valorW0 >= -3) {
                ptoRec--;
            }
            if(valorW0 >= -2) {
                ptoRec--;
            }
            if(valorW0 >= -1) {
                ptoRec--;
            }
        }
        ptoRec *= sinal;
    }

    /* recomendação para velocidade */
    if(!(valorW0 == 0 && valorW1 == 0)) {
        ptoRecVel = valorV;
        if(valorV > 1 && (valorW0 >= -3 && valorW0 <= 3) && (valorW1 >= -3 && valorW1 <= 3)) {
            ptoRecVel--;
        }
        if(valorV > 2 && (valorW0 >= -1 && valorW0 <= 1) && (valorW1 >= -1 && valorW1 <= 1)) {
            ptoRecVel--;
        }
    } else {
        ptoRecVel = 0;
    }

    /* calcular rec Wl e Wr a partir de ptoRec */
    saidaWlWr[0] = ptoRecVel + ptoRec * -1;
    saidaWlWr[1] = ptoRecVel + ptoRec;
    saidaWlWr[0] = evaluateMin(evaluateMax(saidaWlWr[0], -4), 4);
    saidaWlWr[1] = evaluateMin(evaluateMax(saidaWlWr[1], -4), 4);
}

double calculaGrauPertinenciaSV(t_sm_ControladorFuzzy *sm, int valorV, int valorW0, int valorW1) {
    double grau;
    /* Calculo grau de pertinência */
    grau = fmin(sm->svFis.grauDeInferenciaV[valorV - 1], sm->svFis.grauDeInferenciaW[0][valorW0 + 4]);
    grau = fmin(grau, sm->svFis.grauDeInferenciaW[1][valorW1 + 4]);
    return grau;
}

int evaluateMax(int x, int y) {
    return x > y ? x : y;
}

int evaluateMin(int x, int y) {
    return x < y ? x : y;
}

void calculaSaidas(t_sm_ControladorFuzzy *sm) {
    int i;

    for(i=0; i<2; i++) {
        sm->svFis.recW[i] = JUBILEU_MIN_VEL * 0.7 + sm->svFis.recW[i] * (JUBILEU_MAX_VEL - JUBILEU_MIN_VEL * 0.7);
    }

    sm->jubileuInputOutput->vel_l = sm->svFis.recW[0];
    sm->jubileuInputOutput->vel_r = sm->svFis.recW[1];
}
