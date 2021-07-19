#include "SupervisorHibrido.h"
//#include "StateMachineGeneric.h"

// FUN��ES DE TRANSICAO
static void fn_STOP(t_sm_ControladorHibrido *sm) {
    if(!checkEventAtGoal(sm)) {
        resetController(sm,1);
        sm->controladorAtual = ST_AO_AND_GTG;
    }
}

static void fn_GTG(t_sm_ControladorHibrido *sm) {
    if(checkEventAtGoal(sm)) {
        sm->controladorAtual = ST_STOP;
    } else if(!checkEventProgressMade(sm)) {
        if(checkEventSlidingLeft(sm)) {
            sm->fw_direction = SLIDING_LEFT;
            resetController(sm,0);
            sm->controladorAtual = ST_FOLLOW_WALL;
        } else if(checkEventSlidingRight(sm)) {
            sm->fw_direction = SLIDING_RIGHT;
            resetController(sm,0);
            sm->controladorAtual = ST_FOLLOW_WALL;
        }
    } else if(checkEventAtObstacle(sm)) {
        resetController(sm,0);
        sm->controladorAtual = ST_AO_AND_GTG;
    } else if(checkEventUnsafe(sm)) {
        resetController(sm,0);
        sm->controladorAtual = ST_AO;
    }
}

static void fn_AO(t_sm_ControladorHibrido *sm) {
    if(checkEventAtGoal(sm)) {
        sm->controladorAtual = ST_STOP;
    } else if(!checkEventUnsafe(sm)) {
        resetController(sm,0);
        sm->controladorAtual = ST_AO_AND_GTG;
    }
}

static void fn_AO_AND_GTG(t_sm_ControladorHibrido *sm) {
    if(checkEventAtGoal(sm)) {
        sm->controladorAtual = ST_STOP;
    } else if(!checkEventProgressMade(sm)) {
        if(checkEventSlidingLeft(sm)) {
            sm->fw_direction = SLIDING_LEFT;
            resetController(sm,0);
            sm->controladorAtual = ST_FOLLOW_WALL;
        } else if(checkEventSlidingRight(sm)) {
            sm->fw_direction = SLIDING_RIGHT;
            resetController(sm,0);
            sm->controladorAtual = ST_FOLLOW_WALL;
        }
    } else if(checkEventObstacleCleared(sm)) {
        resetController(sm,0);
        sm->controladorAtual = ST_GTG;
    } else if(checkEventUnsafe(sm)) {
        resetController(sm,0);
        sm->controladorAtual = ST_AO;
    }
}

static void fn_FOLLOW_WALL(t_sm_ControladorHibrido *sm) {
    if(checkEventAtGoal(sm)) {
        sm->controladorAtual = ST_STOP;
    } else if(checkEventProgressMade(sm)) {
        if(sm->fw_direction == SLIDING_LEFT && !checkEventSlidingLeft(sm)) {
            resetController(sm,0);
            sm->controladorAtual = ST_AO_AND_GTG;
        } else if(sm->fw_direction == SLIDING_RIGHT && !checkEventSlidingRight(sm)) {
            resetController(sm,0);
            sm->controladorAtual = ST_AO_AND_GTG;
        }
    }
}

// FUN��ES DOS CONTROLADORES:
static void fn_controladorStop(t_sm_ControladorHibrido *sm) {
    sm->jubileuInputOutput->vel_l = 0;
    sm->jubileuInputOutput->vel_r = 0;
}

static void fn_controladorGTG(t_sm_ControladorHibrido *sm) {
    double e_P;

    sm->jubileuInputOutput->v = 1.0f;

    // Vetor GoToGoal:
    calculaVetorGoToGoal(sm);

    // �ngulo entre objetivo e coordenada atual
    e_P = atan2(sm->u_gtg[1], sm->u_gtg[0]);

    // Erro na orienta��o (termo Proporcional).
    e_P -= sm->jubileuInputOutput->CoordTheta;

    // Erro do termo Integrador:
    sm->E_k = sm->E_k + e_P*DT;

    // Erro do termo Derivativo:
    // (e_P - sm->e_k_1)/DT;

    // Controlador PID:
    sm->jubileuInputOutput->w = GTG_Kp*e_P + GTG_Ki*sm->E_k + GTG_Kd*(e_P - sm->e_k_1)/DT;

    // Guardar Erros:
    sm->e_k_1 = e_P;
}

static void fn_controladorAO(t_sm_ControladorHibrido *sm) {
    double e_P, tempX, tempY;

    sm->u_ao[0] = 0;
    sm->u_ao[1] = 0;

    // Verificar condi��o de colis�o frontal.
    if(sm->jubileuInputOutput->DistanciaSensor[2] < D_UNSAFE) {
        calculaVetorEvitarObstaculoParcial(sm);
        sm->jubileuInputOutput->v = 0;
        if((SENSOR_GANHO_1*(sm->jubileuInputOutput->DistanciaSensor[4]
                           -sm->jubileuInputOutput->DistanciaSensor[0])
           +SENSOR_GANHO_2*(sm->jubileuInputOutput->DistanciaSensor[3]
                           -sm->jubileuInputOutput->DistanciaSensor[1])) > 0) {
            // Rota��o 90 graus:
            //sm->u_ao[0] = -SENSOR_GANHO_3*sin(sm->jubileuInputOutput->CoordTheta)*100;
            //sm->u_ao[1] = SENSOR_GANHO_3*cos(sm->jubileuInputOutput->CoordTheta)*100;

            //tempX = sm->u_ao[1];
            //tempY = -sm->u_ao[0];

            tempX = cos(sm->jubileuInputOutput->CoordTheta)*sm->u_ao[1]
                   -sin(sm->jubileuInputOutput->CoordTheta)*(-sm->u_ao[0]);
            tempY = sin(sm->jubileuInputOutput->CoordTheta)*sm->u_ao[1]
                   +cos(sm->jubileuInputOutput->CoordTheta)*(-sm->u_ao[0]);
        } else {
            // Rota��o -90 graus:
            //sm->u_ao[0] = SENSOR_GANHO_3*sin(sm->jubileuInputOutput->CoordTheta)*100;
            //sm->u_ao[1] = -SENSOR_GANHO_3*cos(sm->jubileuInputOutput->CoordTheta)*100;

            //tempX = -sm->u_ao[1];
            //tempY = sm->u_ao[0];

            tempX = cos(sm->jubileuInputOutput->CoordTheta)*(-sm->u_ao[1])
                   -sin(sm->jubileuInputOutput->CoordTheta)*sm->u_ao[0];
            tempY = sin(sm->jubileuInputOutput->CoordTheta)*(-sm->u_ao[1])
                   +cos(sm->jubileuInputOutput->CoordTheta)*sm->u_ao[0];
        }
        /*
        sm->u_ao[0] = tempX;
        sm->u_ao[1] = tempY;

        // World Frame:
        tempX = cos(sm->jubileuInputOutput->CoordTheta)*sm->u_ao[0]
               -sin(sm->jubileuInputOutput->CoordTheta)*sm->u_ao[1];
        tempY = sin(sm->jubileuInputOutput->CoordTheta)*sm->u_ao[0]
               +cos(sm->jubileuInputOutput->CoordTheta)*sm->u_ao[1];
        */

        sm->u_ao[0] = tempX;
        sm->u_ao[1] = tempY;
    } else {
        calculaVetorAvoidObstacles(sm);
    }

    // PID:
    // �ngulo vetor u_ao
    e_P = atan2(sm->u_ao[1], sm->u_ao[0]);

    // Erro na orienta��o (termo Proporcional).
    e_P -= sm->jubileuInputOutput->CoordTheta;
    e_P = atan2(sin(e_P),cos(e_P));

    // Erro do termo Integrador:
    sm->E_k = sm->E_k + e_P*DT;

    // Erro do termo Derivativo:
    // (e_P - sm->e_k_1)/DT;

    // Controlador PID:
    sm->jubileuInputOutput->w = AO_Kp*e_P + AO_Ki*sm->E_k + AO_Kd*(e_P - sm->e_k_1)/DT;

    // Guardar Erros:
    sm->e_k_1 = e_P;
}

static void fn_controladorAO_AND_GTG(t_sm_ControladorHibrido *sm) {
    double e_P, u_gtg_n[2], u_ao_n[2];

    // Diminuir Velocidade
    sm->jubileuInputOutput->v = 0.3;

    // Vetor AvoidObstacles:
    calculaVetorAvoidObstacles(sm);

    // Vetor GoToGoal:
    calculaVetorGoToGoal(sm);

    // Normaliza vetores:
    normalizaVetor(sm->u_gtg, u_gtg_n);
    normalizaVetor(sm->u_ao, u_ao_n);

    // Calcula u_ao_gtg por meio de uma combina��o linear das duas recomenda��es:
    sm->u_ao_gtg[0] = 0.3*u_gtg_n[0] + 0.7*u_ao_n[0];
    sm->u_ao_gtg[1] = 0.3*u_gtg_n[1] + 0.7*u_ao_n[1];

    // PID:
    // �ngulo vetor u_ao_gtg
    e_P = atan2(sm->u_ao_gtg[1], sm->u_ao_gtg[0]);

    // Erro na orienta��o (termo Proporcional).
    e_P -= sm->jubileuInputOutput->CoordTheta;
    e_P = atan2(sin(e_P),cos(e_P));

    // Erro do termo Integrador:
    sm->E_k = sm->E_k + e_P*DT;

    // Erro do termo Derivativo:
    // (e_P - sm->e_k_1)/DT;

    // Controlador PID:
    sm->jubileuInputOutput->w = AO_AND_GTG_Kp*e_P + AO_AND_GTG_Ki*sm->E_k + AO_AND_GTG_Kd*(e_P - sm->e_k_1)/DT;

    // Guardar Erros:
    sm->e_k_1 = e_P;
}

static void fn_controladorFOLLOW_WALL(t_sm_ControladorHibrido *sm) {
    double e_P;

    // Testar direcao:
    sm->fw_direction = SLIDING_LEFT;
    sm->jubileuInputOutput->v = 0.2;

    /* Saturar medidas acima de 60 cm para desconsiderar distancias "grandes". */
    int i;
    for(i=0; i<5; i++) {
        if(sm->jubileuInputOutput->DistanciaSensor[i] > 60.0 &&
                sm->jubileuInputOutput->DistanciaSensor[i] <= 80.0) {
            sm->jubileuInputOutput->DistanciaSensor[i] = 60.0;
        }
    }

    verificaSeguirParedeEsquerdaDireita(sm);

    /* Leds para mostrar sentido de contorno */
    #include "driverlib/gpio.h"
    if(sm->fw_direction == SLIDING_LEFT) {
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0|GPIO_PIN_1, 0xFF); // LEDs on
    } else {
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0|GPIO_PIN_1, 0); // LEDs off
    }

    // Controlador proporcional (apenas P):
    // �ngulo vetor u_fw;
    e_P = atan2((*(sm->u_fw))[1], (*(sm->u_fw))[0]);

    // Erro na orienta��o (termo Proporcional).
    e_P -= sm->jubileuInputOutput->CoordTheta;
    e_P = atan2(sin(e_P),cos(e_P));

    // Conrolador PID:
    sm->jubileuInputOutput->w = FW_Kp*e_P;

    // Guardar Erros:
    sm->e_k_1 = e_P;
}

//____________________________________________
//FUN��O INICIALIZA��O
void Init_SM_Controlador_Hibrido(t_sm_ControladorHibrido *sm, JDInputOutput *data)
{
    // Par�metros inicializ�veis:
    sm->ganhosSensores[0] = SENSOR_GANHO_1;
    sm->ganhosSensores[1] = SENSOR_GANHO_2;
    sm->ganhosSensores[2] = SENSOR_GANHO_3;
    sm->ganhosSensores[3] = SENSOR_GANHO_4;
    sm->ganhosSensores[4] = SENSOR_GANHO_5;
    sm->jubileuInputOutput = data;
    sm->d_prog = 1000;
    sm->E_k = 0;
    sm->e_k_1 = 0;
    sm->fw_direction = SLIDING_LEFT;

    // Estado inicial:
    sm->controladorAtual = ST_STOP;

    // Fun��es de transi��o de estado:
    sm->transicao[ST_STOP] = (t_action_Supervisor)fn_STOP;
    sm->transicao[ST_GTG] = (t_action_Supervisor)fn_GTG;
    sm->transicao[ST_AO] = (t_action_Supervisor)fn_AO;
    sm->transicao[ST_AO_AND_GTG] = (t_action_Supervisor)fn_AO_AND_GTG;
    sm->transicao[ST_FOLLOW_WALL] = (t_action_Supervisor)fn_FOLLOW_WALL;

    // Fun��es para executar controlador atual:
    sm->controlador[ST_STOP] = (t_action_Supervisor)fn_controladorStop;
    sm->controlador[ST_GTG] = (t_action_Supervisor)fn_controladorGTG;
    sm->controlador[ST_AO] = (t_action_Supervisor)fn_controladorAO;
    sm->controlador[ST_AO_AND_GTG] = (t_action_Supervisor)fn_controladorAO_AND_GTG;
    sm->controlador[ST_FOLLOW_WALL] = (t_action_Supervisor)fn_controladorFOLLOW_WALL;

    int i;
    for(i=1; i < 5; i++) {
        //sm->controlador[i] = (t_action_Supervisor)fn_controladorGTG;
        //sm->controlador[i] = (t_action_Supervisor)fn_controladorAO;
        sm->controlador[i] = (t_action_Supervisor)fn_controladorAO_AND_GTG;
        //sm->controlador[i] = (t_action_Supervisor)fn_controladorFOLLOW_WALL;
        //sm->fw_direction = SLIDING_RIGHT;
    }
}

// FUN��O PARA EXECUTAR TRANSI��O
void Exec_SM_ControladorHibrido_Transicao(t_sm_ControladorHibrido *sm)
{
    t_controladores controladorAnterior = sm->controladorAtual;
    sm->transicao[sm->controladorAtual](sm);
    if(controladorAnterior != sm->controladorAtual) {
        Exec_SM_Comm(sm->jubileuInputOutput->stateMachineComm,EV_INFORMARTRANSICAOCONTROLADOR);
        if(sm->controladorAtual == ST_STOP) {
            Exec_SM_Comm(sm->jubileuInputOutput->stateMachineComm,EV_EXIT_LOGGING);
        }
    }
}

// FUN��O PARA EXECUTAR CONTROLADOR
void Exec_SM_ControladorHibrido_Controlador(t_sm_ControladorHibrido *sm)
{
    sm->controlador[sm->controladorAtual](sm);
}

// FUN��O EXECUTAR SUPERVISOR H�BRIDO
void ExecutarSupervisorHibrido(t_sm_ControladorHibrido *sm)
{
    /* Calcula par�metros reutiliz�veis */
    calculaParametrosReutilizaveis(sm);

    /* Verifica transi��o de estado */
    Exec_SM_ControladorHibrido_Transicao(sm);

    /* Executa Controlador: */
    Exec_SM_ControladorHibrido_Controlador(sm);

    if(sm->controladorAtual != ST_STOP) {
        /*
         * Converte Uniciclo Para Acionamento Diferencial
         * Garantindo que satura��o inferior e superior
         * dos motores seja respeitada.
         * ** priorizando w **
         **/
        unicicloParaAcionamentoDiferencialPriorizandoOmega(sm);
    }

    /* Converter rad/s para porcentagem de pwm - regra de 3 */
    converterRadPorSecParaPorcPWM(sm);
}

// Fun��es Auxiliares:
/* Eventos: */
uint8_t checkEventAtGoal(t_sm_ControladorHibrido *sm) {
    return (sm->distanciaObjetivo < D_STOP);
}

uint8_t checkEventProgressMade(t_sm_ControladorHibrido *sm) {
    if(sm->distanciaObjetivo < (sm->d_prog - D_PROG_EPSILON)) {
        sm->d_prog = fmin(sm->distanciaObjetivo, sm->d_prog);
        return 1;
    } else if(fabs(sm->distanciaObjetivo - sm->d_prog) <= D_PROG_EPSILON) {
        return 1;
    }

    return 0;
}

/*
void teste(t_sm_ControladorHibrido *sm) {
    sm->jubileuInputOutput->DistanciaSensor[0] = 80.0;
    sm->jubileuInputOutput->DistanciaSensor[1] = 79.0;
    sm->jubileuInputOutput->DistanciaSensor[2] = 80.0;
    sm->jubileuInputOutput->DistanciaSensor[3] = 80.0;
    sm->jubileuInputOutput->DistanciaSensor[4] = 68.0;
}
*/

uint8_t checkEventSlidingLeft(t_sm_ControladorHibrido *sm) {
    double sigma[2], detA;

    //teste(sm);

    // Vetor GoToGoal
    calculaVetorGoToGoal(sm);
    // Vetor AvoidObstacle
    calculaVetorAvoidObstacles(sm);
    sm->u_ao[0] /= 100;
    sm->u_ao[1] /= 100;

    //Obst�culo presente? Calcula Vetor FollowWall
    if(sm->fw_direction == SLIDING_LEFT) {
        if(!obstaculoEstaPresente(sm)) {
            return 0;
        }
        verificaSeguirParedeEsquerdaDireita(sm);
    } else {
        sm->fw_direction *= -1;
        if(!obstaculoEstaPresente(sm)) {
            sm->fw_direction *= -1;
            return 0;
        }
        verificaSeguirParedeEsquerdaDireita(sm);
        sm->fw_direction *= -1;
    }

    /*
     * A*sigma = B
     * sigma = (A^-1)*B
     * com A = [u_gtg u_ao]
     * sigma = (A^-1)*u_fw
     */
    detA = sm->u_gtg[0]*sm->u_ao[1] - sm->u_gtg[1]*sm->u_ao[0];
    sigma[0] = sm->u_ao[1]*(*sm->u_fw)[0] -sm->u_ao[0]*(*sm->u_fw)[1];
    sigma[1] = -sm->u_gtg[1]*(*sm->u_fw)[0] +sm->u_gtg[0]*(*sm->u_fw)[1];
    sigma[0] /= detA;
    sigma[1] /= detA;

    if(sigma[0] > 0 && sigma[1] > 0) {
        return 1;
    }
    return 0;
}

uint8_t checkEventSlidingRight(t_sm_ControladorHibrido *sm) {
    double sigma[2], detA;

    //teste(sm);

    // Vetor GoToGoal
    calculaVetorGoToGoal(sm);
    // Vetor AvoidObstacle
    calculaVetorAvoidObstacles(sm);
    sm->u_ao[0] /= 100;
    sm->u_ao[1] /= 100;

    //Obst�culo presente? Calcula Vetor FollowWall
    if(sm->fw_direction == SLIDING_RIGHT) {
        if(!obstaculoEstaPresente(sm)) {
            return 0;
        }
        verificaSeguirParedeEsquerdaDireita(sm);
    } else {
        sm->fw_direction *= -1;
        if(!obstaculoEstaPresente(sm)) {
            sm->fw_direction *= -1;
            return 0;
        }
        verificaSeguirParedeEsquerdaDireita(sm);
        sm->fw_direction *= -1;
    }

    /*
     * A*sigma = B
     * sigma = (A^-1)*B
     * com A = [u_gtg u_ao]
     * sigma = (A^-1)*u_fw
     */
    detA = sm->u_gtg[0]*sm->u_ao[1] - sm->u_gtg[1]*sm->u_ao[0];
    sigma[0] = sm->u_ao[1]*(*sm->u_fw)[0] -sm->u_ao[0]*(*sm->u_fw)[1];
    sigma[1] = -sm->u_gtg[1]*(*sm->u_fw)[0] +sm->u_gtg[0]*(*sm->u_fw)[1];
    sigma[0] /= detA;
    sigma[1] /= detA;

    if(sigma[0] > 0 && sigma[1] > 0) {
        return 1;
    }
    return 0;
}

// Se todos s�o maiores que D_AT_OBS
uint8_t checkEventObstacleCleared(t_sm_ControladorHibrido *sm) {
    int i;
    for(i=0; i<5; i++) {
        if(sm->jubileuInputOutput->DistanciaSensor[i] <= D_AT_OBS) {
            return 0;
        }
    }
    return 1;
}

// Se algum � menor que D_AT_OBS
uint8_t checkEventAtObstacle(t_sm_ControladorHibrido *sm) {
    int i;
    for(i=0; i<5; i++) {
        if(sm->jubileuInputOutput->DistanciaSensor[i] < D_AT_OBS) {
            return 1;
        }
    }
    return 0;
}

uint8_t checkEventUnsafe(t_sm_ControladorHibrido *sm) {
    int i;
    for(i=0; i<5; i++) {
        if(sm->jubileuInputOutput->DistanciaSensor[i] < D_UNSAFE) {
            return 1;
        }
    }
    return 0;
}

/* Fun��es utilit�rias */
void resetController(t_sm_ControladorHibrido *sm, uint8_t resetDProg) {
    // Resetar erro acumulado (integrador) e erro anterior (diferencial)
    sm->E_k = 0;
    sm->e_k_1 = 0;
    // d_prog - s� faz sentido mudar ao resetar o objetivo.
    if(resetDProg) {
        sm->d_prog = 1000;
    }
}

double norm(double diffX, double diffY) {
    return sqrt(diffX*diffX + diffY*diffY);
}

void normalizaVetor(double vetorEntrada[2], double vetorNormalizado[2]) {
    double norma = norm(vetorEntrada[0],vetorEntrada[1]);
    vetorNormalizado[0] = vetorEntrada[0]/norma;
    vetorNormalizado[1] = vetorEntrada[1]/norma;
}

void calculaParametrosReutilizaveis(t_sm_ControladorHibrido *sm) {
    sm->distanciaObjetivo = norm(sm->jubileuInputOutput->CoordX - sm->jubileuInputOutput->ObjX,
                                 sm->jubileuInputOutput->CoordY - sm->jubileuInputOutput->ObjY);
}

/*
 * Encontra menores distancias. Percorrendo do sensor lateral para o sensor frontal.
 * valores eh um vetor, mas o ponteiro passado eh sempre o sensor lateral (direito ou esquerdo)
 * sentidoDeCompraracao eh -1 ou 1, de modo que, se o ponteiro valores apontar para o sensor lateral esquerdo,
 * os �ndices avaliados ser�o 0,1,2.
 * Se o ponteiro valores apontar para o sensor lateral direito, os �ndices avaliados ser�o 4,3,2.
 * */
void encontraMenoresDistanciasSeguirParede(volatile double valores[], int sentidoComparacao, int *idx_min, int *idx_min2) {
    // Primeiro valor:
    *idx_min=0;
    // Segundo valor:
    if(valores[sentidoComparacao] < valores[*idx_min]) {
        *idx_min2 = *idx_min;
        *idx_min = sentidoComparacao*1;
    } else {
        *idx_min2 = sentidoComparacao*1;
    }
    // Terceiro valor:
    if(valores[sentidoComparacao*2] < valores[*idx_min]) {
        *idx_min2 = *idx_min;
        *idx_min = sentidoComparacao*2;
    } else if(valores[sentidoComparacao*2] < valores[*idx_min2]) {
        *idx_min2 = sentidoComparacao*2;
    }
}

void calculaVetorGoToGoal(t_sm_ControladorHibrido *sm) {
    sm->u_gtg[0] = sm->jubileuInputOutput->ObjX - sm->jubileuInputOutput->CoordX;
    sm->u_gtg[1] = sm->jubileuInputOutput->ObjY - sm->jubileuInputOutput->CoordY;
}

void calculaVetorAvoidObstacles(t_sm_ControladorHibrido *sm) {
    double tempX, tempY;

    calculaVetorEvitarObstaculoParcial(sm);

    // Equilibrio eixo X:
    sm->u_ao[0] -= SENSOR_GANHO_3*200; // 200 ~= 2*70*sqrt(2)

    // world frame:
    tempX = cos(sm->jubileuInputOutput->CoordTheta)*sm->u_ao[0]
           -sin(sm->jubileuInputOutput->CoordTheta)*sm->u_ao[1];
    tempY = sin(sm->jubileuInputOutput->CoordTheta)*sm->u_ao[0]
           +cos(sm->jubileuInputOutput->CoordTheta)*sm->u_ao[1];

    sm->u_ao[0] = tempX;
    sm->u_ao[1] = tempY;
}

void calculaVetorEvitarObstaculoParcial(t_sm_ControladorHibrido *sm) {
    int i;
    float anguloSensor;
    double ir_distances_rf[2][5];

    sm->u_ao[0] = 0;
    sm->u_ao[1] = 0;
    for(i=0, anguloSensor = M_PI_2; i<5; i++, anguloSensor-=M_PI_4) {
        // robot frame:
        ir_distances_rf[0][i] = cos(anguloSensor)*sm->jubileuInputOutput->DistanciaSensor[i]
                                +sm->jubileuInputOutput->sensorOffSet[0][i];
        ir_distances_rf[1][i] = sin(anguloSensor)*sm->jubileuInputOutput->DistanciaSensor[i]
                                +sm->jubileuInputOutput->sensorOffSet[1][i];

        // Resultante:
        sm->u_ao[0] += sm->ganhosSensores[i] * ir_distances_rf[0][i];
        sm->u_ao[1] += sm->ganhosSensores[i] * ir_distances_rf[1][i];
    }
}

void verificaSeguirParedeEsquerdaDireita(t_sm_ControladorHibrido *sm) {
    int idx_min, idx_min2;

    /*
    // Saturar sensores acima de 60 cm para desconsiderar distancias "grandes".
    volatile double temp[5];
    int i;
    for(i=0; i<5; i++) {
        temp[i] = sm->jubileuInputOutput->DistanciaSensor[i];
        if(sm->jubileuInputOutput->DistanciaSensor[i] > 60.0 &&
                sm->jubileuInputOutput->DistanciaSensor[i] <= 80.0) {
            sm->jubileuInputOutput->DistanciaSensor[i] = 60.0;
        }
    }
    */

    // Pegar dois �ndices referentes �s menores dist�ncias:
    if(sm->fw_direction == SLIDING_RIGHT) {
        encontraMenoresDistanciasSeguirParede(&sm->jubileuInputOutput->DistanciaSensor[4],SENTIDO_COMPARACAO_DIR,&idx_min,&idx_min2);
        idx_min+=4;
        idx_min2+=4;
        if(idx_min < idx_min2) {
            //P2 - idx_min
            //P1 - idx_min2
            calculaVetorFollowWall(sm, idx_min2, idx_min, sm->u_fw_r);
        } else {
            //P2 - idx_min2
            //P1 - idx_min
            calculaVetorFollowWall(sm, idx_min, idx_min2, sm->u_fw_r);
        }
        *sm->u_fw = sm->u_fw_r;
    } else {
        encontraMenoresDistanciasSeguirParede(sm->jubileuInputOutput->DistanciaSensor,SENTIDO_COMPARACAO_ESQ,&idx_min,&idx_min2);
        if(idx_min > idx_min2) {
            //P1 - idx_min2
            //P2 - idx_min
            calculaVetorFollowWall(sm, idx_min2, idx_min, sm->u_fw_l);
        } else {
            //P1 - idx_min
            //P2 - idx_min2
            calculaVetorFollowWall(sm, idx_min, idx_min2, sm->u_fw_l);
        }
        *sm->u_fw = sm->u_fw_l;
    }

    /*
    // Restaurar valores nao saturados
    for(i=0; i<5; i++) {
        sm->jubileuInputOutput->DistanciaSensor[i] = temp[i];
    }
    */
}

void calculaVetorFollowWall(t_sm_ControladorHibrido *sm, int P1, int P2, double resultado[2]) {
    double tempP1[2], modulo;
    double u_fw_p[2], u_fw_t[2];

    // Robot Frame:
    // Dist�ncia P1:
    tempP1[0] = cos(M_PI_2 - P1*M_PI_4)*sm->jubileuInputOutput->DistanciaSensor[P1]
                                                   + sm->jubileuInputOutput->sensorOffSet[0][P1];
    tempP1[1] = sin(M_PI_2 - P1*M_PI_4)*sm->jubileuInputOutput->DistanciaSensor[P1]
                                                   + sm->jubileuInputOutput->sensorOffSet[1][P1];
    // Dist�ncia P2:
    u_fw_t[0] = cos(M_PI_2 - P2*M_PI_4)*sm->jubileuInputOutput->DistanciaSensor[P2]
                                                   + sm->jubileuInputOutput->sensorOffSet[0][P2];
    u_fw_t[1] = sin(M_PI_2 - P2*M_PI_4)*sm->jubileuInputOutput->DistanciaSensor[P2]
                                                   + sm->jubileuInputOutput->sensorOffSet[1][P2];
    // P2 - P1 (vetor paralelo - t de tangente):
    u_fw_t[0] -= tempP1[0];
    u_fw_t[1] -= tempP1[1];
    // Normalizar
    modulo = norm(u_fw_t[0],u_fw_t[1]);
    u_fw_t[0] /= modulo;
    u_fw_t[1] /= modulo;
    // Produto escalar
    tempP1[0] /= 100;
    tempP1[1] /= 100;
    modulo = tempP1[0]*u_fw_t[0] + tempP1[1]*u_fw_t[1];
    // Vetor perpendicular � parede:
    u_fw_p[0] = tempP1[0] - ((modulo)*u_fw_t[0]);
    u_fw_p[1] = tempP1[1] - ((modulo)*u_fw_t[1]);

    // Equil�brio do vetor perpendicular de acordo com dist�ncia desejada:
    modulo = norm(u_fw_p[0],u_fw_p[1]);
    u_fw_p[0] -= ((double)D_FW/100)*u_fw_p[0]/modulo;
    u_fw_p[1] -= ((double)D_FW/100)*u_fw_p[1]/modulo;

    //Solu��o: Combina��o linear dos vetores paralelo e perpendicular.
    resultado[0] = u_fw_t[0] + 5.5*u_fw_p[0];
    resultado[1] = u_fw_t[1] + 5.5*u_fw_p[1];

    // World Frame:
    tempP1[0] = cos(sm->jubileuInputOutput->CoordTheta)*resultado[0]
               -sin(sm->jubileuInputOutput->CoordTheta)*resultado[1]
               +sm->jubileuInputOutput->CoordX;

    tempP1[1] = sin(sm->jubileuInputOutput->CoordTheta)*resultado[0]
               +cos(sm->jubileuInputOutput->CoordTheta)*resultado[1]
               +sm->jubileuInputOutput->CoordY;
    resultado[0] = tempP1[0];
    resultado[1] = tempP1[1];
}

uint8_t obstaculoEstaPresente(t_sm_ControladorHibrido *sm) {
    if(sm->fw_direction == SLIDING_RIGHT) {
        // Sensores Direitos (3 e 4)
        if(sm->jubileuInputOutput->DistanciaSensor[3] < D_SENSOR_SAT) {
            return 1;
        }
        if(sm->jubileuInputOutput->DistanciaSensor[4] < D_SENSOR_SAT) {
            return 1;
        }
    } else {
        // Sensores Esquerdos (0 e 1)
        if(sm->jubileuInputOutput->DistanciaSensor[0] < D_SENSOR_SAT) {
            return 1;
        }
        if(sm->jubileuInputOutput->DistanciaSensor[1] < D_SENSOR_SAT) {
            return 1;
        }
    }

    return 0;
}

void unicicloParaAcionamentoDiferencialPriorizandoOmega(t_sm_ControladorHibrido *sm) {
    double vlim, wlim, velMax, velMin;

    if(fabs(sm->jubileuInputOutput->v) > 0) {
        // Limitar v e w considerando limites m�nimos e m�ximos (espelhado, se forem negativos)
        vlim = fmax(fmin(fabs(sm->jubileuInputOutput->v),JUBILEU_R*JUBILEU_MAX_VEL),JUBILEU_R*JUBILEU_MIN_VEL);
        wlim = fmax(fmin(fabs(sm->jubileuInputOutput->w),(JUBILEU_R/JUBILEU_L)*(JUBILEU_MAX_VEL-JUBILEU_MIN_VEL)),0);

        // Converter uniciclo para acionamento diferencial (com v e w positivos):
        uniToDiff(sm, vlim, wlim);

        // limitar wl e wr caso estejam em situa��o de satura��o:
        velMax = fmax(sm->jubileuInputOutput->vel_l, sm->jubileuInputOutput->vel_r);
        velMin = fmin(sm->jubileuInputOutput->vel_l, sm->jubileuInputOutput->vel_r);
        if(velMax > JUBILEU_MAX_VEL) {
            sm->jubileuInputOutput->vel_l -= (velMax - JUBILEU_MAX_VEL);
            sm->jubileuInputOutput->vel_r -= (velMax - JUBILEU_MAX_VEL);
        } else if(velMin < JUBILEU_MIN_VEL) {
            sm->jubileuInputOutput->vel_l += (JUBILEU_MIN_VEL - velMin);
            sm->jubileuInputOutput->vel_r += (JUBILEU_MIN_VEL - velMin);
        } // else: n�o � necess�rio alterar recomenda��o.

        // Desespelhar:
        if(sm->jubileuInputOutput->v >= 0) {
            vlim=1;
        } else {
            vlim=-1;
        }
        if(sm->jubileuInputOutput->w >= 0) {
            wlim=1;
        } else {
            wlim=-1;
        }
        diffToUni(sm, sm->jubileuInputOutput->vel_l, sm->jubileuInputOutput->vel_r);
        sm->jubileuInputOutput->v *= vlim;
        sm->jubileuInputOutput->w *= wlim;
    } else { // rob� estacion�rio:
        if(fabs(sm->jubileuInputOutput->w) > (JUBILEU_R/JUBILEU_L)*2*JUBILEU_MIN_VEL) {
            if(sm->jubileuInputOutput->w >= 0) {
                sm->jubileuInputOutput->w = fmax(fmin(fabs(sm->jubileuInputOutput->w),(JUBILEU_R/JUBILEU_L)*2*JUBILEU_MAX_VEL),(JUBILEU_R/JUBILEU_L)*2*JUBILEU_MIN_VEL);
            } else {
                sm->jubileuInputOutput->w = -fmax(fmin(fabs(sm->jubileuInputOutput->w),(JUBILEU_R/JUBILEU_L)*2*JUBILEU_MAX_VEL),(JUBILEU_R/JUBILEU_L)*2*JUBILEU_MIN_VEL);
            }
        } else {
            sm->jubileuInputOutput->w = 0;
        }
    }

    // Agora, vel_r e vel_l com valores que garantem w:
    uniToDiff(sm, sm->jubileuInputOutput->v, sm->jubileuInputOutput->w);
}

void uniToDiff(t_sm_ControladorHibrido *sm, double v, double w) {
    sm->jubileuInputOutput->vel_r = (2*v + w * JUBILEU_L)/(2*JUBILEU_R);
    sm->jubileuInputOutput->vel_l = (2*v - w * JUBILEU_L)/(2*JUBILEU_R);
}

void diffToUni(t_sm_ControladorHibrido *sm, double vel_l, double vel_r) {
    sm->jubileuInputOutput->v = JUBILEU_R*(vel_r + vel_l)/2;
    sm->jubileuInputOutput->w = (JUBILEU_R/JUBILEU_L)*(vel_r - vel_l);
}

void converterRadPorSecParaPorcPWM(t_sm_ControladorHibrido *sm) {
    sm->jubileuInputOutput->motorEsquerdo = sm->jubileuInputOutput->vel_l/JUBILEU_MAX_VEL;
    sm->jubileuInputOutput->motorDireito = sm->jubileuInputOutput->vel_r/JUBILEU_MAX_VEL;
}
