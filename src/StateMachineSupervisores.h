#ifndef _SMSup_h
#define _SMSup_h

/*
 * Definições, protótipos e tipos:
 */
#include "main.h"
#include "StateMachineGeneric.h"
#include "SupervisorHibrido.h"
#include "SupervisorFuzzy.h"

typedef enum{
    ST_SUPERVISOR_HIBRIDO=0,
    ST_CONTROLADOR_FUZZY,
    ST_SIZE_SUPERVISORES
} t_states_Sup;

typedef enum{
    EV_ESCOLHE_CONTROLADOR_HIBRIDO=0,
    EV_ESCOLHE_CONTROLADOR_FUZZY
} t_events_Sup;

typedef struct _sm_sup_{
    t_states_Sup        supervisorAtual;
    t_action            transicao[ST_SIZE_SUPERVISORES];
    t_action_Supervisor supervisor[ST_SIZE_SUPERVISORES];

    // Dados para supervisor híbrido e controlador fuzzy:
    void *supervisorDados[ST_SIZE_SUPERVISORES];
    // Estrutura de Dados do Controlador Híbrido:
    t_sm_ControladorHibrido dataControladorHibrido;
    t_sm_ControladorFuzzy dataControladorFuzzy;

    JDInputOutput *jubileuInputOutput;
} t_sm_Sup;

void Init_SM_Sup(t_sm_Sup *sm, JDInputOutput *data);
void Exec_SM_Supervisor_Transicao(t_sm_Sup *sm, unsigned char data);
void Exec_SM_Supervisor_Supervisor(t_sm_Sup *sm);

#endif
