#include "StateMachineSupervisores.h"
//#include "StateMachineGeneric.h"
// DEFINI��ES DOS CONTROLADORES:


// FUN��ES DE TRANSICAO
static void fn_SUPERVISOR_HIBRIDO(t_sm_Sup *sm, unsigned char data) {
    if(data == EV_ESCOLHE_CONTROLADOR_FUZZY) {
        sm->supervisorAtual = ST_CONTROLADOR_FUZZY;
    }
}

static void fn_CONTROLADOR_FUZZY(t_sm_Sup *sm, unsigned char data) {
    if(data == EV_ESCOLHE_CONTROLADOR_HIBRIDO) {
        sm->supervisorAtual = ST_SUPERVISOR_HIBRIDO;
    }
}

//____________________________________________
//FUN��O INICIALIZA��O
void Init_SM_Sup(t_sm_Sup *sm, JDInputOutput *data)
{
    sm->supervisorAtual = ST_SUPERVISOR_HIBRIDO;

    /* Fun��es de transi��o de estado: */
    sm->transicao[ST_SUPERVISOR_HIBRIDO] = (t_action)fn_SUPERVISOR_HIBRIDO;
    sm->transicao[ST_CONTROLADOR_FUZZY] = (t_action)fn_CONTROLADOR_FUZZY;

    /* Fun��es para executar controlador atual: */
    sm->supervisor[ST_SUPERVISOR_HIBRIDO] = (t_action_Supervisor)ExecutarSupervisorHibrido;
    sm->supervisor[ST_CONTROLADOR_FUZZY] = (t_action_Supervisor)ExecutarSupervisorFuzzy;

    /* Estruturas de dados para executar controladores h�brido e fuzzy */
    sm->supervisorDados[ST_SUPERVISOR_HIBRIDO] = &sm->dataControladorHibrido;
    sm->supervisorDados[ST_CONTROLADOR_FUZZY] = &sm->dataControladorFuzzy;

    /* Inicializa Controlador H�brido: */
    sm->jubileuInputOutput = data;
    Init_SM_Controlador_Hibrido(&sm->dataControladorHibrido, data);
    Init_SM_Controlador_Fuzzy(&sm->dataControladorFuzzy, data);
}

// FUN��O PARA EXECUTAR TRANSI��O
void Exec_SM_Supervisor_Transicao(t_sm_Sup *sm, unsigned char data)
{
    sm->transicao[sm->supervisorAtual](sm,data);
}

// FUN��O PARA EXECUTAR CONTROLADOR
void Exec_SM_Supervisor_Supervisor(t_sm_Sup *sm)
{
    sm->supervisor[sm->supervisorAtual](sm->supervisorDados[sm->supervisorAtual]);
}
