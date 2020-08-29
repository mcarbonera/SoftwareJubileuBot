#include "StateMachineComm.h"

//FUNÇÃO TRANSICAO
static void fn_ESPERAMENSAGEMCLIENTE(t_sm_Comm *sm, unsigned char data)
{
    if(data == EV_MENSAGEMRECEBIDA)
    {
        if(temMensagem(sm) && VerificaResposta(sm,"SETCOORDOBJ(_.__,_.__)"))
        {
            sm->state = ST_RESPONDECLIENTE_SETCOORDOBJ;
        } else if(temMensagem(sm) && VerificaResposta(sm,"GETSTATE")) {
            sm->state = ST_RESPONDECLIENTE_GETSTATE;
        } else if(temMensagem(sm) && VerificaResposta(sm,"EXIT")) {
            MontaMensagem(sm, "*\r\n");
            EnviarMensagemUart(sm);
        } else if(temMensagem(sm) && VerificaResposta(sm,"SUPERVISOR_FUZZY")) {
            MontaMensagem(sm, "CONTROLE FUZZY SELECIONADO!\r\n");
            EnviarMensagemUart(sm);
            sm->state = ST_RESPONDECLIENTE_MUDASUPERVISOR_FUZZY;
        } else if(temMensagem(sm) && VerificaResposta(sm,"SUPERVISOR_HIBRIDO")) {
            MontaMensagem(sm, "CONTROLE HIBRIDO SELECIONADO!\r\n");
            EnviarMensagemUart(sm);
            sm->state = ST_RESPONDECLIENTE_MUDASUPERVISOR_HIBRIDO;
        }
    }
}

static void fn_RESPONDECLIENTE_SETCOORDOBJ(t_sm_Comm *sm, unsigned char data)
{
    if(data == EV_ENVIARMENSAGEM)
    {
        MontaMensagem(sm, "Objetivo Alterado!\r\n");
        EnviarMensagemUart(sm);
        sm->state = ST_RESPONDECLIENTE_GETSTATE;
        sm->MensagemUARTData.MensagemWifiRecIdx = 0;
    }
}

static void fn_RESPONDECLIENTE_GETSTATE(t_sm_Comm *sm, unsigned char data)
{
    if(data == EV_ENVIARMENSAGEM)
    {
        EnviarMensagemUart(sm);
    }
    if(data == EV_EXIT_LOGGING) {
        sm->state = ST_ESPERAMENSAGEMCLIENTE;
    }
    if(data == EV_INFORMARTRANSICAOCONTROLADOR) {
        sm->state = ST_RESPONDECLIENTE_MUDACONTROLADOR;
    } else if(temMensagem(sm) && VerificaResposta(sm,"EXIT")) {
        sm->state = ST_ESPERAMENSAGEMCLIENTE;
    }
    else if(temMensagem(sm) && VerificaResposta(sm,"SETCOORDOBJ(_.__,_.__)"))
    {
        sm->state = ST_RESPONDECLIENTE_SETCOORDOBJ;
    }
}

static void fn_RESPONDECLIENTE_MUDACONTROLADOR(t_sm_Comm *sm, unsigned char data)
{
    if(data == EV_ENVIARMENSAGEM) {
        EnviarMensagemUart(sm);
        sm->state = ST_RESPONDECLIENTE_GETSTATE;
    }
}

static void fn_RESPONDECLIENTE_MUDASUPERVISOR_HIBRIDO(t_sm_Comm *sm, unsigned char data)
{
    if(data == EV_INFORMARTRANSICAOSUPERVISOR) {
        sm->state = ST_ESPERAMENSAGEMCLIENTE;
    }
}

static void fn_RESPONDECLIENTE_MUDASUPERVISOR_FUZZY(t_sm_Comm *sm, unsigned char data)
{
    if(data == EV_INFORMARTRANSICAOSUPERVISOR) {
        sm->state = ST_ESPERAMENSAGEMCLIENTE;
    }
}

//____________________________________________
//FUNÇÃO INICIALIZAÇÃO
void Init_SM_Comm(t_sm_Comm *sm)
{
    sm->state = ST_ESPERAMENSAGEMCLIENTE;
    sm->action[ST_ESPERAMENSAGEMCLIENTE] = (t_action)fn_ESPERAMENSAGEMCLIENTE;
    sm->action[ST_RESPONDECLIENTE_SETCOORDOBJ] = (t_action)fn_RESPONDECLIENTE_SETCOORDOBJ;
    sm->action[ST_RESPONDECLIENTE_GETSTATE] = (t_action)fn_RESPONDECLIENTE_GETSTATE;
    sm->action[ST_RESPONDECLIENTE_MUDACONTROLADOR] = (t_action)fn_RESPONDECLIENTE_MUDACONTROLADOR;
    sm->action[ST_RESPONDECLIENTE_MUDASUPERVISOR_HIBRIDO] = (t_action)fn_RESPONDECLIENTE_MUDASUPERVISOR_HIBRIDO;
    sm->action[ST_RESPONDECLIENTE_MUDASUPERVISOR_FUZZY] = (t_action)fn_RESPONDECLIENTE_MUDASUPERVISOR_FUZZY;

    sm->MensagemUARTData.MensagemWifiEnvIdx = 0;
    sm->MensagemUARTData.MensagemWifiRecIdx = 0;

    MontaMensagem(sm, "Bluetooth Inicializado.\r\n");
    EnviarMensagemUart(sm);
}

//FUNÇÃO PARA EXECUTAR TRANSIÇÃO
void Exec_SM_Comm(t_sm_Comm *sm, unsigned char data)
{
    sm->action[sm->state](sm,data);
}

// Funções Auxiliares:
void MontaMensagem(t_sm_Comm *sm, char Mensagem[50])
{
    int i;
    for(i=0; Mensagem[i] != '\n'; i++)
    {
        sm->MensagemUARTData.MensagemWifiEnv[i] = Mensagem[i];
    }
    //sm->MensagemUARTData.MensagemWifiEnv[i] = '\r';
    //i++;
    sm->MensagemUARTData.MensagemWifiEnv[i] = '\n';
    i++;
    sm->MensagemUARTData.MensagemWifiEnvIdx = i;
}

// Funções Auxiliares:
void MontaMensagemComInicio(char MensagemAEnviar[50], char Mensagem[50], int idxInicio)
{
    int i;
    for(i=0; Mensagem[i] != '\n'; i++)
    {
        MensagemAEnviar[idxInicio+i] = Mensagem[i];
    }
    //sm->MensagemUARTData.MensagemWifiEnv[i] = '\r';
    //i++;
    MensagemAEnviar[idxInicio+i] = '\n';
}

int temMensagem(t_sm_Comm *sm) {
    return sm->MensagemUARTData.MensagemWifiRecIdx != 0;
}

int VerificaResposta(t_sm_Comm *sm, char Mensagem[50])
{
    int i;
    int Retorno = 1;
    if(sm->MensagemUARTData.MensagemWifiRecIdx > 0)
    {
        for(i=0; Retorno && (i < sm->MensagemUARTData.MensagemWifiRecIdx); i++)
        {
            if(sm->MensagemUARTData.MensagemWifiRec[i] != Mensagem[i])
            {
                if(Mensagem[i] != '_')
                {
                    Retorno = 0;
                }
            }
        }
    }

    return(Retorno);
}

void EnviarMensagemUart(t_sm_Comm *sm)
{
    int i;

    if(sm->MensagemUARTData.MensagemWifiEnvIdx > 0)
    {
        for(i=0;i<sm->MensagemUARTData.MensagemWifiEnvIdx; i++)
        {
            #ifdef DEBUG_ACTIVE
            UARTCharPut(UART0_BASE, sm->MensagemUARTData.MensagemWifiEnv[i]); // DEBUG
            #endif
            UARTCharPut(UART3_BASE, sm->MensagemUARTData.MensagemWifiEnv[i]);
        }
    }
    sm->MensagemUARTData.MensagemWifiEnvIdx = 0;
}
