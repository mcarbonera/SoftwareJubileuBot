#include "StateMachineComm.h"

//FUN플O TRANSICAO
static void fn_ESPERAMENSAGEMCLIENTE(t_sm *sm, unsigned char data)
{
    if(data == EV_MENSAGEMRECEBIDA)
    {
        if(VerificaResposta(sm,"SETCOORDOBJ(_.__,_.__)"))
        {
            sm->state = ST_RESPONDECLIENTE_SETCOORDOBJ;
        }
        else if(VerificaResposta(sm,"GETSTATE"))
        {
            sm->state = ST_RESPONDECLIENTE_GETSTATE;
        }
        else if(VerificaResposta(sm,"EXIT"))
        {
            MontaMensagem(sm, "*\r\n");
            EnviarMensagemUart(sm);
        }
    }
}

static void fn_RESPONDECLIENTE_SETCOORDOBJ(t_sm *sm, unsigned char data)
{
    if(data == EV_ENVIARMENSAGEM)
    {
        MontaMensagem(sm, "Objetivo Alterado!\r\n");
        EnviarMensagemUart(sm);
        sm->state = ST_RESPONDECLIENTE_GETSTATE;
    }
}

static void fn_RESPONDECLIENTE_GETSTATE(t_sm *sm, unsigned char data)
{
    if(data == EV_ENVIARMENSAGEM)
    {
        EnviarMensagemUart(sm);
    }
    else if(VerificaResposta(sm,"EXIT"))
    {
        //MontaMensagem(sm, "Esperando Mensagem:\r\n");
        //EnviarMensagemUart(sm);
        sm->state = ST_ESPERAMENSAGEMCLIENTE;
    }
    else if(VerificaResposta(sm,"SETCOORDOBJ(_.__,_.__)"))
    {
        sm->state = ST_RESPONDECLIENTE_SETCOORDOBJ;
    }
}

//____________________________________________
//FUN플O INICIALIZA플O
void Init_SM(t_sm *sm, unsigned short addr)
{
    sm->state = ST_ESPERAMENSAGEMCLIENTE;
    sm->my_addr = addr;
    sm->action[ST_ESPERAMENSAGEMCLIENTE] = (t_action)fn_ESPERAMENSAGEMCLIENTE;
    sm->action[ST_RESPONDECLIENTE_SETCOORDOBJ] = (t_action)fn_RESPONDECLIENTE_SETCOORDOBJ;
    sm->action[ST_RESPONDECLIENTE_GETSTATE] = (t_action)fn_RESPONDECLIENTE_GETSTATE;
}

//FUN플O PARA EXECUTAR TRANSI플O
void Exec_SM(t_sm *sm, unsigned char data)
{
    sm->action[sm->state](sm,data);
}

// Fun寤es Auxiliares:
void MontaMensagem(t_sm *sm, char Mensagem[50])
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

int VerificaResposta(t_sm *sm, char Mensagem[50])
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

void EnviarMensagemUart(t_sm *sm)
{
    int i;

    if(sm->MensagemUARTData.MensagemWifiEnvIdx > 0)
    {
        for(i=0;i<sm->MensagemUARTData.MensagemWifiEnvIdx; i++)
        {
            UARTCharPut(UART0_BASE, sm->MensagemUARTData.MensagemWifiEnv[i]); // DEBUG
            UARTCharPut(UART3_BASE, sm->MensagemUARTData.MensagemWifiEnv[i]);
        }
    }
    sm->MensagemUARTData.MensagemWifiEnvIdx = 0;
}
