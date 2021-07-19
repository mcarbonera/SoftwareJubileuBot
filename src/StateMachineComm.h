#ifndef _SMComm_h
#define _SMComm_h

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/uart.h"

/*
 * Definições, protótipos e tipos:
 */
#include "StateMachineGeneric.h"

typedef enum{
    ST_ESPERAMENSAGEMCLIENTE=0,
    ST_RESPONDECLIENTE_SETCOORDOBJ,
    ST_RESPONDECLIENTE_GETSTATE,
    ST_RESPONDECLIENTE_MUDACONTROLADOR,
    ST_RESPONDECLIENTE_MUDASUPERVISOR_HIBRIDO,
    ST_RESPONDECLIENTE_MUDASUPERVISOR_FUZZY,
    ST_SIZE
} t_states_Comm;

typedef enum{
    EV_ENVIARMENSAGEM=0,
    EV_MENSAGEMRECEBIDA,
    EV_INFORMARTRANSICAOCONTROLADOR,
    EV_INFORMARTRANSICAOSUPERVISOR,
    EV_EXIT_LOGGING
} t_events_Comm;

typedef struct Msg_Uart {
    uint32_t MensagemWifiEnv[50];
    uint32_t MensagemWifiRec[50];
    int MensagemWifiEnvIdx;
    int MensagemWifiRecIdx;
    uint8_t LastChar;
    int RecebendoMensagem;
} Msg_Uart;

typedef struct _sm_{
    t_states_Comm   state;
    t_action        action[ST_SIZE];
    Msg_Uart        MensagemUARTData;
} t_sm_Comm;

#include "main.h"

void Init_SM_Comm(t_sm_Comm *sm);
void Exec_SM_Comm(t_sm_Comm *sm, unsigned char data);
void MontaMensagem(t_sm_Comm *sm, char Mensagem[50]);
void MontaMensagemComInicio(char MensagemAEnviar[50], char Mensagem[50], int idxInicio);
int temMensagem(t_sm_Comm *sm);
int VerificaResposta(t_sm_Comm *sm, char Mensagem[50]);
void EnviarMensagemUart(t_sm_Comm *sm);

// #include "StateMachineComm.c"
#endif

