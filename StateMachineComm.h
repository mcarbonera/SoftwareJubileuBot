#ifndef _SM_h
#define _SM_h

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/uart.h"

/*
 * Definições, protótipos e tipos:
 */

//#define STX 0x02
//#define ETX 0x02

typedef void (*t_handler)(void*, unsigned char);
//void*: ponteiro para qualquer coisa (era para ser t_sm*,
                                       //porém, não foi definido)
typedef void (*t_action)(void*, unsigned char);

typedef enum{
    ST_ESPERAMENSAGEMCLIENTE=0,
    ST_RESPONDECLIENTE_SETCOORDOBJ,
    ST_RESPONDECLIENTE_GETSTATE,
    ST_SIZE
} t_states;

typedef enum{
    EV_ENVIARMENSAGEM=0,
    EV_MENSAGEMRECEBIDA
} t_events;

typedef struct Msg_Uart {
    uint32_t MensagemWifiEnv[50];
    uint32_t MensagemWifiRec[50];
    int MensagemWifiEnvIdx;
    int MensagemWifiRecIdx;
    uint8_t LastChar;
    int RecebendoMensagem;
} Msg_Uart;

typedef struct _sm_{
    t_states    state;
    //t_events    event;
    t_action    action[ST_SIZE];
    t_handler   handler[ST_SIZE];
    short       my_addr;
    Msg_Uart    MensagemUARTData;
}t_sm;

void Init_SM(t_sm *sm, unsigned short addr);
void Exec_SM(t_sm *sm, unsigned char data);
void MontaMensagem(t_sm *sm, char Mensagem[50]);
int VerificaResposta(t_sm *sm, char Mensagem[50]);
void EnviarMensagemUart(t_sm *sm);

//#include "StateMachineComm.c"
#endif

