#ifndef _SM_Prot_h
#define _SM_Prot_h

/* Para transi��o de estado: */
//void*: ponteiro para qualquer coisa (enum com estados,
typedef void (*t_action)(void*, unsigned char); // por�m, os tipos n�o foram definidos)

/* Para execu��o do controlador associado ao estado: */
typedef void (*t_action_Supervisor)(void*);

#endif
