#ifndef _SM_Prot_h
#define _SM_Prot_h

/* Para transição de estado: */
//void*: ponteiro para qualquer coisa (enum com estados,
typedef void (*t_action)(void*, unsigned char); // porém, os tipos não foram definidos)

/* Para execução do controlador associado ao estado: */
typedef void (*t_action_Supervisor)(void*);

#endif
