#include "main.h"

#include "inc/tm4c1294ncpdt.h"

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
//#include "inc/hw_ints.h"

#include "driverlib/sysctl.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/adc.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"

#include "StateMachineComm.h"
#include "StateMachineSupervisores.h"

//#include "utils/uartstdio.h"

#define PWM_FREQUENCY 100

#define JUBILEU_TICKS_PER_REVOLUTION 297
#define JUBILEU_DIST_PER_TICK ((2*M_PI*JUBILEU_R)/JUBILEU_TICKS_PER_REVOLUTION)
#define TAMANHO_STRING_ESTADO 49
#define TAMANHO_STRING_CONTROLADOR_SELECIONADO 43

/* Coeficientes do polinômio do sensor infravermelho */
#define IR_COEFF_A 2.7802212625047
#define IR_COEFF_B -35.1150300110164
#define IR_COEFF_C 179.6031433005229
#define IR_COEFF_D -477.9449116299037
#define IR_COEFF_E 706.3400747125301
#define IR_COEFF_F -569.7367375002304
#define IR_COEFF_G 221.2678651473199
#define IR_COEFF_K 0.0008058608059

void InicializaStructRobo(t_sm_Comm *sm);
int IsNumber(char caractere);
float CharAsFloat(char caractere);
char IntAsChar(int num);
void UpdateOdometry();
void CoordenadaParaString();
void DistanciasSensoresParaString();
void AcionarMotor(float Porcentagem_PWM_Esq, float Porcentagem_PWM_Dir);
void PreparaStringEstado(char StringEstado[TAMANHO_STRING_ESTADO], char Mensagem[TAMANHO_STRING_ESTADO]);
void calcularDistanciasSensores(int idxInicio, int idxFim);

typedef struct JD {
    int EncoderEsq, EncoderDir;
    char CoordString[TAMANHO_STRING_ESTADO]; //00.00,00.00,00.00,00.00,00.00,00.00,00.00,00.00\r\n
    char ControladorStr[TAMANHO_STRING_CONTROLADOR_SELECIONADO]; //Novo controlador selecionado: xxxxxxxxxxx\r\n
    t_sm_Sup supervisores;
    JDInputOutput jubileuInputOutput;
} JD;

typedef struct RetornoADC {
    uint8_t ADC0Calculado;
    uint8_t ADC1Calculado;
    uint32_t ui32ADC0Values[4];
    uint32_t ui32ADC1Values[4];
    volatile uint32_t Sensor[5]; // retorno bruto
} RetornoADC;

JD JubileuData;
t_sm_Comm StateMachineComm;

RetornoADC retornoADC;
uint8_t HallEncoder_Esq=0;
uint8_t HallEncoder_Dir=0;
uint32_t ui32SysClkFreq;
uint32_t ui32Period;
volatile uint32_t ui32Load; // PWM period
volatile uint32_t ui32PWMClock; // PWM clock frequency
int countControllerExec = 0;

#ifdef TESTE_MOTOR_X_ENCODER
    float TesteMotor_PWM = -1;
    int TesteSentido = 1;
    int TesteIndice = 0;
#endif

int main(void)
{
    // Configurar clock:
    ui32SysClkFreq = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);

    // 1 - Primeiro habilitar periféricos:
    // Led:
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    // Timer:
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    // ADC para 5 sensores: ADC0 para 1, 2 e 3, ADC1 para 4 e 5:
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); // PE0-4
    // PWM:
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); // PF2 e PF3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG); // PG0 e PG1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    /* Entradas - Sensor de efeito Hall
     *                  Canal A   |   Canal B
     * -> Esquerdo:        PM0          PM1
     * -> Direito:         PH0          PH1
     *                 <INTERRUPT>
     * */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
    // Módulo Bluetooth XM-15B - Comunicação com micro via UART
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    #ifdef DEBUG_ACTIVE
        SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0); //___________________________________DEBUG
    #endif

    // 2 - Configurar periféricos depois:
    // Porta:
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0|GPIO_PIN_1);
    // Timer:
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    ui32Period = ui32SysClkFreq/(CALCULOS_ODOMETRIA*FREQUENCIA_TIMER);
    TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period -1);
    // ADC:
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4);
        // ADC Sequencer:
    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0); // Sample Sequencer 1 do ADC0
    ADCSequenceConfigure(ADC1_BASE, 1, ADC_TRIGGER_PROCESSOR, 0); // Sample Sequencer 1 do ADC1
//                                  ^ Sample Sequencer        ^ Priority of Sample Sequencer
        // Configurar 3 passos no sequencer do ADC0:
    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH3);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH2);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH1|ADC_CTL_IE|ADC_CTL_END);
        // Configurar 2 passos no sequencer do ADC1:
    ADCSequenceStepConfigure(ADC1_BASE, 1, 0, ADC_CTL_CH0);
    ADCSequenceStepConfigure(ADC1_BASE, 1, 1, ADC_CTL_CH9|ADC_CTL_IE|ADC_CTL_END);
        // Sobreamostragem:
    //ADCHardwareOversampleConfigure(ADC0_BASE, 64);
    //ADCHardwareOversampleConfigure(ADC1_BASE, 64);
    // PWM:
    //GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2|GPIO_PIN_3);
    //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2|GPIO_PIN_3, 0x00);
        //Clock PWM (/64 == 1875000):
    PWMClockSet(PWM0_BASE,PWM_SYSCLK_DIV_64);
    //PWMClockSet(PWM0_BASE,PWM_SYSCLK_DIV_64);
        // PWM pinos PG0 e PG1 (Generator 2) PF0 está ligado a led.. Logo, Gen 0 não será usado
    GPIOPinConfigure(GPIO_PG0_M0PWM4);
    GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_0);
    GPIOPinConfigure(GPIO_PG1_M0PWM5);
    GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_1);
        // PWM pinos PF2 e PF3 (Gen 1)
    GPIOPinConfigure(GPIO_PF2_M0PWM2);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);
    GPIOPinConfigure(GPIO_PF3_M0PWM3);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3);
        // Calcular clock:
    ui32PWMClock = ui32SysClkFreq/64; // 120MHz/64 = 1875000
    ui32Load = (ui32PWMClock/PWM_FREQUENCY) - 1; // 1875000/100  =  18750 (1%)
        // Terminar configuração:
        // PWM0 Gen 2 - Portas G0 e G1
    PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC); // no sync necessário para gerar dead-band
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, ui32Load); // Frequencia 100 Hz
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, ui32Load/2); // duty cycle 50 %
        // PWM0 Gen 1 - Portas F2 e F3
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC); // no sync necessário para gerar dead-band
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, ui32Load); // Frequencia 100 Hz
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, ui32Load/2); // duty cycle 50 %
        // Saída Portas
    PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT | PWM_OUT_5_BIT, false); // Saída 0 e 1 do Generator 2
    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT | PWM_OUT_3_BIT, false); // Saída 0 e 1 do Generator 1
        // Deadband
    PWMDeadBandEnable(PWM0_BASE, PWM_GEN_2, 360, 360); // Gerador de banda morta para ponte H:
    PWMDeadBandEnable(PWM0_BASE, PWM_GEN_1, 360, 360); // Gerador de banda morta para ponte H:
    // 360 indica um tempo de 3us entre mudanças de borda, tempo necessário pelo que consegui entender do datasheet do driver l298n
    // Entradas - Sensor de efeito Hall
    //GPIOIntRegisterPin(GPIO_PORTP_BASE, 0, (void)(*GPIO_P0_IntHandler)());
    //GPIOIntRegisterPin(GPIO_PORTP_BASE, 1, (void*)GPIO_P1_IntHandler());
    GPIOPinTypeGPIOInput(GPIO_PORTM_BASE, GPIO_PIN_0 | GPIO_PIN_1); // Esquerda
    GPIOPinTypeGPIOInput(GPIO_PORTH_BASE, GPIO_PIN_0 | GPIO_PIN_1); // Direita
    // Módulo Bluetooth XM-15B - Comunicação com micro via UART
    GPIOPinConfigure(GPIO_PA4_U3RX);
    GPIOPinConfigure(GPIO_PA5_U3TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    UARTConfigSetExpClk(UART3_BASE, ui32SysClkFreq, 9600, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE)); //9600
    //UARTConfigSetExpClk(UART3_BASE, ui32SysClkFreq, 38400, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE)); //38400
    //UARTConfigSetExpClk(UART3_BASE, ui32SysClkFreq, 1200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    #ifdef DEBUG_ACTIVE
        GPIOPinConfigure(GPIO_PA0_U0RX);
        GPIOPinConfigure(GPIO_PA1_U0TX);
        GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
        UARTConfigSetExpClk(UART0_BASE, ui32SysClkFreq, 9600, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE)); //___________________________________DEBUG
        //9600
    #endif

    /* Inicialização de struct */
    //Inicializa Variáveis:
    InicializaStructRobo(&StateMachineComm);

    // 3 - Habilitar interrupções:
    // int Enable
    IntMasterEnable();
    // Habilitar interrupção do timer:
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    // Habilitar interrupção por borda (para sensor de Efeito Hall)
    GPIOIntTypeSet(GPIO_PORTM_BASE, GPIO_PIN_0, GPIO_BOTH_EDGES);
    GPIOIntTypeSet(GPIO_PORTH_BASE, GPIO_PIN_0, GPIO_BOTH_EDGES);
    IntEnable(INT_GPIOM);
    IntEnable(INT_GPIOH);
    GPIOIntEnable(GPIO_PORTM_BASE, GPIO_PIN_0);
    GPIOIntEnable(GPIO_PORTH_BASE, GPIO_PIN_0);
    // UART - MÓDULO BLUETOOTH
    IntEnable(INT_UART3);
    UARTIntEnable(UART3_BASE, UART_INT_RX | UART_INT_RT);
    #ifdef DEBUG_ACTIVE
        IntEnable(INT_UART0);
        UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT); //___________________________________DEBUG
    #endif
    // Interrupção ADC0 e 1 Sample Sequencer 1
    IntEnable(INT_ADC0SS1);
    IntEnable(INT_ADC1SS1);
    ADCIntEnable(ADC0_BASE,1);
    ADCIntEnable(ADC1_BASE,1);

    // 4 - Habilitar periféricos
    // Habilitar Sequencer 1 dos ADCs 0 e 1:
    ADCSequenceEnable(ADC0_BASE, 1);
    ADCSequenceEnable(ADC1_BASE, 1);
    //GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0|GPIO_PIN_1, 0x00);

    // Habilitar PWM
    PWMGenEnable(PWM0_BASE, PWM_GEN_2);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);

    // Habilitar Timer:
    TimerEnable(TIMER0_BASE, TIMER_A);

    while(1)
    {
        /* Cálculo do polinômio - Sensores Infravermelho */
        while(!retornoADC.ADC1Calculado);
        retornoADC.ADC1Calculado = 0;
        calcularDistanciasSensores(3,5);
        while(!retornoADC.ADC0Calculado);
        retornoADC.ADC0Calculado = 0;
        calcularDistanciasSensores(0,3);

        /* Monta Mensagem para envio (se necessário)
        Verificar necessidade de responder cliente (via bluetooth): */
        if(StateMachineComm.state==ST_RESPONDECLIENTE_GETSTATE)
        {
            /* Atualizar String para envio */
            #ifndef TESTE_MOTOR_X_ENCODER /* Se definido, conversão será realizada em UpdateOdometry*/
            CoordenadaParaString();
            DistanciasSensoresParaString();
            #endif
            MontaMensagem(&StateMachineComm, JubileuData.CoordString);
            Exec_SM_Comm(&StateMachineComm,EV_ENVIARMENSAGEM);
        }
        if(StateMachineComm.state==ST_RESPONDECLIENTE_SETCOORDOBJ)
        {
            //SETCOORDOBJ(_.__,_.__)
            if(IsNumber(StateMachineComm.MensagemUARTData.MensagemWifiRec[12]) &&
                    IsNumber(StateMachineComm.MensagemUARTData.MensagemWifiRec[14]) &&
                    IsNumber(StateMachineComm.MensagemUARTData.MensagemWifiRec[15]) &&
                    IsNumber(StateMachineComm.MensagemUARTData.MensagemWifiRec[17]) &&
                    IsNumber(StateMachineComm.MensagemUARTData.MensagemWifiRec[19]) &&
                    IsNumber(StateMachineComm.MensagemUARTData.MensagemWifiRec[20]))
            {
                JubileuData.jubileuInputOutput
                .ObjX = CharAsFloat(StateMachineComm.MensagemUARTData.MensagemWifiRec[12]) +
                        CharAsFloat(StateMachineComm.MensagemUARTData.MensagemWifiRec[14])/10 +
                        CharAsFloat(StateMachineComm.MensagemUARTData.MensagemWifiRec[15])/100;

                JubileuData.jubileuInputOutput
                .ObjY = CharAsFloat(StateMachineComm.MensagemUARTData.MensagemWifiRec[17]) +
                        CharAsFloat(StateMachineComm.MensagemUARTData.MensagemWifiRec[19])/10 +
                        CharAsFloat(StateMachineComm.MensagemUARTData.MensagemWifiRec[20])/100;
            }

            JubileuData.supervisores.dataControladorHibrido.d_prog=1000;
            Exec_SM_Comm(&StateMachineComm,EV_ENVIARMENSAGEM);
        }

        /* Cálculo controlador */
        JubileuData.jubileuInputOutput.v = 1;
        Exec_SM_Supervisor_Supervisor(&JubileuData.supervisores);

        /* Acionamento dos motores */
        AcionarMotor(JubileuData.jubileuInputOutput.motorEsquerdo, JubileuData.jubileuInputOutput.motorDireito);
        //AlterarAcionamento();

        /* Informa transição de controlador, se necessário */
        if(StateMachineComm.state==ST_RESPONDECLIENTE_MUDACONTROLADOR) {
            switch(JubileuData.supervisores.dataControladorHibrido.controladorAtual) {
                case ST_STOP:
                { // {} são necessários: "The C++ standard forbids a goto to bypass an initialization of a non-POD object"
                    MontaMensagemComInicio(JubileuData.ControladorStr, "STOP\r\n",30);
                    break;
                }
                case ST_GTG:
                {
                    MontaMensagemComInicio(JubileuData.ControladorStr, "GTG\r\n",30);
                    break;
                }
                case ST_AO:
                {
                    MontaMensagemComInicio(JubileuData.ControladorStr, "AO\r\n",30);
                    break;
                }
                case ST_AO_AND_GTG:
                {
                    MontaMensagemComInicio(JubileuData.ControladorStr, "AO_AND_GTG\r\n",30);
                    break;
                }
                case ST_FOLLOW_WALL:
                {
                    MontaMensagemComInicio(JubileuData.ControladorStr, "FOLLOW_WALL\r\n",30);
                    break;
                }
            }
            MontaMensagem(&StateMachineComm, JubileuData.ControladorStr);
            Exec_SM_Comm(&StateMachineComm,EV_ENVIARMENSAGEM);
        } else if(StateMachineComm.state==ST_RESPONDECLIENTE_MUDASUPERVISOR_HIBRIDO) {
            Exec_SM_Supervisor_Transicao(&JubileuData.supervisores,EV_ESCOLHE_CONTROLADOR_HIBRIDO);
            Exec_SM_Comm(&StateMachineComm,EV_INFORMARTRANSICAOSUPERVISOR);
        } else if(StateMachineComm.state==ST_RESPONDECLIENTE_MUDASUPERVISOR_FUZZY) {
            Exec_SM_Supervisor_Transicao(&JubileuData.supervisores,EV_ESCOLHE_CONTROLADOR_FUZZY);
            Exec_SM_Comm(&StateMachineComm,EV_INFORMARTRANSICAOSUPERVISOR);
        }
    }
}

/* FUNÇÕES - UART, comunicação, inicialização*/
void InicializaStructRobo(t_sm_Comm *sm)
{
    // OffSet dos Sensores:
    int i;
    for(i=0; i<5; i++) {
        JubileuData.jubileuInputOutput.sensorOffSet[0][i] = 8;
        JubileuData.jubileuInputOutput.sensorOffSet[1][i] = 0;
    }
    JubileuData.jubileuInputOutput.sensorOffSet[0][2] += 8;
    JubileuData.jubileuInputOutput.sensorOffSet[0][1] += 4*sqrt(2);
    JubileuData.jubileuInputOutput.sensorOffSet[0][3] += 4*sqrt(2);
    JubileuData.jubileuInputOutput.sensorOffSet[1][0] = 8;
    JubileuData.jubileuInputOutput.sensorOffSet[1][4] = -8;
    JubileuData.jubileuInputOutput.sensorOffSet[1][1] = 4*sqrt(2);
    JubileuData.jubileuInputOutput.sensorOffSet[1][3] = -4*sqrt(2);

    JubileuData.EncoderEsq = 0;
    JubileuData.EncoderDir = 0;
    JubileuData.jubileuInputOutput.motorEsquerdo = 0;
    JubileuData.jubileuInputOutput.motorDireito = 0;
    JubileuData.jubileuInputOutput.ObjX = 0;
    JubileuData.jubileuInputOutput.ObjY = 0;
    JubileuData.jubileuInputOutput.CoordX = 0;
    JubileuData.jubileuInputOutput.CoordY = 0;
    JubileuData.jubileuInputOutput.CoordTheta = 0;
    PreparaStringEstado(JubileuData.CoordString, "00.00,00.00,00.00,00.00,00.00,00.00,00.00,00.00\r\n");
    PreparaStringEstado(JubileuData.ControladorStr, "Novo controlador selecionado: \n");

    Init_SM_Comm(sm);
    Init_SM_Sup(&JubileuData.supervisores, &JubileuData.jubileuInputOutput);
    JubileuData.jubileuInputOutput.stateMachineComm = sm;

    retornoADC.ADC0Calculado=0;
    retornoADC.ADC1Calculado=0;
    /* Leds
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0|GPIO_PIN_1, 0xFF); // LEDs on
    SysCtlDelay(ui32SysClkFreq / (1)); // delay 1 sec
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0|GPIO_PIN_1, 0); // LEDs off
    */
}

void PreparaStringEstado(char StringEstado[TAMANHO_STRING_ESTADO], char Mensagem[TAMANHO_STRING_ESTADO])
{
    int i;
    for(i=0; Mensagem[i] != '\n'; i++)
    {
        StringEstado[i] = Mensagem[i];
    }
    StringEstado[i] = '\n';
}

int IsNumber(char caractere)
{                                               //  0 -  9
    return(caractere >= 48 && caractere <= 57); // 48 - 57
}

float CharAsFloat(char caractere)
{
    return((float)(caractere-48));
}

char IntAsChar(int num)
{
    return((char)(num+48));
}

void UpdateOdometry()
{
    int Left_Ticks = JubileuData.EncoderEsq;
    int Right_Ticks = JubileuData.EncoderDir;

    float d_left = Left_Ticks*JUBILEU_DIST_PER_TICK/2;
    float d_right = Right_Ticks*JUBILEU_DIST_PER_TICK/2;
    float d_center = (d_left + d_right)/2;
    float d_Phi = (d_right - d_left)/JUBILEU_L;

    JubileuData.jubileuInputOutput.CoordX += d_center*cosf(JubileuData.jubileuInputOutput.CoordTheta);
    JubileuData.jubileuInputOutput.CoordY += d_center*sinf(JubileuData.jubileuInputOutput.CoordTheta);
    JubileuData.jubileuInputOutput.CoordTheta += d_Phi;

    /* Ticks são perdidos após UpdateOdometry()
     * Portanto, em modo teste, deve-se preparar
     * String aqui.
     * */
    #ifdef TESTE_MOTOR_X_ENCODER
        CoordenadaParaString();
    #endif
    JubileuData.EncoderEsq -= Left_Ticks;
    JubileuData.EncoderDir -= Right_Ticks;

    /* FORÇAR CoordTheta para o intervalo [-PI,PI] */
    if(JubileuData.jubileuInputOutput.CoordTheta > M_PI){
        JubileuData.jubileuInputOutput.CoordTheta-=2*M_PI;
    } else if(JubileuData.jubileuInputOutput.CoordTheta < -M_PI){
        JubileuData.jubileuInputOutput.CoordTheta+=2*M_PI;
    }
}

void CoordenadaParaString()
{
    #ifdef TESTE_MOTOR_X_ENCODER // numero de ticks por Período
        int i;
        int TempEncEsq, TempEncDir;

        TempEncEsq = JubileuData.EncoderEsq;
        TempEncDir = JubileuData.EncoderDir;

        if(TempEncEsq < 0){
            TempEncEsq*=-1;
            JubileuData.CoordString[0] = '-';
        } else {
            JubileuData.CoordString[0] = '0';
        }
        if(TempEncDir < 0){
            TempEncDir*=-1;
            JubileuData.CoordString[6] = '-';
        } else {
            JubileuData.CoordString[6] = '0';
        }

        /* Converter valores para string */
        for(i=3; i>=0; i--)
        {                                       // +48
            JubileuData.CoordString[i+1]    = IntAsChar(TempEncEsq%10);     //  4, 3, 2, 1
            JubileuData.CoordString[i+7]    = IntAsChar(TempEncDir%10);     // 10, 9, 8, 7

            TempEncEsq = (TempEncEsq - TempEncEsq%10)/10;
            TempEncDir = (TempEncDir - TempEncDir%10)/10;
        }
    #else // Estimativas em x,y,theta
        int i,j;
        float TempX, TempY, TempTheta;

        TempX = JubileuData.jubileuInputOutput.CoordX;
        TempY = JubileuData.jubileuInputOutput.CoordY;
        TempTheta = JubileuData.jubileuInputOutput.CoordTheta;

        /* Verificar se é valor positivo ou negativo */
        if(TempX < 0){
            TempX*=-1;
            JubileuData.CoordString[0] = '-';
        } else {
            JubileuData.CoordString[0] = '0';
        }
        if(TempY < 0){
            TempY*=-1;
            JubileuData.CoordString[6] = '-';
        } else {
            JubileuData.CoordString[6] = '0';
        }
        if(TempTheta < 0){
            TempTheta*=-1;
            JubileuData.CoordString[12] = '-';
        } else {
            JubileuData.CoordString[12] = '0';
        }

        /* Converter valores para string - esse for ficou fera demais.. que orgulho hein */
        for(i=2,j=0; i>=0; j+=i, i--)
        {                                       // +48
            JubileuData.CoordString[j+1]    = IntAsChar((int)TempX);     //  1,  3,  4
            JubileuData.CoordString[j+7]    = IntAsChar((int)TempY);     //  7,  9, 10
            JubileuData.CoordString[j+13]   = IntAsChar((int)TempTheta); // 13, 15, 16

            TempX = (TempX - (int)TempX)*10;
            TempY = (TempY - (int)TempY)*10;
            TempTheta = (TempTheta - (int)TempTheta)*10;
        }

        /* Identificar Overflow */
        if(fabsf(JubileuData.jubileuInputOutput.CoordX) >= 10){
            JubileuData.CoordString[0] = 'O';
            JubileuData.CoordString[1] = 'F';
        }
        if(fabsf(JubileuData.jubileuInputOutput.CoordY) >= 10){
            JubileuData.CoordString[6] = 'O';
            JubileuData.CoordString[7] = 'F';
        }
    #endif
}

void DistanciasSensoresParaString() {
    int i,j,k;
    float TempDist;

    // Esse for ficou ainda melhor
    for(k=0; k<5; k++) {
        //Tratar Overflow:
        if(JubileuData.jubileuInputOutput.DistanciaSensor[k] >= 100) {
            // Escrever 99.99
            JubileuData.CoordString[k*6 + 18] = IntAsChar(9);
            for(i=2,j=0; i>=0; j+=i, i--) {
                JubileuData.CoordString[k*6 + j+19] = IntAsChar(9);
            }
            continue;
        }

        // Tratar dezenas:
        TempDist = JubileuData.jubileuInputOutput.DistanciaSensor[k] / 10;
        JubileuData.CoordString[k*6 + 18] = IntAsChar((int)TempDist);

        // Tratar restante (x.xx)
        TempDist = JubileuData.jubileuInputOutput.DistanciaSensor[k] - ((int)TempDist)*10;
        for(i=2,j=0; i>=0; j+=i, i--) {
            JubileuData.CoordString[k*6 + j+19] = IntAsChar((int)TempDist); // 0.00
            TempDist = (TempDist - (int)TempDist)*10;
        }
        TempDist=0;
    }
}

/* Porcentagem entre [-1,1] */
void AcionarMotor(float Porcentagem_PWM_Esq, float Porcentagem_PWM_Dir)
{
    /*
     * SysClk = 120 Mhz
     * ClockPWM = SysClk/64 = 1875000
     *
     * Frequencia do PWM é 100 Hz (Gen1 e Gen2).
     * Logo, reset é em 18750
     *
     * Porém, para evitar 0% e 100%,
     * usar 100% == 18749
     * e      0% == 1
     *
     * [-1,1]
     * uint32_t
     * 18748.0f
     * 1874998.0f
     * */
    uint32_t Nivel_Esq, Nivel_Dir;

    /* EVITAR VALORES FORA DE [-1,1] */
    if(Porcentagem_PWM_Esq > 1){
        Porcentagem_PWM_Esq = 1;
    } else if(Porcentagem_PWM_Esq < -1){
        Porcentagem_PWM_Esq = -1;
    }
    if(Porcentagem_PWM_Dir > 1){
        Porcentagem_PWM_Dir = 1;
    } else if(Porcentagem_PWM_Dir < -1){
        Porcentagem_PWM_Dir = -1;
    }

    Porcentagem_PWM_Esq*=-1;
    Porcentagem_PWM_Dir*=-1;

    /* CALCULAR PARAMETROS PARA GERAR PWM */
    if(Porcentagem_PWM_Esq < 0){    /* Para tras */
        Nivel_Esq = (uint32_t)(18748.0f*(1+Porcentagem_PWM_Esq) + 1);
        PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, false);
        PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT, true);
    } else {                        /* Para frente */
        Nivel_Esq = (uint32_t)(18748.0f*Porcentagem_PWM_Esq + 1);
        PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT, false);
        PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, true);
    }

    if(Porcentagem_PWM_Dir < 0) {    /* Para tras */
        Nivel_Dir = (uint32_t)(18748.0f*(1+Porcentagem_PWM_Dir) + 1);
        PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, false);
        PWMOutputState(PWM0_BASE, PWM_OUT_3_BIT, true);
    } else {                        /* Para frente */
        Nivel_Dir = (uint32_t)(18748.0f*Porcentagem_PWM_Dir + 1);
        PWMOutputState(PWM0_BASE, PWM_OUT_3_BIT, false);
        PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
    }

    /* SETAR NÍVEL DO PWM */
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, Nivel_Esq); // Motor Esquerdo
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, Nivel_Dir); // Motor Direito

    /*
    if(Porcentagem_PWM_Esq < 0) {
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, Nivel_Esq); // Motor Esquerdo
    } else {
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, Nivel_Esq); // Motor Esquerdo
    }

    if(Porcentagem_PWM_Dir < 0) {
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, Nivel_Dir); // Motor Direito
    } else {
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, Nivel_Dir); // Motor Direito
    }
    */
}

void calcularDistanciasSensores(int idxInicio, int idxFim)
{
    int i;
    double temp;

    for(i=idxInicio; i<idxFim; i++)
    {
        temp = retornoADC.Sensor[i]*IR_COEFF_K;
        JubileuData
        .jubileuInputOutput
        .DistanciaSensor[i] =   IR_COEFF_A*temp*temp*temp*temp*temp*temp +
                                IR_COEFF_B*temp*temp*temp*temp*temp +
                                IR_COEFF_C*temp*temp*temp*temp +
                                IR_COEFF_D*temp*temp*temp +
                                IR_COEFF_E*temp*temp +
                                IR_COEFF_F*temp +
                                IR_COEFF_G;

        if(JubileuData.jubileuInputOutput.DistanciaSensor[i] > 80.0) {
            JubileuData.jubileuInputOutput.DistanciaSensor[i] = 80.0;
        }
    }
}

/* INTERRUPÇÕES */
void Timer0IntHandler(void) // TIMER
{
    // Clear the timer interrupt
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    countControllerExec++;
    if(countControllerExec == CALCULOS_ODOMETRIA) {
        countControllerExec = 0;
    }

    if(countControllerExec == 0) {
        /* Aciona amostragem dos sensores (ADC0SS1 e ADC1SS1) */
        ADCProcessorTrigger(ADC0_BASE, 1);
        ADCProcessorTrigger(ADC1_BASE, 1);
    }

    /* Cálculo de Odometria*/
    UpdateOdometry();

    //****************************************************
    /* TESTE - ACIONAR MOTOR */
    #ifdef TESTE_MOTOR_X_ENCODER
        TesteIndice++;
        if(TesteIndice == 5)
        {
            #ifdef TESTE_MOTOR_RPS //   Acionamento constante
                AcionarMotor(-1,-1);
            #else //                    Acionamento triangular
                AcionarMotor(TesteMotor_PWM,TesteMotor_PWM);
            #endif

            if(TesteSentido)
            {
                TesteMotor_PWM+=0.01;
                if(TesteMotor_PWM > 1){
                    TesteMotor_PWM=1;
                    TesteSentido=0;
                }
            } else {
                TesteMotor_PWM-=0.01;
                if(TesteMotor_PWM < -1){
                    TesteMotor_PWM=-1;
                    TesteSentido=1;
                }
            }
            TesteIndice = 0;
        }
    #endif
}

void GPIO_M_IntHandler(void) // ENCODER ESQUERDO
{
    // Clear interrupt
    GPIOIntClear(GPIO_PORTM_BASE,GPIO_INT_PIN_0);

    HallEncoder_Esq = GPIOPinRead(GPIO_PORTM_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    /* Canais AB:
     * Se 00:
     *  -> Verdadeiro: incrementa
     *
     * Se 01:
     *  -> Falso: decrementa
     *
     * Se 10:
     *  -> Falso: decrementa
     *
     * Se 11:
     *  -> Verdadeiro: incrementa
     */
    if((GPIO_PIN_0 & HallEncoder_Esq) == (HallEncoder_Esq >> 1))
    {
        JubileuData.EncoderEsq += 1;
    }
    else
    {
        JubileuData.EncoderEsq -= 1;
    }
}

void GPIO_H_IntHandler(void) // ENCODER DIREITO
{
    // Clear interrupt
    GPIOIntClear(GPIO_PORTH_BASE,GPIO_INT_PIN_0);

    HallEncoder_Dir = GPIOPinRead(GPIO_PORTH_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    /* Canais AB:
    * Se 00:
    *  -> Verdadeiro: incrementa
    *
    * Se 01:
    *  -> Falso: decrementa
    *
    * Se 10:
    *  -> Falso: decrementa
    *
    * Se 11:
    *  -> Verdadeiro: incrementa
    */
    if((GPIO_PIN_0 & HallEncoder_Dir) == (HallEncoder_Dir >> 1))
    {
        JubileuData.EncoderDir -= 1; // Sentido contrário ao encoder Esquerdo (180 graus)
    }
    else
    {
        JubileuData.EncoderDir += 1; // Sentido contrário ao encoder Esquerdo (180 graus)
    }
}

void UART3IntHandler(void) // UART MÓDULO BLUETOOTH
{
    uint32_t ui32Status;
    ui32Status = UARTIntStatus(UART3_BASE, true); //get interrupt status
    UARTIntClear(UART3_BASE, ui32Status); //clear the asserted interrupts

    while(UARTCharsAvail(UART3_BASE)) //loop while there are chars
    {
        int32_t charAtual = UARTCharGet(UART3_BASE);
        #ifdef DEBUG_ACTIVE
            UARTCharPut(UART0_BASE, (unsigned char) charAtual);
        #endif

        if(charAtual != '\r' && charAtual != '\n')
        {
            StateMachineComm.MensagemUARTData.MensagemWifiRec[StateMachineComm.MensagemUARTData.MensagemWifiRecIdx] = charAtual;
            StateMachineComm.MensagemUARTData.MensagemWifiRecIdx++;
        }

        if(StateMachineComm.MensagemUARTData.LastChar == '\r' && charAtual == '\n')
        {
            charAtual = '0';
            Exec_SM_Comm(&StateMachineComm,EV_MENSAGEMRECEBIDA);
            StateMachineComm.MensagemUARTData.MensagemWifiRecIdx = 0;
        }

        StateMachineComm.MensagemUARTData.LastChar = charAtual;
    }
}

#ifdef DEBUG_ACTIVE
void UART0IntHandler(void) // UART PARA DEBUG
{
    uint32_t ui32Status;
    ui32Status = UARTIntStatus(UART0_BASE, true); //get interrupt status
    UARTIntClear(UART0_BASE, ui32Status); //clear the asserted interrupts

    while(UARTCharsAvail(UART0_BASE)) //loop while there are chars
    {
        //UARTCharPutNonBlocking(UART3_BASE, UARTCharGetNonBlocking(UART3_BASE)); //echo

        int32_t charAtual = UARTCharGet(UART0_BASE);
        UARTCharPut(UART3_BASE, (unsigned char)charAtual);
        //MensagemWifiRec[MensagemWifiIdx] = (uint8_t)teste;
        //MensagemWifiIdx++;
    }
    //MensagemWifiIdx = 0;
}
#endif

void ADC0SS1IntHandler(void) {
    /* Limpa Interrupção */
    ADCIntClear(ADC0_BASE, 1);
    /* Pega resultado da conversão */
    ADCSequenceDataGet(ADC0_BASE, 1, retornoADC.ui32ADC0Values);
    retornoADC.Sensor[0] = retornoADC.ui32ADC0Values[0];
    retornoADC.Sensor[1] = retornoADC.ui32ADC0Values[1];
    retornoADC.Sensor[2] = retornoADC.ui32ADC0Values[2];
    retornoADC.ADC0Calculado=1;
}

void ADC1SS1IntHandler(void) {
    /* Limpa Interrupção */
    ADCIntClear(ADC1_BASE, 1);
    /* Pega resultado da conversão */
    ADCSequenceDataGet(ADC1_BASE, 1, retornoADC.ui32ADC1Values);
    retornoADC.Sensor[3] = retornoADC.ui32ADC1Values[0];
    retornoADC.Sensor[4] = retornoADC.ui32ADC1Values[1];
    retornoADC.ADC1Calculado=1;
}
