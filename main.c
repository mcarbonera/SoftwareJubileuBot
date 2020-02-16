#ifndef MAIN_H
    #include "main.h"
#endif

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

//#include "utils/uartstdio.h"

#define PWM_FREQUENCY 100
#define APP_PI 3.1415926535897932384626433832795f
#define STEPS 256
#define Jubileu_R 0.034
#define Jubileu_L 0.18
#define Jubileu_TicksPerRevolution 341.2
#define DistPerTick (2*APP_PI*Jubileu_R)/Jubileu_TicksPerRevolution

void InicializaStructRobo(t_sm *sm);
int IsNumber(char caractere);
float CharAsFloat(char caractere);
char IntAsChar(int num);
void UpdateOdometry();
void CoordenadaParaString();
void AcionarMotor(float Porcentagem_PWM_Esq, float Porcentagem_PWM_Dir);

typedef struct JD {
    float ObjX, ObjY;
    float CoordX, CoordY, CoordTheta;
    int Parar;
    int EncoderEsq, EncoderDir;
    char CoordString[19]; //00.00,00.00,00.00\r\n
} JD;

JD JubileuData;
t_sm StateMachineComm;

uint8_t ui8PinData=1;
uint8_t HallEncoder_Esq=0;
uint8_t HallEncoder_Dir=0;
//int EncoderEsq = 0, EncoderDir = 0;

uint32_t ui32SysClkFreq;
int FrequenciaTimer = 10; // 10 Hz

#ifdef TESTE_MOTOR_X_ENCODER
    float TesteMotor_PWM = -1;
    int TesteSentido = 1;
    int TesteIndice = 0;
#endif

int main(void)
{
    int i;
    uint32_t ui32Period;
    //uint32_t ui32SysClkFreq;

    uint32_t ui32ADC0Values[4];
    uint32_t ui32ADC1Values[4];
    volatile uint32_t Sensor[5];
    volatile double DistanciaSensor[5];
    double temp;
    const double a = 2.7802212625047;
    const double b = -35.1150300110164;
    const double c = 179.6031433005229;
    const double d = -477.9449116299037;
    const double e = 706.3400747125301;
    const double f = -569.7367375002304;
    const double g = 221.2678651473199;

    volatile uint32_t ui32Load; // PWM period
    volatile uint32_t ui32BlueLevel; // PWM duty cycle for blue LED
    volatile uint32_t ui32PWMClock; // PWM clock frequency
    volatile uint32_t ui32Index; // Counts the calculation loops
    float fAngle; // Value for sine math (radians)

    int MotEsqRot = 0, MotDirRot = 0;

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
    ui32Period = ui32SysClkFreq/FrequenciaTimer;
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
        //UARTConfigSetExpClk(UART0_BASE, ui32SysClkFreq, 38400, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE)); //38400
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

    // Habilitar Timer:
    TimerEnable(TIMER0_BASE, TIMER_A);

    // Habilitar Sequencer 1 dos ADCs 0 e 1:
    ADCSequenceEnable(ADC0_BASE, 1);
    ADCSequenceEnable(ADC1_BASE, 1);
    //GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0|GPIO_PIN_1, 0x00);

    // Habilitar PWM
    PWMGenEnable(PWM0_BASE, PWM_GEN_2);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);

    ui32Index = 0;
    while(1)
    {
        // Resetar a flag de interrupção:
        ADCIntClear(ADC0_BASE, 1);
        ADCIntClear(ADC1_BASE, 1);

        // Acionar conversão no ADC por software:
        ADCProcessorTrigger(ADC0_BASE, 1);
        ADCProcessorTrigger(ADC1_BASE, 1);

        // Esperar conversão ser realizada:
        //while(!ADCIntStatus(ADC0_BASE, 1, false)); // esta espera
        while(!ADCIntStatus(ADC0_BASE, 1, false) && !ADCIntStatus(ADC1_BASE, 1, false));

        // Pegar dados:
        ADCSequenceDataGet(ADC0_BASE, 1, ui32ADC0Values);
        ADCSequenceDataGet(ADC1_BASE, 1, ui32ADC1Values);

        Sensor[0] = ui32ADC0Values[0];
        Sensor[1] = ui32ADC0Values[1];
        Sensor[2] = ui32ADC0Values[2];
        Sensor[3] = ui32ADC1Values[0];
        Sensor[4] = ui32ADC1Values[1];

        for(i=0; i<5; i++)
        {
            temp = Sensor[i]*0.0008058608059;
            //DistanciaSensor[i] = a*pow(temp,6) + b*pow(temp,5) + c*pow(temp,4) + d*pow(temp,3) + e*pow(temp,2) + f*temp + g;
            DistanciaSensor[i] = a*temp*temp*temp*temp*temp*temp + b*temp*temp*temp*temp*temp + c*temp*temp*temp*temp + d*temp*temp*temp + e*temp*temp + f*temp + g;
        }

        //dist0 = 1/dist0;
        /*
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1, ui8PinData);
        SysCtlDelay(4000000);
        if(ui8PinData==4)
        {
            ui8PinData=1;
        }
        else
        {
            ui8PinData=ui8PinData*2;
        }

        fAngle = ui32Index * (2.0f * APP_PI/STEPS) + 3*APP_PI/2;
        if(MotEsqRot)
        {
            ui32BlueLevel = (uint32_t) (9370.0f * (1 + sinf(fAngle)));
        }
        else
        {
            ui32BlueLevel = (uint32_t) (9370.0f * (1 - sinf(fAngle)));
        }
        //ui32BlueLevel = (uint32_t) (9370.0f * (1 + sinf(fAngle)));
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, ui32BlueLevel + 1); // Motor Esquerdo
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, ui32BlueLevel + 1); // Motor Direito
        if(MotEsqRot)
        {
            // Esq
            PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT, false);
            PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, true);
            // Dir
            PWMOutputState(PWM0_BASE, PWM_OUT_3_BIT, false);
            PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
        }
        else
        {
            // Esq
            PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, false);
            PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT, true);
            // Dir
            PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, false);
            PWMOutputState(PWM0_BASE, PWM_OUT_3_BIT, true);
        }
        ui32Index++;
        if (ui32Index == (STEPS))
        {
            ui32Index = 0;
            // Esq
            PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, false);
            PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT, false);
            // Dir
            PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, false);
            PWMOutputState(PWM0_BASE, PWM_OUT_3_BIT, false);
            if(MotEsqRot)
            {
                MotEsqRot = 0;
            }
            else
            {
                MotEsqRot = 1;
            }
        }
        */
        SysCtlDelay(ui32SysClkFreq/(STEPS));
    }
}

/* FUNÇÕES - UART, comunicação, inicialização*/
void InicializaStructRobo(t_sm *sm)
{
    JubileuData.Parar=1;
    JubileuData.EncoderEsq = 0;
    JubileuData.EncoderDir = 0;
    JubileuData.ObjX = 0;
    JubileuData.ObjY = 0;
    JubileuData.CoordX = 0;
    JubileuData.CoordY = 0;
    JubileuData.CoordString[0] = '0';
    JubileuData.CoordString[1] = '0';
    JubileuData.CoordString[2] = '.';
    JubileuData.CoordString[3] = '0';
    JubileuData.CoordString[4] = '0';
    JubileuData.CoordString[5] = ',';
    JubileuData.CoordString[6] = '0';
    JubileuData.CoordString[7] = '0';
    JubileuData.CoordString[8] = '.';
    JubileuData.CoordString[9] = '0';
    JubileuData.CoordString[10] = '0';
    JubileuData.CoordString[11] = ',';
    JubileuData.CoordString[12] = '0';
    JubileuData.CoordString[13] = '0';
    JubileuData.CoordString[14] = '.';
    JubileuData.CoordString[15] = '0';
    JubileuData.CoordString[16] = '0';
    JubileuData.CoordString[17] = '\r';
    JubileuData.CoordString[18] = '\n';

    Init_SM(sm,1);

    // Leds
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0|GPIO_PIN_1, 0xFF); // LEDs on
    SysCtlDelay(ui32SysClkFreq / (1)); // delay 1 sec
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0|GPIO_PIN_1, 0); // LEDs off

    sm->MensagemUARTData.MensagemWifiEnvIdx = 0;
    sm->MensagemUARTData.MensagemWifiRecIdx = 0;

    MontaMensagem(sm, "Bluetooth Inicializado.\r\n");
    EnviarMensagemUart(sm);
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

    float d_left = Left_Ticks*DistPerTick;
    float d_right = Right_Ticks*DistPerTick;
    float d_center = (d_left + d_right)/2;
    float d_Phi = (d_right - d_left)/Jubileu_L;

    JubileuData.CoordX += d_center*cosf(JubileuData.CoordTheta);
    JubileuData.CoordY += d_center*sinf(JubileuData.CoordTheta);
    JubileuData.CoordTheta += d_Phi;

    /* Ticks são pertidos após UpdateOdometry()
     * Portanto, em modo teste, deve-se preparar
     * String aqui.
     * */
    #ifdef TESTE_MOTOR_X_ENCODER
        CoordenadaParaString();
    #endif
    JubileuData.EncoderEsq -= Left_Ticks;
    JubileuData.EncoderDir -= Right_Ticks;

    /* FORÇAR CoordTheta para o intervalo [-PI,PI] */
    if(JubileuData.CoordTheta > APP_PI){
        JubileuData.CoordTheta-=APP_PI;
    } else if(JubileuData.CoordTheta < -APP_PI){
        JubileuData.CoordTheta+=APP_PI;
    }
}

void CoordenadaParaString()
{
    #ifdef TESTE_MOTOR_X_ENCODER
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
            JubileuData.CoordString[i+1]    = IntAsChar(TempEncEsq%10);     //  1,  3,  4
            JubileuData.CoordString[i+7]    = IntAsChar(TempEncDir%10);     //  7,  9, 10

            TempEncEsq = (TempEncEsq - TempEncEsq%10)/10;
            TempEncDir = (TempEncDir - TempEncDir%10)/10;
        }
    #else
        int i,j;
        float TempX, TempY, TempTheta;

        TempX = JubileuData.CoordX;
        TempY = JubileuData.CoordY;
        TempTheta = JubileuData.CoordTheta;

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

        /* Converter valores para string */
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
        if(fabsf(JubileuData.CoordX) >= 10){
            JubileuData.CoordString[0] = 'O';
            JubileuData.CoordString[1] = 'F';
        }
        if(fabsf(JubileuData.CoordY) >= 10){
            JubileuData.CoordString[6] = 'O';
            JubileuData.CoordString[7] = 'F';
        }
    #endif

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

    if(Porcentagem_PWM_Dir < 0){    /* Para tras */
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
}

/* INTERRUPÇÕES */
void Timer0IntHandler(void) // TIMER
{
    // Clear the timer interrupt
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    /* Cálculo de Odometria*/
    UpdateOdometry();

    /* Verificar necessidade de responder cliente (bluetooth): */
    if(StateMachineComm.state==ST_RESPONDECLIENTE_GETSTATE)
    {
        /* Atualizar String para envio */
        #ifndef TESTE_MOTOR_X_ENCODER /* Se definido, conversão será realizada em UpdateOdometry*/
            CoordenadaParaString();
        #endif
        MontaMensagem(&StateMachineComm, JubileuData.CoordString);
        Exec_SM(&StateMachineComm,EV_ENVIARMENSAGEM);
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
            JubileuData.ObjX = CharAsFloat(StateMachineComm.MensagemUARTData.MensagemWifiRec[12]) +
                    CharAsFloat(StateMachineComm.MensagemUARTData.MensagemWifiRec[14])/10 +
                    CharAsFloat(StateMachineComm.MensagemUARTData.MensagemWifiRec[15])/100;

            JubileuData.ObjY = CharAsFloat(StateMachineComm.MensagemUARTData.MensagemWifiRec[17]) +
                    CharAsFloat(StateMachineComm.MensagemUARTData.MensagemWifiRec[19])/10 +
                    CharAsFloat(StateMachineComm.MensagemUARTData.MensagemWifiRec[20])/100;
        }

        Exec_SM(&StateMachineComm,EV_ENVIARMENSAGEM);
    }

    //****************************************************
    /* ACIONAR MOTOR */
    #ifdef TESTE_MOTOR_X_ENCODER
        TesteIndice++;
        if(TesteIndice == 5)
        {
            AcionarMotor(TesteMotor_PWM,TesteMotor_PWM);
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

    // Read the current state of the GPIO pin and
    // write back the opposite state
    if(GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_1))
    {
        //GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, 0);
    }
    else
    {
        //GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, 2);
    }
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
            Exec_SM(&StateMachineComm,EV_MENSAGEMRECEBIDA);
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
