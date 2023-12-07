/**
 * main.c
 */

#include "F28x_Project.h"
#include "math.h"
//constantes
#define TAMANHO 256
float Vetor_saida[TAMANHO], Vetor_saida2[TAMANHO];

__interrupt void cpu_timer0_isr (void);
void ConfigureTimer(void);
void ConfigureDAC(void);
void ConfigureADC(void);
float lowPassFilter(float input, float prevOutput, float prevEntrada);


//circuito de sincronismo baseado em freq
float f_V =0, f_angulo = 0, f_LPF1_in = 0, f_LPF2_in = 0, f_LPF1_out = 0, f_LPF2_out = 0;
float f_An = 0, f_Bn =0,f_V_linha =0, n =1, f_w1 = 2*3.14159*60;
float f_LPF1_in_anterior = 0, f_LPF2_in_anterior = 0;

// PLL estrutura elementar
float p_V = 0, p_Epd =0, p_V_linha = 0, Kpd =1, kvco = 1, p_Vtf = 0, p_w_linha = 0, p_wc = 2*3.14159*60,p_theta_linha = 0,p_integrador = 0;


// enchanced pll
float ep_V = 0, ep_saida_notch = 0, ep_X  = 0, ep_V_linha =0, ep_epsilon_pd =0, ep_Vf =0, ep_w_linha = 0, ep_theta_linha = 0,ep_integrador1 = 0, ep_integrador2 =0;
float  ep_wc = 2*3.14159*60, ep_K = 1;


//sogi
int16 V_teste;
float z = 0, k =1, V= 0, V_linha = 0, qV_linha = 0, inte_a = 0;
float w = 377;

// da transformada dq
float V_d = 0, V_q = 0, cos_angulo = 0, sin_angulo = 0;

//PLL
float int_srf = 0, omega = 0, angulo = 0, Kp = 2.97, Ki = 793;
int16 saida, V_linha_saida, qV_linha_saida;

//configura  o do timer
void ConfigureTimer(void){
    CpuTimer0Regs.PRD.all = 200000000*0.0001; //Freq do CPU * Period da interrup  o (em sec)
    CpuTimer0Regs.TCR.bit.TSS = 1;  // 1 = Stop timer, 0 = Start/Restart Timer
    CpuTimer0Regs.TCR.bit.TRB = 1;  // 1 = reload timer
    CpuTimer0Regs.TCR.bit.SOFT = 0;
    CpuTimer0Regs.TCR.bit.FREE = 0; // Timer Free Run Disabled
    CpuTimer0Regs.TCR.bit.TIE = 1;  // 0 = Disable/ 1 = Enable Timer Interrupt
}

//configurar o DAC
void ConfigureDAC(void){
    EALLOW;
    DacaRegs.DACCTL.bit.DACREFSEL = 1; // use adc references
    DacaRegs.DACOUTEN.bit.DACOUTEN = 1; // enable dac

    DacbRegs.DACCTL.bit.DACREFSEL = 1; // use adc references
    DacbRegs.DACOUTEN.bit.DACOUTEN = 1; // enable dac
    EDIS;
}

void ConfigureADC(void){
    EALLOW;
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; // set adcclk divider to /4
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6;

    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    AdcaRegs.ADCCTL1.bit.ADCPWDNZ =1;  //power up the adc
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2; // adcina2
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 2;  // ADCINB2


    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 19; //sample duration of 20 sysclk cycles
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 19;

    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 1; /// timer 0
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 5; // ePWM1 SOCA
    EDIS;
}

void main(void)
{

    // Step 1. Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    InitSysCtrl();

    // Step 2. Initialize GPIO:
    InitGpio();
    EALLOW;
    GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1; // output
    GpioCtrlRegs.GPBDIR.bit.GPIO59 = 0; // input
    GpioCtrlRegs.GPBPUD.bit.GPIO59 = 0; // Habilita Pull-up
    GpioCtrlRegs.GPBINV.bit.GPIO59 = 0; // N o inverter
    //GpioCtrlRegs.GPBCTRL.bit.QUALPRD3 = 0x00; // Qualification sampling
    //GpioCtrlRegs.GPBCSEL4.bit.GPIO61 = 3;
    EDIS;

    //configura  o
    ConfigureTimer();
    ConfigureDAC();
    ConfigureADC();


    // Step 3. Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    InitPieCtrl();
    IER=0;
    IFR=0;
    InitPieVectTable();


    CpuTimer0Regs.TCR.bit.TSS = 0;  // 1 = Stop timer, 0 = Start/Restart Timer

    EALLOW;
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    EDIS;


    //o dsp tem um mapa de interrup  es. escolhjer qual interrup  o sera acionada
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

    //interr  o global
    IER |= M_INT1;

    EINT;
    ERTM;

    while(1)
    {

    }
}

__interrupt void cpu_timer0_isr (void)
{
    static int InterruptCount=0 , resultsIndex = 0;
    static float tempo = 0, delta = 0.0001, k =1; //delt   o periodo de amostragem ou passo de integra  o

    InterruptCount++;

    GpioDataRegs.GPATOGGLE.bit.GPIO31=1;

    if(GpioDataRegs.GPBDAT.bit.GPIO59 == 1)
    {GpioDataRegs.GPBSET.bit.GPIO34=1;}

    if(GpioDataRegs.GPBDAT.bit.GPIO59 == 0)
    {GpioDataRegs.GPBCLEAR.bit.GPIO34=1;}

    //2048   proximo de 1.5V  para ver ele basta fazer DacbRegs.DACVALS.all
    DacbRegs.DACVALS.all = (Uint16)(2048 + 2000 * sin(2 * 3.141592 * 60 * tempo)); //registrador que receb o valor que o conversos da vai converter para analogico

   // Vetor_entrada[resultsIndex] = V; //registrador que recebe o valor da convers o


    //olhar o w em todos os caras

    // sogi
    V_teste = ( AdcaResultRegs.ADCRESULT0 - 2048 );
    V = (float)(V_teste)*0.087;    //calibrar a amplitude  para o valor real 180/2048

    //circuito de sincronismo baseado em freq
    f_V= V;  //sinal de
    f_angulo += n*f_w1*delta ; //angulo dos sinais de referencia conhecidos
    if(f_angulo < 0){
        f_angulo = 0;
    }
    if(f_angulo > 6.283185){
        f_angulo = f_angulo - 6.283185;
    }
    f_LPF1_in = cos(f_angulo) * f_V; // do passa baixas superior
    f_LPF2_in = sin(f_angulo) * f_V; // do passa baixas inferior
    f_LPF1_out = lowPassFilter(f_LPF1_in, f_LPF1_out, f_LPF1_in_anterior); //saida do passa baixas superior
    f_LPF2_out = lowPassFilter(f_LPF2_in, f_LPF2_out, f_LPF2_in_anterior); //saida do passa baixas inferior
    f_LPF1_in_anterior = f_LPF1_in; //armazenar  anterior
    f_LPF2_in_anterior = f_LPF2_in;//armazenar entrada anterior
    f_An = 2*   f_LPF1_out; //coenfiente An da serie de fourie normalizado
    f_Bn = 2*   f_LPF2_out; //coeficiente Bn da serie de fourie normalizado
    f_V_linha = f_An*cos(f_angulo) + f_Bn*sin(f_angulo); //saida do circuito sincronizado

    // PLL estrutura elementar
    p_V = V;
    p_Epd =  p_V * p_V_linha * Kpd;
    p_integrador +=  p_Epd* delta;
    p_Vtf =   p_Epd* Kpd + Ki* p_integrador;
    p_w_linha = p_Vtf*kvco + p_wc;
    p_theta_linha += p_w_linha *delta;
    if(p_theta_linha < 0){
        p_theta_linha = 0;
    }
    if(p_theta_linha > 6.283185){
        p_theta_linha = p_theta_linha - 6.283185;
    }
    p_V_linha = cos(p_theta_linha );


    //ePLL
    ep_V = V;
    ep_saida_notch =  ep_V - ep_V_linha;
    ep_X = -cos(ep_theta_linha);
    ep_integrador1 += ep_saida_notch* ep_X * ep_K *delta;
    ep_V_linha = ep_X *  ep_integrador1;
    ep_epsilon_pd = ep_saida_notch * sin(ep_theta_linha);
    ep_integrador2 += ep_epsilon_pd* delta;
    ep_Vf = ep_epsilon_pd*Kp + Ki *ep_integrador2;
    ep_w_linha = ep_Vf - ep_wc;
    ep_theta_linha += ep_w_linha * delta;
    if(ep_theta_linha > 0){
        ep_theta_linha = 0;
    }
    if(ep_theta_linha < -6.283185){
        ep_theta_linha = ep_theta_linha + 6.283185;
    }


    //express o do circuito
    z = k*(V-V_linha);
    V_linha += (z-qV_linha)*w*delta;
    inte_a += V_linha*delta;
    qV_linha= w*inte_a;

    //TRANFORMADA DQ
    cos_angulo = cos(angulo);
    sin_angulo = sin(angulo);
    V_d = (V_linha * cos_angulo) + (qV_linha * sin_angulo);
    V_q = - (V_linha * sin_angulo) + (qV_linha * cos_angulo);

    //LF e FPG
    int_srf += V_q * delta;
    omega = Kp * V_q + Ki*int_srf;
    angulo  += delta * omega;

    //limitar o angulo
    if(angulo < 0){
        angulo = 0;
    }
    if(angulo > 6.283185){
        angulo = angulo - 6.283185;
    }



    //visualizar dac
    saida = angulo *600; //atan((f_Bn)/f_An) * 10; // f_V_linha*300 + 2048;
    DacaRegs.DACVALS.all = (Uint16)(saida);

    Vetor_saida2[resultsIndex] = f_An;
    Vetor_saida[resultsIndex++] = saida;

    if(TAMANHO <= resultsIndex){
        resultsIndex = 0;
    }

    //resetar tudo(libera as flags de interrup  o)
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

    tempo += delta;
}


float lowPassFilter(float input, float prevOutput, float prevEntrada) {
    float B0 = 0.01850082, B1 = 0.01850082, A1 = 0.96299835;

    return (B0 * input + B1 * prevEntrada - A1 * prevOutput);
}




