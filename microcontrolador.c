//
// Included Files
//

#include "F2837xD_device.h"
#include "driverlib.h"
#include "device.h"
#include "stdlib.h"
#include <string.h>

//
// Globals
//
#define N_taps 21
#define N_taps_half 11
#define startMarker 60 //<
#define endMarker 62 //>
#define baudRate 19200
#define maxBufferSize 300
#define separado_filtro ","
#define separado_elemento ";"

//vetores dos coeficientes
static float LP_Coef[N_taps], FP_Coef[N_taps], HP_Coef[N_taps];
static int LP_ganho, FP_ganho, HP_ganho;
static int newData = 0, recebendoDado = 0;
static float buffer_entrada[N_taps] = {0,0,0,0,0}, saida = 0;
//
// Send data for SCI-A
//
char sDataA[maxBufferSize];

char caracter;
char *filtro;
//variaveis desejada
uint16_t bufferIndex = 0;
uint16_t receivedChar;
unsigned char *msg;
int16 entrada = 0;

//
// Function Prototypes
//
__interrupt void sciaTXFIFOISR(void);
__interrupt void sciaRXFIFOISR(void);
__interrupt void cpu_timer1_isr (void);
void initSCIAFIFO(void);
void error(void);
void Desformatar(void);


//configura  o do timer
void ConfigureTimer(void){
    CpuTimer1Regs.PRD.all = 20;//200000000*0.000002; //Freq do CPU * Period da interrup  o (em sec)
    CpuTimer1Regs.TCR.bit.TSS = 1;  // 1 = Stop timer, 0 = Start/Restart Timer
    CpuTimer1Regs.TCR.bit.TRB = 1;  // 1 = reload timer
    CpuTimer1Regs.TCR.bit.SOFT = 0;
    CpuTimer1Regs.TCR.bit.FREE = 0; // Timer Free Run Disabled
    CpuTimer1Regs.TCR.bit.TIE = 1;  // 0 = Disable/ 1 = Enable Timer Interrupt
}

void ConfigureDAC(void){
    EALLOW;
    DacaRegs.DACCTL.bit.DACREFSEL = 1; // use adc references
    DacaRegs.DACOUTEN.bit.DACOUTEN = 1; // enable dac
    EDIS;
}


void ConfigureADC(uint32_t adcBase){
    //AdcaRegs.ADCCTL2.bit.PRESCALE = 6; // set adcclk divider to /4
    ADC_setPrescaler(adcBase, ADC_CLK_DIV_4_0);
    //AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    //ADC_setMode(0x00007400U, 0x00U, 0x00U);
    ADC_setMode(adcBase, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);

    EALLOW;
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ =1;  //power up the adc
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2; // adcina2

    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 19; //sample duration of 20 sysclk cycles

    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 2; /// timer0 = 1; time1 = 2

    EDIS;
}




void main(void)
{

    //
    // Initialize device clock and peripherals
    //
    Device_init();

    //
    // Setup GPIO by disabling pin locks and enabling pullups
    //
    Device_initGPIO();

    ConfigureTimer();
    ConfigureDAC();
    ConfigureADC(ADCA_BASE);

    //
    // Configuration for the SCI Rx pin.
    //
    GPIO_setMasterCore(DEVICE_GPIO_PIN_SCIRXDA, GPIO_CORE_CPU1);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_SCIRXDA);
    GPIO_setDirectionMode(DEVICE_GPIO_PIN_SCIRXDA, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_SCIRXDA, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(DEVICE_GPIO_PIN_SCIRXDA, GPIO_QUAL_ASYNC);

    //
    // Configuration for the SCI Tx pin.
    //
    GPIO_setMasterCore(DEVICE_GPIO_PIN_SCITXDA, GPIO_CORE_CPU1);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_SCITXDA);
    GPIO_setDirectionMode(DEVICE_GPIO_PIN_SCITXDA, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_SCITXDA, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(DEVICE_GPIO_PIN_SCITXDA, GPIO_QUAL_ASYNC);

    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    entrada = 2;
    Interrupt_initVectorTable();

    //
    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this file.
    //
    Interrupt_register(INT_SCIA_RX, sciaRXFIFOISR);
    Interrupt_register(INT_SCIA_TX, sciaTXFIFOISR);

    Interrupt_register(INT_TIMER1, &cpu_timer1_isr);
    entrada = 1;
    //
    // Initialize the Device Peripherals:
    //
    initSCIAFIFO();
    //SCI_performSoftwareReset(SCIA_BASE);

   //------------------------------------------------------------

    Interrupt_enable(INT_SCIA_RX);
    //Interrupt_enable(INT_SCIA_TX);

    Interrupt_enable(INT_TIMER1);

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
    //Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
    CPUTimer_startTimer(CPUTIMER1_BASE);
    // Coisas do adc e dac
    DINT;
    //InitPieCtrl();

    //InitPieVectTable();
    //CpuTimer0Regs.TCR.bit.TSS = 0;  // 1 = Stop timer, 0 = Start/Restart Timer
    //EALLOW;
    //PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    //EDIS;
    //o dsp tem um mapa de interrup  es. escolhjer qual interrup  o sera acionada
    //PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    //PieCtrlRegs.PIEIER1.bit.INTx5 = 7;

    //interr  o global
    //IER |= M_INT1;
    //IER |= M_INT9;

    //
    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    //
    EINT;
    ERTM;

    //
    // IDLE loop. Just sit and loop forever (optional):
    //
    for(;;);
}

//
// error - Function to halt debugger on error
//
void error(void)
{
    Example_Fail = 1;
    asm("     ESTOP0"); // Test failed!! Stop!
    for (;;);
}

__interrupt void cpu_timer1_isr (void)
{
    int i;
    float LP_saida = 0, FP_saida = 0, HP_saida = 0;

    IER |= M_INT13;
    IER |= M_INT9;

    //Coletar dados
    //entrada = ( AdcaResultRegs.ADCRESULT0  );
    entrada = ADC_readResult(ADCARESULT_BASE,
                             ADC_SOC_NUMBER0) -2048;

    //armazenar os dados

    for (i = N_taps - 1; i > 0; i--) {
        buffer_entrada[i] = buffer_entrada[i - 1];
    }
    buffer_entrada[0] = entrada;

    // convoluir
    for(i=0;i<N_taps;i++)
    {
        LP_saida += LP_Coef[i] * buffer_entrada[i];
        FP_saida += FP_Coef[i] * buffer_entrada[i];
        HP_saida += HP_Coef[i] * buffer_entrada[i];
    }
    //Enviar sinal filtrado
    saida = (LP_saida * LP_ganho + FP_saida * FP_ganho + HP_saida * HP_ganho)/2048 * 7 + 2048;
    DacaRegs.DACVALS.all =  (Uint16)(saida);


}


void Desformatar (){
// Step 1: Separate filters
    char* filterTokens[N_taps_half];
    int filterCount = 0, i, j;
    char* token = strtok(sDataA, ",");
    while (token != NULL) {
        filterTokens[filterCount++] = token;
        token = strtok(NULL, ",");
    }

    // Step 2: Separate filter elements
    for (i = 0; i < N_taps_half; ++i) {
        char* elements[N_taps_half + 1];
        int elementCount = 0;

        token = strtok(filterTokens[i], ";");
        while (token != NULL) {
            elements[elementCount++] = token;
            token = strtok(NULL, ";");
        }

        if (elementCount == N_taps_half + 1) {
            int ganho = atoi(elements[0]);
            float Coef[N_taps_half];

            for (j = 1; j <= N_taps_half; ++j) {
                Coef[j - 1] = atof(elements[j]);
            }

            // Step 3: alocar dados
            if (i == 0) {
                LP_ganho = ganho;
                for (j = 0; j < N_taps_half; ++j) {
                    LP_Coef[j] = Coef[j];
                    LP_Coef[N_taps -1 - j] = Coef[j];
                }
            } else if (i == 1) {
                FP_ganho = ganho;
                for (j = 0; j < N_taps_half; ++j) {
                    FP_Coef[j] = Coef[j];
                    FP_Coef[N_taps -1 - j] = Coef[j];
                }
            } else if (i == 2) {
                HP_ganho = ganho;
                for (j = 0; j < N_taps_half; ++j) {
                    HP_Coef[j] = Coef[j];
                    HP_Coef[N_taps -1 - j] = Coef[j];
                }
            }
        }
    }
}

__interrupt void sciaRXFIFOISR(void)
{
    IER |= M_INT9;
    IER &= M_INT9;

    receivedChar = SCI_readCharBlockingFIFO(SCIA_BASE);
    if(recebendoDado == 1 ){
        if(receivedChar != (uint16_t)endMarker ){
            sDataA[bufferIndex] = (char)receivedChar;
            bufferIndex++;
            // Check if buffer is full to avoid overflow
            if (bufferIndex >= maxBufferSize - 1)
            {
                sDataA[maxBufferSize - 1] = '\0';  // Null-terminate the buffer
            }
        }
        else {
            DINT;
            Desformatar();
            sDataA[bufferIndex] = '\0'; // terminate the string
            recebendoDado = 0;
            bufferIndex = 0;
            newData = 1;
            EINT;
        }
    }
    else if(receivedChar == (uint16_t)startMarker){
        recebendoDado = 1;
    }


    SCI_clearOverflowStatus(SCIA_BASE);

    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_RXFF);

    //
    // Issue PIE ack
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);

}

//
// sciaTXFIFOISR - SCIA Transmit FIFO ISR
//
__interrupt void sciaTXFIFOISR(void)
{


    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_TXFF);

    //
    // Issue PIE ACK
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}

//
// initSCIAFIFO - Configure SCIA FIFO
//
void initSCIAFIFO()
{

    SCI_performSoftwareReset(SCIA_BASE);

    //
    // 8 char bits, 1 stop bit, no parity. Baud rate is 9600.
    //
    SCI_setConfig(SCIA_BASE, DEVICE_LSPCLK_FREQ, baudRate, (SCI_CONFIG_WLEN_8 |
                                                        SCI_CONFIG_STOP_ONE |
                                                        SCI_CONFIG_PAR_NONE));
    SCI_enableModule(SCIA_BASE);
    //SCI_enableLoopback(SCIA_BASE);
    SCI_resetChannels(SCIA_BASE);
    SCI_enableFIFO(SCIA_BASE);

    //
    // RX and TX FIFO Interrupts Enabled
    //
    SCI_enableInterrupt(SCIA_BASE, (SCI_INT_RXFF | SCI_INT_TXFF));
    SCI_disableInterrupt(SCIA_BASE, SCI_INT_RXERR);

    SCI_setFIFOInterruptLevel(SCIA_BASE, SCI_FIFO_TX2, SCI_FIFO_RX2);
    SCI_performSoftwareReset(SCIA_BASE);

    SCI_resetTxFIFO(SCIA_BASE);
    SCI_resetRxFIFO(SCIA_BASE);

}
