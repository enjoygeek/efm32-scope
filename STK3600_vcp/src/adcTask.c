/*
 * adcTask.c
 *
 *  Created on: Feb 2, 2016
 *      Author: corviniol
 */

#include "adcTask.h"
#include <stdio.h>

#include "em_cmu.h"
#include "em_adc.h"
#include "em_dac.h"
#include "em_prs.h"
#include "em_timer.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

static SemaphoreHandle_t semADC = NULL;
void ADC0_IRQHandler()
{
    ADC_IntClear(ADC0, ADC_IFC_SCAN);
    BaseType_t woken;
    xSemaphoreGiveFromISR(semADC, &woken);
    portYIELD_FROM_ISR(woken);
}

static void vConfigureTimer(unsigned uSampleRate)
{
    unsigned hfperclks = CMU_ClockFreqGet(cmuClock_TIMER3) / uSampleRate;

    CMU_ClockEnable(cmuClock_TIMER3, true);
    TIMER_Init_TypeDef timer_init = TIMER_INIT_DEFAULT;
    timer_init.prescale = 31 - __builtin_clz(hfperclks >> 16);
    TIMER_TopSet(TIMER3, (CMU_ClockFreqGet(cmuClock_TIMER3) >> timer_init.prescale) / uSampleRate);
    TIMER_Init(TIMER3, &timer_init);
}

static void vConfigurePrs(unsigned uCh)
{
    CMU_ClockEnable(cmuClock_PRS, true);
    PRS_LevelSet(0, 1 << (uCh + _PRS_SWLEVEL_CH0LEVEL_SHIFT));
    PRS_SourceSignalSet(uCh, PRS_CH_CTRL_SOURCESEL_TIMER3, PRS_CH_CTRL_SIGSEL_TIMER0OF, prsEdgePos);
}

static int iConvertAdcReading(int reading, const ADC_InitScan_TypeDef *conf, const int Vdd_mV)
{
    int mult;

    switch (conf->reference)
    {
    case adcRef1V25:
        mult = 1250;
        break;

    case adcRef2V5:
        mult = 2500;
        break;

    case adcRefVDD:
        mult = Vdd_mV;
        break;

    case adcRef2xVDD:
        mult = 2 * Vdd_mV;
        break;

    case adcRef5VDIFF:
        mult = 5000;
        break;

    default:
        printf("External references unsupported\n");
        return 0;
    }

    return (reading * mult) >> 16;
}

void vAdcTask(void *pParameters)
{
    AdcTaskParams_t *pData = pParameters;

    semADC = xSemaphoreCreateBinary();

    CMU_ClockEnable(cmuClock_ADC0, true);
    ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
    init.warmUpMode = adcWarmupKeepADCWarm;
    init.timebase = 0;
    init.prescale = ADC_PrescaleCalc(32000, 0);
    ADC_Init(ADC0, &init);

    NVIC_EnableIRQ(ADC0_IRQn);
    ADC_IntEnable(ADC0, ADC_IF_SCAN);


    int channelsCount = __builtin_popcount(pData->adcChannelsMask);
    ADC_InitScan_TypeDef init_scan =
    {
        .reference = adcRef2V5,
        .input = pData->adcChannelsMask << _ADC_SCANCTRL_INPUTMASK_SHIFT,
        .resolution = adcRes12Bit,
        .acqTime = adcAcqTime32,
        .rep = false,
        .leftAdjust = true,
        .prsEnable = true,
        .prsSel = adcPRSSELCh0 + pData->uPrsChannel
    };
    ADC_InitScan(ADC0, &init_scan);

    vConfigureTimer(pData->uSampleRate);

    vConfigurePrs(pData->uPrsChannel);
    int n = 0;
    int max_n = 31 - __builtin_clz(pData->adcChannelsMask);

    char outstr[64];
    int outIdx = 0;

    while (n != max_n)
    {
        xSemaphoreTake(semADC, portMAX_DELAY);
        n = (ADC0->STATUS & _ADC_STATUS_SCANDATASRC_MASK) >> _ADC_STATUS_SCANDATASRC_SHIFT;
    }

    for (;;)
    {
        BSP_LedToggle(0);
        outIdx = 0;

        for (int chIdx = 0; chIdx < channelsCount; chIdx++)
        {
            xSemaphoreTake(semADC, portMAX_DELAY);
            n = (ADC0->STATUS & _ADC_STATUS_SCANDATASRC_MASK) >> _ADC_STATUS_SCANDATASRC_SHIFT;
            int res = ADC_DataScanGet(ADC0);
            outIdx += sprintf(&outstr[outIdx], "%d: %dmV ", n, iConvertAdcReading(res, &init_scan, 3300));
        }

        puts(outstr);
//      printf("Minimum stack free: %lubytes\n", uxTaskGetStackHighWaterMark(NULL)*4);
    }
}

void vDacTask(void *pParameters)
{
    CMU_ClockEnable(cmuClock_DAC0, true);
    DAC_Init_TypeDef init = DAC_INIT_DEFAULT;
    DAC_InitChannel_TypeDef channel = DAC_INITCHANNEL_DEFAULT;

    init.reference = dacRefVDD;
    init.outMode = dacOutputPin;
    init.prescale = DAC_PrescaleCalc(10000, 0);

    DAC_Init(DAC0, &init);

    channel.refreshEnable = true;
    DAC_InitChannel(DAC0, &channel, 0);
    DAC_InitChannel(DAC0, &channel, 1);

    DAC_Enable(DAC0, 0, true);
    DAC_Enable(DAC0, 1, true);
    DAC_Channel0OutputSet(DAC0, 100);
    DAC_Channel1OutputSet(DAC0, 1000);
}


