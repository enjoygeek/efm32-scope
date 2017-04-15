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
#include "em_dma.h"
#include "dmadrv.h"
#include "../Drivers/segmentlcd.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

typedef struct
{
    uint8_t validChannelsMask;
    uint8_t validChannelsCount;
    int Vdd_mV;
    const ADC_InitScan_TypeDef* conf;
    int readings[8];
} AdcScanResults_t __attribute__((aligned(16)));

void* pvResultFormater(const AdcScanResults_t *results, void* dst);


static SemaphoreHandle_t semADC = NULL;
static AdcScanResults_t scan_results;
static ADC_InitScan_TypeDef init_scan =
{
    .reference = adcRef2V5,
    .input = 0,
    .resolution = adcRes12Bit,
    .acqTime = adcAcqTime4,
    .rep = false,
    .leftAdjust = true,
    .prsEnable = true,
    .prsSel = 0
};

bool synchronized = false;
bool dma = true;


void ADC0_IRQHandler()
{
    ADC_IntClear(ADC0, ADC_IFC_SCAN);
    BaseType_t woken;
    if (!dma || !synchronized)
        xSemaphoreGiveFromISR(semADC, &woken);
    portYIELD_FROM_ISR(woken);
    BSP_LedToggle(0);
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

static void vConfigureADC(const AdcTaskParams_t *pData)
{
    CMU_ClockEnable(cmuClock_ADC0, true);
    ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
    init.warmUpMode = adcWarmupKeepADCWarm;
    init.timebase = 0;
    init.prescale = ADC_PrescaleCalc(32000, 0);
    ADC_Init(ADC0, &init);

    NVIC_EnableIRQ(ADC0_IRQn);
    ADC_IntEnable(ADC0, ADC_IF_SCAN);

    int channelsCount = __builtin_popcount(pData->adcChannelsMask);
    init_scan.input = pData->adcChannelsMask << _ADC_SCANCTRL_INPUTMASK_SHIFT;
    init_scan.prsSel = adcPRSSELCh0 + pData->uPrsChannel;

    ADC_InitScan(ADC0, &init_scan);
    scan_results.conf = &init_scan;
    scan_results.validChannelsMask = pData->adcChannelsMask;
    scan_results.validChannelsCount = channelsCount;
    scan_results.Vdd_mV = 3300;
};

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

void vUnpackMask(int mask, int* indices)
{
    for (int bitIdx = 0; bitIdx < 31; bitIdx++)
    {
        if (mask & (1 << bitIdx))
        {
            *indices++ = bitIdx;
        }
    }
}

void* pvResultFormater(const AdcScanResults_t *results, void* dst)
{
    int outIdx = 0;
    char *buf = dst;

    static int channelsNumbers[8];
    static int prevChannelsMask = 0;

    if (results->validChannelsMask != prevChannelsMask)
    {
        prevChannelsMask = results->validChannelsMask;
        vUnpackMask(results->validChannelsMask, channelsNumbers);
    }

    for (int chIdx = 0; chIdx < results->validChannelsCount; chIdx++)
    {
        outIdx += sprintf(&buf[outIdx], "%d: %dmV ", channelsNumbers[chIdx], iConvertAdcReading(results->readings[chIdx], results->conf, results->Vdd_mV));
    }
    return dst;
}



static unsigned uDmaChannel;

static bool symbol_gecko = false;

bool vDmaCallback(unsigned int channel, unsigned int seqNumber, void *userParam)
{
//	SegmentLCD_LowerNumber(seqNumber*10 + channel);

    BaseType_t woken;
    xSemaphoreGiveFromISR(semADC, &woken);
    portYIELD_FROM_ISR(woken);
//    SegmentLCD_Symbol(LCD_SYMBOL_GECKO, symbol_gecko);
//    symbol_gecko = !symbol_gecko;
    BSP_LedToggle(1);
	return true;
}

void vConfigureDma()
{
    CMU_ClockEnable(cmuClock_DMA, true);
	DMADRV_Init();
	DMADRV_AllocateChannel(&uDmaChannel, NULL);
}


extern bool portOpen;
void vAdcTask(void *pParameters)
{
    while (!portOpen)
    {
    	vTaskDelay(100);
    }
//    vTaskDelay(20000);
//	puts(__FUNCTION__);
//    vTaskDelay(20000);
	puts(__FUNCTION__);
	printf("staring %s\n", __FUNCTION__);
    AdcTaskParams_t *pData = pParameters;

    semADC = xSemaphoreCreateBinary();


//    vTaskDelay(10000);

    vConfigureDma();
    printf("DMA Channel %d allocated\n", uDmaChannel);
    SegmentLCD_LowerNumber(NVIC_GetPriority(DMA_IRQn));
    vTaskDelay(1000);


    vConfigureADC(pData);
    vConfigureTimer(pData->uSampleRate);
    vConfigurePrs(pData->uPrsChannel);

    for (;;)
    {
        if (!synchronized)
        {
//            int n = 0;
//            int max_n = 31 - __builtin_clz(scan_results.validChannelsMask);
//            while (n != max_n)
//            {
//                xSemaphoreTake(semADC, portMAX_DELAY);
//                n = (ADC0->STATUS & _ADC_STATUS_SCANDATASRC_MASK) >> _ADC_STATUS_SCANDATASRC_SHIFT;
//            }
            synchronized = true;

            printf("synchronized...\n");
            if (dma)
            {
                xSemaphoreTake(semADC, 10);
                xSemaphoreTake(semADC, 10);
                printf("Disabling ADC irq");
                ADC_IntDisable(ADC0, ADC_IF_SCAN);
                NVIC_DisableIRQ(ADC0_IRQn);

            }
        }



        char channels[4] = {0};
        if (dma)
        {
//            bool status = false;
//            DMADRV_TransferActive(uDmaChannel, &status);
//            if (!status)
            {
                Ecode_t ret = DMADRV_PeripheralMemory(uDmaChannel, DMAREQ_ADC0_SCAN, &scan_results.readings[0], &(ADC0->SCANDATA),
                                        true, scan_results.validChannelsCount, dmaDataSize4, vDmaCallback, NULL);

                xSemaphoreTake(semADC, portMAX_DELAY);
            }
        }
        else
        {
            for (int chIdx = 0; chIdx < scan_results.validChannelsCount; chIdx++)
            {
                xSemaphoreTake(semADC, portMAX_DELAY);
                scan_results.readings[chIdx] = ADC_DataScanGet(ADC0);
                channels[chIdx] = '0' + ((ADC0->STATUS & _ADC_STATUS_SCANDATASRC_MASK) >> _ADC_STATUS_SCANDATASRC_SHIFT);
            }
        }

        if (synchronized)
        {
            char outstr[64];
            pvResultFormater(&scan_results, outstr);
            puts(outstr);
        }

//        SegmentLCD_Number(uxTaskGetStackHighWaterMark(NULL));
//        SegmentLCD_Write(channels);
    }
}

void vDacTask(void *pParameters)
{
    CMU_ClockEnable(cmuClock_DAC0, true);
    DAC_Init_TypeDef init = DAC_INIT_DEFAULT;
    DAC_InitChannel_TypeDef channel = DAC_INITCHANNEL_DEFAULT;

    init.reference = dacRefVDD;
    init.outMode = dacOutputPin;
    init.prescale = DAC_PrescaleCalc(100, 0);

    DAC_Init(DAC0, &init);

    channel.refreshEnable = true;
    DAC_InitChannel(DAC0, &channel, 0);
    DAC_InitChannel(DAC0, &channel, 1);

    DAC_Enable(DAC0, 0, true);
    DAC_Enable(DAC0, 1, true);
    DAC_Channel0OutputSet(DAC0, 100);
    DAC_Channel1OutputSet(DAC0, 1000);
}


