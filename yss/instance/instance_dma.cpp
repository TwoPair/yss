////////////////////////////////////////////////////////////////////////////////////////
//
// 저작권 표기 License_ver_2.0
// 본 소스코드의 소유권은 yss Embedded Operating System 네이버 카페 관리자와 운영진에게 있습니다.
// 운영진이 임의로 코드의 권한을 타인에게 양도할 수 없습니다.
// 본 소스코드는 아래 사항에 동의할 경우에 사용 가능합니다.
// 아래 사항에 대해 동의하지 않거나 이해하지 못했을 경우 사용을 금합니다.
// 본 소스코드를 사용하였다면 아래 사항을 모두 동의하는 것으로 자동 간주 합니다.
// 본 소스코드의 상업적 또는 비상업적 이용이 가능합니다.
// 본 소스코드의 내용을 임의로 수정하여 재배포하는 행위를 금합니다.
// 본 소스코드의 내용을 무단 전재하는 행위를 금합니다.
// 본 소스코드의 사용으로 인해 발생하는 모든 사고에 대해서 어떤한 법적 책임을 지지 않습니다.
//
//	Home Page : http://cafe.naver.com/yssoperatingsystem
//	Copyright 2021.	yss Embedded Operating System all right reserved.
//
//  주담당자 : 아이구 (mymy49@nate.com) 2021.02.11 ~ 현재
//  부담당자 : -
//
////////////////////////////////////////////////////////////////////////////////////////

#include <instance/instance_clock.h>
#include <instance/instance_dma.h>

#include <config.h>
#include <yss/yss.h>

#if defined(DMA1)
static void setDmaClockEn(bool en)
{
    clock.peripheral.setDmaEn(en);
}

static void setDmaIntEn(bool en)
{
#if defined(DMA1_STREAM0_ENABLE) && defined(DMA1_Stream0)
    nvic.setDma1Stream0En(en);
#endif
#if defined(DMA1_STREAM1_ENABLE) && (defined(DMA1_Channel1) || defined(DMA1_Stream1))
    nvic.setDma1Stream1En(en);
#endif
#if defined(DMA1_STREAM2_ENABLE) && (defined(DMA1_Channel2) || defined(DMA1_Stream2))
    nvic.setDma1Stream2En(en);
#endif
#if defined(DMA1_STREAM3_ENABLE) && (defined(DMA1_Channel3) || defined(DMA1_Stream3))
    nvic.setDma1Stream3En(en);
#endif
#if defined(DMA1_STREAM4_ENABLE) && (defined(DMA1_Channel4) || defined(DMA1_Stream4))
    nvic.setDma1Stream4En(en);
#endif
#if defined(DMA1_STREAM5_ENABLE) && (defined(DMA1_Channel5) || defined(DMA1_Stream5))
    nvic.setDma1Stream5En(en);
#endif
#if defined(DMA1_STREAM6_ENABLE) && (defined(DMA1_Channel6) || defined(DMA1_Stream6))
    nvic.setDma1Stream6En(en);
#endif
#if defined(DMA1_STREAM7_ENABLE) && (defined(DMA1_Channel7) || defined(DMA1_Stream7))
    nvic.setDma1Stream7En(en);
#endif

#if defined(DMA2_STREAM0_ENABLE) && defined(DMA2_Stream0)
    nvic.setDma2Stream0En(en);
#endif
#if defined(DMA2_STREAM1_ENABLE) && (defined(DMA2_Channel1) || defined(DMA2_Stream1))
    nvic.setDma2Stream1En(en);
#endif
#if defined(DMA2_STREAM2_ENABLE) && (defined(DMA2_Channel2) || defined(DMA2_Stream2))
    nvic.setDma2Stream2En(en);
#endif
#if defined(DMA2_STREAM3_ENABLE) && (defined(DMA2_Channel3) || defined(DMA2_Stream3))
    nvic.setDma2Stream3En(en);
#endif
#if defined(DMA2_STREAM4_ENABLE) && (defined(DMA2_Channel4) || defined(DMA2_Stream4))
    nvic.setDma2Stream4En(en);
#endif
#if defined(DMA2_STREAM5_ENABLE) && (defined(DMA2_Channel5) || defined(DMA2_Stream5))
    nvic.setDma2Stream5En(en);
#endif
#if defined(DMA2_STREAM6_ENABLE) && (defined(DMA2_Channel6) || defined(DMA2_Stream6))
    nvic.setDma2Stream6En(en);
#endif
#if defined(DMA2_STREAM7_ENABLE) && (defined(DMA2_Channel7) || defined(DMA2_Stream7))
    nvic.setDma2Stream7En(en);
#endif
}

drv::Dma dma(setDmaClockEn, setDmaIntEn);
#endif

#if defined(DMA1_STREAM0_ENABLE) && defined(DMA1_Stream0)
static void setDma1Stream0IntEn(bool en)
{
    nvic.setDma1Stream0En(en);
}

drv::Stream dma1Stream0(DMA1, DMA1_Stream0, 0, setDma1Stream0IntEn, 0);

extern "C"
{

#if defined(YSS_DRV_DMA_ST_TYPE_A__H_)
    void DMA1_Stream0_IRQHandler(void)
#endif
    {
        dma1Stream0.isr0();
    }
}
#endif

#if defined(DMA1_STREAM1_ENABLE) && (defined(DMA1_Channel1) || defined(DMA1_Stream1))
static void setDma1Stream1IntEn(bool en)
{
    nvic.setDma1Stream1En(en);
}

#if defined(YSS_DRV_DMA_ST_TYPE_A__H_)
drv::Stream dma1Stream1(DMA1, DMA1_Stream1, 0, setDma1Stream1IntEn, 1);
#elif defined(YSS_DRV_DMA_ST_TYPE_B__H_)
drv::Stream dma1Stream1(DMA1, DMA1_Channel1, 0, setDma1Stream1IntEn, 1);
#endif
extern "C"
{

#if defined(YSS_DRV_DMA_ST_TYPE_A__H_)
    void DMA1_Stream1_IRQHandler(void)
#elif defined(YSS_DRV_DMA_ST_TYPE_B__H_)
    void DMA1_Channel1_IRQHandler(void)
#endif
    {
        dma1Stream1.isr1();
    }
}

#endif

#if defined(DMA1_STREAM2_ENABLE) && (defined(DMA1_Channel2) || defined(DMA1_Stream2))
static void setDma1Stream2IntEn(bool en)
{
    nvic.setDma1Stream2En(en);
}

#if defined(YSS_DRV_DMA_ST_TYPE_A__H_)
drv::Stream dma1Stream2(DMA1, DMA1_Stream2, 0, setDma1Stream2IntEn, 2);
#elif defined(YSS_DRV_DMA_ST_TYPE_B__H_)
drv::Stream dma1Stream2(DMA1, DMA1_Channel2, 0, setDma1Stream2IntEn, 2);
#endif

extern "C"
{
#if defined(YSS_DRV_DMA_ST_TYPE_A__H_)
    void DMA1_Stream2_IRQHandler(void)
#elif defined(YSS_DRV_DMA_ST_TYPE_B__H_)
    void DMA1_Channel2_IRQHandler(void)
#endif
    {
        dma1Stream2.isr2();
    }
}

#endif

#if defined(DMA1_STREAM3_ENABLE) && (defined(DMA1_Channel3) || defined(DMA1_Stream3))
static void setDma1Stream3IntEn(bool en)
{
    nvic.setDma1Stream3En(en);
}

#if defined(YSS_DRV_DMA_ST_TYPE_A__H_)
drv::Stream dma1Stream3(DMA1, DMA1_Stream3, 0, setDma1Stream3IntEn, 3);
#elif defined(YSS_DRV_DMA_ST_TYPE_B__H_)
drv::Stream dma1Stream3(DMA1, DMA1_Channel3, 0, setDma1Stream3IntEn, 3);
#endif

extern "C"
{
#if defined(YSS_DRV_DMA_ST_TYPE_A__H_)
    void DMA1_Stream3_IRQHandler(void)
#elif defined(YSS_DRV_DMA_ST_TYPE_B__H_)
    void DMA1_Channel3_IRQHandler(void)
#endif
    {
        dma1Stream3.isr3();
    }
}
#endif

#if defined(DMA1_STREAM4_ENABLE) && (defined(DMA1_Channel4) || defined(DMA1_Stream4))
static void setDma1Stream4IntEn(bool en)
{
    nvic.setDma1Stream4En(en);
}

#if defined(YSS_DRV_DMA_ST_TYPE_A__H_)
drv::Stream dma1Stream4(DMA1, DMA1_Stream4, 0, setDma1Stream4IntEn, 4);
#elif defined(YSS_DRV_DMA_ST_TYPE_B__H_)
drv::Stream dma1Stream4(DMA1, DMA1_Channel4, 0, setDma1Stream4IntEn, 4);
#endif

extern "C"
{
#if defined(YSS_DRV_DMA_ST_TYPE_A__H_)
    void DMA1_Stream4_IRQHandler(void)
#elif defined(YSS_DRV_DMA_ST_TYPE_B__H_)
    void DMA1_Channel4_IRQHandler(void)
#endif
    {
        dma1Stream4.isr4();
    }
}
#endif

#if defined(DMA1_STREAM5_ENABLE) && (defined(DMA1_Channel5) || defined(DMA1_Stream5))
static void setDma1Stream5IntEn(bool en)
{
    nvic.setDma1Stream5En(en);
}

#if defined(YSS_DRV_DMA_ST_TYPE_A__H_)
drv::Stream dma1Stream5(DMA1, DMA1_Stream5, 0, setDma1Stream5IntEn, 5);
#elif defined(YSS_DRV_DMA_ST_TYPE_B__H_)
drv::Stream dma1Stream5(DMA1, DMA1_Channel5, 0, setDma1Stream5IntEn, 5);
#endif

extern "C"
{
#if defined(YSS_DRV_DMA_ST_TYPE_A__H_)
    void DMA1_Stream5_IRQHandler(void)
#elif defined(YSS_DRV_DMA_ST_TYPE_B__H_)
    void DMA1_Channel5_IRQHandler(void)
#endif
    {
        dma1Stream5.isr5();
    }
}
#endif

#if defined(DMA1_STREAM6_ENABLE) && (defined(DMA1_Channel6) || defined(DMA1_Stream6))
static void setDma1Stream6IntEn(bool en)
{
    nvic.setDma1Stream6En(en);
}

#if defined(YSS_DRV_DMA_ST_TYPE_A__H_)
drv::Stream dma1Stream6(DMA1, DMA1_Stream6, 0, setDma1Stream6IntEn, 6);
#elif defined(YSS_DRV_DMA_ST_TYPE_B__H_)
drv::Stream dma1Stream6(DMA1, DMA1_Channel6, 0, setDma1Stream6IntEn, 6);
#endif

extern "C"
{
#if defined(YSS_DRV_DMA_ST_TYPE_A__H_)
    void DMA1_Stream6_IRQHandler(void)
#elif defined(YSS_DRV_DMA_ST_TYPE_B__H_)
    void DMA1_Channel6_IRQHandler(void)
#endif
    {
        dma1Stream6.isr6();
    }
}
#endif

#if defined(DMA1_STREAM7_ENABLE) && (defined(DMA1_Channel7) || defined(DMA1_Stream7))
static void setDma1Stream7IntEn(bool en)
{
    nvic.setDma1Stream7En(en);
}

#if defined(YSS_DRV_DMA_ST_TYPE_A__H_)
drv::Stream dma1Stream7(DMA1, DMA1_Stream7, 0, setDma1Stream7IntEn, 7);
#elif defined(YSS_DRV_DMA_ST_TYPE_B__H_)
drv::Stream dma1Stream7(DMA1, DMA1_Channel7, 0, setDma1Stream7IntEn, 7);
#endif

extern "C"
{
#if defined(YSS_DRV_DMA_ST_TYPE_A__H_)
    void DMA1_Stream7_IRQHandler(void)
#elif defined(YSS_DRV_DMA_ST_TYPE_B__H_)
    void DMA1_Channel7_IRQHandler(void)
#endif
    {
        dma1Stream7.isr7();
    }
}

#endif

#if defined(DMA2_STREAM0_ENABLE) && defined(DMA2_Stream0)
static void setDma2Stream0IntEn(bool en)
{
    nvic.setDma2Stream0En(en);
}

#if defined(YSS_DRV_DMA_ST_TYPE_A__H_)
drv::Stream dma2Stream0(DMA2, DMA2_Stream0, 0, setDma2Stream0IntEn, 0);
#endif

extern "C"
{
#if defined(YSS_DRV_DMA_ST_TYPE_A__H_)
    void DMA2_Stream0_IRQHandler(void)
#endif
    {
        dma2Stream0.isr0();
    }
}
#endif

#if defined(DMA2_STREAM1_ENABLE) && (defined(DMA2_Channel1) || defined(DMA2_Stream1))
static void setDma2Stream1IntEn(bool en)
{
    nvic.setDma2Stream1En(en);
}

#if defined(YSS_DRV_DMA_ST_TYPE_A__H_)
drv::Stream dma2Stream1(DMA2, DMA2_Stream1, 0, setDma2Stream1IntEn, 1);
#elif defined(YSS_DRV_DMA_ST_TYPE_B__H_)
drv::Stream dma2Stream1(DMA2, DMA2_Channel1, 0, setDma2Stream1IntEn, 1);
#endif

extern "C"
{
#if defined(YSS_DRV_DMA_ST_TYPE_A__H_)
    void DMA2_Stream1_IRQHandler(void)
#elif defined(YSS_DRV_DMA_ST_TYPE_B__H_)
    void DMA2_Channel1_IRQHandler(void)
#endif
    {
        dma2Stream1.isr1();
    }
}
#endif

#if defined(DMA2_STREAM2_ENABLE) && (defined(DMA2_Channel2) || defined(DMA2_Stream2))
static void setDma2Stream2IntEn(bool en)
{
    nvic.setDma2Stream2En(en);
}

#if defined(YSS_DRV_DMA_ST_TYPE_A__H_)
drv::Stream dma2Stream2(DMA2, DMA2_Stream2, 0, setDma2Stream2IntEn, 2);
#elif defined(YSS_DRV_DMA_ST_TYPE_B__H_)
drv::Stream dma2Stream2(DMA2, DMA2_Channel2, 0, setDma2Stream2IntEn, 2);
#endif

extern "C"
{
#if defined(YSS_DRV_DMA_ST_TYPE_A__H_)
    void DMA2_Stream2_IRQHandler(void)
#elif defined(YSS_DRV_DMA_ST_TYPE_B__H_)
    void DMA2_Channel2_IRQHandler(void)
#endif
    {
        dma2Stream2.isr2();
    }
}
#endif

#if defined(DMA2_STREAM3_ENABLE) && (defined(DMA2_Channel3) || defined(DMA2_Stream3))
static void setDma2Stream3IntEn(bool en)
{
    nvic.setDma2Stream3En(en);
}

#if defined(YSS_DRV_DMA_ST_TYPE_A__H_)
drv::Stream dma2Stream3(DMA2, DMA2_Stream3, 0, setDma2Stream3IntEn, 3);
#elif defined(YSS_DRV_DMA_ST_TYPE_B__H_)
drv::Stream dma2Stream3(DMA2, DMA2_Channel3, 0, setDma2Stream3IntEn, 3);
#endif

extern "C"
{
#if defined(YSS_DRV_DMA_ST_TYPE_A__H_)
    void DMA2_Stream3_IRQHandler(void)
#elif defined(YSS_DRV_DMA_ST_TYPE_B__H_)
    void DMA2_Channel3_IRQHandler(void)
#endif
    {
        dma2Stream3.isr3();
    }
}
#endif

#if defined(DMA2_STREAM4_ENABLE) && (defined(DMA2_Channel4) || defined(DMA2_Stream4))
static void setDma2Stream4IntEn(bool en)
{
    nvic.setDma2Stream4En(en);
}

#if defined(YSS_DRV_DMA_ST_TYPE_A__H_)
drv::Stream dma2Stream4(DMA2, DMA2_Stream4, 0, setDma2Stream4IntEn, 4);
#elif defined(YSS_DRV_DMA_ST_TYPE_B__H_)
drv::Stream dma2Stream4(DMA2, DMA2_Channel4, 0, setDma2Stream4IntEn, 4);
#endif

extern "C"
{
#if defined(YSS_DRV_DMA_ST_TYPE_A__H_)
    void DMA2_Stream4_IRQHandler(void)
    {
        dma2Stream4.isr4();
    }
#elif defined(YSS_DRV_DMA_ST_TYPE_B__H_)
#if !defined(DMA2_STREAM5_ENABLE) && !defined(DMA2_Channel5)
    void DMA2_Channel4_5_IRQHandler(void)
    {
        dma2Stream4.isr4();
    }
#endif
#endif
}
#endif

#if defined(DMA2_STREAM5_ENABLE) && (defined(DMA2_Channel5) || defined(DMA2_Stream5))
static void setDma2Stream5IntEn(bool en)
{
    nvic.setDma2Stream5En(en);
}

#if defined(YSS_DRV_DMA_ST_TYPE_A__H_)
drv::Stream dma2Stream5(DMA2, DMA2_Stream5, 0, setDma2Stream5IntEn, 5);
#elif defined(YSS_DRV_DMA_ST_TYPE_B__H_)
drv::Stream dma2Stream5(DMA2, DMA2_Channel5, 0, setDma2Stream5IntEn, 5);
#endif

extern "C"
{
#if defined(YSS_DRV_DMA_ST_TYPE_A__H_)
    void DMA2_Stream5_IRQHandler(void)
    {
        dma2Stream5.isr5();
    }
#elif defined(YSS_DRV_DMA_ST_TYPE_B__H_)
    void DMA2_Channel4_5_IRQHandler(void)
    {
        dma2Stream4.isr4();
        dma2Stream5.isr5();
    }
#endif
}
#endif

#if defined(DMA2_STREAM6_ENABLE) && defined(DMA2_Stream6)
static void setDma2Stream6IntEn(bool en)
{
    nvic.setDma2Stream6En(en);
}

#if defined(YSS_DRV_DMA_ST_TYPE_A__H_)
drv::Stream dma2Stream6(DMA2, DMA2_Stream6, 0, setDma2Stream6IntEn, 6);
#endif

extern "C"
{
#if defined(YSS_DRV_DMA_ST_TYPE_A__H_)
    void DMA2_Stream6_IRQHandler(void)
#endif
    {
        dma2Stream6.isr6();
    }
}
#endif

#if defined(DMA2_STREAM7_ENABLE) && defined(DMA2_Stream7)
static void setDma2Stream7IntEn(bool en)
{
    nvic.setDma2Stream7En(en);
}

#if defined(YSS_DRV_DMA_ST_TYPE_A__H_)
drv::Stream dma2Stream7(DMA2, DMA2_Stream7, 0, setDma2Stream7IntEn, 7);
#endif

extern "C"
{
#if defined(YSS_DRV_DMA_ST_TYPE_A__H_)
    void DMA2_Stream7_IRQHandler(void)
#endif
    {
        dma2Stream7.isr7();
    }
}
#endif

extern "C"
{

#if defined(STM32F100xB) || defined(STM32F100xE) ||                                                 \
    defined(STM32F101x6) || defined(STM32F101xB) || defined(STM32F101xE) || defined(STM32F101xG) || \
    defined(STM32F102x6) || defined(STM32F102xB) ||                                                 \
    defined(STM32F103x6) || defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || \
    defined(STM32F105xC) ||                                                                         \
    defined(STM32F107xC)

#elif defined(STM32L010x4) || defined(STM32L010x6) || defined(STM32L010x8) || defined(STM32L010xB) || \
    defined(STM32L011xx) || defined(STM32L021xx) ||                                                   \
    defined(STM32L031xx) || defined(STM32L041xx) ||                                                   \
    defined(STM32L051xx) || defined(STM32L052xx) || defined(STM32L053xx) ||                           \
    defined(STM32L061xx) || defined(STM32L062xx) || defined(STM32L063xx) ||                           \
    defined(STM32L071xx) || defined(STM32L072xx) || defined(STM32L073xx) ||                           \
    defined(STM32L081xx) || defined(STM32L082xx) || defined(STM32L083xx)

#if (defined(DMA1_STREAM2_ENABLE) || defined(DMA1_STREAM3_ENABLE)) && defined(DMA1)
    void DMA1_Channel2_3_IRQHandler(void)
    {
        unsigned long sr = getDmaStream2Sr(DMA1);
        clrDmaStream2Sr(DMA1, sr);
        if (checkError(sr))
            dma1Stream2.setError();
        if (checkComplete(sr))
            dma1Stream2.setComplete();

        sr = getDmaStream3Sr(DMA1);
        if (checkError(sr))
            dma1Stream3.setError();
        if (checkComplete(sr))
            dma1Stream3.setComplete();
    }

    void DMA1_Channel4_5_6_7_IRQHandler(void)
    {
        unsigned long sr = getDmaStream4Sr(DMA1);
        clrDmaStream4Sr(DMA1, sr);
        if (checkError(sr))
            dma1Stream4.setError();
        if (checkComplete(sr))
            dma1Stream4.setComplete();

        sr = getDmaStream5Sr(DMA1);
        if (checkError(sr))
            dma1Stream5.setError();
        if (checkComplete(sr))
            dma1Stream5.setComplete();

        sr = getDmaStream6Sr(DMA1);
        if (checkError(sr))
            dma1Stream6.setError();
        if (checkComplete(sr))
            dma1Stream6.setComplete();

        sr = getDmaStream7Sr(DMA1);
        if (checkError(sr))
            dma1Stream7.setError();
        if (checkComplete(sr))
            dma1Stream7.setComplete();
    }

#endif

#endif
}