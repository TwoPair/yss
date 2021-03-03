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
//  주담당자 : 아이구 (mymy49@nate.com) 2021.02.06 ~ 현재
//  부담당자 : -
//
////////////////////////////////////////////////////////////////////////////////////////

#ifndef	YSS_INSTANCE_DMA__H_
#define	YSS_INSTANCE_DMA__H_

#include <drv/drv_Dma.h>

#if defined(DMA1)
extern drv::Dma dma;
#endif

#if defined(DMA1_STREAM0_ENABLE) && defined(DMA1_Stream0)
extern drv::Stream	dma1Stream0;
#endif

#if defined(DMA1_STREAM1_ENABLE) && (defined(DMA1_Stream1) || defined(DMA1_Channel1))
extern drv::Stream	dma1Stream1;
#endif

#if defined(DMA1_STREAM2_ENABLE) && (defined(DMA1_Stream2) || defined(DMA1_Channel2))
extern drv::Stream	dma1Stream2;
#endif

#if defined(DMA1_STREAM3_ENABLE) && (defined(DMA1_Stream3) || defined(DMA1_Channel3))
extern drv::Stream	dma1Stream3;
#endif

#if defined(DMA1_STREAM4_ENABLE) && (defined(DMA1_Stream4) || defined(DMA1_Channel4))
extern drv::Stream	dma1Stream4;
#endif

#if defined(DMA1_STREAM5_ENABLE) && (defined(DMA1_Stream5) || defined(DMA1_Channel5))
extern drv::Stream	dma1Stream5;
#endif

#if defined(DMA1_STREAM6_ENABLE) && (defined(DMA1_Stream6) || defined(DMA1_Channel6))
extern drv::Stream	dma1Stream6;
#endif

#if defined(DMA1_STREAM7_ENABLE) && (defined(DMA1_Stream7) || defined(DMA1_Channel7))
extern drv::Stream	dma1Stream7;
#endif

#if defined(DMA2_STREAM0_ENABLE) && defined(DMA2_Stream0)
extern drv::Stream	dma2Stream0;
#endif

#if defined(DMA2_STREAM1_ENABLE) && (defined(DMA2_Stream1) || defined(DMA2_Channel1))
extern drv::Stream	dma2Stream1;
#endif

#if defined(DMA2_STREAM2_ENABLE) && (defined(DMA2_Stream2) || defined(DMA2_Channel2))
extern drv::Stream	dma2Stream2;
#endif

#if defined(DMA2_STREAM3_ENABLE) && (defined(DMA2_Stream3) || defined(DMA2_Channel3))
extern drv::Stream	dma2Stream3;
#endif

#if defined(DMA2_STREAM4_ENABLE) && (defined(DMA2_Stream4) || defined(DMA2_Channel4))
extern drv::Stream	dma2Stream4;
#endif

#if defined(DMA2_STREAM5_ENABLE) && (defined(DMA2_Stream5) || defined(DMA2_Channel5))
extern drv::Stream	dma2Stream5;
#endif

#if defined(DMA2_STREAM6_ENABLE) && (defined(DMA2_Stream6) || defined(DMA2_Channel6))
extern drv::Stream	dma2Stream6;
#endif

#if defined(DMA2_STREAM7_ENABLE) && (defined(DMA2_Stream7) || defined(DMA2_Channel7))
extern drv::Stream	dma2Stream7;
#endif

#endif