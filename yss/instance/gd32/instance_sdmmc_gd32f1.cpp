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
//  Home Page : http://cafe.naver.com/yssoperatingsystem
//  Copyright 2021. yss Embedded Operating System all right reserved.
//
// 주담당자 : 아이구 (mymy49@nate.com) 2021.03.05 ~ 현재
// 부담당자 : -
//
////////////////////////////////////////////////////////////////////////////////////////

#include <yss/instance.h>
#include <config.h>

#if defined(GD32F10X_XD) || defined(GD32F10X_HD)

#define PRIORITY_POS	12
#define MWIDTH_POS		10
#define PWIDTH_POS		8
#define DIR_POS			4

#if defined(SDMMC_ENABLE) & defined(SDIO)
static void setClockEn(bool en)
{
	clock.peripheral.setSdmmcEn(en);
}

static void setInterruptEn(bool en)
{
//	nvic.setSd(en);
}

static void reset(void)
{
	clock.peripheral.resetSdmmc();
}

static const Drv::Config gDrvConfig
{
	setClockEn,		//void (*clockFunc)(bool en);
	setInterruptEn,	//void (*nvicFunc)(bool en);
	reset			//void (*resetFunc)(void);
};

static const drv::Dma::DmaInfo gRxDmaInfo = 
{
	(define::dma::priorityLevel::LOW << PRIORITY_POS) | // unsigned int controlRegister1
	(define::dma::size::BYTE << MWIDTH_POS) |
	(define::dma::size::BYTE << PWIDTH_POS) |
	DMA_CTLR_MNAGA | 
	(define::dma::dir::PERI_TO_MEM << DIR_POS) | 
	DMA_CTLR_TCIE | 
	DMA_CTLR_ERRIE | 
	DMA_CTLR_CHEN ,
	0,													// unsigned int controlRegister2
	0,													// unsigned int controlRegister3
	(void*)&SDIO->FIFO,									//void *dataRegister;
};

static const drv::Dma::DmaInfo gTxDmaInfo = 
{
	(define::dma::priorityLevel::LOW << PRIORITY_POS) | // unsigned int controlRegister1
	(define::dma::size::BYTE << MWIDTH_POS) |
	(define::dma::size::BYTE << PWIDTH_POS) |
	DMA_CTLR_MNAGA | 
	(define::dma::dir::MEM_TO_PERI << DIR_POS) | 
	DMA_CTLR_TCIE | 
	DMA_CTLR_ERRIE | 
	DMA_CTLR_CHEN ,
	0,													// unsigned int controlRegister2
	0,													// unsigned int controlRegister3
	(void*)&SDIO->FIFO,									//void *dataRegister;
};

static const drv::Sdmmc::Config gConfig
{
	SDIO,			//YSS_SDMMC_Peri *peri;
	dmaChannel12,	//Dma &txDma;
	gTxDmaInfo,		//Dma::DmaInfo txDmaInfo;
	dmaChannel12,	//Dma &rxDma;
	gRxDmaInfo		//Dma::DmaInfo rxDmaInfo;
};

drv::Sdmmc sdmmc(gDrvConfig, gConfig);
#endif

#endif
