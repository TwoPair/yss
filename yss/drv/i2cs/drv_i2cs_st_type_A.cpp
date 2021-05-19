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
//  주담당자 : 아이구 (mymy49@nate.com) 2021.05.18 ~ 현재
//  부담당자 : -
//
////////////////////////////////////////////////////////////////////////////////////////

#include <yss/mcu.h>

#if defined(STM32F7) || defined(STM32F0)

#include <__cross_studio_io.h>

#include <drv/i2cs/drv_st_i2cs_type_A.h>
#include <yss/thread.h>

namespace drv
{

I2cs::I2cs(I2C_TypeDef *peri, void (*clockFunc)(bool en), void (*nvicFunc)(bool en), void (*resetFunc)(void), unsigned int (*getClockFrequencyFunc)(void)) : Drv(clockFunc, nvicFunc, resetFunc)
{
    mPeri = peri;
	mRcvBuf = 0;
	mRcvBufSize = 0;
}

bool I2cs::setSpeed(unsigned char speed)
{
    register unsigned int reg;

    switch (speed)
    {
    case define::i2cs::speed::STANDARD:
#if defined(STM32F0)
        reg = 10 << I2C_TIMINGR_PRESC_Pos |
              0x13 << I2C_TIMINGR_SCLL_Pos |
              0x0F << I2C_TIMINGR_SCLH_Pos |
              0x02 << I2C_TIMINGR_SDADEL_Pos |
              0x04 << I2C_TIMINGR_SCLDEL_Pos;
#elif defined(STM32F7)
        reg = 3 << I2C_TIMINGR_PRESC_Pos |
              0x13 << I2C_TIMINGR_SCLL_Pos |
              0x0F << I2C_TIMINGR_SCLH_Pos |
              0x02 << I2C_TIMINGR_SDADEL_Pos |
              0x04 << I2C_TIMINGR_SCLDEL_Pos;
#endif
        break;
    case define::i2cs::speed::FAST:
#if defined(STM32F0)
        reg = 0 << I2C_TIMINGR_PRESC_Pos |
              0x09 << I2C_TIMINGR_SCLL_Pos |
              0x03 << I2C_TIMINGR_SCLH_Pos |
              0x01 << I2C_TIMINGR_SDADEL_Pos |
              0x03 << I2C_TIMINGR_SCLDEL_Pos;
#elif defined(STM32F7)
        reg = 1 << I2C_TIMINGR_PRESC_Pos |
              0x09 << I2C_TIMINGR_SCLL_Pos |
              0x03 << I2C_TIMINGR_SCLH_Pos |
              0x02 << I2C_TIMINGR_SDADEL_Pos |
              0x03 << I2C_TIMINGR_SCLDEL_Pos;
#endif
        break;
    case define::i2cs::speed::FAST_PLUS:
#if defined(STM32F0)
        reg = 0 << I2C_TIMINGR_PRESC_Pos |
              0x06 << I2C_TIMINGR_SCLL_Pos |
              0x03 << I2C_TIMINGR_SCLH_Pos |
              0x00 << I2C_TIMINGR_SDADEL_Pos |
              0x01 << I2C_TIMINGR_SCLDEL_Pos;
#elif defined(STM32F7)
        reg = 0 << I2C_TIMINGR_PRESC_Pos |
              0x04 << I2C_TIMINGR_SCLL_Pos |
              0x02 << I2C_TIMINGR_SCLH_Pos |
              0x00 << I2C_TIMINGR_SDADEL_Pos |
              0x02 << I2C_TIMINGR_SCLDEL_Pos;
#endif
        break;
    default:
        return false;
    }

    mPeri->TIMINGR = reg;

    return true;
}

bool I2cs::init(unsigned char speed, void *rcvBuf, unsigned short rcvBufSize, unsigned char addr1, unsigned char addr2)
{
    // 장치 비활성화
    mPeri->CR1 = 0;

    // 주소 레지스터1, 2 비활성화
    mPeri->OAR1 = 0;
    mPeri->OAR2 = 0;

    // 타이밍 설정
    if (setSpeed(speed) == false)
        return false;

    // 주소 레지스터1 설정
    mPeri->OAR1 = I2C_OAR1_OA1EN_Msk | (addr1 & 0xFE) << I2C_OAR1_OA1_Pos;

    // 주소 레지스터2에 값이 있으면 설정
    if (addr2 > 0)
        mPeri->OAR2 = I2C_OAR2_OA2EN_Msk | (addr2 & 0xFE);

    // DMA_RX, DMA_TX, 주소 일치 인터럽트, 장치 활성화 설정
    mPeri->CR1 =  I2C_CR1_ADDRIE_Msk | I2C_CR1_PE_Msk;

	mRcvBuf = (unsigned char*)rcvBuf;
	mRcvBufSize = rcvBufSize;

    return true;
}

inline void waitUntilComplete(I2C_TypeDef *peri)
{
    while ((peri->ISR & I2C_ISR_TC) == false)
        thread::yield();
}

#define setNbytes(data, x) setRegField(data, 0xFFUL, x, 16)
#define setSaddr(data, x) setRegField(data, 0x3FFUL, x, 0)

bool I2cs::setSendBuffer(void *src, unsigned int size)
{
	return true;
}

void I2cs::isr(void)
{
	register int isr = mPeri->ISR, cr1 = mPeri->CR1;

	if(cr1 & I2C_CR1_ADDRIE_Msk && isr & I2C_ISR_ADDR_Msk)
	{
		mPeri->ICR = I2C_ICR_ADDRCF_Msk;
		if((mPeri->OAR1 & I2C_OAR2_OA2_Msk) >> I2C_OAR2_OA2_Pos == (isr & I2C_ISR_ADDCODE_Msk) >> I2C_ISR_ADDCODE_Pos)
			mSelectedAddr = define::i2cs::selectedAddr::ADDR1;
		else
			mSelectedAddr = define::i2cs::selectedAddr::ADDR2;
		
		if(isr & I2C_ISR_DIR_Msk) // 쓰기 모드
		{

		}
		else // 읽기 모드
		{
			mRcvBufIndex = 0;
			mPeri->CR1 |= I2C_CR1_RXIE_Msk;
		}
	}

	if(cr1 & I2C_CR1_RXIE_Msk && isr & I2C_ISR_RXNE_Msk)
	{
		mRcvBuf[mRcvBufIndex++] = (unsigned char)mPeri->RXDR;
		if(mRcvBufIndex > mRcvBufSize)
		{
			mPeri->CR1 &= I2C_CR1_RXIE_Msk;
		}
	}
}
}

#endif