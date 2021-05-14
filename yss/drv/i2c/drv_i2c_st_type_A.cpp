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
//  주담당자 : 아이구 (mymy49@nate.com) 2016.04.30 ~ 현재
//  부담당자 : -
//
////////////////////////////////////////////////////////////////////////////////////////

#include <yss/mcu.h>

#if defined(STM32F7) || defined(STM32F0)

#include <__cross_studio_io.h>

#include <drv/i2c/drv_st_i2c_type_A.h>
#include <drv/i2c/drv_st_i2c_type_A_register.h>
#include <yss/thread.h>

namespace drv
{
I2c::I2c(I2C_TypeDef *peri, void (*clockFunc)(bool en), void (*nvicFunc)(bool en), void (*resetFunc)(void), Stream *txStream, Stream *rxStream, unsigned char txChannel, unsigned char rxChannel, unsigned int (*getClockFrequencyFunc)(void), unsigned short priority) : Drv(clockFunc, nvicFunc, resetFunc)
{
    this->set(txChannel, rxChannel, (void *)&(peri->TXDR), (void *)&(peri->RXDR), priority);

    mTxStream = txStream;
    mRxStream = rxStream;
    mPeri = peri;
}

bool I2c::setSpeed(unsigned char speed)
{
    register unsigned int reg;

    switch (speed)
    {
    case define::i2c::speed::STANDARD:
#if defined(STM32F0)
        reg = 1 << I2C_TIMINGR_PRESC_Pos |
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
    case define::i2c::speed::FAST:
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
    case define::i2c::speed::FAST_PLUS:
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

bool I2c::init(unsigned char speed)
{
    // 장치 비활성화
    mPeri->CR1 = 0;

    // 타이밍 설정
    if (setSpeed(speed) == false)
        return false;

    // DMA_RX, DMA_TX, 장치 활성화 설정
    mPeri->CR1 = I2C_CR1_PE_Msk | I2C_CR1_RXDMAEN_Msk | I2C_CR1_TXDMAEN_Msk;

    return true;
}

bool I2c::initAsSlave(unsigned char speed, void *rcvBuf, unsigned short rcvBufSize, unsigned char addr1, unsigned char addr2)
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
    mPeri->CR1 = I2C_CR1_RXDMAEN_Msk | I2C_CR1_TXDMAEN_Msk | I2C_CR1_ADDRIE_Msk | I2C_CR1_PE_Msk;

    return true;
}

inline void waitUntilComplete(I2C_TypeDef *peri)
{
    while ((peri->ISR & I2C_ISR_TC) == false)
        thread::yield();
}

#define setNbytes(data, x) setRegField(data, 0xFFUL, x, 16)
#define setSaddr(data, x) setRegField(data, 0x3FFUL, x, 0)

bool I2c::send(unsigned char addr, void *src, unsigned int size, unsigned int timeout)
{
    register unsigned int cr2 = I2C_CR2_START;
    register unsigned int isr;
    bool rt;

    if (mTxStream)
    {
        mPeri->ICR = 0xffff;
//		cr2 = (cr2 & ~(I2C_CR2_NBYTES_Msk | I2C_CR2_NBYTES_Msk)) | ((size & 0xFF) << I2C_CR2_NBYTES_Pos | );
        setNbytes(cr2, size);
        setSaddr(cr2, addr);
        mPeri->CR2 = cr2;

#if !defined(__CORE_CM0_H_GENERIC)
        thread::delayUs(2);
#else
        thread::yield();
#endif

        do
        {
            isr = mPeri->ISR;

            if (isr & I2C_ISR_NACKF)
                return false;

            thread::yield();
        } while ((isr & I2C_ISR_TXIS) == false);

        rt = mTxStream->send(this, src, size, timeout);

        waitUntilComplete(mPeri);

        return rt;
    }
    else
        return false;
}

bool I2c::receive(unsigned char addr, void *des, unsigned int size, unsigned int timeout)
{
    unsigned long cr2 = I2C_CR2_START | I2C_CR2_RD_WRN;
    volatile unsigned long isr;
    bool rt;

    if (mRxStream)
    {
        mPeri->ICR = 0xffff;
        setNbytes(cr2, size);
        setSaddr(cr2, addr);
        mPeri->CR2 = cr2;

#if !defined(__CORE_CM0_H_GENERIC)
        thread::delayUs(2);
#else
        thread::yield();
#endif

        do
        {
            isr = mPeri->ISR;

            if (isr & I2C_ISR_NACKF)
                return false;

            thread::yield();
        } while ((isr & I2C_ISR_RXNE) == false);

        rt = mRxStream->receive(this, des, size, timeout);
        waitUntilComplete(mPeri);

        return true;
    }
    else
        return false;
}

void I2c::stop(void)
{
    setI2cStop(mPeri);
}
}

#endif