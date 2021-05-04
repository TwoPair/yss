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

#if defined(STM32F7) || defined(STM32F4) || defined(STM32G4) || defined(STM32L0) || defined(STM32L4) || defined(STM32F0)

#include <__cross_studio_io.h>

#include <drv/gpio/drv_st_gpio_type_A.h>
#include <drv/gpio/drv_st_gpio_type_A_register.h>
#include <drv/syscfg/drv_st_syscfg_type_A_register.h>

namespace drv
{
Gpio::Gpio(GPIO_TypeDef *peri, void (*clockFunc)(bool en), void (*resetFunc)(void), unsigned char exti) : Drv(clockFunc, 0, resetFunc)
{
    mPeri = peri;
    mExti = exti;
}

void Gpio::setExti(unsigned char pin)
{
    unsigned char field = pin % 4 * 4;
    unsigned int reg = SYSCFG->EXTICR[pin / 4];
    reg &= 0xF << field;
    reg |= mExti << field;
    SYSCFG->EXTICR[pin / 4] = reg;
}

void Gpio::setAsAltFunc(unsigned char pin, unsigned char altFunc, unsigned char ospeed, bool otype)
{
    setGpioMode(mPeri, pin, define::gpio::mode::ALT_FUNC);
    setGpioAltfunc(mPeri, pin, altFunc);
    setGpioOspeed(mPeri, pin, ospeed);
    setGpioOtype(mPeri, pin, otype);
}

void Gpio::setAsInput(unsigned char pin, unsigned char pullUpDown)
{
    setGpioMode(mPeri, pin, define::gpio::mode::INPUT);
    setGpioPullUpDown(mPeri, pin, pullUpDown);
}

void Gpio::setAsAltFunc(config::gpio::AltFunc *altport, unsigned char numOfPort, unsigned char ospeed, bool otype)
{
    GPIO_TypeDef *port;
    unsigned char pin;
    unsigned char func;

    for (unsigned char i = 0; i < numOfPort; i++)
    {
        port = altport[i].port;
        pin = altport[i].pin;
        func = altport[i].func;

        setGpioMode(port, pin, define::gpio::mode::ALT_FUNC);
        setGpioAltfunc(port, pin, func);
        setGpioOspeed(port, pin, ospeed);
        setGpioOtype(port, pin, otype);
    }
}

void Gpio::setAsOutput(unsigned char pin, unsigned char ospeed, bool otype)
{
    setGpioMode(mPeri, pin, define::gpio::mode::OUTPUT);
    setGpioOspeed(mPeri, pin, ospeed);
    setGpioOtype(mPeri, pin, otype);
}

void Gpio::setOutput(unsigned char pin, bool data)
{
    setRegBit(mPeri->ODR, data, pin);
}

void Gpio::setPullUpDown(unsigned char pin, unsigned char pupd)
{
    setGpioPullUpDown(mPeri, pin, pupd);
}

void Gpio::setAsAnalog(unsigned char pin)
{
    mPeri->MODER |= 0x03 << (pin * 2);
}

bool Gpio::getData(unsigned char pin)
{
    return getGpioInputData(mPeri, pin);
}
}

#endif