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
//	Copyright 2020.	yss Embedded Operating System all right reserved.
//
//  주담당자 : 아이구 (mymy49@nate.com) 2016.04.30 ~ 현재
//  부담당자 : -
//
////////////////////////////////////////////////////////////////////////////////////////

#if defined(STM32L010x4) || defined(STM32L010x6) || defined(STM32L010x8) || defined(STM32L010xB) || \
    defined(STM32L011xx) || defined(STM32L021xx) ||                                                 \
    defined(STM32L031xx) || defined(STM32L041xx) ||                                                 \
    defined(STM32L051xx) || defined(STM32L052xx) || defined(STM32L053xx) ||                         \
    defined(STM32L061xx) || defined(STM32L062xx) || defined(STM32L063xx) ||                         \
    defined(STM32L071xx) || defined(STM32L072xx) || defined(STM32L073xx) ||                         \
    defined(STM32L081xx) || defined(STM32L082xx) || defined(STM32L083xx)

#include <config.h>

#if YSS_USE_DEFAULT_MSP == true

#include <drv/peripherals.h>

void initSystem(void)
{
    signed int hseFreq = HSE_CLOCK_FREQ, mul = -1, div = -1, freq;
    const int mulTable[9] = {3, 4, 6, 8, 12, 16, 24, 32, 48};
    const int divTable[3] = {1, 2, 3};

    using namespace define::clock;

    clock.setVosRange(vos::RANGE1);

    clock.enableHsi();

    clock.enableHse(HSE_CLOCK_FREQ);

    for (int i = 0; i < 9; i++)
    {
        freq = hseFreq * mulTable[i];

        if (freq == ec::clock::sysclk::MAX_FREQ / 1000000 * 2)
        {
            mul = i;
            div = 1;
            break;
        }
        else if (freq == ec::clock::sysclk::MAX_FREQ / 1000000 * 3)
        {
            mul = i;
            div = 2;
            break;
        }
    }

    if (mul >= 0 && div >= 0)
    {
        clock.pll.enable(
            define::clock::pll::src::HSE, // unsigned char src;
            mul,                          // unsigned char mul;
            div                           // unsigned char div;
        );

        clock.setSysclk(
            define::clock::sysclk::src::PLL,       // unsigned char sysclkSrc;
            define::clock::divFactor::ahb::NO_DIV, // unsigned char ahb;
            define::clock::divFactor::apb::NO_DIV, // unsigned char apb1;
            define::clock::divFactor::apb::NO_DIV  // unsigned char apb2;
        );
    }

    flash.setPrefetchEn(true);
    flash.setPreReadEn(true);

#if defined(GPIOA)
    clock.peripheral.setGpioAEn(true);
#endif
#if defined(GPIOB)
    clock.peripheral.setGpioBEn(true);
#endif
#if defined(GPIOC)
    clock.peripheral.setGpioCEn(true);
#endif
#if defined(GPIOD)
    clock.peripheral.setGpioDEn(true);
#endif
#if defined(GPIOE)
    clock.peripheral.setGpioEEn(true);
#endif
#if defined(GPIOH)
    clock.peripheral.setGpioHEn(true);
#endif
}

#endif

#endif

