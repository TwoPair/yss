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

#ifndef YSS_DRV_CLOCK_ST_TYPE_B_DEFINE__H_
#define YSS_DRV_CLOCK_ST_TYPE_B_DEFINE__H_

#if defined(STM32L010x4) || defined(STM32L010x6) || defined(STM32L010x8) || defined(STM32L010xB) || \
    defined(STM32L011xx) || defined(STM32L021xx) ||                                                 \
    defined(STM32L031xx) || defined(STM32L041xx) ||                                                 \
    defined(STM32L051xx) || defined(STM32L052xx) || defined(STM32L053xx) ||                         \
    defined(STM32L061xx) || defined(STM32L062xx) || defined(STM32L063xx) ||                         \
    defined(STM32L071xx) || defined(STM32L072xx) || defined(STM32L073xx) ||                         \
    defined(STM32L081xx) || defined(STM32L082xx) || defined(STM32L083xx)

namespace define
{
namespace clock
{
namespace pll
{
namespace src
{
enum
{
    HSI = 0,
    HSE = 1
};
}

namespace mul
{
enum
{
    MUL_X3 = 0,
    MUL_X4 = 1,
    MUL_X6 = 2,
    MUL_X8 = 3,
    MUL_X12 = 4,
    MUL_X16 = 5,
    MUL_X24 = 6,
    MUL_X32 = 7,
    MUL_X48 = 8,
};
}

namespace div
{
enum
{
    DIV_2 = 1,
    DIV_3 = 2,
    DIV_4 = 3,
};
}
}

namespace usbclk
{
namespace src
{
enum
{
    MAIN_PLL = 0,
    SAI_PLL = 1,
};
}
}

namespace sysclk
{
namespace src
{
enum
{
    HSI = 0,
    HSE = 1,
    PLL = 2
};
}
}

namespace divFactor
{
namespace ahb
{
enum
{
    NO_DIV = 0,
    DIV2 = 0x8,
    DIV4 = 0x9,
    DIV8 = 0xa,
    DIV16 = 0xb,
    DIV64 = 0xc,
    DIV128 = 0xd,
    DIV256 = 0xe,
    DIV512 = 0xf
};
}

namespace apb
{
enum
{
    NO_DIV = 0,
    DIV2 = 0x4,
    DIV4 = 0x5,
    DIV8 = 0x6,
    DIV16 = 0x7,
};
}

}
}
}

#endif

#endif