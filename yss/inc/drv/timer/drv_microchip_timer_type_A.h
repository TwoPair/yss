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

#ifndef	YSS_DRV_TIMER_MAXIM_TYPE_A__H_
#define	YSS_DRV_TIMER_MAXIM_TYPE_A__H_

#if defined(__SAML21E15A__) || defined(__SAML21E15B__) || defined(__SAML21E16A__) || defined(__SAML21E16B__) || \
    defined(__SAML21E17A__) || defined(__SAML21E17B__) || defined(__SAML21E18B__) || defined(__SAML21G16A__) || \
    defined(__SAML21G16B__) || defined(__SAML21G17A__) || defined(__SAML21G17B__) || defined(__SAML21G18A__) || \
    defined(__SAML21G18B__) || defined(__SAML21J16A__) || defined(__SAML21J16B__) || defined(__SAML21J17A__) || \
    defined(__SAML21J17B__) || defined(__SAML21J18A__) || defined(__SAML21J18B__)

#include <config.h>
#include <drv/Drv.h>
#include <yss/mcu.h>

namespace drv
{
	class Timer : public Drv
	{
		Tc *mPeri;
		unsigned int (*mGetClockFreq)(void);
		unsigned int mDiv;
		void (*mIsrUpdate)(void);

	public :
		Timer(Tc *peri, void (*clockFunc)(bool en), void (*nvicFunc)(bool en), unsigned int (*getClockFreq)(void));

		void setUpdateIsr(void (*isr)(void));

		void init(unsigned int freq);
		void init(unsigned int psc, unsigned int arr);
		void initSystemTime(void);

		void setUpdateIntEn(bool en);

		void start(void);
		void stop(void);

		unsigned int getClockFreq(void);

		void isrUpdate(void);

		unsigned int getCounterValue(void);
		unsigned int getOverFlowCount(void);
	};
}

#if defined(TIM0_ENABLE) && defined(TC0)
extern drv::Timer timer0;
#endif

#if defined(TIM1_ENABLE) && defined(TC1)
extern drv::Timer timer1;
#endif

#if defined(TIM2_ENABLE) && defined(TC2)
extern drv::Timer timer2;
#endif

#if defined(TIM3_ENABLE) && defined(TC3)
extern drv::Timer timer3;
#endif

#if defined(TIM4_ENABLE) && defined(TC4)
extern drv::Timer timer4;
#endif

#endif

#endif