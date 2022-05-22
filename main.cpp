#include <cstddef>
#include <cstdint>
#include <utility>
#include <type_traits>

namespace stm32 {

static inline void sleep()
{
	__asm__ __inline__("wfi");
}


template<class Derived, std::uint32_t Addr>
class Device {
protected:
	static constexpr uint32_t address = Addr;
	static Derived& instance() {return *reinterpret_cast<Derived*>(address);}
};


using Register = volatile std::uint32_t;


namespace f1xx {

enum base_address {
	flash = 0x08000000UL, /*!< FLASH base address in the alias region */
	flash_bank1_end = 0x0801FFFFUL, /*!< FLASH END address of bank1 */
	sram = 0x20000000UL, /*!< SRAM base address in the alias region */
	periph = 0x40000000UL, /*!< PERIPH base address in the alias region */
	sram_bb = 0x22000000UL, /*!< SRAM base address in the bit-band region */
	periph_bb = 0x42000000UL, /*!< Peripheral base address in the bit-band region */
/*!< Peripheral memory map */
	apb1periph = periph,
	apb2periph = periph + 0x00010000UL,
	ahbperiph = periph + 0x00020000UL,
/*!< System devices memory map */
	sys_tick = 0xE000E010
}; // enum base_address

enum Port_address {
		port_a = apb2periph + 0x00000800UL,
		port_b = apb2periph + 0x00000C00UL,
		port_c = apb2periph + 0x00001000UL,
		port_d = apb2periph + 0x00001400UL,
		port_e = apb2periph + 0x00001800UL
};

namespace {
constexpr int apb2enr_index(const Port_address p)
{
	return p == port_a ? 2 : (p == port_b ? 3 : (p == port_c ? 4 : (p == port_d ? 5 : (6))));
}
} // Anonymous


namespace system {

class SysTick: public Device<SysTick, sys_tick>
{
public:
	static void start(std::uint32_t count, bool divide_by_8 = true, bool enable_interrupt = true)
	{
		SysTick& st = instance();
		st.ctrl_ = (enable_interrupt ? 2 : 0) | (divide_by_8 ? 0 : 4);
		st.load_ = count;
		st.val_ = 0;
		st.ctrl_ = st.ctrl_ | 1;
	}

	static void stop()
	{
		instance().ctrl_ = instance().ctrl_ & ~1;
	}

	static bool reloaded() {return instance().ctrl_ & (1 << 16);}
	static std::uint32_t calibration() {return instance().calib_;}

private:
	Register ctrl_;     /* SYSTICK control and status register, Address offset: 0x00 */
	Register load_;     /* SYSTICK reload value register,       Address offset: 0x04 */
	Register val_;      /* SYSTICK current value register,      Address offset: 0x08 */
	Register calib_;    /* SYSTICK calibration value register,  Address offset: 0x0C */
}; // class SysTick

static_assert(sizeof(SysTick) == 4 * sizeof(Register));

class Flash: public Device<Flash, ahbperiph + 0x00002000UL>
{
public:
	enum Latency {
		wait0 = 0,
		wait1,
		wait2
	};

	static void access_control(Latency wait_states, bool prefetch=true, bool half_cycle=false)
	{
		Flash& f = instance();
		f.acr_ = (f.acr_ & ~(0x1f)) | wait_states | (prefetch ? 0x10 : 0) | (half_cycle ? 8 : 0);
	}

private:
	Register acr_;
	Register keyr_;
	Register optkeyr_;
	Register sr_;
	Register cr_;
	Register ar_;
	Register reserved_;
	Register obr_;
	Register wrpr_;
}; // struct Flash

static_assert(sizeof(Flash) == 9 * sizeof(Register));

} // namespace system




namespace clock {

class Rcc: public Device<Rcc, ahbperiph + 0x00001000UL>
{
public:
	enum HsSource {
		internal = 1,
		external = 1 << 16,
		bypass = external | (1 << 18)
	};

	enum PllMul {
		x4 = 2 << 18,
		x5 = 3 << 18,
		x6 = 4 << 18,
		x7 = 5 << 18,
		x8 = 6 << 18,
		x9 = 7 << 18,
		x6_5 = 0xd << 18
	};

	enum AdcPre {
		adc_by2 = 0,
		adc_by4 = 1 << 14,
		adc_by6 = 2 << 14,
		adc_by8 = 3 << 14
	};

	enum Apb1Pre {
		apb1_by1 = 0,
		apb1_by2 = 4 << 8,
		apb1_by4 = 5 << 8,
		apb1_by8 = 6 << 8,
		apb1_by16 = 7 << 8
	};

	enum Apb2Pre {
		apb2_by1 = 0,
		apb2_by2 = 4 << 11,
		apb2_by4 = 5 << 11,
		apb2_by8 = 6 << 11,
		apb2_by16 = 7 << 11
	};


	static void high_speed_source(HsSource src)
	{
		Rcc& r = instance();
		r.cr_ = (r.cr_ & ~(internal | external | bypass)) | src;
		// If clock is set to external, wait until HSE settles down
		if (src & external) {
			while (!(r.cr_ & (1 << 17))) {}
		}
	}

	static void hs_pll(PllMul mul, AdcPre adc_prediv, Apb1Pre apb1_prediv,
				Apb2Pre apb2_prediv)
	{
		Rcc& r = instance();
		r.cfgr_ = (r.cfgr_ & ~0x3cff00) | (1 << 16) | mul | adc_prediv | apb1_prediv |apb2_prediv;
		// Enable PLL
		r.cr_ = r.cr_ | (1 << 24);
		// Wait untill PLL settles down
		while (!(r.cr_ & (1 << 25))) {}
		// Set the PLL as clock source
		r.cfgr_ = r.cfgr_ | 2;
		// Wait until PLL is selected as system clock
		while ((r.cfgr_ & 0xc) != 8) {}
	}

	static void enable_port(Port_address p)
	{
		instance().apb2enr_ = instance().apb2enr_ | (1 << apb2enr_index(p));
	}

	static void disable_port(Port_address p)
	{
		instance().apb2enr_ = instance().apb2enr_ & ~(1 << apb2enr_index(p));
	}

private:
	Register cr_;
	Register cfgr_;
	Register cir_;
	Register apb2rstr_;
	Register apb1rstr_;
	Register ahbenr_;
	Register apb2enr_;
	Register apb1enr_;
	Register bdcr_;
	Register csr_;
};

static_assert(sizeof(Rcc) == 10 * sizeof(Register));

} // namespace clock


namespace gpio {

enum mode {
	input = 0,
	output10MHz,
	output2MHz,
	output50MHz
};

enum cnf {
/* input configuration */
	analog = 0,
	floating,
	pull_up_down,
	reserved,
/* output configuration */
	gpio_push_pull = 0,
	gpio_open_drain,
	alternate_push_pull,
	alternate_open_drain
};



template<Port_address PortAddr>
class Port: public Device<Port<PortAddr>, PortAddr> {
    using ThisType = Port<PortAddr>;

public:
	template<unsigned PinNr> requires (PinNr < 16)
	struct Pin {
		static constexpr std::uint32_t no = PinNr < 8 ? PinNr * 4 : (PinNr - 8) * 4;
		static constexpr std::uint32_t cfgmask = ~(0x0f << no);

		static void configure(mode m, cnf c)
		{
			const std::uint32_t cnf_mode = ((c << 2) | m) << no;
			if constexpr (PinNr < 8) ThisType::instance().crl_ = (ThisType::instance().crl_ & cfgmask) | cnf_mode;
			else ThisType::instance().crh_ = (ThisType::instance().crh_ & cfgmask) | cnf_mode;
		}

		static void set(bool value=true)
		{
			if (value) ThisType::instance().bsrr_ = 1 << PinNr;
			else ThisType::instance().brr_ = 1 << PinNr;
		}

		static void toggle()
		{
			set(!(ThisType::instance().idr_ & (1 << PinNr)));
		}

		static void clear()
		{
			ThisType::instance().brr_ = 1 << PinNr;
		}

		static bool get() {return ThisType::instance().idr_ & (1 << PinNr);}


		Pin(mode m=input, cnf c=floating)
		{
			ThisType::enable();
			configure(m, c);
		}

		void operator=(bool value) {set(value);}

		operator bool() const {return get();}
	}; // struct pin

	static void enable()
	{
		clock::Rcc::enable_port(PortAddr);
	}

	static void disable()
	{
		clock::Rcc::disable_port(PortAddr);
	}

	void set_pins(const std::uint32_t pins) {bsrr_ = pins;}
	void clear_pins(const std::uint32_t pins) {brr_ = pins;}
	std::uint32_t get_pins() const {return idr_;}

	operator std::uint32_t() const {return idr_;}
	void operator=(std::uint32_t value) {odr_ = value;}

private:
	Register crl_;
	Register crh_;
	Register idr_;
	Register odr_;
	Register bsrr_;
	Register brr_;
	Register lckr_;
   
}; // class Port

static_assert(sizeof(Port<port_a>) == 7 * sizeof(Register));

} //namespace gpio

}} // namespace stm32::f1xx




/*
 * Set up clock at 72 MHz (HSE-PLL)
 *
 * 8M ->   /1    ->   *9   ->   /1         -> 72 MHz
 * HSE -> PreDiv -> PllMul -> AHBprescaler -> System Clock
 *
 * 8 MHz External clock is selected as the source clock (HSE)
 * It is muliplied by 9 with PllMul
 * Then choose Pll as the clock source
 */
static void set_sysclock_pll_72Mhz()
{
	using Rcc = stm32::f1xx::clock::Rcc;
	using Flash = stm32::f1xx::system::Flash;
	// Necessary wait states for Flash for high speeds
	Flash::access_control(Flash::wait2);
	// Enable HSE
	Rcc::high_speed_source(Rcc::external);
	// Set PLL to HSE, multiplication factor to 9, ADC prescaler /6,
	// APB1 prescaler to /2 and APB2 prescaler to /1
	Rcc::hs_pll(Rcc::x9, Rcc::adc_by6, Rcc::apb1_by2, Rcc::apb2_by1);
}


// SysTick interrupt handler
extern "C" void SysTick_Handler(void)
{
}



using PortC = stm32::f1xx::gpio::Port<stm32::f1xx::port_c>;
using LedPin = PortC::Pin<13>;
using TickPin = PortC::Pin<14>;

int main()
{
	namespace sfg = stm32::f1xx::gpio;
	using SysTick = stm32::f1xx::system::SysTick;

	set_sysclock_pll_72Mhz();
	LedPin led{sfg::output50MHz, sfg::gpio_push_pull};
	TickPin tick{sfg::output50MHz, sfg::gpio_push_pull};
	SysTick::start(8999); // (72MHz / 8) / 9000 = 1ms
	tick = false;
	while (true) {
		led = true;
		for (std::uint32_t counter = 0; counter < 1000; ++counter) {
			while(!SysTick::reloaded()) stm32::sleep();
			tick = !tick;
			if (counter == 950) led = false;
		}
	}
}
