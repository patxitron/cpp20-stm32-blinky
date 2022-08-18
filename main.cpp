#include <cstddef>
#include <cstdint>
#include <utility>
#include <type_traits>
#include <concepts>

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
    return p == port_a ? 2
                       : (p == port_b ? 3
                                      : (p == port_c ? 4
                                                     : (p == port_d ? 5
                                                                    : (6))));
}
} // Anonymous


namespace system {

class SysTick: public Device<SysTick, sys_tick>
{
public:
    static std::uint32_t hz;

    static void start(std::uint32_t count, bool divide_by_8 = true
                     ,bool enable_interrupt = true)
    {
        SysTick& st = instance();
        st.ctrl_ = (enable_interrupt ? 2 : 0) | (divide_by_8 ? 0 : 4);
        st.load_ = count;
        st.val_ = 0;
        st.ctrl_ = st.ctrl_ | 1;
        us_ = 1000000 * (8 * divide_by_8) / (hz / (count + 1));
        ms_ = 1000 * (8 * divide_by_8) / (hz / (count + 1));
    }

    static void stop()
    {
        instance().ctrl_ = instance().ctrl_ & ~1;
    }

    static bool reloaded() {return instance().ctrl_ & (1 << 16);}
    static std::uint32_t calibration() {return instance().calib_;}

    static std::uint32_t microseconds() {return us_;}
    static std::uint32_t milliseconds() {return ms_;}

private:
    static std::uint32_t us_;
    static std::uint32_t ms_;
    Register ctrl_;         /* SYSTICK control and status register, Address offset: 0x00 */
    Register load_;         /* SYSTICK reload value register,             Address offset: 0x04 */
    Register val_;            /* SYSTICK current value register,            Address offset: 0x08 */
    Register calib_;        /* SYSTICK calibration value register,    Address offset: 0x0C */
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

    static void access_control(Latency wait_states, bool prefetch=true
                              ,bool half_cycle=false)
    {
        Flash& f = instance();
        f.acr_ = (f.acr_ & ~(0x1f)) | wait_states | (prefetch ? 0x10 : 0)
                                    | (half_cycle ? 8 : 0);
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

    static void hs_pll(PllMul mul, AdcPre adc_prediv, Apb1Pre apb1_prediv
                      ,Apb2Pre apb2_prediv)
    {
        Rcc& r = instance();
        r.cfgr_ = (r.cfgr_ & ~0x3cff00) | (1 << 16) | mul | adc_prediv
                                        | apb1_prediv | apb2_prediv;
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
    static constexpr Port_address addr = PortAddr;

    static void enable() {clock::Rcc::enable_port(PortAddr);}
    static void disable() {clock::Rcc::disable_port(PortAddr);}

    static void set_pins(const std::uint32_t pins)
    {
        ThisType::instance().bsrr_ = pins;
    }
    static void clear_pins(const std::uint32_t pins)
    {
        ThisType::instance().brr_ = pins;
    }
    static std::uint32_t get_pins() {return ThisType::instance().idr_;}
    template<std::uint_fast8_t Pn> static void configure_pin(mode m, cnf c)
            requires (Pn < 16)
    {
        static constexpr std::uint32_t no = Pn < 8 ? Pn * 4 : (Pn - 8) * 4;
        static constexpr std::uint32_t cfgmask = ~(0x0f << no);
        const std::uint32_t cnf_mode = ((c << 2) | m) << no;
        ThisType& p = ThisType::instance();
        if constexpr (Pn < 8) p.crl_ = (p.crl_ & cfgmask) | cnf_mode;
        else p.crh_ = (p.crh_ & cfgmask) | cnf_mode;
    }

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

template<typename P>
concept gpio_port = requires(P p) {
    []<Port_address A>(const Port<A>& pa){}(p);
};

template<gpio_port Gp, std::uint_fast8_t PinNr> requires (PinNr < 16)
class Pin {
    static constexpr std::uint32_t no = PinNr < 8 ? PinNr * 4 : (PinNr - 8) * 4;
    static constexpr std::uint32_t cfgmask = ~(0x0f << no);

public:
    using Port = Gp;
    static constexpr std::uint_fast8_t pin = PinNr;

    static void configure(mode m, cnf c)
    {
        Port::template configure_pin<pin>(m, c);
    }
    static void set(bool value=true)
    {
        if (value) Port::set_pins(1 << pin);
        else Port::clear_pins(1 << pin);
    }
    static void toggle()
    {
        set(!( Port::get_pins() & (1 << pin)));
    }
    static void clear()
    {
        Port::clear_pins(1 << pin);
    }
    static bool get() {return Port::get_pins() & (1 << pin);}

    Pin(mode m=input, cnf c=floating)
    {
        Port::enable();
        configure(m, c);
    }

    void operator=(bool value) {set(value);}

    operator bool() const {return get();}
}; // struct pin

template<typename P>
concept gpio_pin = requires(P p) {
    []<gpio_port A, std::uint_fast8_t N>(const Pin<A, N>& pn){}(p);
};

} //namespace gpio

}} // namespace stm32::f1xx

/**
 * Monotonic clock class. It could never go backwards nor be resetted.
 *
 */
struct Uptime {
    using systick_t = std::uint_fast64_t;
    using SysTick = stm32::f1xx::system::SysTick;

    Uptime(): ticks_{0} {}
    Uptime(const Uptime&) = delete;
    Uptime(Uptime&&) = delete;

    Uptime& operator=(const Uptime&) = delete;
    Uptime& operator=(Uptime&&) = delete;
    void operator++() {increment();}
    void operator++(int) {increment();}
    void operator+=(uint32_t ammount) {increment(ammount);}
    operator double() const {return seconds();}
    operator systick_t() const {return ticks_;}

    void increment(uint32_t ammount=1) {ticks_ = ticks_ + ammount;}

    systick_t ticks() const {return ticks_;}
    double seconds(systick_t since=0) const
    {
        if (since > ticks_) return 0.0;
        return (static_cast<double>(ticks_ - since) * 1000000.0)
               / SysTick::microseconds();
    }
    template<std::unsigned_integral Rt=std::uint32_t>
    Rt milliseconds(systick_t since=0) const
    {
        if (since > ticks_) return 0;
        return static_cast<Rt>((ticks_ - since) / SysTick::milliseconds());
    }

private:
    volatile systick_t ticks_;
};

static Uptime uptime;

/*
 * Set up clock at 72 MHz (HSE-PLL)
 *
 * 8M ->     /1        ->     *9     ->     /1                 -> 72 MHz
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
    stm32::f1xx::system::SysTick::hz = 72000000;
}

// SysTick interrupt handler
extern "C" void SysTick_Handler(void)
{
    uptime++;
}

template<stm32::f1xx::gpio::gpio_pin Ld, stm32::f1xx::gpio::gpio_pin Tk>
class MyApp
{
public:
    using LedPin = Ld;
    using TickPin = Tk;

    MyApp(const Uptime& upt)
            :upt_{upt}
            ,led_{stm32::f1xx::gpio::output50MHz
                 ,stm32::f1xx::gpio::gpio_push_pull}
            ,tick_{stm32::f1xx::gpio::output50MHz
                  ,stm32::f1xx::gpio::gpio_push_pull}
            ,counter_{0}
    {
        tick_ = false;
        led_ = false;
    }

    void operator()()
    {
        tick_ = !tick_;
        ++counter_;
        if (counter_ == 950) led_ = true;
        else if (counter_ >= 1000) {
            led_ = false;
            counter_ = 0;
        }
    }

private:
    const Uptime& upt_;
    LedPin led_;
    TickPin tick_;
    std::uint_fast16_t counter_;
};


int main()
{
    using SysTick = stm32::f1xx::system::SysTick;
    using PortC = stm32::f1xx::gpio::Port<stm32::f1xx::port_c>;
    using LedPin = stm32::f1xx::gpio::Pin<PortC, 13>;
    using TickPin = stm32::f1xx::gpio::Pin<PortC, 14>;

    set_sysclock_pll_72Mhz();
    MyApp<LedPin, TickPin> app(uptime);
    SysTick::start(8999); // (72MHz / 8) / 9000 = 1ms
    while (true) {
        while(!SysTick::reloaded()) stm32::sleep();
        app();
    }
}
