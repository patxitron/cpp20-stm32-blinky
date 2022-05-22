CC = arm-unknown-eabi-gcc
CXX = arm-unknown-eabi-g++
FLASH = st-flash
OBJCOPY = arm-unknown-eabi-objcopy
OBJDUMP = arm-unknown-eabi-objdump
TARGET_MCU = STM32F103xB
CFLAGS = -mcpu=cortex-m3 -mthumb -std=c11 -pedantic -Wall -Wextra -D$(TARGET_MCU)
CXXFLAGS = -mcpu=cortex-m3 -mthumb -std=c++20 -pedantic -Wall -Wextra -Wconversion -Werror -fno-exceptions -fno-rtti -D$(TARGET_MCU)
LDFLAGS = --specs=nosys.specs -nostdlib -pedantic -Wall -Wextra -Wl,--print-memory-usage -Xlinker -Map=$(TARGET).map
DEBUGFLAGS = -O0 -g
OPTFLAGS = -Os

CXXSOURCES = main.cpp
ASMSOURCES = core.S

CXXOBJECTS = $(CXXSOURCES:%.cpp=%.o)
ASMOBJECTS = $(ASMSOURCES:%.S=%.o)

TARGET = firmware

.PHONY: clean all release debug

all: debug

release: CFLAGS += $(OPTFLAGS)
release: CXXFLAGS += $(OPTFLAGS)
release: LDFLAGS += $(OPTFLAGS)
release: $(TARGET).elf $(TARGET).bin $(TARGET).hex $(TARGET).lst

debug: CFLAGS += $(DEBUGFLAGS)
debug: CXXFLAGS += $(DEBUGFLAGS)
debug: LDFLAGS += $(DEBUGFLAGS)
debug: $(TARGET).elf $(TARGET).bin $(TARGET).hex $(TARGET).lst

flash: $(TARGET).bin
	$(FLASH) write $(TARGET).bin 0x08000000

%.hex: %.elf
	$(OBJCOPY) -O ihex $< $@

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@

%.lst: %.elf
	$(OBJDUMP) -x -S $(TARGET).elf > $@

$(TARGET).elf: $(ASMOBJECTS) $(CXXOBJECTS) $(LDSCRIPT)
	$(CXX) -o $@ $(LDFLAGS) -T./$(TARGET_MCU).ld $(ASMOBJECTS) $(CXXOBJECTS)

$(CXXOBJECTS): %.o: %.cpp
	$(CXX) -o $@ $(CXXFLAGS) -c $<

$(ASMOBJECTS): %o: %S
	$(CC) -o $@ -x assembler-with-cpp $(CFLAGS) -c $<

clean:
	rm -f *.elf *.bin *.map *.lst *.hex *.o
