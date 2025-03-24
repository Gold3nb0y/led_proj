#define a series of objects in an array
OBJS := build/main.o build/common.o build/write.o build/i2c.o build/lcd.o build/gyro.o
CC = arm-none-eabi-gcc
CFLAGS = -g -ggdb -Wall -O0 -ffreestanding -ffunction-sections -fdata-sections -fsingle-precision-constant -Wall -Wextra -Wpedantic -Wundef -Wshadow -Wredundant-decls -Wstrict-prototypes -Wmissing-prototypes -Wno-variadic-macros -Wno-unused-result -Wno-unused-parameter -Wno-unused-label -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -nostdlib
LIBS = -I./include/ -I./libs/libopencm3/include/ -L./libs/libopencm3/lib/
LFLAGS = -Wl,--no-warn-rwx-segment -specs=nano.specs -specs=nosys.specs -lnosys -lm
target = blink

all: $(target)
	arm-none-eabi-objcopy -O binary $(target).elf $(target).bin
	arm-none-eabi-objcopy -O ihex $(target).elf $(target).hex

#link everything together
$(target): $(OBJS)# all requires the object files to run
	$(CC) $(CFLAGS) $(LIBS) -o $(target).elf $(OBJS) -T./linker.ld -l:libopencm3_stm32f4.a -v $(LFLAGS)

build/%.o : src/%.c
	$(CC) -c $(CFLAGS) $(LIBS) $< -o $@ 

build/%.o : src/%.S
	$(CC) -c $(CFLAGS) $(LIBS) $< -o $@

clean:
	rm build/* $(target).* 

flash:
	openocd -f tigard-swd.cfg -f target/stm32f4x.cfg -c "program $(target).elf"
