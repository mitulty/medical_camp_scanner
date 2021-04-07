LIB=./lib
DEVICE=atmega2560
CFLAGS?=-g -Wno-unused-value -Wno-unused-result 
AVR_GCC= avr-gcc -Wall -O0 -mmcu=$(DEVICE) -std=gnu99
all: compile upload clean
compile: adc_sensors.o buzzer_bargraph.o color_sensor.o lcd.o motor_control.o uart.o position_control_interrupt.o main.c path_finder.o atmega_to_esp32_uart.o
	$(AVR_GCC) $(CFLAGS)  -c main.c -o main.o
	$(AVR_GCC) -o robot_controller.elf main.o path_finder.o adc_sensors.o buzzer_bargraph.o color_sensor.o lcd.o motor_control.o uart.o position_control_interrupt.o atmega_to_esp32_uart.o
	avr-objcopy -j .text -j .data -O ihex robot_controller.elf robot_controller.hex
	avr-size --format=avr --mcu=$(DEVICE) robot_controller.elf

upload:
	avrdude -c stk500v2 -p m2560 -P /dev/ttyACM0 -U flash:w:robot_controller.hex:i

adc_sensors.o: $(LIB)/adc_sensors.c
		$(AVR_GCC) $(CFLAGS) -c $(LIB)/adc_sensors.c

buzzer_bargraph.o: $(LIB)/buzzer_bargraph.c
		$(AVR_GCC) $(CFLAGS) -c $(LIB)/buzzer_bargraph.c

color_sensor.o: $(LIB)/color_sensor.c
		$(AVR_GCC) $(CFLAGS) -c $(LIB)/color_sensor.c

lcd.o: $(LIB)/lcd.c
		$(AVR_GCC) $(CFLAGS) -c $(LIB)/lcd.c

motor_control.o: $(LIB)/motor_control.c
		$(AVR_GCC) $(CFLAGS) -c $(LIB)/motor_control.c

uart.o: $(LIB)/uart.c 
		$(AVR_GCC) $(CFLAGS) -c $(LIB)/uart.c
		
atmega_to_esp32_uart.o: $(LIB)/atmega_to_esp32_uart.c 
		$(AVR_GCC) $(CFLAGS) -c $(LIB)/atmega_to_esp32_uart.c

position_control_interrupt.o: $(LIB)/position_control_interrupt.c
		$(AVR_GCC) $(CFLAGS) -c $(LIB)/position_control_interrupt.c

path_finder.o: path_finder.c
		$(AVR_GCC) $(CFLAGS) -c path_finder.c -lm

clean:
		$(RM) -rf *.o
		$(RM) -rf robot_controller.elf
		$(RM) -rf robot_controller.hex
		$(RM) -rf path_finder