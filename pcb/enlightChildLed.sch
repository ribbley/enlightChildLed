EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:Worldsemi
LIBS:stm32
LIBS:switches
LIBS:battery_management
LIBS:enlightChildLed-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 3
Title "Enlight Child LED (worktitle)"
Date ""
Rev "0.0.1"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Sheet
S 1100 900  3300 2550
U 5A1F6059
F0 "Control and Power" 60
F1 "control_and_power.sch" 60
F2 "SER_OUT" I R 4400 1100 60 
$EndSheet
$Sheet
S 9550 850  1300 900 
U 5A1F955C
F0 "LED Matrix" 60
F1 "led_matrix.sch" 60
F2 "SER_INPUT" I L 9550 1550 60 
$EndSheet
Wire Wire Line
	4400 1100 6650 1100
Wire Wire Line
	6650 1100 6650 1550
Wire Wire Line
	6650 1550 9550 1550
$EndSCHEMATC
