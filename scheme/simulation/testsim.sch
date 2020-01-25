EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L pspice:R R1
U 1 1 5E190987
P 3700 2700
F 0 "R1" H 3768 2746 50  0000 L CNN
F 1 "1k" H 3768 2655 50  0000 L CNN
F 2 "" H 3700 2700 50  0001 C CNN
F 3 "~" H 3700 2700 50  0001 C CNN
	1    3700 2700
	1    0    0    -1  
$EndComp
$Comp
L pspice:0 #GND02
U 1 1 5E190CBC
P 3700 3250
F 0 "#GND02" H 3700 3150 50  0001 C CNN
F 1 "0" H 3700 3339 50  0000 C CNN
F 2 "" H 3700 3250 50  0001 C CNN
F 3 "~" H 3700 3250 50  0001 C CNN
	1    3700 3250
	1    0    0    -1  
$EndComp
$Comp
L Simulation_SPICE:VSIN V1
U 1 1 5E19171F
P 2700 2850
F 0 "V1" H 2830 2941 50  0000 L CNN
F 1 "VSIN" H 2830 2850 50  0000 L CNN
F 2 "" H 2700 2850 50  0001 C CNN
F 3 "~" H 2700 2850 50  0001 C CNN
F 4 "Y" H 2700 2850 50  0001 L CNN "Spice_Netlist_Enabled"
F 5 "V" H 2700 2850 50  0001 L CNN "Spice_Primitive"
F 6 "sin(0 20m 1k)" H 2830 2759 50  0000 L CNN "Spice_Model"
	1    2700 2850
	1    0    0    -1  
$EndComp
$Comp
L pspice:0 #GND01
U 1 1 5E191ED0
P 2700 3250
F 0 "#GND01" H 2700 3150 50  0001 C CNN
F 1 "0" H 2700 3339 50  0000 C CNN
F 2 "" H 2700 3250 50  0001 C CNN
F 3 "~" H 2700 3250 50  0001 C CNN
	1    2700 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 3050 2700 3250
$Comp
L tl:tl072 U1
U 1 1 5E192915
P 4250 1800
F 0 "U1" H 4594 1846 50  0000 L CNN
F 1 "tl072" H 4594 1755 50  0000 L CNN
F 2 "" H 4250 1800 50  0001 C CNN
F 3 "~" H 4250 1800 50  0001 C CNN
F 4 "X" H 4250 1800 50  0001 C CNN "Spice_Primitive"
F 5 "TL072" H 4250 1800 50  0001 C CNN "Spice_Model"
F 6 "Y" H 4250 1800 50  0001 C CNN "Spice_Netlist_Enabled"
F 7 "/home/vc/Downloads/TL072.301" H 4250 1800 50  0001 C CNN "Spice_Lib_File"
	1    4250 1800
	1    0    0    -1  
$EndComp
$Comp
L pspice:VSOURCE V2
U 1 1 5E1931C7
P 5950 1900
F 0 "V2" H 6178 1946 50  0000 L CNN
F 1 "15" H 6178 1855 50  0000 L CNN
F 2 "" H 5950 1900 50  0001 C CNN
F 3 "~" H 5950 1900 50  0001 C CNN
	1    5950 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	5950 900  5400 900 
Wire Wire Line
	4150 900  4150 1500
Wire Wire Line
	5950 3000 5400 3000
Wire Wire Line
	4150 3000 4150 2100
$Comp
L pspice:R R2
U 1 1 5E194E9D
P 4600 2450
F 0 "R2" V 4395 2450 50  0000 C CNN
F 1 "10k" V 4486 2450 50  0000 C CNN
F 2 "" H 4600 2450 50  0001 C CNN
F 3 "~" H 4600 2450 50  0001 C CNN
	1    4600 2450
	0    1    1    0   
$EndComp
Wire Wire Line
	3950 1900 3950 2450
Wire Wire Line
	3950 2450 3700 2450
Wire Wire Line
	4850 2450 4850 1800
Wire Wire Line
	4850 1800 4550 1800
Wire Wire Line
	4350 2450 3950 2450
Connection ~ 3950 2450
$Comp
L pspice:VSOURCE V4
U 1 1 5E198706
P 2700 2200
F 0 "V4" H 2928 2246 50  0000 L CNN
F 1 "500m" H 2928 2155 50  0000 L CNN
F 2 "" H 2700 2200 50  0001 C CNN
F 3 "~" H 2700 2200 50  0001 C CNN
	1    2700 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 2500 2700 2650
$Comp
L pspice:C C1
U 1 1 5E1999AF
P 2950 1500
F 0 "C1" V 2635 1500 50  0000 C CNN
F 1 "0.1u" V 2726 1500 50  0000 C CNN
F 2 "" H 2950 1500 50  0001 C CNN
F 3 "~" H 2950 1500 50  0001 C CNN
	1    2950 1500
	0    1    1    0   
$EndComp
Wire Wire Line
	2700 1500 2700 1900
Wire Wire Line
	3200 1500 3400 1500
Wire Wire Line
	3950 1500 3950 1700
$Comp
L pspice:R R3
U 1 1 5E19B7DE
P 3400 1900
F 0 "R3" H 3468 1946 50  0000 L CNN
F 1 "100k" H 3468 1855 50  0000 L CNN
F 2 "" H 3400 1900 50  0001 C CNN
F 3 "~" H 3400 1900 50  0001 C CNN
	1    3400 1900
	1    0    0    -1  
$EndComp
$Comp
L pspice:0 #GND04
U 1 1 5E19BFD6
P 3400 2300
F 0 "#GND04" H 3400 2200 50  0001 C CNN
F 1 "0" H 3400 2389 50  0000 C CNN
F 2 "" H 3400 2300 50  0001 C CNN
F 3 "~" H 3400 2300 50  0001 C CNN
	1    3400 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	3400 2300 3400 2150
Wire Wire Line
	3400 1650 3400 1500
Connection ~ 3400 1500
Wire Wire Line
	3400 1500 3950 1500
Wire Wire Line
	5950 1600 5950 900 
Wire Wire Line
	5950 2200 5950 3000
$Comp
L pspice:R R4
U 1 1 5E19F52F
P 5400 1400
F 0 "R4" H 5468 1446 50  0000 L CNN
F 1 "100" H 5468 1355 50  0000 L CNN
F 2 "" H 5400 1400 50  0001 C CNN
F 3 "~" H 5400 1400 50  0001 C CNN
	1    5400 1400
	1    0    0    -1  
$EndComp
$Comp
L pspice:R R5
U 1 1 5E19FA60
P 5400 2600
F 0 "R5" H 5468 2646 50  0000 L CNN
F 1 "100" H 5468 2555 50  0000 L CNN
F 2 "" H 5400 2600 50  0001 C CNN
F 3 "~" H 5400 2600 50  0001 C CNN
	1    5400 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 1150 5400 900 
Connection ~ 5400 900 
Wire Wire Line
	5400 900  5200 900 
Wire Wire Line
	5400 3000 5400 2850
Connection ~ 5400 3000
Wire Wire Line
	5400 3000 4150 3000
Wire Wire Line
	5400 2350 5400 2250
$Comp
L pspice:0 #GND03
U 1 1 5E1A0BAA
P 5300 2050
F 0 "#GND03" H 5300 1950 50  0001 C CNN
F 1 "0" H 5300 2139 50  0000 C CNN
F 2 "" H 5300 2050 50  0001 C CNN
F 3 "~" H 5300 2050 50  0001 C CNN
	1    5300 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 2050 5300 2000
Wire Wire Line
	5300 2000 5400 2000
Connection ~ 5400 2000
Wire Wire Line
	5400 2000 5400 1650
$Comp
L tl:tl072 U2
U 1 1 5E1A19C9
P 5300 3450
F 0 "U2" H 5644 3496 50  0000 L CNN
F 1 "tl072" H 5644 3405 50  0000 L CNN
F 2 "" H 5300 3450 50  0001 C CNN
F 3 "~" H 5300 3450 50  0001 C CNN
F 4 "X" H 5300 3450 50  0001 C CNN "Spice_Primitive"
F 5 "TL072" H 5300 3450 50  0001 C CNN "Spice_Model"
F 6 "Y" H 5300 3450 50  0001 C CNN "Spice_Netlist_Enabled"
F 7 "/home/vc/Downloads/TL072.301" H 5300 3450 50  0001 C CNN "Spice_Lib_File"
	1    5300 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 3150 5200 900 
Connection ~ 5200 900 
Wire Wire Line
	5200 900  4150 900 
Wire Wire Line
	5200 3750 6050 3750
Wire Wire Line
	6050 3750 6050 3000
Wire Wire Line
	6050 3000 5950 3000
Connection ~ 5950 3000
Wire Wire Line
	5000 1800 4850 1800
Connection ~ 4850 1800
Wire Wire Line
	5000 3350 5000 1800
Wire Wire Line
	5000 3550 4800 3550
$Comp
L pspice:R R6
U 1 1 5E1A64DD
P 6700 3350
F 0 "R6" H 6768 3396 50  0000 L CNN
F 1 "1k" H 6768 3305 50  0000 L CNN
F 2 "" H 6700 3350 50  0001 C CNN
F 3 "~" H 6700 3350 50  0001 C CNN
	1    6700 3350
	1    0    0    -1  
$EndComp
$Comp
L pspice:0 #GND05
U 1 1 5E1A7636
P 6700 3800
F 0 "#GND05" H 6700 3700 50  0001 C CNN
F 1 "0" H 6700 3889 50  0000 C CNN
F 2 "" H 6700 3800 50  0001 C CNN
F 3 "~" H 6700 3800 50  0001 C CNN
	1    6700 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	5600 3450 6350 3450
Wire Wire Line
	6350 3450 6350 3100
Wire Wire Line
	6350 3100 6700 3100
Wire Wire Line
	6700 3600 6700 3800
Wire Wire Line
	3700 2950 3700 3250
Wire Wire Line
	4800 3550 4800 2750
Wire Wire Line
	4800 2750 5300 2750
Wire Wire Line
	5300 2750 5300 2250
Wire Wire Line
	5300 2250 5400 2250
Connection ~ 5400 2250
Wire Wire Line
	5400 2250 5400 2000
$EndSCHEMATC