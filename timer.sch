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
L nrg3-cache:GND #PWR07
U 1 1 5DFB4824
P 2400 4375
F 0 "#PWR07" H 2400 4125 50  0001 C CNN
F 1 "GND" H 2405 4202 50  0000 C CNN
F 2 "" H 2400 4375 50  0001 C CNN
F 3 "" H 2400 4375 50  0001 C CNN
	1    2400 4375
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR06
U 1 1 5DFB4CDE
P 2400 1525
F 0 "#PWR06" H 2400 1375 50  0001 C CNN
F 1 "+5V" H 2415 1698 50  0000 C CNN
F 2 "" H 2400 1525 50  0001 C CNN
F 3 "" H 2400 1525 50  0001 C CNN
	1    2400 1525
	1    0    0    -1  
$EndComp
$Comp
L Display_Character:KCSC02-105 U5
U 1 1 5DFB5EC3
P 7425 1575
F 0 "U5" H 7425 2242 50  0000 C CNN
F 1 "KCSC02-105" H 7425 2151 50  0000 C CNN
F 2 "Display_7Segment:KCSC02-105" H 7425 975 50  0001 C CNN
F 3 "http://www.kingbright.com/attachments/file/psearch/000/00/00/KCSC02-105(Ver.9A).pdf" H 6925 2050 50  0001 L CNN
	1    7425 1575
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC595 U2
U 1 1 5DFC1911
P 6200 1675
F 0 "U2" H 6325 2375 50  0000 C CNN
F 1 "74HC595" H 6425 2250 50  0000 C CNN
F 2 "Package_SO:SOIC-16_3.9x9.9mm_P1.27mm" H 6200 1675 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/sn74hc595.pdf" H 6200 1675 50  0001 C CNN
	1    6200 1675
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R8
U 1 1 5DFC4222
P 6825 1275
F 0 "R8" V 6775 1400 50  0000 C CNN
F 1 "150R" V 6675 1275 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6825 1275 50  0001 C CNN
F 3 "~" H 6825 1275 50  0001 C CNN
	1    6825 1275
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R9
U 1 1 5DFC4816
P 6825 1375
F 0 "R9" V 6775 1500 50  0000 C CNN
F 1 "R_Small" V 6720 1375 50  0001 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6825 1375 50  0001 C CNN
F 3 "~" H 6825 1375 50  0001 C CNN
	1    6825 1375
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R10
U 1 1 5DFC58B5
P 6825 1475
F 0 "R10" V 6775 1600 50  0000 C CNN
F 1 "R_Small" V 6720 1475 50  0001 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6825 1475 50  0001 C CNN
F 3 "~" H 6825 1475 50  0001 C CNN
	1    6825 1475
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R11
U 1 1 5DFC5A5F
P 6825 1575
F 0 "R11" V 6775 1700 50  0000 C CNN
F 1 "R_Small" V 6720 1575 50  0001 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6825 1575 50  0001 C CNN
F 3 "~" H 6825 1575 50  0001 C CNN
	1    6825 1575
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R12
U 1 1 5DFC5CF0
P 6825 1675
F 0 "R12" V 6775 1800 50  0000 C CNN
F 1 "R_Small" V 6720 1675 50  0001 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6825 1675 50  0001 C CNN
F 3 "~" H 6825 1675 50  0001 C CNN
	1    6825 1675
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R13
U 1 1 5DFC5ED9
P 6825 1775
F 0 "R13" V 6775 1900 50  0000 C CNN
F 1 "R_Small" V 6720 1775 50  0001 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6825 1775 50  0001 C CNN
F 3 "~" H 6825 1775 50  0001 C CNN
	1    6825 1775
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R14
U 1 1 5DFC616A
P 6825 1875
F 0 "R14" V 6775 2000 50  0000 C CNN
F 1 "R_Small" V 6720 1875 50  0001 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6825 1875 50  0001 C CNN
F 3 "~" H 6825 1875 50  0001 C CNN
	1    6825 1875
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R15
U 1 1 5DFC650C
P 6825 1975
F 0 "R15" V 6775 2100 50  0000 C CNN
F 1 "R_Small" V 6720 1975 50  0001 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6825 1975 50  0001 C CNN
F 3 "~" H 6825 1975 50  0001 C CNN
	1    6825 1975
	0    1    1    0   
$EndComp
Wire Wire Line
	6600 1275 6725 1275
Wire Wire Line
	6600 1375 6725 1375
Wire Wire Line
	6600 1475 6725 1475
Wire Wire Line
	6600 1575 6725 1575
Wire Wire Line
	6600 1675 6725 1675
Wire Wire Line
	6600 1775 6725 1775
Wire Wire Line
	6600 1875 6725 1875
Wire Wire Line
	6600 1975 6725 1975
Wire Wire Line
	6925 1275 7125 1275
Wire Wire Line
	6925 1375 7125 1375
Wire Wire Line
	6925 1475 7125 1475
Wire Wire Line
	6925 1575 7125 1575
Wire Wire Line
	6925 1675 7125 1675
Wire Wire Line
	6925 1775 7125 1775
Wire Wire Line
	6925 1875 7125 1875
Wire Wire Line
	6925 1975 7125 1975
$Comp
L power:GND #PWR016
U 1 1 5DFCAD26
P 6200 2450
F 0 "#PWR016" H 6200 2200 50  0001 C CNN
F 1 "GND" H 6205 2277 50  0000 C CNN
F 2 "" H 6200 2450 50  0001 C CNN
F 3 "" H 6200 2450 50  0001 C CNN
	1    6200 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	6200 2450 6200 2400
$Comp
L power:+5V #PWR015
U 1 1 5DFCB4EC
P 6200 1000
F 0 "#PWR015" H 6200 850 50  0001 C CNN
F 1 "+5V" H 6215 1173 50  0000 C CNN
F 2 "" H 6200 1000 50  0001 C CNN
F 3 "" H 6200 1000 50  0001 C CNN
	1    6200 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	6200 1000 6200 1050
$Comp
L Display_Character:KCSC02-105 U6
U 1 1 5DFD2178
P 7450 3525
F 0 "U6" H 7450 4192 50  0000 C CNN
F 1 "KCSC02-105" H 7450 4101 50  0000 C CNN
F 2 "Display_7Segment:KCSC02-105" H 7450 2925 50  0001 C CNN
F 3 "http://www.kingbright.com/attachments/file/psearch/000/00/00/KCSC02-105(Ver.9A).pdf" H 6950 4000 50  0001 L CNN
	1    7450 3525
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC595 U3
U 1 1 5DFD217E
P 6225 3625
F 0 "U3" H 6350 4325 50  0000 C CNN
F 1 "74HC595" H 6450 4200 50  0000 C CNN
F 2 "Package_SO:SOIC-16_3.9x9.9mm_P1.27mm" H 6225 3625 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/sn74hc595.pdf" H 6225 3625 50  0001 C CNN
	1    6225 3625
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R16
U 1 1 5DFD2184
P 6850 3225
F 0 "R16" V 6800 3350 50  0000 C CNN
F 1 "150R" V 6700 3225 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6850 3225 50  0001 C CNN
F 3 "~" H 6850 3225 50  0001 C CNN
	1    6850 3225
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R17
U 1 1 5DFD218A
P 6850 3325
F 0 "R17" V 6800 3450 50  0000 C CNN
F 1 "R_Small" V 6745 3325 50  0001 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6850 3325 50  0001 C CNN
F 3 "~" H 6850 3325 50  0001 C CNN
	1    6850 3325
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R18
U 1 1 5DFD2190
P 6850 3425
F 0 "R18" V 6800 3550 50  0000 C CNN
F 1 "R_Small" V 6745 3425 50  0001 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6850 3425 50  0001 C CNN
F 3 "~" H 6850 3425 50  0001 C CNN
	1    6850 3425
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R19
U 1 1 5DFD2196
P 6850 3525
F 0 "R19" V 6800 3650 50  0000 C CNN
F 1 "R_Small" V 6745 3525 50  0001 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6850 3525 50  0001 C CNN
F 3 "~" H 6850 3525 50  0001 C CNN
	1    6850 3525
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R20
U 1 1 5DFD219C
P 6850 3625
F 0 "R20" V 6800 3750 50  0000 C CNN
F 1 "R_Small" V 6745 3625 50  0001 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6850 3625 50  0001 C CNN
F 3 "~" H 6850 3625 50  0001 C CNN
	1    6850 3625
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R21
U 1 1 5DFD21A2
P 6850 3725
F 0 "R21" V 6800 3850 50  0000 C CNN
F 1 "R_Small" V 6745 3725 50  0001 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6850 3725 50  0001 C CNN
F 3 "~" H 6850 3725 50  0001 C CNN
	1    6850 3725
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R22
U 1 1 5DFD21A8
P 6850 3825
F 0 "R22" V 6800 3950 50  0000 C CNN
F 1 "R_Small" V 6745 3825 50  0001 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6850 3825 50  0001 C CNN
F 3 "~" H 6850 3825 50  0001 C CNN
	1    6850 3825
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R23
U 1 1 5DFD21AE
P 6850 3925
F 0 "R23" V 6800 4050 50  0000 C CNN
F 1 "R_Small" V 6745 3925 50  0001 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6850 3925 50  0001 C CNN
F 3 "~" H 6850 3925 50  0001 C CNN
	1    6850 3925
	0    1    1    0   
$EndComp
Wire Wire Line
	6625 3225 6750 3225
Wire Wire Line
	6625 3325 6750 3325
Wire Wire Line
	6625 3425 6750 3425
Wire Wire Line
	6625 3525 6750 3525
Wire Wire Line
	6625 3625 6750 3625
Wire Wire Line
	6625 3725 6750 3725
Wire Wire Line
	6625 3825 6750 3825
Wire Wire Line
	6625 3925 6750 3925
Wire Wire Line
	6950 3225 7150 3225
Wire Wire Line
	6950 3325 7150 3325
Wire Wire Line
	6950 3425 7150 3425
Wire Wire Line
	6950 3525 7150 3525
Wire Wire Line
	6950 3625 7150 3625
Wire Wire Line
	6950 3725 7150 3725
Wire Wire Line
	6950 3825 7150 3825
Wire Wire Line
	6950 3925 7150 3925
$Comp
L power:GND #PWR018
U 1 1 5DFD21C4
P 6225 4400
F 0 "#PWR018" H 6225 4150 50  0001 C CNN
F 1 "GND" H 6230 4227 50  0000 C CNN
F 2 "" H 6225 4400 50  0001 C CNN
F 3 "" H 6225 4400 50  0001 C CNN
	1    6225 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	6225 4400 6225 4350
$Comp
L power:+5V #PWR017
U 1 1 5DFD21CB
P 6225 2950
F 0 "#PWR017" H 6225 2800 50  0001 C CNN
F 1 "+5V" H 6240 3123 50  0000 C CNN
F 2 "" H 6225 2950 50  0001 C CNN
F 3 "" H 6225 2950 50  0001 C CNN
	1    6225 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	6225 2950 6225 2975
$Comp
L Display_Character:KCSC02-105 U7
U 1 1 5DFD73B0
P 7475 5475
F 0 "U7" H 7475 6142 50  0000 C CNN
F 1 "KCSC02-105" H 7475 6051 50  0000 C CNN
F 2 "Display_7Segment:KCSC02-105" H 7475 4875 50  0001 C CNN
F 3 "http://www.kingbright.com/attachments/file/psearch/000/00/00/KCSC02-105(Ver.9A).pdf" H 6975 5950 50  0001 L CNN
	1    7475 5475
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC595 U4
U 1 1 5DFD73B6
P 6250 5575
F 0 "U4" H 6375 6275 50  0000 C CNN
F 1 "74HC595" H 6475 6150 50  0000 C CNN
F 2 "Package_SO:SOIC-16_3.9x9.9mm_P1.27mm" H 6250 5575 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/sn74hc595.pdf" H 6250 5575 50  0001 C CNN
	1    6250 5575
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R24
U 1 1 5DFD73BC
P 6875 5175
F 0 "R24" V 6825 5300 50  0000 C CNN
F 1 "150R" V 6725 5175 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6875 5175 50  0001 C CNN
F 3 "~" H 6875 5175 50  0001 C CNN
	1    6875 5175
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R25
U 1 1 5DFD73C2
P 6875 5275
F 0 "R25" V 6825 5400 50  0000 C CNN
F 1 "R_Small" V 6770 5275 50  0001 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6875 5275 50  0001 C CNN
F 3 "~" H 6875 5275 50  0001 C CNN
	1    6875 5275
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R26
U 1 1 5DFD73C8
P 6875 5375
F 0 "R26" V 6825 5500 50  0000 C CNN
F 1 "R_Small" V 6770 5375 50  0001 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6875 5375 50  0001 C CNN
F 3 "~" H 6875 5375 50  0001 C CNN
	1    6875 5375
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R27
U 1 1 5DFD73CE
P 6875 5475
F 0 "R27" V 6825 5600 50  0000 C CNN
F 1 "R_Small" V 6770 5475 50  0001 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6875 5475 50  0001 C CNN
F 3 "~" H 6875 5475 50  0001 C CNN
	1    6875 5475
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R28
U 1 1 5DFD73D4
P 6875 5575
F 0 "R28" V 6825 5700 50  0000 C CNN
F 1 "R_Small" V 6770 5575 50  0001 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6875 5575 50  0001 C CNN
F 3 "~" H 6875 5575 50  0001 C CNN
	1    6875 5575
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R29
U 1 1 5DFD73DA
P 6875 5675
F 0 "R29" V 6825 5800 50  0000 C CNN
F 1 "R_Small" V 6770 5675 50  0001 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6875 5675 50  0001 C CNN
F 3 "~" H 6875 5675 50  0001 C CNN
	1    6875 5675
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R30
U 1 1 5DFD73E0
P 6875 5775
F 0 "R30" V 6825 5900 50  0000 C CNN
F 1 "R_Small" V 6770 5775 50  0001 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6875 5775 50  0001 C CNN
F 3 "~" H 6875 5775 50  0001 C CNN
	1    6875 5775
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R31
U 1 1 5DFD73E6
P 6875 5875
F 0 "R31" V 6825 6000 50  0000 C CNN
F 1 "R_Small" V 6770 5875 50  0001 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6875 5875 50  0001 C CNN
F 3 "~" H 6875 5875 50  0001 C CNN
	1    6875 5875
	0    1    1    0   
$EndComp
Wire Wire Line
	6650 5175 6775 5175
Wire Wire Line
	6650 5275 6775 5275
Wire Wire Line
	6650 5375 6775 5375
Wire Wire Line
	6650 5475 6775 5475
Wire Wire Line
	6650 5575 6775 5575
Wire Wire Line
	6650 5675 6775 5675
Wire Wire Line
	6650 5775 6775 5775
Wire Wire Line
	6650 5875 6775 5875
Wire Wire Line
	6975 5175 7175 5175
Wire Wire Line
	6975 5275 7175 5275
Wire Wire Line
	6975 5375 7175 5375
Wire Wire Line
	6975 5475 7175 5475
Wire Wire Line
	6975 5575 7175 5575
Wire Wire Line
	6975 5675 7175 5675
Wire Wire Line
	6975 5775 7175 5775
Wire Wire Line
	6975 5875 7175 5875
$Comp
L power:GND #PWR020
U 1 1 5DFD73FC
P 6250 6350
F 0 "#PWR020" H 6250 6100 50  0001 C CNN
F 1 "GND" H 6255 6177 50  0000 C CNN
F 2 "" H 6250 6350 50  0001 C CNN
F 3 "" H 6250 6350 50  0001 C CNN
	1    6250 6350
	1    0    0    -1  
$EndComp
Wire Wire Line
	6250 6350 6250 6300
$Comp
L power:+5V #PWR019
U 1 1 5DFD7403
P 6250 4900
F 0 "#PWR019" H 6250 4750 50  0001 C CNN
F 1 "+5V" H 6265 5073 50  0000 C CNN
F 2 "" H 6250 4900 50  0001 C CNN
F 3 "" H 6250 4900 50  0001 C CNN
	1    6250 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	6250 4900 6250 4950
Wire Wire Line
	6600 2175 6775 2175
Wire Wire Line
	6775 2175 6775 2700
Wire Wire Line
	6775 2700 5650 2700
Wire Wire Line
	5650 2700 5650 3225
Wire Wire Line
	5650 3225 5825 3225
Wire Wire Line
	6625 4125 6825 4125
Wire Wire Line
	6825 4125 6825 4625
Wire Wire Line
	6825 4625 5675 4625
Wire Wire Line
	5675 4625 5675 5175
Wire Wire Line
	5675 5175 5850 5175
NoConn ~ 6650 6075
Wire Wire Line
	5800 1875 5625 1875
Wire Wire Line
	5625 1875 5625 2400
Wire Wire Line
	5625 2400 6200 2400
Connection ~ 6200 2400
Wire Wire Line
	6200 2400 6200 2375
Wire Wire Line
	5825 3825 5625 3825
Wire Wire Line
	5625 3825 5625 4350
Wire Wire Line
	5625 4350 6225 4350
Connection ~ 6225 4350
Wire Wire Line
	6225 4350 6225 4325
Wire Wire Line
	5850 5775 5675 5775
Wire Wire Line
	5675 5775 5675 6300
Wire Wire Line
	5675 6300 6250 6300
Connection ~ 6250 6300
Wire Wire Line
	6250 6300 6250 6275
$Comp
L Device:R_Small R6
U 1 1 5DFFF039
P 5325 3525
F 0 "R6" V 5521 3525 50  0000 C CNN
F 1 "1K" V 5430 3525 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 5325 3525 50  0001 C CNN
F 3 "~" H 5325 3525 50  0001 C CNN
	1    5325 3525
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_Small R7
U 1 1 5DFFFD55
P 5350 5475
F 0 "R7" V 5154 5475 50  0000 C CNN
F 1 "1K" V 5245 5475 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 5350 5475 50  0001 C CNN
F 3 "~" H 5350 5475 50  0001 C CNN
	1    5350 5475
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C7
U 1 1 5E000A2D
P 5975 1050
F 0 "C7" V 5746 1050 50  0000 C CNN
F 1 "0.1uF" V 5837 1050 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 5975 1050 50  0001 C CNN
F 3 "~" H 5975 1050 50  0001 C CNN
	1    5975 1050
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C8
U 1 1 5E00193B
P 6000 2975
F 0 "C8" V 5771 2975 50  0000 C CNN
F 1 "0.1uF" V 5862 2975 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6000 2975 50  0001 C CNN
F 3 "~" H 6000 2975 50  0001 C CNN
	1    6000 2975
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C9
U 1 1 5E001E04
P 6050 4950
F 0 "C9" V 5821 4950 50  0000 C CNN
F 1 "0.1uF" V 5912 4950 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6050 4950 50  0001 C CNN
F 3 "~" H 6050 4950 50  0001 C CNN
	1    6050 4950
	0    1    1    0   
$EndComp
Wire Wire Line
	5375 1575 5800 1575
Wire Wire Line
	5425 3525 5825 3525
Wire Wire Line
	5450 5475 5850 5475
Wire Wire Line
	6100 2975 6225 2975
Connection ~ 6225 2975
Wire Wire Line
	6225 2975 6225 3025
Wire Wire Line
	6150 4950 6250 4950
Connection ~ 6250 4950
Wire Wire Line
	6250 4950 6250 4975
Wire Wire Line
	6075 1050 6200 1050
Connection ~ 6200 1050
Wire Wire Line
	6200 1050 6200 1075
Text GLabel 5175 1575 0    50   Input ~ 0
VCC
Text GLabel 5225 3525 0    50   Input ~ 0
VCC
Text GLabel 5250 5475 0    50   Input ~ 0
VCC
Text GLabel 5950 4950 0    50   Input ~ 0
GND
Text GLabel 5900 2975 0    50   Input ~ 0
GND
Text GLabel 5875 1050 0    50   Input ~ 0
GND
$Comp
L Switch:SW_Push SW1
U 1 1 5E0131C0
P 2050 5250
F 0 "SW1" H 2050 5535 50  0000 C CNN
F 1 "UP" H 2050 5444 50  0000 C CNN
F 2 "Button_Switch_SMD:SW_SPST_EVPBF" H 2050 5450 50  0001 C CNN
F 3 "~" H 2050 5450 50  0001 C CNN
	1    2050 5250
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R1
U 1 1 5E013E54
P 1450 5025
F 0 "R1" H 1509 5071 50  0000 L CNN
F 1 "10K" H 1509 4980 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 1450 5025 50  0001 C CNN
F 3 "~" H 1450 5025 50  0001 C CNN
	1    1450 5025
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C2
U 1 1 5E017BB4
P 1450 5500
F 0 "C2" H 1542 5546 50  0000 L CNN
F 1 "0.1uF" H 1542 5455 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 1450 5500 50  0001 C CNN
F 3 "~" H 1450 5500 50  0001 C CNN
	1    1450 5500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR03
U 1 1 5E01B6F3
P 1450 5675
F 0 "#PWR03" H 1450 5425 50  0001 C CNN
F 1 "GND" H 1455 5502 50  0000 C CNN
F 2 "" H 1450 5675 50  0001 C CNN
F 3 "" H 1450 5675 50  0001 C CNN
	1    1450 5675
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR010
U 1 1 5E01B9A1
P 2450 5675
F 0 "#PWR010" H 2450 5425 50  0001 C CNN
F 1 "GND" H 2455 5502 50  0000 C CNN
F 2 "" H 2450 5675 50  0001 C CNN
F 3 "" H 2450 5675 50  0001 C CNN
	1    2450 5675
	1    0    0    -1  
$EndComp
Wire Wire Line
	1450 5675 1450 5600
Wire Wire Line
	2250 5250 2450 5250
Wire Wire Line
	2450 5250 2450 5675
Text GLabel 1350 5250 0    50   Input ~ 0
UP
Wire Wire Line
	1450 5400 1450 5250
Wire Wire Line
	1350 5250 1450 5250
Connection ~ 1450 5250
$Comp
L Device:R_Small R3
U 1 1 5E034322
P 1450 6425
F 0 "R3" H 1509 6471 50  0000 L CNN
F 1 "10K" H 1509 6380 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 1450 6425 50  0001 C CNN
F 3 "~" H 1450 6425 50  0001 C CNN
	1    1450 6425
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C5
U 1 1 5E034328
P 1450 6900
F 0 "C5" H 1542 6946 50  0000 L CNN
F 1 "0.1uF" H 1542 6855 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 1450 6900 50  0001 C CNN
F 3 "~" H 1450 6900 50  0001 C CNN
	1    1450 6900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR09
U 1 1 5E03432E
P 1450 7075
F 0 "#PWR09" H 1450 6825 50  0001 C CNN
F 1 "GND" H 1455 6902 50  0000 C CNN
F 2 "" H 1450 7075 50  0001 C CNN
F 3 "" H 1450 7075 50  0001 C CNN
	1    1450 7075
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR013
U 1 1 5E034334
P 2450 7075
F 0 "#PWR013" H 2450 6825 50  0001 C CNN
F 1 "GND" H 2455 6902 50  0000 C CNN
F 2 "" H 2450 7075 50  0001 C CNN
F 3 "" H 2450 7075 50  0001 C CNN
	1    2450 7075
	1    0    0    -1  
$EndComp
Wire Wire Line
	1450 7075 1450 7000
Wire Wire Line
	2250 6650 2450 6650
Wire Wire Line
	2450 6650 2450 7075
Text GLabel 1350 6650 0    50   Input ~ 0
DOWN
Wire Wire Line
	1450 6800 1450 6650
Wire Wire Line
	1350 6650 1450 6650
Connection ~ 1450 6650
$Comp
L Switch:SW_Push SW3
U 1 1 5E038B5D
P 3875 5250
F 0 "SW3" H 3875 5535 50  0000 C CNN
F 1 "START" H 3875 5444 50  0000 C CNN
F 2 "Button_Switch_SMD:SW_SPST_EVPBF" H 3875 5450 50  0001 C CNN
F 3 "~" H 3875 5450 50  0001 C CNN
	1    3875 5250
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R4
U 1 1 5E038B63
P 3275 5025
F 0 "R4" H 3334 5071 50  0000 L CNN
F 1 "10K" H 3334 4980 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3275 5025 50  0001 C CNN
F 3 "~" H 3275 5025 50  0001 C CNN
	1    3275 5025
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C6
U 1 1 5E038B69
P 3275 5500
F 0 "C6" H 3367 5546 50  0000 L CNN
F 1 "0.1uF" H 3367 5455 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3275 5500 50  0001 C CNN
F 3 "~" H 3275 5500 50  0001 C CNN
	1    3275 5500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR012
U 1 1 5E038B6F
P 3275 5675
F 0 "#PWR012" H 3275 5425 50  0001 C CNN
F 1 "GND" H 3280 5502 50  0000 C CNN
F 2 "" H 3275 5675 50  0001 C CNN
F 3 "" H 3275 5675 50  0001 C CNN
	1    3275 5675
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR014
U 1 1 5E038B75
P 4275 5675
F 0 "#PWR014" H 4275 5425 50  0001 C CNN
F 1 "GND" H 4280 5502 50  0000 C CNN
F 2 "" H 4275 5675 50  0001 C CNN
F 3 "" H 4275 5675 50  0001 C CNN
	1    4275 5675
	1    0    0    -1  
$EndComp
Wire Wire Line
	3275 5675 3275 5600
Wire Wire Line
	4075 5250 4275 5250
Wire Wire Line
	4275 5250 4275 5675
Wire Wire Line
	3275 5400 3275 5250
Wire Wire Line
	3175 5250 3275 5250
Connection ~ 3275 5250
Text GLabel 3175 5250 0    50   Input ~ 0
START
Text GLabel 3000 2625 2    50   Input ~ 0
MOSI
Text GLabel 3000 2725 2    50   Input ~ 0
MISO
Text GLabel 3000 2825 2    50   Input ~ 0
Clock
Text GLabel 1425 2125 0    50   Input ~ 0
RST
$Comp
L Connector_Generic:Conn_02x03_Odd_Even J2
U 1 1 5E058F4A
P 8875 1300
F 0 "J2" H 8925 1617 50  0000 C CNN
F 1 "ISP" H 8925 1526 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x03_P2.54mm_Vertical" H 8875 1300 50  0001 C CNN
F 3 "~" H 8875 1300 50  0001 C CNN
	1    8875 1300
	1    0    0    -1  
$EndComp
Text GLabel 8675 1200 0    50   Input ~ 0
MISO
Text GLabel 8675 1300 0    50   Input ~ 0
Clock
Text GLabel 8675 1400 0    50   Input ~ 0
RST
Text GLabel 9175 1300 2    50   Input ~ 0
MOSI
Text GLabel 9175 1200 2    50   Input ~ 0
VCC
Text GLabel 9175 1400 2    50   Input ~ 0
GND
$Comp
L Connector_Generic:Conn_01x02 J3
U 1 1 5E05A544
P 10025 2375
F 0 "J3" H 10105 2367 50  0000 L CNN
F 1 "OUT" H 10105 2276 50  0000 L CNN
F 2 "TerminalBlock_TE-Connectivity:TerminalBlock_TE_282834-2_1x02_P2.54mm_Horizontal" H 10025 2375 50  0001 C CNN
F 3 "~" H 10025 2375 50  0001 C CNN
	1    10025 2375
	1    0    0    -1  
$EndComp
Text GLabel 8700 2850 0    50   Input ~ 0
OUT
$Comp
L power:GND #PWR021
U 1 1 5E05B36B
P 7800 2075
F 0 "#PWR021" H 7800 1825 50  0001 C CNN
F 1 "GND" H 7805 1902 50  0000 C CNN
F 2 "" H 7800 2075 50  0001 C CNN
F 3 "" H 7800 2075 50  0001 C CNN
	1    7800 2075
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR022
U 1 1 5E05B76D
P 7825 4025
F 0 "#PWR022" H 7825 3775 50  0001 C CNN
F 1 "GND" H 7830 3852 50  0000 C CNN
F 2 "" H 7825 4025 50  0001 C CNN
F 3 "" H 7825 4025 50  0001 C CNN
	1    7825 4025
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR023
U 1 1 5E05BD6D
P 7875 5975
F 0 "#PWR023" H 7875 5725 50  0001 C CNN
F 1 "GND" H 7880 5802 50  0000 C CNN
F 2 "" H 7875 5975 50  0001 C CNN
F 3 "" H 7875 5975 50  0001 C CNN
	1    7875 5975
	1    0    0    -1  
$EndComp
Wire Wire Line
	7775 5775 7875 5775
Wire Wire Line
	7875 5775 7875 5875
Wire Wire Line
	7775 5875 7875 5875
Connection ~ 7875 5875
Wire Wire Line
	7875 5875 7875 5975
Wire Wire Line
	7750 3825 7825 3825
Wire Wire Line
	7825 3825 7825 3925
Wire Wire Line
	7750 3925 7825 3925
Connection ~ 7825 3925
Wire Wire Line
	7825 3925 7825 4025
Wire Wire Line
	7725 1875 7800 1875
Wire Wire Line
	7800 1875 7800 1975
Wire Wire Line
	7725 1975 7800 1975
Connection ~ 7800 1975
Wire Wire Line
	7800 1975 7800 2075
$Comp
L Device:R_Small R33
U 1 1 5E07F17B
P 9175 2850
F 0 "R33" V 9371 2850 50  0000 C CNN
F 1 "2.2K" V 9280 2850 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9175 2850 50  0001 C CNN
F 3 "~" H 9175 2850 50  0001 C CNN
	1    9175 2850
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR028
U 1 1 5E07FA48
P 9675 2225
F 0 "#PWR028" H 9675 2075 50  0001 C CNN
F 1 "+5V" H 9690 2398 50  0000 C CNN
F 2 "" H 9675 2225 50  0001 C CNN
F 3 "" H 9675 2225 50  0001 C CNN
	1    9675 2225
	1    0    0    -1  
$EndComp
Wire Wire Line
	8700 2850 8900 2850
Wire Wire Line
	9275 2850 9375 2850
Text GLabel 3000 2125 2    50   Input ~ 0
UP
Text GLabel 3000 2225 2    50   Input ~ 0
DOWN
Text GLabel 3000 2325 2    50   Input ~ 0
START
Text GLabel 3000 2425 2    50   Input ~ 0
OUT
NoConn ~ 3000 3625
$Comp
L Device:LED_Small D1
U 1 1 5E0C7D8B
P 9850 4125
F 0 "D1" H 9850 4360 50  0000 C CNN
F 1 "LED_Small" H 9850 4269 50  0000 C CNN
F 2 "LED_SMD:LED_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 9850 4125 50  0001 C CNN
F 3 "~" V 9850 4125 50  0001 C CNN
	1    9850 4125
	-1   0    0    -1  
$EndComp
$Comp
L Device:R_Small R34
U 1 1 5E0C8A62
P 9450 4125
F 0 "R34" V 9646 4125 50  0000 C CNN
F 1 "150R" V 9555 4125 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9450 4125 50  0001 C CNN
F 3 "~" H 9450 4125 50  0001 C CNN
	1    9450 4125
	0    -1   -1   0   
$EndComp
Text GLabel 9100 4125 0    50   Input ~ 0
VCC
Text GLabel 10100 4125 2    50   Input ~ 0
GND
Wire Wire Line
	9100 4125 9225 4125
Wire Wire Line
	9550 4125 9750 4125
Wire Wire Line
	9950 4125 10050 4125
$Comp
L power:+5V #PWR025
U 1 1 5E0D9328
P 9225 3950
F 0 "#PWR025" H 9225 3800 50  0001 C CNN
F 1 "+5V" H 9240 4123 50  0000 C CNN
F 2 "" H 9225 3950 50  0001 C CNN
F 3 "" H 9225 3950 50  0001 C CNN
	1    9225 3950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR030
U 1 1 5E0D9901
P 10050 4275
F 0 "#PWR030" H 10050 4025 50  0001 C CNN
F 1 "GND" H 10055 4102 50  0000 C CNN
F 2 "" H 10050 4275 50  0001 C CNN
F 3 "" H 10050 4275 50  0001 C CNN
	1    10050 4275
	1    0    0    -1  
$EndComp
Wire Wire Line
	10050 4275 10050 4125
Connection ~ 10050 4125
Wire Wire Line
	10050 4125 10100 4125
Wire Wire Line
	9225 3950 9225 4125
Connection ~ 9225 4125
Wire Wire Line
	9225 4125 9350 4125
$Comp
L Device:R_Small R2
U 1 1 5E10ED66
P 1600 1750
F 0 "R2" H 1659 1796 50  0000 L CNN
F 1 "10K" H 1659 1705 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 1600 1750 50  0001 C CNN
F 3 "~" H 1600 1750 50  0001 C CNN
	1    1600 1750
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR05
U 1 1 5E10FBC1
P 1600 1525
F 0 "#PWR05" H 1600 1375 50  0001 C CNN
F 1 "+5V" H 1615 1698 50  0000 C CNN
F 2 "" H 1600 1525 50  0001 C CNN
F 3 "" H 1600 1525 50  0001 C CNN
	1    1600 1525
	1    0    0    -1  
$EndComp
Wire Wire Line
	1425 2125 1600 2125
Wire Wire Line
	1600 1850 1600 2125
Connection ~ 1600 2125
Wire Wire Line
	1600 2125 1800 2125
Wire Wire Line
	1600 1525 1600 1650
Wire Wire Line
	2400 4025 2400 4375
Wire Wire Line
	2400 1525 2400 1675
$Comp
L timer-rescue:Barrel_Jack-Connector J1
U 1 1 5E174E70
P 8825 5375
F 0 "J1" H 8882 5700 50  0000 C CNN
F 1 "DC" H 8882 5609 50  0000 C CNN
F 2 "Connector_BarrelJack:BarrelJack_Horizontal" H 8875 5335 50  0001 C CNN
F 3 "~" H 8875 5335 50  0001 C CNN
	1    8825 5375
	1    0    0    -1  
$EndComp
$Comp
L Device:CP_Small C10
U 1 1 5E17668F
P 10100 5500
F 0 "C10" H 10188 5546 50  0000 L CNN
F 1 "100uF" H 10188 5455 50  0000 L CNN
F 2 "Capacitor_Tantalum_SMD:CP_EIA-7343-31_Kemet-D_Pad2.25x2.55mm_HandSolder" H 10100 5500 50  0001 C CNN
F 3 "~" H 10100 5500 50  0001 C CNN
	1    10100 5500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR026
U 1 1 5E176871
P 9325 5750
F 0 "#PWR026" H 9325 5500 50  0001 C CNN
F 1 "GND" H 9330 5577 50  0000 C CNN
F 2 "" H 9325 5750 50  0001 C CNN
F 3 "" H 9325 5750 50  0001 C CNN
	1    9325 5750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR032
U 1 1 5E177163
P 10100 5725
F 0 "#PWR032" H 10100 5475 50  0001 C CNN
F 1 "GND" H 10105 5552 50  0000 C CNN
F 2 "" H 10100 5725 50  0001 C CNN
F 3 "" H 10100 5725 50  0001 C CNN
	1    10100 5725
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR031
U 1 1 5E182F1B
P 10100 5075
F 0 "#PWR031" H 10100 4925 50  0001 C CNN
F 1 "+5V" H 10115 5248 50  0000 C CNN
F 2 "" H 10100 5075 50  0001 C CNN
F 3 "" H 10100 5075 50  0001 C CNN
	1    10100 5075
	1    0    0    -1  
$EndComp
Wire Wire Line
	9125 5475 9325 5475
Wire Wire Line
	9325 5475 9325 5750
Wire Wire Line
	10100 5275 10100 5075
Wire Wire Line
	10100 5400 10100 5275
Connection ~ 10100 5275
Wire Wire Line
	10100 5600 10100 5725
$Comp
L Device:Crystal_GND24_Small Y1
U 1 1 5E1B9C7C
P 1350 2750
F 0 "Y1" H 1400 2450 50  0000 R CNN
F 1 "12MHz" H 1475 2550 50  0000 R CNN
F 2 "Crystal:Crystal_SMD_SeikoEpson_FA238V-4Pin_3.2x2.5mm_HandSoldering" H 1350 2750 50  0001 C CNN
F 3 "~" H 1350 2750 50  0001 C CNN
	1    1350 2750
	1    0    0    1   
$EndComp
$Comp
L Device:C_Small C1
U 1 1 5E1BB18F
P 1075 3025
F 0 "C1" H 1167 3071 50  0000 L CNN
F 1 "22pF" H 1167 2980 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 1075 3025 50  0001 C CNN
F 3 "~" H 1075 3025 50  0001 C CNN
	1    1075 3025
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C3
U 1 1 5E1BB81B
P 1575 3025
F 0 "C3" H 1667 3071 50  0000 L CNN
F 1 "22pF" H 1667 2980 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 1575 3025 50  0001 C CNN
F 3 "~" H 1575 3025 50  0001 C CNN
	1    1575 3025
	1    0    0    -1  
$EndComp
NoConn ~ 1350 2850
NoConn ~ 1350 2650
$Comp
L nrg3-cache:GND #PWR01
U 1 1 5E1CF08A
P 1075 3275
F 0 "#PWR01" H 1075 3025 50  0001 C CNN
F 1 "GND" H 1080 3102 50  0000 C CNN
F 2 "" H 1075 3275 50  0001 C CNN
F 3 "" H 1075 3275 50  0001 C CNN
	1    1075 3275
	1    0    0    -1  
$EndComp
$Comp
L nrg3-cache:GND #PWR04
U 1 1 5E1CF338
P 1575 3275
F 0 "#PWR04" H 1575 3025 50  0001 C CNN
F 1 "GND" H 1580 3102 50  0000 C CNN
F 2 "" H 1575 3275 50  0001 C CNN
F 3 "" H 1575 3275 50  0001 C CNN
	1    1575 3275
	1    0    0    -1  
$EndComp
Wire Wire Line
	1075 2925 1075 2750
Wire Wire Line
	1075 2325 1800 2325
Wire Wire Line
	1800 2525 1575 2525
Wire Wire Line
	1575 2525 1575 2750
Wire Wire Line
	1450 2750 1575 2750
Connection ~ 1575 2750
Wire Wire Line
	1575 2750 1575 2925
Wire Wire Line
	1250 2750 1075 2750
Connection ~ 1075 2750
Wire Wire Line
	1075 2750 1075 2325
Wire Wire Line
	1075 3125 1075 3275
Wire Wire Line
	1575 3125 1575 3275
$Comp
L Device:R_Small R32
U 1 1 5E202CEA
P 8900 3075
F 0 "R32" H 8959 3121 50  0000 L CNN
F 1 "10K" H 8959 3030 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 8900 3075 50  0001 C CNN
F 3 "~" H 8900 3075 50  0001 C CNN
	1    8900 3075
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR024
U 1 1 5E2032E7
P 8900 3325
F 0 "#PWR024" H 8900 3075 50  0001 C CNN
F 1 "GND" H 8905 3152 50  0000 C CNN
F 2 "" H 8900 3325 50  0001 C CNN
F 3 "" H 8900 3325 50  0001 C CNN
	1    8900 3325
	1    0    0    -1  
$EndComp
Wire Wire Line
	8900 2975 8900 2850
Connection ~ 8900 2850
Wire Wire Line
	8900 2850 9075 2850
Wire Wire Line
	8900 3325 8900 3175
Wire Wire Line
	9825 2475 9675 2475
Wire Wire Line
	9675 2475 9675 2650
Wire Wire Line
	9675 2375 9825 2375
Wire Wire Line
	9675 2225 9675 2375
$Comp
L power:GND #PWR029
U 1 1 5E22578C
P 9675 3325
F 0 "#PWR029" H 9675 3075 50  0001 C CNN
F 1 "GND" H 9680 3152 50  0000 C CNN
F 2 "" H 9675 3325 50  0001 C CNN
F 3 "" H 9675 3325 50  0001 C CNN
	1    9675 3325
	1    0    0    -1  
$EndComp
Wire Wire Line
	9675 3050 9675 3325
$Comp
L power:+5V #PWR02
U 1 1 5E25F67D
P 1450 4850
F 0 "#PWR02" H 1450 4700 50  0001 C CNN
F 1 "+5V" H 1465 5023 50  0000 C CNN
F 2 "" H 1450 4850 50  0001 C CNN
F 3 "" H 1450 4850 50  0001 C CNN
	1    1450 4850
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR011
U 1 1 5E25FB84
P 3275 4850
F 0 "#PWR011" H 3275 4700 50  0001 C CNN
F 1 "+5V" H 3290 5023 50  0000 C CNN
F 2 "" H 3275 4850 50  0001 C CNN
F 3 "" H 3275 4850 50  0001 C CNN
	1    3275 4850
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR08
U 1 1 5E25FFF8
P 1450 6225
F 0 "#PWR08" H 1450 6075 50  0001 C CNN
F 1 "+5V" H 1465 6398 50  0000 C CNN
F 2 "" H 1450 6225 50  0001 C CNN
F 3 "" H 1450 6225 50  0001 C CNN
	1    1450 6225
	1    0    0    -1  
$EndComp
Wire Wire Line
	1450 4850 1450 4925
Wire Wire Line
	1450 5125 1450 5250
Wire Wire Line
	1450 6225 1450 6325
Wire Wire Line
	1450 6525 1450 6650
Wire Wire Line
	1450 6650 1850 6650
Wire Wire Line
	3275 5250 3675 5250
Wire Wire Line
	3275 5125 3275 5250
Wire Wire Line
	3275 4850 3275 4925
Wire Wire Line
	1450 5250 1850 5250
$Comp
L Device:Q_NMOS_GSD Q1
U 1 1 5E2146F7
P 9575 2850
F 0 "Q1" H 9781 2896 50  0000 L CNN
F 1 "IRLML2502TRPBF" H 9781 2805 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 9825 2775 50  0001 L CIN
F 3 "http://www.irf.com/product-info/datasheets/data/irf540n.pdf" H 9575 2850 50  0001 L CNN
	1    9575 2850
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C4
U 1 1 5DFCEB59
P 2225 1675
F 0 "C4" V 1996 1675 50  0000 C CNN
F 1 "0.1uF" V 2087 1675 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 2225 1675 50  0001 C CNN
F 3 "~" H 2225 1675 50  0001 C CNN
	1    2225 1675
	0    1    1    0   
$EndComp
Text GLabel 2125 1675 0    50   Input ~ 0
GND
Wire Wire Line
	2325 1675 2400 1675
Connection ~ 2400 1675
Wire Wire Line
	2400 1675 2400 1825
Wire Wire Line
	9125 5275 10100 5275
$Comp
L timer-rescue:ATtiny2313A-SU-MCU_Microchip_ATtiny U1
U 1 1 5DFB35AE
P 2400 2925
F 0 "U1" H 2525 4175 50  0000 C CNN
F 1 "ATtiny2313A-SU" H 2800 4025 50  0000 C CNN
F 2 "Package_SO:SOIC-20W_7.5x12.8mm_P1.27mm" H 2400 2925 50  0001 C CIN
F 3 "https://static.chipdip.ru/lib/223/DOC000223773.pdf" H 2400 2925 50  0001 C CNN
	1    2400 2925
	1    0    0    -1  
$EndComp
Text GLabel 3000 3025 2    50   Input ~ 0
RXD
Text GLabel 3000 3125 2    50   Input ~ 0
TXD
$Comp
L Connector_Generic:Conn_01x04 J4
U 1 1 5E220B0A
P 10175 1300
F 0 "J4" H 10225 1617 50  0000 C CNN
F 1 "UART" H 10225 1526 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 10175 1300 50  0001 C CNN
F 3 "~" H 10175 1300 50  0001 C CNN
	1    10175 1300
	1    0    0    -1  
$EndComp
Text GLabel 9975 1200 0    50   Input ~ 0
VCC
Text GLabel 9975 1500 0    50   Input ~ 0
GND
Text GLabel 9975 1300 0    50   Input ~ 0
RXD
Text GLabel 9975 1400 0    50   Input ~ 0
TXD
$Comp
L Device:Buzzer BZ1
U 1 1 5E222430
P 3650 6675
F 0 "BZ1" H 3802 6704 50  0000 L CNN
F 1 "Buzzer" H 3802 6613 50  0000 L CNN
F 2 "Buzzer_Beeper:Buzzer_12x9.5RM7.6" V 3625 6775 50  0001 C CNN
F 3 "~" V 3625 6775 50  0001 C CNN
	1    3650 6675
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R5
U 1 1 5DFFAADF
P 5275 1575
F 0 "R5" V 5471 1575 50  0000 C CNN
F 1 "1K" V 5380 1575 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 5275 1575 50  0001 C CNN
F 3 "~" H 5275 1575 50  0001 C CNN
	1    5275 1575
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5800 1275 5525 1275
Wire Wire Line
	5800 1775 5525 1775
Text Label 5525 1275 0    50   ~ 0
Data
Text Label 5525 1775 0    50   ~ 0
Latch
Wire Wire Line
	5825 3725 5575 3725
Text Label 5575 3725 0    50   ~ 0
Latch
Wire Wire Line
	5850 5675 5600 5675
Text Label 5600 5675 0    50   ~ 0
Latch
Wire Wire Line
	3000 3225 3375 3225
Text Label 3375 3225 2    50   ~ 0
Data
Wire Wire Line
	3000 3325 3375 3325
Text Label 3375 3325 2    50   ~ 0
Latch
Wire Wire Line
	3000 3425 3375 3425
Text Label 3375 3425 2    50   ~ 0
Sync
Wire Wire Line
	5800 1475 5525 1475
Text Label 5525 1475 0    50   ~ 0
Sync
Wire Wire Line
	5825 3425 5575 3425
Text Label 5575 3425 0    50   ~ 0
Sync
Wire Wire Line
	3000 3525 3375 3525
Text Label 3375 3525 2    50   ~ 0
Buzzer
Wire Wire Line
	3550 6575 3300 6575
Text Label 3300 6575 0    50   ~ 0
Buzzer
NoConn ~ 3000 2525
Wire Wire Line
	5850 5375 5600 5375
Text Label 5600 5375 0    50   ~ 0
Sync
Wire Wire Line
	3550 6775 3300 6775
Text Label 3300 6775 0    50   ~ 0
GND
$Comp
L Switch:SW_Push SW2
U 1 1 5E03431C
P 2050 6650
F 0 "SW2" H 2050 6935 50  0000 C CNN
F 1 "DOWN" H 2050 6844 50  0000 C CNN
F 2 "Button_Switch_SMD:SW_SPST_EVPBF" H 2050 6850 50  0001 C CNN
F 3 "~" H 2050 6850 50  0001 C CNN
	1    2050 6650
	1    0    0    -1  
$EndComp
$EndSCHEMATC
