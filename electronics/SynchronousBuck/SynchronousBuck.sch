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
L Diode:BAS20 D101
U 1 1 60B9C2F8
P 5050 2850
F 0 "D101" H 5150 2750 50  0000 R CNN
F 1 "BAS20" H 5200 2650 50  0001 R CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 5050 2675 50  0001 C CNN
F 3 "https://www.diodes.com/assets/Datasheets/Ds12004.pdf" H 5050 2850 50  0001 C CNN
	1    5050 2850
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR0105
U 1 1 60BAC4C6
P 6450 2650
F 0 "#PWR0105" H 6450 2500 50  0001 C CNN
F 1 "VCC" H 6465 2823 50  0000 C CNN
F 2 "" H 6450 2650 50  0001 C CNN
F 3 "" H 6450 2650 50  0001 C CNN
	1    6450 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	7750 3400 7650 3400
Wire Wire Line
	4700 4450 4700 4350
Wire Wire Line
	4700 4250 4600 4250
$Comp
L Device:R_Small R107
U 1 1 60BC1223
P 7750 4050
F 0 "R107" H 7809 4096 50  0000 L CNN
F 1 "1k2" H 7809 4005 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 7750 4050 50  0001 C CNN
F 3 "~" H 7750 4050 50  0001 C CNN
	1    7750 4050
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R108
U 1 1 60BC3269
P 7750 4700
F 0 "R108" H 7809 4746 50  0000 L CNN
F 1 "1k2" H 7809 4655 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 7750 4700 50  0001 C CNN
F 3 "~" H 7750 4700 50  0001 C CNN
	1    7750 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 4550 5050 4550
Wire Wire Line
	7750 4450 7750 4500
Wire Wire Line
	7750 4800 7750 4850
$Comp
L power:GNDA #PWR0108
U 1 1 60BC6C9E
P 7750 4850
F 0 "#PWR0108" H 7750 4600 50  0001 C CNN
F 1 "GNDA" H 7755 4677 50  0000 C CNN
F 2 "" H 7750 4850 50  0001 C CNN
F 3 "" H 7750 4850 50  0001 C CNN
	1    7750 4850
	1    0    0    -1  
$EndComp
$Comp
L Device:C C106
U 1 1 60BC828F
P 5550 4800
F 0 "C106" H 5665 4846 50  0000 L CNN
F 1 "33p" H 5665 4755 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5588 4650 50  0001 C CNN
F 3 "~" H 5550 4800 50  0001 C CNN
F 4 "-prod" H 5550 4800 50  0001 C CNN "Config"
	1    5550 4800
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R104
U 1 1 60BC8C39
P 5050 4700
F 0 "R104" H 5109 4746 50  0000 L CNN
F 1 "178k" H 5109 4655 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 5050 4700 50  0001 C CNN
F 3 "~" H 5050 4700 50  0001 C CNN
F 4 "-prod" H 5050 4700 50  0001 C CNN "Config"
	1    5050 4700
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C104
U 1 1 60BC9A24
P 5050 4950
F 0 "C104" H 5142 4996 50  0000 L CNN
F 1 "560p" H 5142 4905 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5050 4950 50  0001 C CNN
F 3 "~" H 5050 4950 50  0001 C CNN
F 4 "-prod" H 5050 4950 50  0001 C CNN "Config"
	1    5050 4950
	1    0    0    -1  
$EndComp
Wire Wire Line
	5550 4650 5550 4550
Wire Wire Line
	5050 4550 5050 4600
Wire Wire Line
	5050 5150 5550 5150
Wire Wire Line
	5550 5150 5550 4950
Wire Wire Line
	5050 4800 5050 4850
Wire Wire Line
	5050 5050 5050 5150
Connection ~ 5050 4550
Wire Wire Line
	5050 4550 5550 4550
Wire Wire Line
	5050 5150 4850 5150
Wire Wire Line
	4850 5150 4850 4700
Wire Wire Line
	4850 4700 4600 4700
Connection ~ 5050 5150
Wire Wire Line
	6450 3300 6450 3400
Connection ~ 6450 3400
$Comp
L Device:C_Small C105
U 1 1 60BDF776
P 4750 3250
F 0 "C105" H 4842 3296 50  0000 L CNN
F 1 "1u" H 4842 3205 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4750 3250 50  0001 C CNN
F 3 "~" H 4750 3250 50  0001 C CNN
	1    4750 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 2700 5250 2700
$Comp
L Device:C_Small C107
U 1 1 60C09E68
P 5600 2700
F 0 "C107" V 5371 2700 50  0000 C CNN
F 1 "1u" V 5462 2700 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5600 2700 50  0001 C CNN
F 3 "~" H 5600 2700 50  0001 C CNN
	1    5600 2700
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0104
U 1 1 60C0D43D
P 5800 2800
F 0 "#PWR0104" H 5800 2550 50  0001 C CNN
F 1 "GND" H 5805 2627 50  0000 C CNN
F 2 "" H 5800 2800 50  0001 C CNN
F 3 "" H 5800 2800 50  0001 C CNN
	1    5800 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	5250 2700 5500 2700
Wire Wire Line
	5800 2700 5800 2800
Wire Wire Line
	5700 2700 5800 2700
Connection ~ 5250 2700
$Comp
L power:GND #PWR0106
U 1 1 60C16D88
P 6550 4200
F 0 "#PWR0106" H 6550 3950 50  0001 C CNN
F 1 "GND" H 6555 4027 50  0000 C CNN
F 2 "" H 6550 4200 50  0001 C CNN
F 3 "" H 6550 4200 50  0001 C CNN
	1    6550 4200
	1    0    0    -1  
$EndComp
Connection ~ 7750 3400
Text Label 4700 4250 0    50   ~ 0
VOUT
Connection ~ 5550 4550
Connection ~ 7750 4550
Wire Wire Line
	7750 4550 7750 4600
Wire Wire Line
	5550 4550 7750 4550
Wire Wire Line
	2900 4500 2900 4600
Wire Wire Line
	3600 4500 2900 4500
$Comp
L power:GNDA #PWR0102
U 1 1 60BCEB61
P 2900 4800
F 0 "#PWR0102" H 2900 4550 50  0001 C CNN
F 1 "GNDA" H 2905 4627 50  0000 C CNN
F 2 "" H 2900 4800 50  0001 C CNN
F 3 "" H 2900 4800 50  0001 C CNN
	1    2900 4800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 60BCDA49
P 3100 4900
F 0 "#PWR0103" H 3100 4650 50  0001 C CNN
F 1 "GND" H 3105 4727 50  0000 C CNN
F 2 "" H 3100 4900 50  0001 C CNN
F 3 "" H 3100 4900 50  0001 C CNN
	1    3100 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 4700 3100 4900
Wire Wire Line
	3600 4700 3100 4700
Wire Wire Line
	3600 4600 3100 4600
Wire Wire Line
	3100 4600 3100 4700
Connection ~ 3100 4700
$Comp
L Device:C_Small C102
U 1 1 60C2B032
P 3150 3800
F 0 "C102" V 2921 3800 50  0000 C CNN
F 1 "1.8p" V 3012 3800 50  0000 C CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 3150 3800 50  0001 C CNN
F 3 "~" H 3150 3800 50  0001 C CNN
F 4 "C140677" V 3150 3800 50  0001 C CNN "LCSC Pn"
	1    3150 3800
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C103
U 1 1 60C2BD6E
P 3150 3950
F 0 "C103" V 3000 3950 50  0000 C CNN
F 1 "100n" V 2900 3950 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3150 3950 50  0001 C CNN
F 3 "~" H 3150 3950 50  0001 C CNN
F 4 "C386106" V 3150 3950 50  0001 C CNN "LCSC Pn"
	1    3150 3950
	0    1    -1   0   
$EndComp
Wire Wire Line
	2900 4500 2900 4050
Wire Wire Line
	2900 3950 3050 3950
Wire Wire Line
	3250 3950 3600 3950
Connection ~ 2900 4500
Wire Wire Line
	3600 3800 3250 3800
Wire Wire Line
	3050 3800 2900 3800
$Comp
L Device:R_Small R103
U 1 1 60C33F82
P 2900 3550
F 0 "R103" H 2959 3596 50  0000 L CNN
F 1 "9k1" H 2959 3505 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 2900 3550 50  0001 C CNN
F 3 "~" H 2900 3550 50  0001 C CNN
	1    2900 3550
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R101
U 1 1 60C3C21E
P 2550 3200
F 0 "R101" V 2650 3150 39  0000 L CNN
F 1 "24k" V 2550 3150 39  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 2550 3200 50  0001 C CNN
F 3 "~" H 2550 3200 50  0001 C CNN
	1    2550 3200
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_Small R102
U 1 1 60C3E2C9
P 3300 3200
F 0 "R102" V 3400 3150 39  0000 L CNN
F 1 "1k2" V 3300 3150 39  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 3300 3200 50  0001 C CNN
F 3 "~" H 3300 3200 50  0001 C CNN
	1    3300 3200
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2900 3800 2900 3950
Connection ~ 2900 3800
Connection ~ 2900 3950
$Comp
L power:VCC #PWR0101
U 1 1 60C4C0C4
P 1500 2400
F 0 "#PWR0101" H 1500 2250 50  0001 C CNN
F 1 "VCC" H 1515 2573 50  0000 C CNN
F 2 "" H 1500 2400 50  0001 C CNN
F 3 "" H 1500 2400 50  0001 C CNN
	1    1500 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	1500 2400 1500 2700
NoConn ~ 2500 4350
Wire Wire Line
	3600 3100 3500 3100
$Comp
L Device:R_Small R106
U 1 1 60C5B10D
P 6950 4100
F 0 "R106" H 7009 4146 50  0000 L CNN
F 1 "R_Small" H 7009 4055 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 6950 4100 50  0001 C CNN
F 3 "~" H 6950 4100 50  0001 C CNN
F 4 "-prod" H 6950 4100 50  0001 C CNN "Config"
	1    6950 4100
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C108
U 1 1 60C5BDF5
P 6950 3900
F 0 "C108" H 7042 3946 50  0000 L CNN
F 1 "C_Small" H 7050 3850 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6950 3900 50  0001 C CNN
F 3 "~" H 6950 3900 50  0001 C CNN
F 4 "-prod" H 6950 3900 50  0001 C CNN "Config"
	1    6950 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	6950 3400 6950 3800
$Comp
L power:GND #PWR0107
U 1 1 60C5F52D
P 6950 4200
F 0 "#PWR0107" H 6950 3950 50  0001 C CNN
F 1 "GND" H 6955 4027 50  0000 C CNN
F 2 "" H 6950 4200 50  0001 C CNN
F 3 "" H 6950 4200 50  0001 C CNN
	1    6950 4200
	1    0    0    -1  
$EndComp
$Comp
L Device:C C111
U 1 1 60C835F9
P 1500 2950
F 0 "C111" H 1615 2996 50  0000 L CNN
F 1 "2u2" H 1615 2905 50  0000 L CNN
F 2 "Capacitor_SMD:C_1210_3225Metric" H 1538 2800 50  0001 C CNN
F 3 "~" H 1500 2950 50  0001 C CNN
F 4 "100 V" H 1500 2950 50  0001 C CNN "Voltage"
F 5 "C153036" H 1500 2950 50  0001 C CNN "LCSC Pn"
	1    1500 2950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0109
U 1 1 60C84522
P 1500 3200
F 0 "#PWR0109" H 1500 2950 50  0001 C CNN
F 1 "GND" H 1505 3027 50  0000 C CNN
F 2 "" H 1500 3200 50  0001 C CNN
F 3 "" H 1500 3200 50  0001 C CNN
	1    1500 3200
	1    0    0    -1  
$EndComp
Wire Wire Line
	1500 3150 1500 3200
Wire Wire Line
	1500 2700 1500 2750
Wire Wire Line
	1500 3100 1500 3150
Connection ~ 1500 2700
Wire Wire Line
	1900 2700 1900 2800
Wire Wire Line
	1900 3150 1500 3150
Wire Wire Line
	1900 3100 1900 3150
Wire Wire Line
	1900 2700 1700 2700
Connection ~ 1500 3150
Text Notes 3200 3500 0    50   ~ 0
fsw 330 kHz
Text Notes 750  2400 0    50   ~ 0
Vin 30-50V
Wire Wire Line
	5200 2850 5250 2850
Wire Wire Line
	5250 2700 5250 2850
Wire Wire Line
	4750 3150 4750 2850
Wire Wire Line
	4750 2850 4900 2850
Wire Wire Line
	4750 2850 4600 2850
Connection ~ 4750 2850
Wire Wire Line
	4600 3400 4750 3400
Wire Wire Line
	4750 3350 4750 3400
Connection ~ 4750 3400
Wire Wire Line
	4750 3400 6450 3400
$Comp
L power:GND #PWR0114
U 1 1 60BC1D81
P 9900 3650
F 0 "#PWR0114" H 9900 3400 50  0001 C CNN
F 1 "GND" H 9905 3477 50  0000 C CNN
F 2 "" H 9900 3650 50  0001 C CNN
F 3 "" H 9900 3650 50  0001 C CNN
	1    9900 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	7750 3400 8000 3400
$Comp
L Device:CP1_Small C112
U 1 1 60BEDCBC
P 9100 3550
F 0 "C112" H 9191 3596 50  0000 L CNN
F 1 "100u" H 9191 3505 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_8x10" H 9100 3550 50  0001 C CNN
F 3 "~" H 9100 3550 50  0001 C CNN
F 4 "C487360" H 9100 3550 50  0001 C CNN "LCSC Pn"
F 5 "50V" H 9100 3550 50  0001 C CNN "Voltage"
	1    9100 3550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0112
U 1 1 60BFEE85
P 9100 3650
F 0 "#PWR0112" H 9100 3400 50  0001 C CNN
F 1 "GND" H 9105 3477 50  0000 C CNN
F 2 "" H 9100 3650 50  0001 C CNN
F 3 "" H 9100 3650 50  0001 C CNN
	1    9100 3650
	1    0    0    -1  
$EndComp
Connection ~ 8000 3400
Wire Wire Line
	8000 3400 8100 3400
Connection ~ 8900 3400
Wire Wire Line
	8900 3400 9100 3400
Wire Wire Line
	9100 3450 9100 3400
Connection ~ 9100 3400
Wire Wire Line
	9100 3400 9500 3400
Wire Wire Line
	9500 3450 9500 3400
Connection ~ 9500 3400
Wire Wire Line
	9500 3400 9900 3400
Wire Wire Line
	9900 3450 9900 3400
Connection ~ 9900 3400
Wire Wire Line
	9900 3400 10300 3400
$Comp
L Device:C C101
U 1 1 60C18BB2
P 1900 2950
F 0 "C101" H 2015 2996 50  0000 L CNN
F 1 "100n" H 2015 2905 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 1938 2800 50  0001 C CNN
F 3 "~" H 1900 2950 50  0001 C CNN
F 4 "50V" H 1900 2950 50  0001 C CNN "Voltage"
F 5 "C386106" H 1900 2950 50  0001 C CNN "LCSC Pn"
	1    1900 2950
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:BSC070N10NS5 Q101
U 1 1 60C33CB4
P 6350 3100
F 0 "Q101" H 6554 3146 50  0000 L CNN
F 1 "BSC070N10NS5" H 6554 3055 50  0001 L CNN
F 2 "Package_TO_SOT_SMD:TDSON-8-1" H 6550 3025 50  0001 L CIN
F 3 "http://www.infineon.com/dgdl/Infineon-BSC070N10NS5-DS-v02_01-EN.pdf?fileId=5546d4624a0bf290014a0fc62d9d6b3c" V 6350 3100 50  0001 L CNN
F 4 "C151549" H 6350 3100 50  0001 C CNN "LCSC Pn"
	1    6350 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 3100 6150 3100
Wire Wire Line
	6450 2650 6450 2900
$Comp
L Connector_Generic:Conn_01x02 J101
U 1 1 60C5EBC6
P 700 2850
F 0 "J101" H 618 2525 50  0000 C CNN
F 1 "Conn_01x02" H 618 2616 50  0000 C CNN
F 2 "PCBDecals:2xM4" H 700 2850 50  0001 C CNN
F 3 "~" H 700 2850 50  0001 C CNN
F 4 "-prod" H 700 2850 50  0001 C CNN "Config"
	1    700  2850
	-1   0    0    1   
$EndComp
Connection ~ 1500 2750
Wire Wire Line
	1500 2750 1500 2800
Wire Wire Line
	1500 3100 1300 3100
Wire Wire Line
	1300 3100 1300 2850
Wire Wire Line
	1300 2850 900  2850
Connection ~ 1500 3100
$Comp
L Device:R_Small R113
U 1 1 60BAF4E0
P 5000 3950
F 0 "R113" V 4950 3800 50  0000 C CNN
F 1 "30k" V 5000 3950 39  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 5000 3950 50  0001 C CNN
F 3 "~" H 5000 3950 50  0001 C CNN
	1    5000 3950
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R112
U 1 1 60BB3017
P 5000 3750
F 0 "R112" V 4950 3600 50  0000 C CNN
F 1 "30k" V 5000 3750 39  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 5000 3750 50  0001 C CNN
F 3 "~" H 5000 3750 50  0001 C CNN
	1    5000 3750
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R114
U 1 1 60BCCE76
P 5000 4100
F 0 "R114" V 4950 3950 50  0000 C CNN
F 1 "10k" V 5000 4100 39  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 5000 4100 50  0001 C CNN
F 3 "~" H 5000 4100 50  0001 C CNN
	1    5000 4100
	0    1    1    0   
$EndComp
$Comp
L Device:R_Shunt R105
U 1 1 60BB20E6
P 6550 4000
F 0 "R105" H 6462 4046 50  0000 R CNN
F 1 "0R002" H 6462 3955 50  0000 R CNN
F 2 "PCBDecals:R_2512_6332Metric_SHUNT" V 6480 4000 50  0001 C CNN
F 3 "~" H 6550 4000 50  0001 C CNN
F 4 "C457118" H 6550 4000 50  0001 C CNN "LCSC Pn"
	1    6550 4000
	-1   0    0    -1  
$EndComp
Wire Wire Line
	5300 3750 5300 3900
Wire Wire Line
	5300 3900 6400 3900
Wire Wire Line
	6400 4100 5200 4100
Wire Wire Line
	4600 4100 4900 4100
Connection ~ 5200 4100
Wire Wire Line
	4600 3950 4900 3950
Wire Wire Line
	5100 4100 5200 4100
Wire Wire Line
	5100 3950 5200 3950
Wire Wire Line
	5200 3950 5200 4100
Wire Wire Line
	4900 3750 4600 3750
Wire Wire Line
	5100 3750 5300 3750
Wire Wire Line
	7650 3400 7650 4450
Wire Wire Line
	7650 4450 4700 4450
$Comp
L Device:R_Small R115
U 1 1 60C1F42C
P 7750 4350
F 0 "R115" H 7809 4396 50  0000 L CNN
F 1 "24k" H 7809 4305 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 7750 4350 50  0001 C CNN
F 3 "~" H 7750 4350 50  0001 C CNN
	1    7750 4350
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R109
U 1 1 60C2D444
P 2800 3200
F 0 "R109" V 2900 3150 39  0000 L CNN
F 1 "1k2" V 2800 3150 39  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 2800 3200 50  0001 C CNN
F 3 "~" H 2800 3200 50  0001 C CNN
	1    2800 3200
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_Small R110
U 1 1 60C2DBB1
P 3050 3200
F 0 "R110" V 3150 3150 39  0000 L CNN
F 1 "1k2" V 3050 3150 39  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 3050 3200 50  0001 C CNN
F 3 "~" H 3050 3200 50  0001 C CNN
	1    3050 3200
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_Small R111
U 1 1 60C3B8F2
P 2400 3900
F 0 "R111" H 2459 3946 50  0000 L CNN
F 1 "1k2" H 2459 3855 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 2400 3900 50  0001 C CNN
F 3 "~" H 2400 3900 50  0001 C CNN
	1    2400 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 3200 2700 3200
Wire Wire Line
	2400 4050 2900 4050
Wire Wire Line
	2900 3200 2950 3200
Wire Wire Line
	3150 3200 3200 3200
Wire Wire Line
	2400 4000 2400 4050
Connection ~ 2900 4050
Wire Wire Line
	2900 4050 2900 3950
Wire Wire Line
	2400 3750 2750 3750
Wire Wire Line
	2400 3750 2400 3800
Wire Wire Line
	2900 3650 2900 3800
Wire Wire Line
	1900 2700 2100 2700
Connection ~ 1900 2700
Connection ~ 2400 2700
Wire Wire Line
	2400 2700 3500 2700
$Comp
L Device:R_Small R116
U 1 1 60C62B1C
P 8050 4350
F 0 "R116" H 8109 4396 50  0000 L CNN
F 1 "24k" H 8109 4305 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 8050 4350 50  0001 C CNN
F 3 "~" H 8050 4350 50  0001 C CNN
	1    8050 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	7750 4150 7750 4200
Wire Wire Line
	7750 3400 7750 3950
Wire Wire Line
	7750 4200 8050 4200
Wire Wire Line
	8050 4200 8050 4250
Connection ~ 7750 4200
Wire Wire Line
	7750 4200 7750 4250
Wire Wire Line
	8050 4450 8050 4500
Wire Wire Line
	8050 4500 7750 4500
Connection ~ 7750 4500
Wire Wire Line
	7750 4500 7750 4550
Wire Wire Line
	6450 3400 6550 3400
$Comp
L Misc:BSC067N06LS3G Q102
U 1 1 60C8D212
P 6450 3600
F 0 "Q102" H 6655 3600 50  0000 L CNN
F 1 "BSC067N06LS3G" H 6654 3555 50  0001 L CNN
F 2 "Package_TO_SOT_SMD:TDSON-8-1" H 6650 3525 50  0001 L CIN
F 3 "https://www.infineon.com/dgdl/Infineon-BSC067N06LS3-DS-v02_04-en.pdf" V 6450 3600 50  0001 L CNN
F 4 "C24199" H 6450 3600 50  0001 C CNN "LCSC Pn"
	1    6450 3600
	1    0    0    -1  
$EndComp
Connection ~ 6550 3400
Wire Wire Line
	6550 3400 6950 3400
Wire Wire Line
	4600 3600 6250 3600
Wire Wire Line
	3500 3100 3500 2700
Connection ~ 3500 2700
Wire Wire Line
	3500 2700 3600 2700
Wire Wire Line
	3100 4600 2900 4600
Connection ~ 3100 4600
Connection ~ 2900 4600
Wire Wire Line
	2900 4600 2900 4800
Text Label 5550 3400 0    50   ~ 0
SW
Text Label 5550 3100 0    50   ~ 0
HO
Text Label 5550 3600 0    50   ~ 0
LO
Wire Wire Line
	4600 4350 4700 4350
Connection ~ 4700 4350
Wire Wire Line
	4700 4350 4700 4250
Connection ~ 8450 3400
Wire Wire Line
	8450 3400 8550 3400
$Comp
L Device:R_Small R117
U 1 1 60C8DD8C
P 1200 5750
F 0 "R117" H 1259 5796 50  0000 L CNN
F 1 "24k" H 1259 5705 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 1200 5750 50  0001 C CNN
F 3 "~" H 1200 5750 50  0001 C CNN
	1    1200 5750
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R118
U 1 1 60C8F75B
P 1200 6100
F 0 "R118" H 1259 6146 50  0000 L CNN
F 1 "1k2" H 1259 6055 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 1200 6100 50  0001 C CNN
F 3 "~" H 1200 6100 50  0001 C CNN
	1    1200 6100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0116
U 1 1 60C9810A
P 1200 6300
F 0 "#PWR0116" H 1200 6050 50  0001 C CNN
F 1 "GND" H 1205 6127 50  0000 C CNN
F 2 "" H 1200 6300 50  0001 C CNN
F 3 "" H 1200 6300 50  0001 C CNN
	1    1200 6300
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 5850 1200 5950
Wire Wire Line
	1200 6200 1200 6300
Wire Wire Line
	1200 5550 1550 5550
Wire Wire Line
	1550 5550 1550 5650
Wire Wire Line
	1550 5950 1200 5950
Wire Wire Line
	1550 5850 1550 5950
Wire Wire Line
	1200 5550 1200 5650
Connection ~ 1200 5950
Wire Wire Line
	1200 5950 1200 6000
Connection ~ 1550 5950
$Comp
L Device:R_Small R121
U 1 1 60CCC409
P 10650 4600
F 0 "R121" H 10709 4646 50  0000 L CNN
F 1 "1k2" H 10709 4555 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 10650 4600 50  0001 C CNN
F 3 "~" H 10650 4600 50  0001 C CNN
	1    10650 4600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0118
U 1 1 60CE6A53
P 10650 4800
F 0 "#PWR0118" H 10650 4550 50  0001 C CNN
F 1 "GND" H 10655 4627 50  0000 C CNN
F 2 "" H 10650 4800 50  0001 C CNN
F 3 "" H 10650 4800 50  0001 C CNN
	1    10650 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	10650 4800 10650 4700
Wire Wire Line
	10650 4500 10650 4450
Connection ~ 10650 3400
Connection ~ 10650 4450
Wire Wire Line
	10650 4450 10650 4400
$Comp
L Device:R_Small R119
U 1 1 60C8F240
P 1550 5750
F 0 "R119" H 1609 5796 50  0000 L CNN
F 1 "24k" H 1609 5705 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 1550 5750 50  0001 C CNN
F 3 "~" H 1550 5750 50  0001 C CNN
	1    1550 5750
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:Fiducial FID101
U 1 1 60BC714B
P 5150 7350
F 0 "FID101" H 5235 7350 50  0000 L CNN
F 1 "Fiducial" H 5235 7305 50  0001 L CNN
F 2 "Fiducial:Fiducial_0.5mm_Mask1mm" H 5150 7350 50  0001 C CNN
F 3 "~" H 5150 7350 50  0001 C CNN
	1    5150 7350
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:Fiducial FID102
U 1 1 60BC7A84
P 5150 7500
F 0 "FID102" H 5235 7500 50  0000 L CNN
F 1 "Fiducial" H 5235 7455 50  0001 L CNN
F 2 "Fiducial:Fiducial_0.5mm_Mask1mm" H 5150 7500 50  0001 C CNN
F 3 "~" H 5150 7500 50  0001 C CNN
	1    5150 7500
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:Fiducial FID103
U 1 1 60BC8240
P 5150 7650
F 0 "FID103" H 5235 7650 50  0000 L CNN
F 1 "Fiducial" H 5235 7605 50  0001 L CNN
F 2 "Fiducial:Fiducial_0.5mm_Mask1mm" H 5150 7650 50  0001 C CNN
F 3 "~" H 5150 7650 50  0001 C CNN
	1    5150 7650
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H102
U 1 1 60BC9D5D
P 5150 7100
F 0 "H102" H 5250 7146 50  0000 L CNN
F 1 "MountingHole" H 5250 7055 50  0000 L CNN
F 2 "MountingHole:MountingHole_4.3mm_M4_DIN965" H 5150 7100 50  0001 C CNN
F 3 "~" H 5150 7100 50  0001 C CNN
	1    5150 7100
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H104
U 1 1 60BCAEF4
P 5950 7100
F 0 "H104" H 6050 7146 50  0000 L CNN
F 1 "MountingHole" H 6050 7055 50  0000 L CNN
F 2 "MountingHole:MountingHole_4.3mm_M4_DIN965" H 5950 7100 50  0001 C CNN
F 3 "~" H 5950 7100 50  0001 C CNN
	1    5950 7100
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H101
U 1 1 60BCB683
P 5150 6850
F 0 "H101" H 5250 6896 50  0000 L CNN
F 1 "MountingHole" H 5250 6805 50  0000 L CNN
F 2 "MountingHole:MountingHole_4.3mm_M4_DIN965" H 5150 6850 50  0001 C CNN
F 3 "~" H 5150 6850 50  0001 C CNN
	1    5150 6850
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H103
U 1 1 60BCBF1C
P 5950 6850
F 0 "H103" H 6050 6896 50  0000 L CNN
F 1 "MountingHole" H 6050 6805 50  0000 L CNN
F 2 "MountingHole:MountingHole_4.3mm_M4_DIN965" H 5950 6850 50  0001 C CNN
F 3 "~" H 5950 6850 50  0001 C CNN
	1    5950 6850
	1    0    0    -1  
$EndComp
Wire Notes Line
	4800 7800 4800 6550
Wire Notes Line
	4800 6550 6950 6550
Text Notes 5750 6650 0    50   ~ 0
Mechanical\n
$Comp
L Connector_Generic:Conn_01x02 J103
U 1 1 60C1A8FF
P 11050 3400
F 0 "J103" H 11050 3650 50  0000 C CNN
F 1 "Conn_01x02" H 10900 3550 50  0000 C CNN
F 2 "PCBDecals:2xM4" H 11050 3400 50  0001 C CNN
F 3 "~" H 11050 3400 50  0001 C CNN
F 4 "-prod" H 11050 3400 50  0001 C CNN "Config"
	1    11050 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	10650 3400 10850 3400
Wire Wire Line
	10850 3500 10800 3500
Wire Wire Line
	10800 3500 10800 3850
$Comp
L power:GND #PWR0119
U 1 1 60C2A146
P 10800 3900
F 0 "#PWR0119" H 10800 3650 50  0001 C CNN
F 1 "GND" H 10805 3727 50  0000 C CNN
F 2 "" H 10800 3900 50  0001 C CNN
F 3 "" H 10800 3900 50  0001 C CNN
	1    10800 3900
	1    0    0    -1  
$EndComp
Text Label 10000 3400 0    50   ~ 0
VOUT
Wire Wire Line
	10050 5250 10050 4450
Wire Wire Line
	10050 5250 10250 5250
Wire Wire Line
	10250 5350 10050 5350
Wire Wire Line
	10050 5350 10050 5450
Wire Wire Line
	10250 5450 10050 5450
Connection ~ 10050 5450
Wire Wire Line
	10050 5450 10050 5550
$Comp
L power:GND #PWR0117
U 1 1 60CA65E8
P 10050 5550
F 0 "#PWR0117" H 10050 5300 50  0001 C CNN
F 1 "GND" H 10055 5377 50  0000 C CNN
F 2 "" H 10050 5550 50  0001 C CNN
F 3 "" H 10050 5550 50  0001 C CNN
	1    10050 5550
	1    0    0    -1  
$EndComp
Wire Wire Line
	10150 5950 10150 5550
Wire Wire Line
	10150 5550 10250 5550
Wire Wire Line
	1550 5950 10150 5950
$Comp
L Misc:LM5116MH U101
U 1 1 60B7B47C
P 4100 3200
F 0 "U101" H 4100 3967 50  0000 C CNN
F 1 "LM5116MH" H 4100 3876 50  0000 C CNN
F 2 "Package_SO:Texas_PWP0020A" H 4800 3850 50  0001 L CNN
F 3 "https://www.ti.com/lit/ds/symlink/lm5116.pdf" H 6100 2750 50  0001 C CNN
F 4 "C13755" H 5250 3500 50  0001 C CNN "LSCS Part#"
F 5 "Texas Instruments" H 5450 3700 50  0001 C CNN "Manufacturer"
F 6 "LM5116MH/NOPB" H 5450 3600 50  0001 C CNN "Part#"
	1    4100 3200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0113
U 1 1 60BFE6DC
P 9500 3650
F 0 "#PWR0113" H 9500 3400 50  0001 C CNN
F 1 "GND" H 9505 3477 50  0000 C CNN
F 2 "" H 9500 3650 50  0001 C CNN
F 3 "" H 9500 3650 50  0001 C CNN
	1    9500 3650
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1_Small C113
U 1 1 60BF0C3F
P 9500 3550
F 0 "C113" H 9591 3596 50  0000 L CNN
F 1 "100u" H 9591 3505 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_8x10" H 9500 3550 50  0001 C CNN
F 3 "~" H 9500 3550 50  0001 C CNN
F 4 "C487360" H 9500 3550 50  0001 C CNN "LCSC Pn"
F 5 "50" H 9500 3550 50  0001 C CNN "Voltage"
	1    9500 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	10300 3450 10300 3400
Connection ~ 10300 3400
Wire Wire Line
	10300 3400 10450 3400
$Comp
L power:GND #PWR0121
U 1 1 60D5973C
P 10300 3650
F 0 "#PWR0121" H 10300 3400 50  0001 C CNN
F 1 "GND" H 10305 3477 50  0000 C CNN
F 2 "" H 10300 3650 50  0001 C CNN
F 3 "" H 10300 3650 50  0001 C CNN
	1    10300 3650
	1    0    0    -1  
$EndComp
Text Notes 3200 4900 0    50   ~ 0
Connect AGND and GND at exposed pad\n
$Comp
L Device:C_Small C117
U 1 1 60D5A9A4
P 8550 3200
F 0 "C117" H 8665 3246 50  0000 L CNN
F 1 "2u2" H 8665 3155 50  0000 L CNN
F 2 "Capacitor_SMD:C_1210_3225Metric" H 8588 3050 50  0001 C CNN
F 3 "~" H 8550 3200 50  0001 C CNN
F 4 "100 V" H 8550 3200 50  0001 C CNN "Voltage"
F 5 "C153036" H 8550 3200 50  0001 C CNN "LCSC Pn"
	1    8550 3200
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C116
U 1 1 60D5B2FE
P 8100 3200
F 0 "C116" H 8215 3246 50  0000 L CNN
F 1 "2u2" H 8215 3155 50  0000 L CNN
F 2 "Capacitor_SMD:C_1210_3225Metric" H 8138 3050 50  0001 C CNN
F 3 "~" H 8100 3200 50  0001 C CNN
F 4 "100 V" H 8100 3200 50  0001 C CNN "Voltage"
F 5 "C153036" H 8100 3200 50  0001 C CNN "LCSC Pn"
F 6 "-prod" H 8100 3200 50  0001 C CNN "Config"
	1    8100 3200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0120
U 1 1 60D62397
P 8950 3100
F 0 "#PWR0120" H 8950 2850 50  0001 C CNN
F 1 "GND" H 8955 2927 50  0000 C CNN
F 2 "" H 8950 3100 50  0001 C CNN
F 3 "" H 8950 3100 50  0001 C CNN
	1    8950 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	8950 3100 8950 3050
Wire Wire Line
	8950 3050 8550 3050
Wire Wire Line
	8100 3050 8100 3100
Wire Wire Line
	8100 3300 8100 3400
Connection ~ 8100 3400
Wire Wire Line
	8100 3400 8450 3400
Wire Wire Line
	8550 3050 8550 3100
Wire Wire Line
	8550 3300 8550 3400
Connection ~ 8550 3050
Wire Wire Line
	8550 3050 8100 3050
Connection ~ 8550 3400
Wire Wire Line
	8550 3400 8900 3400
$Comp
L Connector:Conn_01x04_Male J102
U 1 1 60CAFF80
P 10450 5350
F 0 "J102" H 10422 5232 50  0000 R CNN
F 1 "Conn_01x04_Male" H 10422 5323 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 10450 5350 50  0001 C CNN
F 3 "~" H 10450 5350 50  0001 C CNN
F 4 "-prod" H 10450 5350 50  0001 C CNN "Config"
	1    10450 5350
	-1   0    0    -1  
$EndComp
Wire Wire Line
	8450 3400 8450 3450
Wire Wire Line
	8000 3450 8000 3400
$Comp
L power:GND #PWR0115
U 1 1 60C7C685
P 8450 3650
F 0 "#PWR0115" H 8450 3400 50  0001 C CNN
F 1 "GND" H 8455 3477 50  0000 C CNN
F 2 "" H 8450 3650 50  0001 C CNN
F 3 "" H 8450 3650 50  0001 C CNN
	1    8450 3650
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C115
U 1 1 60C776D1
P 8450 3550
F 0 "C115" H 8565 3596 50  0000 L CNN
F 1 "2u2" H 8565 3505 50  0000 L CNN
F 2 "Capacitor_SMD:C_1210_3225Metric" H 8488 3400 50  0001 C CNN
F 3 "~" H 8450 3550 50  0001 C CNN
F 4 "100 V" H 8450 3550 50  0001 C CNN "Voltage"
F 5 "C153036" H 8450 3550 50  0001 C CNN "LCSC Pn"
	1    8450 3550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0110
U 1 1 60C004A2
P 8000 3650
F 0 "#PWR0110" H 8000 3400 50  0001 C CNN
F 1 "GND" H 8005 3477 50  0000 C CNN
F 2 "" H 8000 3650 50  0001 C CNN
F 3 "" H 8000 3650 50  0001 C CNN
	1    8000 3650
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C109
U 1 1 60BB967B
P 8000 3550
F 0 "C109" H 8115 3596 50  0000 L CNN
F 1 "2u2" H 8115 3505 50  0000 L CNN
F 2 "Capacitor_SMD:C_1210_3225Metric" H 8038 3400 50  0001 C CNN
F 3 "~" H 8000 3550 50  0001 C CNN
F 4 "100 V" H 8000 3550 50  0001 C CNN "Voltage"
F 5 "C153036" H 8000 3550 50  0001 C CNN "LCSC Pn"
	1    8000 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	8900 3850 8900 3400
$Comp
L power:GND #PWR0111
U 1 1 60BFF42C
P 8900 4050
F 0 "#PWR0111" H 8900 3800 50  0001 C CNN
F 1 "GND" H 8905 3877 50  0000 C CNN
F 2 "" H 8900 4050 50  0001 C CNN
F 3 "" H 8900 4050 50  0001 C CNN
	1    8900 4050
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1_Small C110
U 1 1 60BF703D
P 8900 3950
F 0 "C110" H 8991 3996 50  0000 L CNN
F 1 "100u" H 8991 3905 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_6.3x7.7" H 8900 3950 50  0001 C CNN
F 3 "~" H 8900 3950 50  0001 C CNN
F 4 "C134220" H 8900 3950 50  0001 C CNN "LCSC Pn"
F 5 "35" H 8900 3950 50  0001 C CNN "Voltage"
	1    8900 3950
	1    0    0    -1  
$EndComp
$Comp
L Misc:IND_7443640680_WURTH L101
U 1 1 60B94479
P 7350 3400
F 0 "L101" H 7450 3500 60  0000 C CNN
F 1 "IND_7443640680_WURTH" H 7600 3050 60  0001 C CNN
F 2 "Misc:WE-HCF" H 7550 3600 60  0001 L CNN
F 3 "https://www.we-online.com/catalog/datasheet/7443640680.pdf" H 7550 3700 60  0001 L CNN
F 4 "Fixed Inductors" H 7550 4100 60  0001 L CNN "Family"
F 5 "FIXED IND 6.8UH 30A 2.4mOHM" H 7550 4400 60  0001 L CNN "Description"
F 6 "Wurth Elektronik" H 7550 4500 60  0001 L CNN "Manufacturer"
F 7 "Active" H 7550 4600 60  0001 L CNN "Status"
F 8 "7443640680" H 7300 3150 50  0001 C CNN "Manufacturer #"
F 9 "6.8 uH" H 7350 3293 50  0000 C CNN "Component Value"
F 10 "-prod" H 7350 3400 50  0001 C CNN "Config"
	1    7350 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	7600 3400 7650 3400
Connection ~ 7650 3400
Wire Wire Line
	7100 3400 6950 3400
Connection ~ 6950 3400
$Comp
L power:GND #PWR0122
U 1 1 60C0D9B0
P 7350 3650
F 0 "#PWR0122" H 7350 3400 50  0001 C CNN
F 1 "GND" H 7355 3477 50  0000 C CNN
F 2 "" H 7350 3650 50  0001 C CNN
F 3 "" H 7350 3650 50  0001 C CNN
	1    7350 3650
	1    0    0    -1  
$EndComp
Text Notes 9600 5250 0    50   ~ 0
26V=4.33V
Text Notes 1600 5950 0    50   ~ 0
2.727 - 4.545 V
$Comp
L Device:R_Small R123
U 1 1 60BEA4B3
P 10650 4300
F 0 "R123" H 10709 4346 50  0000 L CNN
F 1 "24k" H 10709 4255 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 10650 4300 50  0001 C CNN
F 3 "~" H 10650 4300 50  0001 C CNN
	1    10650 4300
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J104
U 1 1 60C12715
P 700 3300
F 0 "J104" H 700 3050 50  0000 C CNN
F 1 "Conn_01x02" H 700 3400 50  0000 C CNN
F 2 "Misc:1x2_3.5_pinHeader" H 700 3300 50  0001 C CNN
F 3 "~" H 700 3300 50  0001 C CNN
F 4 "-prod" H 700 3300 50  0001 C CNN "Config"
	1    700  3300
	-1   0    0    -1  
$EndComp
Wire Wire Line
	900  3300 950  3300
Wire Wire Line
	900  3400 1300 3400
Wire Wire Line
	1300 3400 1300 3100
Connection ~ 1300 3100
Wire Wire Line
	900  2750 1200 2750
Wire Wire Line
	1200 2750 1200 3300
Connection ~ 1200 2750
Wire Wire Line
	1200 2750 1250 2750
Connection ~ 1200 5550
$Comp
L Device:C_Small C119
U 1 1 60C36F59
P 1050 3300
F 0 "C119" V 900 3300 50  0000 C CNN
F 1 "10n" V 800 3300 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 1050 3300 50  0001 C CNN
F 3 "~" H 1050 3300 50  0001 C CNN
F 4 "C57112" V 1050 3300 50  0001 C CNN "LCSC Pn"
F 5 "-prod" V 1050 3300 50  0001 C CNN "Config"
	1    1050 3300
	0    1    -1   0   
$EndComp
Wire Wire Line
	1150 3300 1200 3300
Connection ~ 1200 3300
Wire Wire Line
	1200 3300 1200 5550
$Comp
L Device:C_Small C120
U 1 1 60C54B09
P 1700 2550
F 0 "C120" H 1815 2596 50  0000 L CNN
F 1 "2u2" H 1815 2505 50  0000 L CNN
F 2 "Capacitor_SMD:C_1210_3225Metric" H 1738 2400 50  0001 C CNN
F 3 "~" H 1700 2550 50  0001 C CNN
F 4 "100 V" H 1700 2550 50  0001 C CNN "Voltage"
F 5 "C153036" H 1700 2550 50  0001 C CNN "LCSC Pn"
F 6 "-prod" H 1700 2550 50  0001 C CNN "Config"
	1    1700 2550
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C121
U 1 1 60C9A675
P 2100 2550
F 0 "C121" H 2215 2596 50  0000 L CNN
F 1 "2u2" H 2215 2505 50  0000 L CNN
F 2 "Capacitor_SMD:C_1210_3225Metric" H 2138 2400 50  0001 C CNN
F 3 "~" H 2100 2550 50  0001 C CNN
F 4 "100 V" H 2100 2550 50  0001 C CNN "Voltage"
F 5 "C153036" H 2100 2550 50  0001 C CNN "LCSC Pn"
F 6 "-prod" H 2100 2550 50  0001 C CNN "Config"
	1    2100 2550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0123
U 1 1 60C9AD2F
P 2550 2450
F 0 "#PWR0123" H 2550 2200 50  0001 C CNN
F 1 "GND" H 2555 2277 50  0000 C CNN
F 2 "" H 2550 2450 50  0001 C CNN
F 3 "" H 2550 2450 50  0001 C CNN
	1    2550 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 2450 2550 2400
Wire Wire Line
	2550 2400 2100 2400
Wire Wire Line
	2100 2400 2100 2450
Wire Wire Line
	2100 2650 2100 2700
Connection ~ 2100 2700
Wire Wire Line
	2100 2700 2400 2700
Wire Wire Line
	2100 2400 1700 2400
Wire Wire Line
	1700 2400 1700 2450
Connection ~ 2100 2400
Wire Wire Line
	1700 2650 1700 2700
Connection ~ 1700 2700
Wire Wire Line
	1700 2700 1500 2700
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 60CC90E6
P 1250 2750
F 0 "#FLG0101" H 1250 2825 50  0001 C CNN
F 1 "PWR_FLAG" H 1250 2923 50  0000 C CNN
F 2 "" H 1250 2750 50  0001 C CNN
F 3 "~" H 1250 2750 50  0001 C CNN
	1    1250 2750
	1    0    0    -1  
$EndComp
Connection ~ 1250 2750
Wire Wire Line
	1250 2750 1500 2750
$Comp
L power:PWR_FLAG #FLG0102
U 1 1 60CC9856
P 10450 3400
F 0 "#FLG0102" H 10450 3475 50  0001 C CNN
F 1 "PWR_FLAG" H 10450 3573 50  0000 C CNN
F 2 "" H 10450 3400 50  0001 C CNN
F 3 "~" H 10450 3400 50  0001 C CNN
	1    10450 3400
	1    0    0    -1  
$EndComp
Connection ~ 10450 3400
Wire Wire Line
	10450 3400 10650 3400
$Comp
L power:PWR_FLAG #FLG0103
U 1 1 60CCBADB
P 11000 3800
F 0 "#FLG0103" H 11000 3875 50  0001 C CNN
F 1 "PWR_FLAG" H 11000 3973 50  0000 C CNN
F 2 "" H 11000 3800 50  0001 C CNN
F 3 "~" H 11000 3800 50  0001 C CNN
	1    11000 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	10800 3850 11000 3850
Wire Wire Line
	11000 3850 11000 3800
Connection ~ 10800 3850
Wire Wire Line
	10800 3850 10800 3900
Wire Wire Line
	10650 3400 10650 4200
Wire Wire Line
	10050 4450 10650 4450
$Comp
L Device:CP1_Small C114
U 1 1 60D3F955
P 9900 3550
F 0 "C114" H 9991 3596 50  0000 L CNN
F 1 "100u" H 9991 3505 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_8x10" H 9900 3550 50  0001 C CNN
F 3 "~" H 9900 3550 50  0001 C CNN
F 4 "C487360" H 9900 3550 50  0001 C CNN "LCSC Pn"
F 5 "50" H 9900 3550 50  0001 C CNN "Voltage"
	1    9900 3550
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1_Small C118
U 1 1 60D4025D
P 10300 3550
F 0 "C118" H 10391 3596 50  0000 L CNN
F 1 "100u" H 10391 3505 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_8x10" H 10300 3550 50  0001 C CNN
F 3 "~" H 10300 3550 50  0001 C CNN
F 4 "C487360" H 10300 3550 50  0001 C CNN "LCSC Pn"
F 5 "50" H 10300 3550 50  0001 C CNN "Voltage"
	1    10300 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	2750 3750 2750 3300
Wire Wire Line
	2750 3300 3450 3300
Wire Wire Line
	2400 3200 2450 3200
Wire Wire Line
	2400 2700 2400 3200
Wire Wire Line
	2900 3450 2900 3400
Wire Wire Line
	2900 3400 3550 3400
Wire Wire Line
	3550 3400 3550 3200
Wire Wire Line
	3550 3200 3600 3200
Wire Wire Line
	3450 3300 3450 3200
Wire Wire Line
	3450 2900 3600 2900
Wire Wire Line
	3400 3200 3450 3200
Connection ~ 3450 3200
Wire Wire Line
	3450 3200 3450 2900
$EndSCHEMATC
