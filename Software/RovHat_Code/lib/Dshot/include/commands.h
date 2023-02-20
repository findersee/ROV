#ifndef _COMMANDS_H_
#define _COMMANDS_H_

#define	DSHOT_CMD_MOTOR_STOP	0x0
#define	DSHOT_CMD_BEEP1	0x1	//	1	Wait at least length of beep (260ms) before next command
#define	DSHOT_CMD_BEEP2	0x2	//	2	Wait at least length of beep (260ms) before next command
#define	DSHOT_CMD_BEEP3	0x3	//	3	Wait at least length of beep (260ms) before next command
#define	DSHOT_CMD_BEEP4	0x4	//	4	Wait at least length of beep (260ms) before next command
#define	DSHOT_CMD_BEEP5	0x5	//	5	Wait at least length of beep (260ms) before next command
#define	DSHOT_CMD_ESC_INFO	0x6	//	6	Wait at least 12ms before next command
#define	DSHOT_CMD_SPIN_DIRECTION_1	0x7	//	7	Need 6x
#define	DSHOT_CMD_SPIN_DIRECTION_2	0x8	//	8	Need 6x
#define	DSHOT_CMD_3D_MODE_OFF	0x9	//	9	Need 6x
#define	DSHOT_CMD_3D_MODE_ON	0xA	//	10	Need 6x
#define	DSHOT_CMD_SETTINGS_REQUEST	0xB
#define	DSHOT_CMD_SAVE_SETTINGS	0xC	//	12	Need 6x, wait at least 35ms before next command
#define	DSHOT_EXTENDED_TELEMETRY_ENABLE	0xD	//	13	Need 6x (only on EDT enabed firmware)
#define	DSHOT_EXTENDED_TELEMETRY_DISABLE	0xE	//	14	Need 6x (only on EDT enabed firmware)
#define	DSHOT_CMD_SPIN_DIRECTION_NORMAL	0x14	//	20	Need 6x
#define	DSHOT_CMD_SPIN_DIRECTION_REVERSED	0x15	//	21	Need 6x
#define	DSHOT_CMD_LED0_ON	0x16
#define	DSHOT_CMD_LED1_ON	0x17
#define	DSHOT_CMD_LED2_ON	0x18
#define	DSHOT_CMD_LED3_ON	0x19
#define	DSHOT_CMD_LED0_OFF	0x1A
#define	DSHOT_CMD_LED1_OFF	0x1B
#define	DSHOT_CMD_LED2_OFF	0x1C
#define	DSHOT_CMD_LED3_OFF	0x1D
#define	DSHOT_CMD_SIGNAL_LINE_TELEMETRY_DISABLE	0x20	//	32	Need 6x. Disables commands 42 to 47
#define	DSHOT_CMD_SIGNAL_LINE_TELEMETRY_ENABLE	0x21	//	33	Need 6x. Enables commands 42 to 47
#define	DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_TELEMETRY	0x22	//	34	Need 6x. Enables commands 42 to 47 and sends erpm if normal Dshot frame
#define	DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_PERIOD_TELEMETRY	0x23	//	35	Need 6x. Enables commands 42 to 47 and sends erpm period if normal Dshot frame
#define	DSHOT_CMD_SIGNAL_LINE_TEMPERATURE_TELEMETRY	0x2A	//	42	1°C per LSB
#define	DSHOT_CMD_SIGNAL_LINE_VOLTAGE_TELEMETRY	0x2B	//	43	10mV per LSB, 40.95V max
#define	DSHOT_CMD_SIGNAL_LINE_CURRENT_TELEMETRY	0x2C	//	44	100mA per LSB, 409.5A max
#define	DSHOT_CMD_SIGNAL_LINE_CONSUMPTION_TELEMETRY	0x2D	//	45	10mAh per LSB, 40.95Ah max
#define	DSHOT_CMD_SIGNAL_LINE_ERPM_TELEMETRY	0x2E	//	46	100erpm per LSB, 409500erpm max
#define	DSHOT_CMD_SIGNAL_LINE_ERPM_PERIOD_TELEMETRY	0x2F	//	47	16us per LSB, 65520us max TBD

#endif