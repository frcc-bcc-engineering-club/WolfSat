#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "sensirion_arch_config.h"
#include "sensirion_uart.h"

#define BAUDRATE 115200

	s16 sensirion_uart_open()
	{
		Serial2.begin(BAUDRATE);
		while (!Serial2)
		{
			delay(100);
		}
		return 0;
	}

	s16 sensirion_uart_close()
	{
		Serial2.end();
		return 0;
	}

	s16 sensirion_uart_tx(u16 data_len, const u8 *data)
	{
		return Serial2.write(data, data_len);
	}

	s16 sensirion_uart_rx(u16 max_data_len, u8* data)
	{
		s16 i = 0;

		while ((Serial2.available() > 0) && (i < max_data_len))
		{
			data[i] = (u8)Serial2.read();
			i++;
		}

		return i;
	}

	void sensirion_sleep_usec(u32 useconds)
	{
		delay((useconds / 1000) + 1);
	}

#ifdef __cplusplus
}	//extern "C"
#endif