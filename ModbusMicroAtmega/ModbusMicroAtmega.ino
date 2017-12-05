/**
*	Exemplo de utilização da biblioteca Modbus RTU
*
*/
#include "Modbus.h"

/**
 *	Holding Registers
 */
enum {
	MB_REG_A,
	MB_REG_B,
	MB_REG_C,
	MB_REG_D,
	MB_REG_E,
	MB_REG_F,
	MB_REG_TOTAL
};
uint16_t holdingRegisters[MB_REG_TOTAL];
const uint8_t txPin = 1;
const uint32_t commBaudRate = 9600;
const uint8_t slaveId = 1;

unsigned long wdog = 0; /* watchdog */

Modbus modbus = Modbus(txPin, commBaudRate,  MB_PARITY_N, slaveId, holdingRegisters, MB_REG_TOTAL);

void setup() {
	// configura o Modbus
	modbus.configure_mb_slave();
}

void loop() {
	// Verificando se tem alguma coisa
	if (modbus.update_mb_slave()) {
		wdog = millis();
	}
}
