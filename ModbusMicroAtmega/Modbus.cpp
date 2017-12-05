/**
Este exemplo � de dom�nio p�blico
Baseado na biblioteca de Juan Pablo Zometa : jpmzometa@gmail.com
http://sites.google.com/site/jpmzometa/
and Samuel Marco: sammarcoarmengol@gmail.com
and Andras Tucsni.
As fun��es do protocolo MODBUS implementadas neste c�digo:
3 - Read holding registers;
6 - Preset single register;
16 - Preset multiple registers.
This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
The functions included here have been derived from the
Modicon Modbus Protocol Reference Guide
which can be obtained from Schneider at www.schneiderautomation.com.
This code has its origins with
who wrote a small program to read 100 registers from a modbus slave.
*/
#include "Modbus.h"

/**
* \brief
* \param txPin
* \param commBaudRate
* \param PARITY
* \param mbSlaveId
* \param holdingRegisters
* \param holdingRegistersSize
*/
Modbus::Modbus(uint8_t txPin, uint32_t commBaudRate, ModbusParity parity, uint8_t mbSlaveId, uint16_t* holdingRegisters,
	uint16_t holdingRegistersSize) {
	this->parity = uint8_t(parity);
	this->lastBytesReceived = 0;
	this->holdingRegisters = holdingRegisters;
	this->holdingRegistersSize = holdingRegistersSize;
	this->txPin = txPin;
	this->commBaudRate = commBaudRate;
	this->parity = parity;
	this->mbSlaveId = mbSlaveId;
}


/**************************************
CRC
INPUTS:
buf -> Matriz contendo a mensagem a ser enviada para o controlador mestre.
start -> In�cio do loop no crc do contador, normalmente 0.
cnt -> Quantidade de bytes na mensagem a ser enviada para o controlador mestre
OUTPUTS:
temp -> Retorna byte crc para a mensagem.
COMMENT�RIOS:
Esta rotina calcula o byte crc alto e baixo de uma mensagem.
Note que este CRC � usado somente para Modbus, n�o em Modbus PLUS ou TCP.
****************************************************************************/
uint16_t Modbus::crc(uint8_t* buf, uint8_t start, uint8_t cnt) const {
	uint8_t i, j;
	unsigned temp, temp2, flag;
	temp = 0xFFFF;
	for (i = start; i < cnt; i++) {
		temp = temp ^ buf[i];
		for (j = 1; j <= 8; j++) {
			flag = temp & 0x0001;
			temp = temp >> 1;
			if (flag)
				temp = temp ^ 0xA001;
		}
	}
	/* Inverter a ordem dos bytes. */
	temp2 = temp >> 8;
	temp = (temp << 8) | temp2;

	temp &= 0xFFFF;
	return (temp);
}

/***********************************************************************
*
* As seguintes fun��es constroem o frame de
* um pacote de consulta modbus.
*
***********************************************************************/
/*
* In�cio do pacote de uma resposta read_holding_register
*/
void Modbus::build_read_packet(uint8_t slave, uint8_t function, uint8_t count,
	uint8_t* packet) {
	packet[SLAVE] = slave;
	packet[FUNC] = function;
	packet[2] = count * 2;
}

/*
* In�cio do pacote de uma resposta preset_multiple_register
*/
void Modbus::build_write_packet(uint8_t slave, uint8_t function, uint16_t start_addr,
	uint8_t count, uint8_t* packet) {
	packet[SLAVE] = slave;
	packet[FUNC] = function;
	packet[START_H] = start_addr >> 8;
	packet[START_L] = start_addr & 0x00ff;
	packet[REGS_H] = 0x00;
	packet[REGS_L] = count;
}

/*
* In�cio do pacote de uma resposta write_single_register
*/
void Modbus::build_write_single_packet(uint8_t slave, uint8_t function, uint16_t write_addr,
	uint16_t reg_val, uint8_t* packet) {
	packet[SLAVE] = slave;
	packet[FUNC] = function;
	packet[START_H] = write_addr >> 8;
	packet[START_L] = write_addr & 0x00ff;
	packet[REGS_H] = reg_val >> 8;
	packet[REGS_L] = reg_val & 0x00ff;
}

/*
* In�cio do pacote de uma resposta excep��o
*/
void Modbus::build_error_packet(uint8_t slave, uint8_t function, uint8_t exception,
	uint8_t* packet) {
	packet[SLAVE] = slave;
	packet[FUNC] = function + 0x80;
	packet[2] = exception;
}

/*************************************************************************
*
* modbus_query( packet, length)
*
* Fun��o para adicionar uma soma de verifica��o para o fim de um pacote.
* Por favor, note que a matriz pacote deve ser de pelo menos 2 campos mais do que
* String_length.
**************************************************************************/
void Modbus::modbus_reply(uint8_t* packet, uint8_t string_length) {
	int temp_crc;
	temp_crc = crc(packet, 0, string_length);
	packet[string_length] = temp_crc >> 8;
	string_length++;
	packet[string_length] = temp_crc & 0x00FF;
}

/***********************************************************************
*
* send_reply( query_string, query_length )
*
* Fun��o para enviar uma resposta a um mestre Modbus.
* Retorna: o n�mero total de caracteres enviados
************************************************************************/
int Modbus::send_reply(uint8_t* query, uint8_t string_length) {
	uint8_t i;
	if (txPin > 1) {
		// coloca o MAX485 no modo de transmiss�o
		UCSR0A = UCSR0A | (1 << TXC0);
		digitalWrite(txPin, HIGH);
		delayMicroseconds(3640); // aguarda silencio de 3.5 caracteres em 9600bps
	}
	modbus_reply(query, string_length);
	string_length += 2;
	for (i = 0; i < string_length; i++) {
		Serial.write(byte(query[i]));
	}
	if (txPin > 1) {
		// coloca o MAX485 no modo de recep��o
		while (!(UCSR0A & (1 << TXC0)));
		digitalWrite(txPin, LOW);
	}
	return i; /* isso n�o significa que a grava��o foi bem sucedida */
}

/***********************************************************************
*
* receive_request( array_for_data )
*
* Fun��o para monitorar um pedido do mestre modbus.
*
* Retorna: N�mero total de caracteres recebidos se OK
* 0 se n�o houver nenhum pedido
* Um c�digo de erro negativo em caso de falha
***********************************************************************/
int Modbus::receive_request(uint8_t* received_string) {
	int bytes_received = 0;
	/* FIXME: n�o Serial.available esperar 1.5T ou 3.5T antes de sair do loop? */
	while (Serial.available()) {
		received_string[bytes_received] = Serial.read();
		bytes_received++;
		if (bytes_received >= MAX_MESSAGE_LENGTH)
			return NO_REPLY; /* erro de porta */
	}
	return (bytes_received);
}

/*********************************************************************
*
* modbus_request(slave_id, request_data_array)
*
* Fun��o que � retornada quando o pedido est� correto
* e a soma de verifica��o est� correto.
* Retorna: string_length se OK
* 0 se n�o
* Menos de 0 para erros de exce��o
*
* Nota: Todas as fun��es usadas para enviar ou receber dados via
* Modbus devolver esses valores de retorno.
*
**********************************************************************/
int Modbus::modbus_request(uint8_t slave, uint8_t* data) {
	int response_length;
	uint16_t crc_calc;
	uint16_t crc_received;
	uint8_t recv_crc_hi;
	uint8_t recv_crc_lo;
	response_length = receive_request(data);
	if (response_length > 0) {
		crc_calc = crc(data, 0, response_length - 2);
		recv_crc_hi = unsigned(data[response_length - 2]);
		recv_crc_lo = unsigned(data[response_length - 1]);
		crc_received = data[response_length - 2];
		crc_received = unsigned(crc_received) << 8;
		crc_received =
			crc_received | unsigned(data[response_length - 1]);
		/*********** verificar CRC da resposta ************/
		if (crc_calc != crc_received) {
			return NO_REPLY;
		}
		/* verificar a ID do escravo */
		if (slave != data[SLAVE]) {
			return NO_REPLY;
		}
	}
	return (response_length);
}

/*********************************************************************
*
* validate_request(request_data_array, request_length, available_regs)
*
* Fun��o para verificar se o pedido pode ser processado pelo escravo.
*
* Retorna: 0 se OK
* Um c�digo de exce��o negativa em caso de erro
*
**********************************************************************/
int Modbus::validate_request(uint8_t* data, uint8_t length, uint16_t regs_size) {
	int i, fcnt = 0;
	uint16_t regs_num;
	uint16_t start_addr;
	uint8_t max_regs_num = 0;
	/* verificar o c�digo de fun��o */
	for (i = 0; i < sizeof(funcSupported); i++) {
		if (funcSupported[i] == data[FUNC]) {
			fcnt = 1;
			break;
		}
	}
	if (0 == fcnt)
		return EXC_FUNC_CODE;
	if (FC_WRITE_REG == data[FUNC]) {
		/* Para a fun��o de escrever um reg �nico, este � o registro alvo.*/
		regs_num = TO_UINT_16_B(data[START_H], data[START_L]);
		//regs_num = ((int)data[START_H] << 8) + (int)data[START_L];
		if (regs_num >= regs_size)
			return EXC_ADDR_RANGE;
		return 0;
	}
	/* Para as fun��es de leitura / escrita de registros, este � o intervalo. */
	regs_num = TO_UINT_16_B(data[START_H], data[START_L]);
	/* verifica a quantidade de registros */
	if (FC_READ_REGS == data[FUNC])
		max_regs_num = MAX_READ_REGS;
	else if (FC_WRITE_REGS == data[FUNC])
		max_regs_num = MAX_WRITE_REGS;

	if ((regs_num < 1) || (regs_num > max_regs_num))
		return EXC_REGS_QUANT;
	/* verificar� a quantidade de registros, endere�o inicial � 0 */
	start_addr = TO_UINT_16_B(data[START_H], data[START_L]);
	if ((start_addr + regs_num) > regs_size)
		return EXC_ADDR_RANGE;
	return 0; /* OK, sem exce��o */
}

/************************************************************************
*
* write_regs(first_register, data_array, registers_array)
*
* escreve nos registradores do escravo os dados em consulta,
* A partir de start_addr.
*
* Retorna: o n�mero de registros escritos
************************************************************************/
int Modbus::write_regs(uint16_t start_addr, uint8_t* query, uint16_t* regs) {
	int temp;
	unsigned int i;
	for (i = 0; i < query[REGS_L]; i++) {
		/* mudar reg hi_byte para temp */
		temp = int(query[(BYTE_CNT + 1) + i * 2]) << 8;
		/* OR com lo_byte */
		temp = temp | int(query[(BYTE_CNT + 2) + i * 2]);
		regs[start_addr + i] = temp;
	}
	return i;
}

/************************************************************************
*
* preset_multiple_registers(slave_id, first_register, number_of_registers,
* data_array, registers_array)
*
* Escreva os dados na matriz dos registos do escravo.
*
*************************************************************************/
int Modbus::preset_multiple_registers(uint8_t slave, uint16_t start_addr, uint8_t count,
	uint8_t* query, uint16_t* regs) {
	uint8_t function = FC_WRITE_REGS; /* Escrever em m�ltiplos registros */
	int status = 0;
	uint8_t packet[RESPONSE_SIZE + CHECKSUM_SIZE];
	build_write_packet(slave, function, start_addr, count, packet);
	if (write_regs(start_addr, query, regs)) {
		status = send_reply(packet, RESPONSE_SIZE);
	}
	return (status);
}

/************************************************************************
*
* write_single_register(slave_id, write_addr, data_array, registers_array)
*
* Escrever um �nico valor inteiro em um �nico registo do escravo.
*
*************************************************************************/
int Modbus::write_single_register(uint8_t slave, uint16_t write_addr, uint8_t* query, uint16_t* regs) {
	uint8_t function = FC_WRITE_REG; /* Fun��o: Write Single Register */
	unsigned int reg_val;
	uint8_t packet[RESPONSE_SIZE + CHECKSUM_SIZE];
	reg_val = query[REGS_H] << 8 | query[REGS_L];
	build_write_single_packet(slave, function, write_addr, reg_val, packet);
	regs[write_addr] = (int)reg_val;
	/*
	written.start_addr=write_addr;
	written.num_regs=1;
	*/
	return (send_reply(packet, RESPONSE_SIZE));

}

/************************************************************************
*
* read_holding_registers(slave_id, first_register, number_of_registers,
* registers_array)
*
* l� os registros do escravo e envia para o mestre Modbus
*
*************************************************************************/
int Modbus::read_holding_registers(uint8_t slave, uint16_t start_addr, uint8_t reg_count, uint16_t* regs) {
	uint8_t function = 0x03; /* Fun��o 03: Read Holding Registers */
	int packet_size = 3;
	int status;
	unsigned int i;
	uint8_t packet[MAX_MESSAGE_LENGTH];
	build_read_packet(slave, function, reg_count, packet);
	for (i = start_addr; i < (start_addr + (uint16_t)reg_count);
		i++) {
		packet[packet_size] = regs[i] >> 8;
		packet_size++;
		packet[packet_size] = regs[i] & 0x00FF;
		packet_size++;
	}
	status = send_reply(packet, packet_size);
	return (status);
}

//long baud, char parity, char txenpin
void Modbus::configure_mb_slave() {
	Serial.begin(commBaudRate);
	switch (parity) {
	case 'e': // 8E1
		UCSR0C |= ((1 << UPM01) | (1 << UCSZ01) | (1 << UCSZ00));
		// UCSR0C &= ~((1<<UPM00) | (1<<UCSZ02) | (1<<USBS0));
		break;
	case 'o': // 8O1
		UCSR0C |= ((1 << UPM01) | (1 << UPM00) | (1 << UCSZ01) | (1 << UCSZ00));
		// UCSR0C &= ~((1<<UCSZ02) | (1<<USBS0));
		break;
	case 'n': // 8N1
		UCSR0C |= ((1 << UCSZ01) | (1 << UCSZ00));
		// UCSR0C &= ~((1<<UPM01) | (1<<UPM00) | (1<<UCSZ02) | (1<<USBS0));
		break;
	default:
		break;
	}
	if (parity > 1) {
		// pino 0 & pino 1 s�o reservados para RX/TX
		parity = parity; /* definir vari�vel global */
		pinMode(parity, OUTPUT);
		digitalWrite(parity, LOW);
	}
	return;
}

/*
* update_mb_slave(slave_id, holding_regs_array, number_of_regs)
*
* verifica se h� qualquer pedido v�lido do mestre modbus. Se houver,
* executa a a��o solicitada
*/
int Modbus::update_mb_slave() {
	uint8_t query[MAX_MESSAGE_LENGTH];
	uint8_t errpacket[EXCEPTION_SIZE + CHECKSUM_SIZE];
	if (int length = Serial.available()) {
		const unsigned long now = millis();
		if (lastBytesReceived != length) {
			lastBytesReceived = length;
			Nowdt = now + T35;
			return 0;
		}
		if (now < Nowdt) {
			return 0;
		}
		lastBytesReceived = 0;
		length = modbus_request(SLAVE, query);
		if (length < 1)
			return length;
		if (const int exception = validate_request(query, length, holdingRegistersSize)) {
			build_error_packet(SLAVE, query[FUNC], exception, errpacket);
			send_reply(errpacket, EXCEPTION_SIZE);
			return (exception);
		}
		const uint16_t start_addr = (int(query[START_H]) << 8) + int(query[START_L]);
		switch (query[FUNC]) {
		case FC_READ_REGS:
			return read_holding_registers(SLAVE, start_addr, query[REGS_L], holdingRegisters);
		case FC_WRITE_REGS:
			return preset_multiple_registers(SLAVE, start_addr, query[REGS_L], query, holdingRegisters);
		case FC_WRITE_REG:
			return write_single_register(SLAVE, start_addr, query, holdingRegisters);
		default:
			return 0;
		}
	}
	else {
		lastBytesReceived = 0;
		return 0;
	}
}
