﻿/**
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
Modbus::Modbus(uint8_t txPin, uint32_t commBaudRate, ModbusParity parity,
               uint8_t mbSlaveId, uint16_t* holdingRegisters, uint16_t holdingRegistersSize) :
	commBaudRate(commBaudRate), parity(parity),
	mbSlaveId(mbSlaveId), holdingRegisters(holdingRegisters),
	holdingRegistersSize(holdingRegistersSize) {

	/* Os pinos 0 e 1 sao reservador para RXTX*/
	if (txPin > 1) {
		this->txPin = txPin;
		pinMode(parity, OUTPUT);
		digitalWrite(parity, LOW);
	}
};

/**
 * \brief Configuração para o uso de RS232
 * \param commBaudRate 
 * \param parity 
 * \param mbSlaveId 
 * \param holdingRegisters 
 * \param holdingRegistersSize 
 */
Modbus::Modbus(uint32_t commBaudRate, ModbusParity parity,
               uint8_t mbSlaveId, uint16_t* holdingRegisters, uint16_t holdingRegistersSize) :
	commBaudRate(commBaudRate), parity(parity),
	mbSlaveId(mbSlaveId), holdingRegisters(holdingRegisters),
	holdingRegistersSize(holdingRegistersSize), txPin(0) {

};

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
* Inicio do pacote de uma resposta read_holding_register
*/
void Modbus::buildReadPacket(uint8_t function, uint8_t count, uint8_t* packet) {
	packet[POS_VEC_SLAVE] = slaveId;
	packet[POS_VEC_FUNC] = function;
	packet[2] = count * 2;
}

/*
* In�cio do pacote de uma resposta preset_multiple_register
*/
void Modbus::buildWritePacket(uint8_t function, uint16_t start_addr, uint8_t count, uint8_t* packet) {
	packet[POS_VEC_SLAVE] = slaveId;
	packet[POS_VEC_FUNC] = function;
	packet[POS_VEC_START_H] = start_addr >> 8;
	packet[POS_VEC_START_L] = start_addr & 0x00ff;
	packet[POS_VEC_REGS_H] = 0x00;
	packet[POS_VEC_REGS_L] = count;
}

/*
* In�cio do pacote de uma resposta writeSingleRegister
*/
void Modbus::buildWriteSinglePacket(uint8_t function, uint16_t write_addr, uint16_t reg_val, uint8_t* packet) {
	packet[POS_VEC_SLAVE] = slaveId;
	packet[POS_VEC_FUNC] = function;
	packet[POS_VEC_START_H] = write_addr >> 8;
	packet[POS_VEC_START_L] = write_addr & 0x00ff;
	packet[POS_VEC_REGS_H] = reg_val >> 8;
	packet[POS_VEC_REGS_L] = reg_val & 0x00ff;
}

/*
* In�cio do pacote de uma resposta excep��o
*/
void Modbus::buildErrorPacket(uint8_t function, uint8_t exception, uint8_t* packet) {
	packet[POS_VEC_SLAVE] = slaveId;
	packet[POS_VEC_FUNC] = function + 0x80;
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
void Modbus::modbusReply(uint8_t* packet, uint8_t string_length) {
	int temp_crc;
	temp_crc = crc(packet, 0, string_length);
	packet[string_length] = temp_crc >> 8;
	string_length++;
	packet[string_length] = temp_crc & 0x00FF;
}

/***********************************************************************
*
* sendReply( query_string, query_length )
*
* Fun��o para enviar uma resposta a um mestre Modbus.
* Retorna: o n�mero total de caracteres enviados
************************************************************************/
int Modbus::sendReply(uint8_t* query, uint8_t string_length) {
	uint8_t i;
	if (txPin > 1) {
		// coloca o MAX485 no modo de transmissao
		UCSR0A = UCSR0A | (1 << TXC0);
		digitalWrite(txPin, HIGH);
		delayMicroseconds(3640); // aguarda silencio de 3.5 caracteres em 9600bps
	}
	modbusReply(query, string_length);
	string_length += 2;
	for (i = 0; i < string_length; i++) {
		Serial.write(byte(query[i]));
	}
	if (txPin > 1) {
		// coloca o MAX485 no modo de recepcao
		while (!(UCSR0A & (1 << TXC0)));
		digitalWrite(txPin, LOW);
	}
	return i; /* isso n�o significa que a grava��o foi bem sucedida */
}

/***********************************************************************
*
* receiveRequest( array_for_data )
*
* Fun��o para monitorar um pedido do mestre modbus.
*
* Retorna: N�mero total de caracteres recebidos se MB_RES_OK
* 0 se n�o houver nenhum pedido
* Um c�digo de erro negativo em caso de falha
***********************************************************************/
int Modbus::receiveRequest(uint8_t* received_string) {
	int bytes_received = 0;
	/* FIXME: n�o Serial.available esperar 1.5T ou 3.5T antes de sair do loop? */
	while (Serial.available()) {
		received_string[bytes_received] = Serial.read();
		bytes_received++;
		if (bytes_received >= MAX_MESSAGE_LENGTH)
			return MB_RES_NO_REPLY; /* erro de porta */
	}
	return (bytes_received);
}

/*********************************************************************
*
* modbusRequest(slave_id, request_data_array)
*
* Fun��o que � retornada quando o pedido est� correto
* e a soma de verifica��o est� correto.
* Retorna: string_length se MB_RES_OK
* 0 se n�o
* Menos de 0 para erros de exce��o
*
* Nota: Todas as fun��es usadas para enviar ou receber dados via
* Modbus devolver esses valores de retorno.
*
**********************************************************************/
int Modbus::modbusRequest(uint8_t* data) {
	int response_length;
	uint16_t crc_calc;
	uint16_t crc_received;
	uint8_t recv_crc_hi;
	uint8_t recv_crc_lo;
	response_length = receiveRequest(data);
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
			return MB_RES_NO_REPLY;
		}
		/* verificar a ID do escravo */
		if (slaveId != data[POS_VEC_SLAVE]) {
			return MB_RES_NO_REPLY;
		}
	}
	return (response_length);
}

/*********************************************************************
*
* validateRequest(request_data_array, request_length, available_regs)
*
* Fun��o para verificar se o pedido pode ser processado pelo escravo.
*
* Retorna: 0 se MB_RES_OK
* Um codigo de excecao negativa em caso de erro
*
**********************************************************************/
Modbus::ModbusResponse Modbus::validateRequest(uint8_t* data, uint8_t length, uint16_t regs_size) {
	int i, fcnt = 0;
	uint16_t regs_num;
	uint16_t start_addr;
	uint8_t max_regs_num = 0;
	/* verificar o c�digo de fun��o */
	for (i = 0; i < sizeof(funcSupported); i++) {
		if (funcSupported[i] == data[POS_VEC_FUNC]) {
			fcnt = 1;
			break;
		}
	}
	if (0 == fcnt)
		return MB_RES_EXC_FUNC_CODE;
	if (FC_WRITE_REG == data[POS_VEC_FUNC]) {
		/* Para a fun��o de escrever um reg �nico, este � o registro alvo.*/
		regs_num = TO_UINT_16_B(data[POS_VEC_START_H], data[POS_VEC_START_L]);
		//regs_num = ((int)data[START_H] << 8) + (int)data[START_L];
		if (regs_num >= regs_size)
			return MB_RES_EXC_ADDR_RANGE;
		return MB_RES_OK;
	}
	/* Para as fun��es de leitura / escrita de registros, este � o intervalo. */
	regs_num = TO_UINT_16_B(data[POS_VEC_START_H], data[POS_VEC_START_L]);
	/* verifica a quantidade de registros */
	if (FC_READ_REGS == data[POS_VEC_FUNC])
		max_regs_num = MAX_READ_REGS;
	else if (FC_WRITE_REGS == data[POS_VEC_FUNC])
		max_regs_num = MAX_WRITE_REGS;

	if ((regs_num < 1) || (regs_num > max_regs_num))
		return MB_RES_EXC_REGS_QUANT;
	/* verificar� a quantidade de registros, endere�o inicial � 0 */
	start_addr = TO_UINT_16_B(data[POS_VEC_START_H], data[POS_VEC_START_L]);
	if ((start_addr + regs_num) > regs_size)
		return MB_RES_EXC_ADDR_RANGE;
	return MB_RES_OK; /* MB_RES_OK, sem exce��o */
}

/************************************************************************
*
* writeRegs(first_register, data_array, registers_array)
*
* escreve nos registradores do escravo os dados em consulta,
* A partir de start_addr.
*
* Retorna: o n�mero de registros escritos
************************************************************************/
int Modbus::writeRegs(uint16_t start_addr, uint8_t* query, uint16_t* regs) {
	int temp;
	unsigned int i;
	for (i = 0; i < query[POS_VEC_REGS_L]; i++) {
		/* mudar reg hi_byte para temp */
		temp = int(query[(POS_VEC_BYTE_CNT + 1) + i * 2]) << 8;
		/* OR com lo_byte */
		temp = temp | int(query[(POS_VEC_BYTE_CNT + 2) + i * 2]);
		regs[start_addr + i] = temp;
	}
	return i;
}

/************************************************************************
*
* presetMultipleRegisters(slave_id, first_register, number_of_registers,
* data_array, registers_array)
*
* Escreva os dados na matriz dos registos do escravo.
*
*************************************************************************/
int Modbus::presetMultipleRegisters(uint16_t start_addr, uint8_t count, uint8_t* query, uint16_t* regs) {
	uint8_t function = FC_WRITE_REGS; /* Escrever em m�ltiplos registros */
	int status = 0;
	uint8_t packet[RESPONSE_SIZE + CHECKSUM_SIZE];
	buildWritePacket(function, start_addr, count, packet);
	if (writeRegs(start_addr, query, regs)) {
		status = sendReply(packet, RESPONSE_SIZE);
	}
	return (status);
}

/************************************************************************
*
* writeSingleRegister(slave_id, write_addr, data_array, registers_array)
*
* Escrever um �nico valor inteiro em um �nico registo do escravo.
*
*************************************************************************/
int Modbus::writeSingleRegister(uint16_t write_addr, uint8_t* query, uint16_t* regs) {
	uint8_t function = FC_WRITE_REG; /* Fun��o: Write Single Register */
	unsigned int reg_val;
	uint8_t packet[RESPONSE_SIZE + CHECKSUM_SIZE];
	reg_val = query[POS_VEC_REGS_H] << 8 | query[POS_VEC_REGS_L];
	buildWriteSinglePacket(function, write_addr, reg_val, packet);
	regs[write_addr] = int(reg_val);
	/*
	written.start_addr=write_addr;
	written.num_regs=1;
	*/
	return (sendReply(packet, RESPONSE_SIZE));

}

/**
* \brief Verifica se há algum pedido válido do mestre. Executa as devidas ações.
* \return -1 No reply; 0 nenhum pedido do mestre; 1-4 para exceptions; (>4) numero de bytes enviados como resposta se ok
*/
int Modbus::readHoldingRegisters(uint16_t start_addr, uint8_t reg_count, uint16_t* regs) {
	uint8_t function = 0x03; /* Fun��o 03: Read Holding Registers */
	int packet_size = 3;
	int status;
	unsigned int i;
	uint8_t packet[MAX_MESSAGE_LENGTH];
	buildReadPacket(function, reg_count, packet);
	for (i = start_addr; i < (start_addr + uint16_t(reg_count));
	     i++) {
		packet[packet_size] = regs[i] >> 8;
		packet_size++;
		packet[packet_size] = regs[i] & 0x00FF;
		packet_size++;
	}
	status = sendReply(packet, packet_size);
	return (status);
}

//long baud, char parity, char txenpin
void Modbus::configure_mb_slave() {
	Serial.begin(commBaudRate);
	switch (parity) {
	case MB_PARITY_E: // 8E1
		UCSR0C |= ((1 << UPM01) | (1 << UCSZ01) | (1 << UCSZ00));
		// UCSR0C &= ~((1<<UPM00) | (1<<UCSZ02) | (1<<USBS0));
		break;
	case MB_PARITY_O: // 8O1
		UCSR0C |= ((1 << UPM01) | (1 << UPM00) | (1 << UCSZ01) | (1 << UCSZ00));
		// UCSR0C &= ~((1<<UCSZ02) | (1<<USBS0));
		break;
	case MB_PARITY_N: // 8N1
		UCSR0C |= ((1 << UCSZ01) | (1 << UCSZ00));
		// UCSR0C &= ~((1<<UPM01) | (1<<UPM00) | (1<<UCSZ02) | (1<<USBS0));
		break;
	default:
		break;
	}
}

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
		length = modbusRequest(query);
		if (length < 1)
			return length;
		if (const int exception = validateRequest(query, length, holdingRegistersSize)) {
			buildErrorPacket(query[POS_VEC_FUNC], exception, errpacket);
			sendReply(errpacket, EXCEPTION_SIZE);
			return (exception);
		}
		const uint16_t start_addr = (int(query[POS_VEC_START_H]) << 8) + int(query[POS_VEC_START_L]);
		switch (query[POS_VEC_FUNC]) {
		case FC_READ_REGS:
			return readHoldingRegisters(start_addr, query[POS_VEC_REGS_L], holdingRegisters);
		case FC_WRITE_REGS:
			return presetMultipleRegisters(start_addr, query[POS_VEC_REGS_L], query, holdingRegisters);
		case FC_WRITE_REG:
			return writeSingleRegister(start_addr, query, holdingRegisters);
		default:
			return 0;
		}
	} else {
		lastBytesReceived = 0;
		return 0;
	}
}
