﻿/**
 * Este exemplo é de domínio público
  Testado na IDE 1.0.1
  Baseado na biblioteca de Juan Pablo Zometa : jpmzometa@gmail.com
  http://sites.google.com/site/jpmzometa/
  and Samuel Marco: sammarcoarmengol@gmail.com and Andras Tucsni.
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
#pragma once

#define TO_UINT_16_B(uint8_H, uint8_L) ((uint8_t(uint8_H) << 8) + uint8_t(uint8_L))

class Modbus {

public:
	/** Paridade utilizado */
	enum ModbusParity : uint8_t {
		MB_PARITY_E = 'e',
		MB_PARITY_O = 'o',
		MB_PARITY_N = 'n'
	};

private:
	/** Células dentro da matriz de consulta|resposta */
	enum ModbusPosVec : uint8_t {
		POS_VEC_SLAVE = 0,
		POS_VEC_FUNC,
		POS_VEC_START_H,
		POS_VEC_START_L,
		POS_VEC_REGS_H,
		POS_VEC_REGS_L,
		POS_VEC_BYTE_CNT
	};

	/**
	* Funcoes Modbus suportadas.
	*/
	enum FuncModbus {
		FC_READ_REGS = 0x03,
		FC_WRITE_REG = 0x06,
		FC_WRITE_REGS = 0x10
	};
	const FuncModbus funcSupported[3] = {FC_READ_REGS, FC_WRITE_REG, FC_WRITE_REGS};

	enum {
		MAX_READ_REGS = 0x7D,
		MAX_WRITE_REGS = 0x7B,
		MAX_MESSAGE_LENGTH = 256
	};

	enum :uint8_t{
		RESPONSE_SIZE = 6,
		EXCEPTION_SIZE = 3,
		CHECKSUM_SIZE = 2,
	};

	enum ModbusResponse : uint8_t {
		/** Código de exceptions */
		MB_RES_NO_REPLY = -1,
		MB_RES_OK = 0,
		MB_RES_EXC_FUNC_CODE = 1,
		MB_RES_EXC_ADDR_RANGE = 2,
		MB_RES_EXC_REGS_QUANT = 3,
		MB_RES_EXC_EXECUT = 4
	};

	uint8_t mbSlaveId;
	uint8_t parity;
	uint32_t commBaudRate;
	/** Definir o pino usado para colocar o driver RS485 em modo de transmiss�o, utilizado somente em redes RS485 quando colocar em 0 ou 1 para redes RS232 */
	unsigned int txPin;
	unsigned int lastBytesReceived;
	unsigned long Nowdt = 0;
	const unsigned long T35 = 5;

	/** Vetor de registradores 16bit */
	uint16_t* holdingRegisters;
	int holdingRegistersSize;


public:
	/**
	* \brief
	* \param txPin
	* \param COMM_BPS
	* \param PARITY
	* \param mbSlaveId
	* \param holdingRegisters
	* \param holdingRegistersSize
	*/
	Modbus(uint8_t txPin, uint32_t COMM_BPS, ModbusParity PARITY, uint8_t mbSlaveId, uint16_t* holdingRegisters,
	       uint16_t holdingRegistersSize);
	Modbus(uint32_t COMM_BPS, ModbusParity PARITY, uint8_t mbSlaveId, uint16_t* holdingRegisters,
	       uint16_t holdingRegistersSize);

	/**
	 * \brief Verifica se há algum pedido válido do mestre. Executa as devidas ações.
	 * \return -1 No reply; 0 nenhum pedido do mestre; 1-4 para exceptions; (>4) numero de bytes enviados como resposta se ok
	 */
	int update_mb_slave();
	/**
	*
	* configuracao dos parametros da porta serial.
	*
	* baud: taxa de transmiss�o em bps (valores t�picos entre 9600, 19200... 115200)
	* parity: seta o modo de paridade:
	* 'n' sem paridade (8N1); 'e' paridede impar (8E1), 'o' paridade par (8O1).
	* tx_en_pin: pino do arduino que controla a transmiss�o/recep��o em uma linha RS485.
	* 0 or 1 desliga esta fun��o (para rede RS232)
	* >2 para uma rede multiponto.
	*/
	void configure_mb_slave();
private:
	/**
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

	*/
	unsigned int crc(uint8_t* buf, uint8_t start, uint8_t cnt) const;

	/***********************************************************************
	*
	* As seguintes fun��es constroem o frame de
	* um pacote de consulta modbus.
	*
	***********************************************************************/

	/**
	* In�cio do pacote de uma resposta read_holding_register
	*/
	void buildReadPacket(uint8_t function, uint8_t count, uint8_t* packet);

	/**
	* In�cio do pacote de uma resposta preset_multiple_register
	*/
	void buildWritePacket(uint8_t function, uint16_t start_addr, uint8_t count, uint8_t* packet);

	/**
	* In�cio do pacote de uma resposta writeSingleRegister
	*/
	void buildWriteSinglePacket(uint8_t function, uint16_t write_addr, uint16_t reg_val, uint8_t* packet);

	/**
	* In�cio do pacote de uma resposta excep��o
	*/
	void buildErrorPacket(uint8_t function, uint8_t exception, uint8_t* packet);

	/*************************************************************************
	*
	* modbus_query( packet, length)
	*
	* Fun��o para adicionar uma soma de verifica��o para o fim de um pacote.
	* Por favor, note que a matriz pacote deve ser de pelo menos 2 campos mais do que
	* String_length.
	**************************************************************************/
	void modbusReply(uint8_t* packet, uint8_t string_length);

	/***********************************************************************
	*
	* sendReply( query_string, query_length )
	*
	* Fun��o para enviar uma resposta a um mestre Modbus.
	* Retorna: o n�mero total de caracteres enviados
	************************************************************************/
	int sendReply(uint8_t* query, uint8_t string_length);

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
	int receiveRequest(uint8_t* received_string);

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
	int modbusRequest(uint8_t* data);

	/*********************************************************************
	*
	* validateRequest(request_data_array, request_length, available_regs)
	*
	* Fun��o para verificar se o pedido pode ser processado pelo escravo.
	*
	* Retorna: 0 se MB_RES_OK
	* Um c�digo de exce��o negativa em caso de erro
	*
	**********************************************************************/
	ModbusResponse validateRequest(uint8_t* data, uint8_t length, uint16_t regs_size);

	/************************************************************************
	*
	* writeRegs(first_register, data_array, registers_array)
	*
	* escreve nos registradores do escravo os dados em consulta,
	* A partir de start_addr.
	*
	* Retorna: o n�mero de registros escritos
	************************************************************************/
	int writeRegs(uint16_t start_addr, uint8_t* query, uint16_t* regs);

	/************************************************************************
	*
	* presetMultipleRegisters(slave_id, first_register, number_of_registers,
	* data_array, registers_array)
	*
	* Escreva os dados na matriz dos registos do escravo.
	*
	*************************************************************************/
	int presetMultipleRegisters(uint16_t start_addr, uint16_t count, uint8_t* query, uint16_t* regs);
	/************************************************************************
	*
	* writeSingleRegister(slave_id, write_addr, data_array, registers_array)
	*
	* Escrever um �nico valor inteiro em um �nico registo do escravo.
	*
	*************************************************************************/
	int writeSingleRegister(uint16_t write_addr, uint8_t* query, uint16_t* regs);

	/************************************************************************
	*
	* readHoldingRegisters(slave_id, first_register, number_of_registers,
	* registers_array)
	*
	* l� os registros do escravo e envia para o mestre Modbus
	*
	*************************************************************************/
	int readHoldingRegisters(uint16_t start_addr, uint16_t reg_count, uint16_t* regs);
};
