/*
	MessageComIR.cpp
	
	MessageComIR is a library to facilitate communication between devices via infrared.
	MessageComIR uses the Arduino - IRremote library by Ken Shirriff.
	Links:
		https://github.com/shirriff/Arduino-IRremote
    http://arcfn.com/2009/08/multi-protocol-infrared-remote-library.html
  
	@version 0.5
	@date 14 Nov 2013

	@author master[at]link-igor[dot]de Igor Milutinovic

	@link https://github.com/sigger/MessageComIR

	Copyright 2013 Igor Milutinovic. All rights reserved.

	This library is free software; you can redistribute it and/or
	modify it under the terms of the GNU Lesser General Public
	License as published by the Free Software Foundation; either
	version 2.1 of the License, or (at your option) any later version.

	This library is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
	Lesser General Public License for more details.

	You should have received a copy of the GNU Lesser General Public
	License along with this library; if not, write to the Free Software
	Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <MessageComIR.h>

// a different number of bits for each message-types
#define IRTASK 12
#define IRACK 13
#define IRQUANTITY 14
#define IRTERM 15
#define IRCHECKSUM 16
#define IRDATA 32

#define IRMAXTRY 5
#define IRDELAY 50
#define IRMAXQUANTITY 64

#define IRTERMVAL 0xe2d
#define IRBADTASK 0xbad

MessageComIR::MessageComIR(IRsend *irsend, IRrecv *irrecv, decode_results *results) {
	_irsend = irsend;
	_irrecv = irrecv;
	_results = results;
}

// ACK combined with success and failed state via MSB
uint16_t MessageComIR::addStateToAck(uint16_t taskValue, boolean state) {
	// add status bit from "state" to the taskValue at MSB - true or false
	bitWrite(taskValue, 12, bitRead(state, 0));
	delay(2);

	return taskValue;
}
boolean MessageComIR::getStateFromAckValue(uint16_t taskValue) {
	// get status bit from "taskValue" at MSB - true or false
	return bitRead(taskValue, 12);
}
boolean MessageComIR::checkAck(uint16_t taskValue, uint16_t rawTask) {
	if(getStateFromAckValue(taskValue)) {
		bitClear(taskValue, 12);
		delay(8);

		if(taskValue == rawTask)
			return 1;
	}
	return 0;
}
uint16_t MessageComIR::calcCrc(long data) {
	uint16_t retval = 0xffff;

	uint8_t n=4;
	uint8_t tmp[4];
	uint8_t cnt=0;
	
	for(uint8_t byteI=0; byteI<4; byteI++)
		for(uint8_t bitI=0; bitI<8; bitI++)
			bitWrite(tmp[byteI], bitI, bitRead(data, cnt++));

	while(n--)
		retval = _crc_ccitt_update(retval, (uint16_t)tmp[n]);

	return retval;
}


// TX
void MessageComIR::snd(uint16_t var, uint8_t type) {
	_irsend->sendSony(var, type);
}
void MessageComIR::snd(long var, uint8_t type) {
	_irsend->sendSony(var, type);
}

void MessageComIR::sendAck(uint16_t task, boolean state) {
	delay(6);

	delay(IRDELAY);
	snd(addStateToAck(task, state), IRACK);
	delay(IRDELAY);
}

// send * and try to receive ACK with boolean result
boolean MessageComIR::sendTask(uint16_t task) {
	delay(1);
	for(uint8_t atry=0; atry<IRMAXTRY; atry++) {
		delay(2);

		// send task
		snd(task, IRTASK);
		delay(IRDELAY);
		
		// receive ack
		int ackState = receiveAck(task);
		if(ackState != -1)
			return ackState;

		delay(IRDELAY);
	}
	return 0;
}
boolean MessageComIR::sendQuantity(uint16_t quantity, uint16_t task) {
	delay(8);

	for(uint8_t atry=0; atry<IRMAXTRY; atry++) {
		delay(6);

		// send quantity
		snd(quantity, IRQUANTITY);
		// delay(IRDELAY);
		
		// receive ack
		int ackState = receiveAck(task);
		if(ackState != -1)
			return ackState;

		delay(IRDELAY);
	}
	return 0;
}
boolean MessageComIR::sendTerm(uint16_t task) {
	delay(7);

	for(uint8_t atry=0; atry<IRMAXTRY; atry++) {
		delay(4);

		// send IRTERMVAL
		snd((uint16_t) IRTERMVAL, IRTERM);
		// delay(IRDELAY);
		
		// receive ack
		int ackState = receiveAck(task);
		if(ackState != -1)
			return ackState;

		delay(IRDELAY);
	}
	return 0;
}

boolean MessageComIR::sendDataAndChecksum(long data, uint16_t task) {
	delay(12);

	for(uint8_t atry=0; atry<IRMAXTRY; atry++) {
		delay(7);
		// send data
		snd(data, IRDATA);
		delay(IRDELAY);

		// send checksum := calcCrc(data)
		delay(IRDELAY);
		snd(calcCrc(data), IRCHECKSUM);
		delay(IRDELAY);
		
		// receive ack
		int ackState = receiveAck(task);
		if(ackState != -1)
			return ackState;

		delay(IRDELAY);
	}
	return 0;
}

boolean MessageComIR::send(uint16_t task, uint16_t quantity, long *data) {
	// send Task
	if(sendTask(task)) {
		// send MAC-address(es)
		// delay(IRDELAY);
		if(sendQuantity(quantity, task)) {
			for(uint16_t i=0; i<quantity; i++) {
				// delay(IRDELAY);
				// send one part of the data
				if(!sendDataAndChecksum(data[i], task))
					return 0;
			}
			// delay(IRDELAY);
			// send terminate
			if(sendTerm(task))
				return 1;
		}
	}
	return 0;
}


// RX
int MessageComIR::receiveAck(uint16_t rawTask) {
	if(recv()) {
		if(_results->bits == IRACK) {
			delay(12);

			// _results->value := (state incl. TASK)
			// rawTask := TASK
			return checkAck(_results->value, rawTask);
		}
	}
	return -1;
}

uint16_t MessageComIR::receiveTask() {
	if(recv()) {
		if(_results->bits == IRTASK) {
			delay(6); 
			sendAck(_results->value, 1);
			// _results->value := TASK
			return _results->value;
		}
	}
	sendAck((uint16_t) IRBADTASK, 0);
	return 0;
}
uint16_t MessageComIR::receiveQuantity(uint16_t task) {
	if(recv()) {
		if(_results->bits == IRQUANTITY) {
			delay(15);

			sendAck(task, 1);
			// _results->value := quantity
			return _results->value;
		}
	}
	sendAck(task, 0);
	return 0;
}
boolean MessageComIR::receiveTerm(uint16_t task) {
	if(recv()) {
		if(_results->bits == IRTERM) {
			delay(12);

			sendAck(task, 1);
			// _results->value := IRTERMVAL
			return (_results->value == IRTERMVAL);
		}
	}
	sendAck(task, 0);
	return 0;
}
boolean MessageComIR::receiveChecksum(uint16_t checksum, uint16_t task) {
	if(recv()) {
		if(_results->bits == IRCHECKSUM) {
			delay(14);

			sendAck(task, 1);
			// _results->value := checksum
			return (_results->value == checksum);
		}
	}
	sendAck(task, 0);
	return 0;
}
boolean MessageComIR::receiveDataAndChecksum(long *recvData, uint16_t task) {
	if(recv()) {
		if(_results->bits == IRDATA) {
			delay(14);

			*recvData = (long) _results->value;
			return receiveChecksum(calcCrc(*recvData), task);
		}
	}
	sendAck(task, 0);
	return 0;
}


boolean MessageComIR::recv() {
	delay(2);
	
	// Start the receiver
	_irrecv->enableIRIn();

	for(uint8_t atry=0; atry<IRMAXTRY; atry++) {
		// Receive the next value
		_irrecv->resume();

		delay(2);

		for(uint16_t i=0; i<1000; i++){
			if(_irrecv->decode(_results)) {
				delay(6);

				return 1;
			}
			delay(1);
		}
		delay(IRDELAY);
	}
	delay(2);
	return 0;
}


boolean MessageComIR::receive(uint16_t &task, uint16_t &quantity, long *data) {
	// receive Task
	task = receiveTask();
	// receive the quantity of data
	if(task > 0) {
		quantity = receiveQuantity(task);
		if(quantity > 0) {
			if(quantity <= IRMAXQUANTITY) {
				for(uint16_t i=0; i<quantity; i++) {
					if(!receiveDataAndChecksum(&data[i], task))
						return 0;
				}
				if(receiveTerm(task))
					return 1;
			}
		}
	}
	return 0;
}
