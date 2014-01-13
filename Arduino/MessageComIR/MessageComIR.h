/*
	MessageComIR.h
	
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

#ifndef MessageComIR_h
#define MessageComIR_h


#include <Arduino.h>
#include <IRremote.h>
#include <util/crc16.h>

class MessageComIR {
	private:
		IRsend *_irsend;
		IRrecv *_irrecv;
		decode_results *_results;
	public:
		MessageComIR(IRsend *irsend, IRrecv *irrecv, decode_results *results);
		
		uint16_t addStateToAck(uint16_t, boolean);
		boolean getStateFromAckValue(uint16_t);
		boolean checkAck(uint16_t, uint16_t);
		uint16_t calcCrc(long);

		// TX
		void snd(uint16_t, uint8_t);
		void snd(long, uint8_t);

		void sendAck(uint16_t, boolean);

		boolean sendTask(uint16_t);
		boolean sendQuantity(uint16_t, uint16_t);
		boolean sendTerm(uint16_t);
		boolean sendDataAndChecksum(long, uint16_t);

		boolean send(uint16_t, uint16_t, long*);

		// RX
		int receiveAck(uint16_t);

		uint16_t receiveTask();
		uint16_t receiveQuantity(uint16_t);
		boolean receiveTerm(uint16_t);
		boolean receiveChecksum(uint16_t, uint16_t);
		boolean receiveDataAndChecksum(long*, uint16_t);

		boolean recv();
		
		boolean receive(uint16_t&, uint16_t&, long*);
};

#endif
