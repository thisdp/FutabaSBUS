#include "FutabaSBUS.h"

void FutabaSBUS::begin(){
	uint8_t loc_sbusData[25] = {0x0f,0x01,0x04,0x20,0x00,0xff,0x07,0x40,0x00,0x02,0x10,0x80,0x2c,0x64,0x21,0x0b,0x59,0x08,0x40,0x00,0x02,0x10,0x80,0x00,0x00};
	int16_t loc_channels[18] = {1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,0,0};
	int16_t loc_servos[18] = {1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,0,0};
	serial.begin(BAUDRATE,SERIAL_8E2);
	memcpy(sbusData,loc_sbusData,25);
	memcpy(channels,loc_channels,18);
	memcpy(servos,loc_servos,18);
	failSafeStatus = SBUS_SIGNAL_OK;
	passThrough = 1;
	toChannels = 0;
	bufferIndex=0;
	feedState = 0;
}

int16_t FutabaSBUS::getChannelValue(uint8_t ch) {
	// Read channel data
	if ((ch>0)&&(ch<=16)){
		return channels[ch-1];
	}
	else{
		return 1023;
	}
}

uint8_t FutabaSBUS::getDigitalChannelValue(uint8_t ch) {
	// Read digital channel data
	if ((ch>0) && (ch<=2)){
		return channels[15+ch];
	}
	else{
		return 0;
	}
}

void FutabaSBUS::setChannelValue(uint8_t ch, int16_t position) {
	// Set servo position
	if ((ch>0)&&(ch<=16)) {
		if (position>2048) {
			position=2048;
		}
		servos[ch-1] = position;
	}
}
void FutabaSBUS::setDigitalChannelValue(uint8_t ch, uint8_t position) {
	// Set digital servo position
	if ((ch>0) && (ch<=2)) {
		if (position>1) {
			position=1;
		}
		servos[15+ch] = position;
	}
}
uint8_t FutabaSBUS::getFailSafe(void) {
	return failSafeStatus;
}

void FutabaSBUS::setPassThroughMode(int mode) {
	// Set passtrough mode, if true, received channel data is send to servos
	passThrough = mode;
}

int FutabaSBUS::getPassThroughMode(void) {
	// Return current passthrough mode
	return passThrough;
}

void FutabaSBUS::UpdateServos(void) {
	// Send data to servos
	// Passtrough mode = false >> send own servo data
	// Passtrough mode = true >> send received channel data
	uint8_t i;
	if (passThrough==0) {
		// clear received channel data
		for (i=1; i<24; i++) {
			sbusData[i] = 0;
		}
		// reset counters
		ch = 0;
		bit_in_servo = 0;
		byte_in_sbus = 1;
		bit_in_sbus = 0;
		// store servo data
		for (i=0; i<176; i++) {
			if (servos[ch] & (1<<bit_in_servo)) {
				sbusData[byte_in_sbus] |= (1<<bit_in_sbus);
			}
			bit_in_sbus++;
			bit_in_servo++;

			if (bit_in_sbus == 8) {
				bit_in_sbus =0;
				byte_in_sbus++;
			}
			if (bit_in_servo == 11) {
				bit_in_servo =0;
				ch++;
			}
		}
		// DigiChannel 1
		if (channels[16] == 1) {
			sbusData[23] |= (1<<0);
		}
		// DigiChannel 2
		if (channels[17] == 1) {
			sbusData[23] |= (1<<1);
		}
		// Failsafe
		if (failSafeStatus == SBUS_SIGNAL_LOST) {
			sbusData[23] |= (1<<2);
		}
		if (failSafeStatus == SBUS_SIGNAL_FAILSAFE) {
			sbusData[23] |= (1<<2);
			sbusData[23] |= (1<<3);
		}
	}
	// send data out
	//serialPort.write(sbusData,25);
	for (i=0;i<25;i++) {
		serial.write(sbusData[i]);
	}
}

void FutabaSBUS::UpdateChannels(void) {
	channels[0]	= ((sbusData[1]|sbusData[2]<< 8) & 0x07FF);
	channels[1]	= ((sbusData[2]>>3|sbusData[3]<<5) & 0x07FF);
	channels[2]	= ((sbusData[3]>>6|sbusData[4]<<2|sbusData[5]<<10) & 0x07FF);
	channels[3]	= ((sbusData[5]>>1|sbusData[6]<<7) & 0x07FF);
	channels[4]	= ((sbusData[6]>>4|sbusData[7]<<4) & 0x07FF);
	channels[5]	= ((sbusData[7]>>7|sbusData[8]<<1|sbusData[9]<<9) & 0x07FF);
	channels[6]	= ((sbusData[9]>>2|sbusData[10]<<6) & 0x07FF);
	channels[7]	= ((sbusData[10]>>5|sbusData[11]<<3) & 0x07FF);
	channels[8]	= ((sbusData[12]|sbusData[13]<< 8) & 0x07FF);
	channels[9]	= ((sbusData[13]>>3|sbusData[14]<<5) & 0x07FF);
	channels[10] = ((sbusData[14]>>6|sbusData[15]<<2|sbusData[16]<<10) & 0x07FF);
	channels[11] = ((sbusData[16]>>1|sbusData[17]<<7) & 0x07FF);
	channels[12] = ((sbusData[17]>>4|sbusData[18]<<4) & 0x07FF);
	channels[13] = ((sbusData[18]>>7|sbusData[19]<<1|sbusData[20]<<9) & 0x07FF);
	channels[14] = ((sbusData[20]>>2|sbusData[21]<<6) & 0x07FF);
	channels[15] = ((sbusData[21]>>5|sbusData[22]<<3) & 0x07FF);
	channels[16] = (sbusData[23] & 0x01);
	channels[17] = (sbusData[23]>>1 & 0x01);
	failSafeStatus = SBUS_SIGNAL_OK;
	if (sbusData[23] & 0x04) {
		failSafeStatus = SBUS_SIGNAL_LOST;
	}
	if (sbusData[23] & 0x08) {
		failSafeStatus = SBUS_SIGNAL_FAILSAFE;
	}
}

void FutabaSBUS::buildPacketAndSend()
{
	byte sbusData[25];
	// SBUS header
	sbusData[0] = 0x0F; 
	// 16 channels of 11 bit data
	sbusData[1]  = (unsigned char) ((channels[0] & 0x07FF));
	sbusData[2]  = (unsigned char) ((channels[0] & 0x07FF)>>8   | (channels[1] & 0x07FF)<<3);
	sbusData[3]  = (unsigned char) ((channels[1] & 0x07FF)>>5   | (channels[2] & 0x07FF)<<6);
	sbusData[4]  = (unsigned char) ((channels[2] & 0x07FF)>>2);
	sbusData[5]  = (unsigned char) ((channels[2] & 0x07FF)>>10  | (channels[3] & 0x07FF)<<1);
	sbusData[6]  = (unsigned char) ((channels[3] & 0x07FF)>>7   | (channels[4] & 0x07FF)<<4);
	sbusData[7]  = (unsigned char) ((channels[4] & 0x07FF)>>4   | (channels[5] & 0x07FF)<<7);
	sbusData[8]  = (unsigned char) ((channels[5] & 0x07FF)>>1);
	sbusData[9]  = (unsigned char) ((channels[5] & 0x07FF)>>9   | (channels[6] & 0x07FF)<<2);
	sbusData[10] = (unsigned char) ((channels[6] & 0x07FF)>>6   | (channels[7] & 0x07FF)<<5);
	sbusData[11] = (unsigned char) ((channels[7] & 0x07FF)>>3);
	sbusData[12] = (unsigned char) ((channels[8] & 0x07FF));
	sbusData[13] = (unsigned char) ((channels[8] & 0x07FF)>>8   | (channels[9] & 0x07FF)<<3);
	sbusData[14] = (unsigned char) ((channels[9] & 0x07FF)>>5   | (channels[10] & 0x07FF)<<6);
	sbusData[15] = (unsigned char) ((channels[10] & 0x07FF)>>2);
	sbusData[16] = (unsigned char) ((channels[10] & 0x07FF)>>10 | (channels[11] & 0x07FF)<<1);
	sbusData[17] = (unsigned char) ((channels[11] & 0x07FF)>>7  | (channels[12] & 0x07FF)<<4);
	sbusData[18] = (unsigned char) ((channels[12] & 0x07FF)>>4  | (channels[13] & 0x07FF)<<7);
	sbusData[19] = (unsigned char) ((channels[13] & 0x07FF)>>1);
	sbusData[20] = (unsigned char) ((channels[13] & 0x07FF)>>9  | (channels[14] & 0x07FF)<<2);
	sbusData[21] = (unsigned char) ((channels[14] & 0x07FF)>>6  | (channels[15] & 0x07FF)<<5);
	sbusData[22] = (unsigned char) ((channels[15] & 0x07FF)>>3);
	// Digital
	sbusData[23] = channels[16]; 
	sbusData[23] |= channels[17]<<1; 
    // flags
	if(failSafeStatus == SBUS_SIGNAL_LOST)
		sbusData[23] |= 0x04;
	if(failSafeStatus == SBUS_SIGNAL_FAILSAFE)
		sbusData[23] |= 0x08;
	// footer
	sbusData[24] = 0x00;
	Serial.write(sbusData, 25);
}

void FutabaSBUS::FeedLine(void){
	channelLen = serial.available();
	if (serial.available() > 24){
		while(serial.available() > 0){
			inData = serial.read();
			switch (feedState){
			case 0:
				if (inData != 0x0f){
					while(serial.available() > 0){
						inData = serial.read();
					}
					return;
				}
				else{
					bufferIndex = 0;
					inBuffer[bufferIndex] = inData;
					inBuffer[24] = 0xff;
					feedState = 1;
				}
				break;
			case 1:
				bufferIndex ++;
				inBuffer[bufferIndex] = inData;
				if (bufferIndex < 24 && serial.available() == 0){
					feedState = 0;
				}
				if (bufferIndex == 24){
					feedState = 0;
					if (inBuffer[0]==0x0f && inBuffer[24] == 0x00){
						memcpy(sbusData,inBuffer,25);
						toChannels = channelLen;
					}
				}
				break;
			}
		}
	}
}