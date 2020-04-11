#ifndef FutabaSBUS_h
	#define FutabaSBUS_h
	#include <Arduino.h>
	#define SBUS_SIGNAL_OK          0x00
	#define SBUS_SIGNAL_LOST        0x01
	#define SBUS_SIGNAL_FAILSAFE    0x03
	#define BAUDRATE 100000
	#ifndef USE_SOFTWARE_SERIAL
	#define SerialX HardwareSerial
	#else
	#define SerialX SoftwareSerial
	#endif
	class FutabaSBUS
	{
		public:
			uint8_t sbusData[25];
			int16_t channels[18];
			int16_t servos[18];
			uint8_t failSafeStatus;
			int passThrough;
			int toChannels;
			void begin(void);
			int16_t getChannelValue(uint8_t ch);
			uint8_t getDigitalChannelValue(uint8_t ch);
			void setChannelValue(uint8_t ch, int16_t position);
			void setDigitalChannelValue(uint8_t ch, uint8_t position);
			uint8_t getFailSafe(void);
			void setPassThroughMode(int mode);
			int getPassThroughMode(void);
			void UpdateServos(void);
			void UpdateChannels(void);
			void FeedLine(void);
			void buildPacketAndSend(void);
			FutabaSBUS(SerialX &serial_) : serial(serial_){};
			int channelLen;
		private:
			SerialX &serial;
			uint8_t byte_in_sbus;
			uint8_t bit_in_sbus;
			uint8_t ch;
			uint8_t bit_in_channel;
			uint8_t bit_in_servo;
			uint8_t inBuffer[25];
			int bufferIndex;
			uint8_t inData;
			int feedState;

	};

#endif