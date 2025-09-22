#include "em4100.h"

const int SHD = 17;
const int MOD = 16;
const int DEMODOUT = 18;
const int READYCLK = 15;

Em4095 rfid(SHD, MOD, DEMODOUT, READYCLK);


void setup()
{
    Serial.begin(115200);
    Serial.printf("starting...");
    rfid.Init();
    rfid.Enable(); 
    delay(200);
    double freq = rfid.calcResonantFreq();
    Serial.printf("\nRDYCLK pulses per ms ~ %.2f\n", freq);
}

void loop()

{
    uint64_t id40 = 0;
    if (rfid.ReadEM4100(&id40)) {
       // In dạng 10 nibble hex (40-bit)
        Serial.print("EM4100 ID (40-bit): 0x");
        for (int i = 9; i >= 0; --i) {
          uint8_t nib = (id40 >> (i*4)) & 0xF;
          Serial.print(nib, HEX);
          }
        Serial.println();
    } else {
        Serial.println("Không tìm thấy thẻ...");
    }
    delay(200);
}
