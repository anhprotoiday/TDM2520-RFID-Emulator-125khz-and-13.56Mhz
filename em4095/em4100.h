//Most of this code is derived from
//https://github.com/Proxmark/proxmark3
#ifndef RFID_H
#define RFID_H
#include <Arduino.h>
#define DELAYVAL 384  // 384 //standard delay for manchster decode
#define TIMEOUT 10000 // standard timeout for manchester decode at  160mhz is 1000000
#define MAX_DEMOD_BUF_LEN 1024
struct RfidResult {             // Structure declaration
  int data;         // Member (int variable)
  bool error;   // Member (string variable)
} ;
class Em4095
{
public:
    void Enable();
    void Disable();
    double calcResonantFreq();
    Em4095(byte shd, byte mod, byte demodOut, byte rdyClk);  // <-- cần có
    void Init();
    bool ReadEM4100(uint64_t *cardID);
    bool ReadEM4100Stable(uint64_t *cardID, uint8_t minRepeats=2, uint8_t maxTries=5); //EM4100 read with stability check

private:
    // pin configuration

    int demodOut;
    int shd ;
    int mod;
    int rdyClk;

    int DemodBufferLen = 0;
    unsigned char DemodBuffer[MAX_DEMOD_BUF_LEN];
    bool preambleSearchEx(uint8_t *BitStream, uint8_t *preamble, size_t pLen, size_t *size, size_t *startIdx, bool findone) ;
    void RecordEM4100Raw(uint16_t halfBits); // record raw data from antenna EM4100 mode  
    void setDemodBuf(uint8_t *buff, size_t size, size_t startIdx);
    void RecordFromAntenna(uint32_t numberOfBits);
    void turn_read_lf_off(uint32_t microseconds);
    void turn_read_lf_on(uint32_t microseconds);


};

#endif
