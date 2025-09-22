    // code is adapted from https://github.com/Proxmark/proxmark3
    #include "em4100.h"
    const char *RF_TAG = "RFID";
    volatile unsigned int clkCount = 0;
    static void IRAM_ATTR onClk() { clkCount++; }  
    volatile bool lastEdgeWasFalling = false;

    // some defines to make copy pasted code from proxmark function
    bool g_debugMode = false;
    #define prnt(s) ESP_LOGI(RF_TAG, s)
    #define prnt(s, v) ESP_LOGI(RF_TAG, s, v)
    #define PrintAndLog2(f, v) ESP_LOGI(RF_TAG, f, v)
    #define PrintAndLog(f) ESP_LOGI(RF_TAG, f)
    #define WaitUS delayMicroseconds

    double Em4095::calcResonantFreq()
    {
    unsigned int countStart = clkCount;
    delay(100);
    unsigned int elapsedCount= clkCount- countStart;
    return elapsedCount/100.0;
    }

    void Em4095::Enable()
    {
        digitalWrite(shd, LOW);
    }
    void Em4095::Disable()
    {
        digitalWrite(shd, HIGH);
    }
    //====================================================================
    // prepares command bits
    // see EM4469 spec
    //====================================================================
    //--------------------------------------------------------------------
    //  VALUES TAKEN FROM EM4x function: SendForward
    //  START_GAP = 440;       (55*8) cycles at 125kHz (8us = 1cycle)
    //  WRITE_GAP = 128;       (16*8)
    //  WRITE_1   = 256 32*8;  (32*8)

    //  These timings work for 4469/4269/4305 (with the 55*8 above)

    // search for given preamble in given BitStream and return success=1 or fail=0 and startIndex (where it was found) and length if not fineone
    // fineone does not look for a repeating preamble for em4x05/4x69 sends preamble once, so look for it once in the first pLen bits
    bool Em4095::preambleSearchEx(uint8_t *BitStream, uint8_t *preamble, size_t pLen, size_t *size, size_t *startIdx, bool findone)
    {
        // Sanity check.  If preamble length is bigger than bitstream length.
        if (*size <= pLen)
            return false;

        uint8_t foundCnt = 0;
        for (size_t idx = 0; idx < *size - pLen; idx++)
        {
            if (memcmp(BitStream + idx, preamble, pLen) == 0)
            {
                // first index found
                foundCnt++;
                if (foundCnt == 1)
                {
                    if (g_debugMode)
                        prnt("DEBUG: preamble found at %u", idx);
                    *startIdx = idx;
                    if (findone)
                        return true;
                }
                else if (foundCnt == 2)
                {
                    *size = idx - *startIdx;
                    return true;
                }
            }
        }
        return false;
    }
    static inline bool oddparity32(uint32_t x)
    {
    #if !defined __GNUC__
        x ^= x >> 16;
        x ^= x >> 8;
        return oddparity8(x);
    #else
        return !__builtin_parity(x);
    #endif
    }
    // by marshmellow
    // pass bits to be tested in bits, length bits passed in bitLen, and parity type (even=0 | odd=1) in pType
    // returns 1 if passed
    bool parityTest(uint32_t bits, uint8_t bitLen, uint8_t pType)
    {
        return oddparity32(bits) ^ pType;
    }
    // by marshmellow
    // takes a array of binary values, start position, length of bits per parity (includes parity bit - MAX 32),
    //   Parity Type (1 for odd; 0 for even; 2 for Always 1's; 3 for Always 0's), and binary Length (length to run)
    size_t removeParity(uint8_t *BitStream, size_t startIdx, uint8_t pLen, uint8_t pType, size_t bLen)
    {
        uint32_t parityWd = 0;
        size_t bitCnt = 0;
        for (int word = 0; word < (bLen); word += pLen)
        {
            for (int bit = 0; bit < pLen; bit++)
            {
                if (word + bit >= bLen)
                    break;
                parityWd = (parityWd << 1) | BitStream[startIdx + word + bit];
                BitStream[bitCnt++] = (BitStream[startIdx + word + bit]);
            }
            if (word + pLen > bLen)
                break;

            bitCnt--; // overwrite parity with next data
            // if parity fails then return 0
            switch (pType)
            {
            case 3:
                if (BitStream[bitCnt] == 1)
                {
                    return 0;
                }
                break; // should be 0 spacer bit
            case 2:
                if (BitStream[bitCnt] == 0)
                {
                    return 0;
                }
                break; // should be 1 spacer bit
            default:
                if (parityTest(parityWd, pLen, pType) == 0)
                {
                    return 0;
                }
                break; // test parity
            }
            parityWd = 0;
        }
        // if we got here then all the parities passed
        // return size
        return bitCnt;
    }
    bool EM_EndParityTest(uint8_t *BitStream, size_t size, uint8_t rows, uint8_t cols, uint8_t pType)
    {
        if (rows * cols > size)
            return false;
        uint8_t colP = 0;
        // assume last col is a parity and do not test
        for (uint8_t colNum = 0; colNum < cols - 1; colNum++)
        {
            for (uint8_t rowNum = 0; rowNum < rows; rowNum++)
            {
                colP ^= BitStream[(rowNum * cols) + colNum];
            //Serial.print(BitStream[(rowNum * cols) + colNum]);
            }
            //Serial.println();
            if (colP != pType)
                return false;
        }
        return true;
    }



    //====================================================================

    // set the demod buffer with given array of binary (one bit per byte)
    // by marshmellow
    void Em4095::setDemodBuf(uint8_t *buff, size_t size, size_t startIdx)
    {
        if (buff == NULL)
            return;

        if (size > MAX_DEMOD_BUF_LEN - startIdx)
            size = MAX_DEMOD_BUF_LEN - startIdx;

        size_t i = 0;
        for (; i < size; i++)
        {
            DemodBuffer[i] = buff[startIdx++];
        }
        DemodBufferLen = size;
        return;
    }
    // least significant bit first
    uint32_t bytebits_to_byteLSBF(uint8_t *src, size_t numbits)
    {
        uint32_t num = 0;
        for (int i = 0; i < numbits; i++)
        {
            num = (num << 1) | *(src + (numbits - (i + 1)));
        }
        return num;
    }
    void Em4095::Init()
    {
        pinMode(shd, OUTPUT);
        pinMode(mod, OUTPUT);
        pinMode(demodOut, INPUT);
        pinMode(rdyClk, INPUT);
        digitalWrite(mod, LOW);
        // todo only attach on read command dont want to affect timming while sending commands
        attachInterrupt(rdyClk, onClk, RISING);
    }
    Em4095::Em4095(byte shd,byte mod, byte demodOut,byte rdyClk){
        this->shd=shd;
        this->mod =mod;
        this->demodOut=demodOut;
        this->rdyClk= rdyClk;
    }

    void Em4095::RecordFromAntenna(uint32_t numberOfBits)
    {
        if (numberOfBits > MAX_DEMOD_BUF_LEN)
        {
            return;
        }
        // preturn this is wrong
        while (0 == digitalRead(demodOut))
        { // sync on falling edge
        }
        while (1 == digitalRead(demodOut))
        { // sync on falling edge
        }
        clkCount = 0;
        while (clkCount < 5)
            ; //read offset a few lcocks to avoid the edge
        for (int i = 0; i < numberOfBits; i++)
        {
            DemodBuffer[i] = digitalRead(demodOut);
            clkCount = 0;
            while (clkCount < 64)
                ;
        }
        DemodBufferLen = numberOfBits;
        //print raw buffer
        // String data = "";

        // for (int i = 0; i < numberOfBits; i++)
        // {
        //     data += DemodBuffer[i];
        // }
        // ESP_LOGI(RF_TAG, "Read Buffer: %s", data.c_str());
    }

    void Em4095::turn_read_lf_off(uint32_t microSeconds) {
        digitalWrite(mod, HIGH);            
        delayMicroseconds(microSeconds);
    }

    void Em4095::turn_read_lf_on(uint32_t microSeconds) {
        digitalWrite(mod, LOW);            
        delayMicroseconds(microSeconds);
    }


    // --- EM4100 support (Manchester @ fc/64) ---
    // Samples DEMOD_OUT at half-bit resolution (32 RDY/CLK cycles per half-bit)
    void Em4095::RecordEM4100Raw(uint16_t halfBits) {
        if (halfBits > MAX_DEMOD_BUF_LEN) halfBits = MAX_DEMOD_BUF_LEN;

        // Sync to một cạnh sạch trên DEMOD_OUT, có timeout để không bao giờ treo
        uint32_t t0 = micros();
        while (digitalRead(demodOut) == 0) {
            if (micros() - t0 > 2000) { DemodBufferLen = 0; return; }  // ~2ms
        }
        while (digitalRead(demodOut) == 1) {
            if (micros() - t0 > 4000) { DemodBufferLen = 0; return; }  // ~4ms tổng
        }

        clkCount = 0;
        while (clkCount < 5) ; // offset 

        for (uint16_t i = 0; i < halfBits; i++) {
            uint8_t sum = 0;
            clkCount = 0;

            // Lấy 3 mẫu đều nhau trong 32 xung RDY/CLK của một half-bit
            while (clkCount < 8)  ;
            sum += digitalRead(demodOut);
            while (clkCount < 16) ;
            sum += digitalRead(demodOut);
            while (clkCount < 24) ;
            sum += digitalRead(demodOut);

            while (clkCount < 32) ; // hoàn tất half-bit

            // Majority vote
            DemodBuffer[i] = (sum >= 2) ? 1 : 0;
        }
        DemodBufferLen = halfBits;
    }

    // Decodes EM4100 and returns 40-bit ID in *cardID (lower 40 bits)
    bool Em4095::ReadEM4100(uint64_t *cardID) {
        if (!cardID) return false;

        // 1) Capture ~320 half-bits (~160 bits) to safely contain a 64-bit frame
        RecordEM4100Raw(384);

        auto decode_with_mapping = [&](bool one_is_01) -> bool {
            // Convert half-bits -> bits
            static uint8_t bits[600];
            int nb = 0;
            for (int i = 0; i + 1 < DemodBufferLen; i += 2) {
                uint8_t a = DemodBuffer[i];
                uint8_t b = DemodBuffer[i + 1];
                if (one_is_01) {
                    // 01 -> 1, 10 -> 0
                    if (a == 0 && b == 1) bits[nb++] = 1;
                    else if (a == 1 && b == 0) bits[nb++] = 0;
                    else return false; // invalid Manchester pair
                } else {
                    // 01 -> 0, 10 -> 1 (inverted polarity)
                    if (a == 0 && b == 1) bits[nb++] = 0;
                    else if (a == 1 && b == 0) bits[nb++] = 1;
                    else return false;
                }
                if (nb >= 512) break;
            }
            if (nb < 64) return false;

            // 2) Find preamble of 9 consecutive '1' bits
            int start = -1;
            for (int i = 0; i + 64 <= nb; i++) {
                bool ok = true;
                for (int j = 0; j < 9; j++) {
                    if (bits[i + j] != 1) { ok = false; break; }
                }
                if (ok) { start = i; break; }
            }
            if (start < 0) return false;

            // 3) Validate frame parity: 10 groups of 4 data + 1 row parity (even),
            //    then 4 column parity bits (even), then stop bit 0
            if (bits[start + 63] != 0) return false;               // stop

            for (int g = 0; g < 10; g++) {                         // row parity
                int base = start + 9 + g * 5;
                uint8_t sum = 0;
                for (int j = 0; j < 4; j++) sum ^= bits[base + j];
                if ((sum & 1) != bits[base + 4]) return false;
            }
            for (int col = 0; col < 4; col++) {                    // column parity
                uint8_t sum = 0;
                for (int g = 0; g < 10; g++) sum ^= bits[start + 9 + g * 5 + col];
                if ((sum & 1) != bits[start + 59 + col]) return false;
            }

            // 4) Extract 10 nibbles = 40 bits
            uint64_t id40 = 0;
            for (int g = 0; g < 10; g++) {
                int base = start + 9 + g * 5;
                uint8_t nib = 0;
                for (int j = 0; j < 4; j++) nib = (nib << 1) | bits[base + j];
                id40 = (id40 << 4) | (uint64_t)nib;
            }
            *cardID = id40; // lower 40 bits valid
            return true;
        };

        // Try both Manchester polarities
        if (decode_with_mapping(true)) return true;
        if (decode_with_mapping(false)) return true;

        return false;
    }

    bool Em4095::ReadEM4100Stable(uint64_t *cardID, uint8_t minRepeats, uint8_t maxTries) {
        if (!cardID || minRepeats == 0 || maxTries == 0) return false;

        uint64_t last = 0;
        uint8_t sameCount = 0;

        for (uint8_t i = 0; i < maxTries; i++) {
            uint64_t cur = 0;
            if (ReadEM4100(&cur)) {
                if (sameCount == 0) {
                    last = cur;
                    sameCount = 1;
                } else if (cur == last) {
                    sameCount++;
                    if (sameCount >= minRepeats) {
                        *cardID = last;
                        return true;
                    }
                } else {
                    last = cur;
                    sameCount = 1;
                }
            }
            // nghỉ ngắn để tránh lock pha khi lấy liên tiếp
            delay(10);
        }
        return false;
    }
