#include <algorithm>
#include <cstring>
#ifndef ARDUINO
#include <string>
#endif
#include "IRrecv.h"
#include "IRsend.h"
#include "IRtext.h"
#include "IRutils.h"

// Constants
const uint16_t kWtc801HdrMark = 6790;
const uint16_t kWtc801HdrSpace = 3398;
const uint16_t kWtc801OneSpace = 1266;
const uint16_t kWtc801BitMark = 434;
const uint16_t kWtc801ZeroSpace = 444;
const uint16_t kWtc801MessageGap = 32292;

using irutils::addIntToString;
using irutils::addBoolToString;
using irutils::addModeToString;
using irutils::addFanToString;
using irutils::addTempToString;
using irutils::sumBytes;


#if DECODE_WTC801
/// Decode the supplied Wtc801 message.
/// Status: TEST
/// @param[in,out] results Ptr to the data to decode & where to store the decode
/// @param[in] offset The starting index to use when attempting to decode the
///   raw data. Typically/Defaults to kStartOffset.
/// @param[in] nbits The number of data bits to expect.
/// @param[in] strict Flag indicating if we should perform strict matching.
/// @return A boolean. True if it can decode it, false if it can't.
bool IRrecv::decodeWtc801(decode_results *results, uint16_t offset,
                          const uint16_t nbits, const bool strict) {

    uint16_t bits = 2 * (nbits + kHeader);  
    if (results->rawlen < bits) {
        DPRINT("Not enough bits to be Wtc801 - Rawlen: ");
        DPRINT(results->rawlen);
        DPRINT(" Expected: ");
        DPRINTLN(bits);
        return false;  // Too short a message to match.
    }
    
    if (strict && nbits != kWtc801Bits) {
        DPRINTLN("Bits incorrect");
        return false;
    }

    // Pre-Header
    // Sequence begins with a bit mark and a zero space.
    if (!matchMark(results->rawbuf[offset++], kWtc801HdrMark)) return false;
    if (!matchSpace(results->rawbuf[offset++], kWtc801HdrSpace)) return false;

    // Header + Data + Footer
    uint16_t res = matchGeneric(&(results->rawbuf[offset]), results->state,
                                results->rawlen - offset, nbits,
                                0  , 0,
                                kWtc801BitMark   , kWtc801OneSpace,
                                kWtc801BitMark   , kWtc801ZeroSpace ,                     
                                kWtc801BitMark   , kWtc801MessageGap,
                                true, kUseDefTol, kMarkExcess, false);
    if(!res) {
                      DPRINTLN("MatchGeneric Failed");
                      return false;
    }

    // Compliance
    if (strict && ((uint8_t)(sumBytes(results->state,kWtc801StateLength) ^ 0x1FE))) return false;
    
    // Success
    results->decode_type = decode_type_t::WTC801;
    results->bits = nbits; 
    results->command = irutils::getBit(results->state[2],7);
    return true;
}
#endif  // DECODE_WTC801   

#if SEND_WTC801
    
    void IRsend::sendWtc801(const uint64_t data, 
                            const uint16_t nbits,
                            const uint16_t repeat) {
      if (nbits < kWtc801Bits)
        return;  // Not enough bytes to send a proper message.    
  
      for (uint16_t r = 0; r <= repeat; r++) {
        sendGeneric(kWtc801HdrMark, kWtc801HdrSpace, 
                    kWtc801BitMark   , kWtc801OneSpace,
                    kWtc801ZeroSpace , kWtc801BitMark,                    
                    kWtc801BitMark   , kDefaultMessageGap,
                    data, nbits, 
                    38000,    // Complete guess of the modulation frequency.
                    false, 0, 100);
   
      }
    }

#endif
