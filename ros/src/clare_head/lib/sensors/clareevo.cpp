#include "clareevo.h"

ClareEvo::ClareEvo(){
}

void ClareEvo::setupEvo(Logging* lggr, Stream* serial) {
    channel = serial;
    logger = lggr;

    const byte PRINTOUT_BINARY[4]             = {0x00,0x11,0x02,0x4C};
    const byte PRINTOUT_TEXT[4]               = {0x00,0x11,0x01,0x45};
    const byte RUNMODE_SINGLE_PIXEL[4]        = {0x00,0x21,0x01,0xBC};
    const byte RUNMODE_TWO_PIXEL[4]           = {0x00,0x21,0x03,0xB2};
    const byte RUNMODE_TWO_BY_TWO_PIXEL[4]    = {0x00,0x21,0x02,0xB5};
    const byte RANGEMODE_SHORT[4]             = {0x00,0x61,0x01,0xE7};
    const byte RANGEMODE_LONG[4]              = {0x00,0x61,0x03,0xE9};

    //channel->write(RUNMODE_SINGLE_PIXEL, 4); // Set the TeraRanger in single-pixel mode
    //channel->write(RUNMODE_TWO_PIXEL, 4); // Set the TeraRanger in two-pixel mode
    channel->write(RUNMODE_TWO_BY_TWO_PIXEL, 4); // Set the TeraRanger in two-by-two pixel mode

    channel->write(RANGEMODE_LONG, 4); // Set the Teraranger in long range mode
    //channel->write(RANGEMODE_SHORT, 4); // Set the Teraranger in short range mode

    channel->write(PRINTOUT_BINARY, 4);// Set the TeraRanger in Binary mode, text mode is not supported
}

/*
 * Brief : Calculate a Cyclic Redundancy Checks of 8 bits
 * Param1 : (*p) pointer to receive buffer
 * Param2 : (len) number of bytes returned by the TeraRanger
 * Return : (crc & 0xFF) checksum calculated locally
 */
uint8_t crc8(uint8_t *p, uint8_t len) {
  uint8_t i;
  uint8_t crc = 0x0;
  while (len--) {
    i = (crc ^ *p++) & 0xFF;
    crc = (crc_table[i] ^ (crc << 8)) & 0xFF;
  }
  return crc & 0xFF;
}

void ClareEvo::readState(float &x1, float &x2, float &x3, float &x4) {
    const int BUFFER_LENGTH = 10;
    uint8_t Framereceived[BUFFER_LENGTH];// The variable "Framereceived[]" will contain the frame sent by the TeraRanger
    uint8_t indexx = 0;// The variable "indexx" will contain the number of actual bytes in the frame to treat in the main loop
   
    while(true){
      if (channel->available() > 0) {
      // Send data only when you receive data
      uint8_t inChar = channel->read();
      if (indexx == 0) {
        if (inChar == 'T') {
          // Looking for frame start 'T'
          Framereceived[indexx++] = inChar;
        } else {
          continue;
        }
      } else if ((indexx > 0) && (indexx < 10)) {
        // Gathering data
        Framereceived[indexx++] = inChar;
      }

      // Check if the frame is single-pixel
      if (indexx == 4) {
        if (crc8(Framereceived, 3) == Framereceived[3]) {
          // Convert bytes to distance
          x1 = (Framereceived[1] << 8) + Framereceived[2];
          break;
        }
      }
      // Check if the frame is two-pixel
      else if (indexx == 6) {
        if (crc8(Framereceived, 5) == Framereceived[5]) {
          // Convert bytes to distances
          x1 = (Framereceived[1] << 8) + Framereceived[2];
          x2 = (Framereceived[3] << 8) + Framereceived[4];
          break;
        }
      }
      // Check if the frame is two-by-two-pixel
      else if (indexx == 10) {
        if (crc8(Framereceived, 9) == Framereceived[9]) {
          // Convert bytes to distances
          x1 = (Framereceived[1] << 8) + Framereceived[2];
          x2 = (Framereceived[3] << 8) + Framereceived[4];
          x3 = (Framereceived[5] << 8) + Framereceived[6];
          x4 = (Framereceived[7] << 8) + Framereceived[8];

          break;
        } else {
          logger->error(
              "CRC checks failed. Couldn't find valid frame in buffer length");
          break;
        }
      }
    }
  }  
  indexx = 0;
  Framereceived[0] = 0;
}