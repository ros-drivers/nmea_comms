#include "nmea_comms/checksum.h"
#include <stdio.h>

void compute_checksum(const char* sentence_body, char checksum_out[2])
{
  char checksum = 0;
  while(*sentence_body) {
    checksum ^= *sentence_body;
    sentence_body++;
  }
  sprintf(checksum_out, "%2X", checksum);
}
