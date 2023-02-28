#ifndef OTOMO_SERIAL_KISS_TNC_H
#define OTOMO_SERIAL_KISS_TNC_H

#include <vector>
#include <stdint.h>

namespace otomo_serial
{

class KissOutputStream
{
public:
  KissOutputStream() = default;
  ~KissOutputStream() = default;

  int encode_buffer(const std::vector<uint8_t> & input, std::vector<uint8_t> & output);

private:
  std::vector<uint8_t> buf_;
  
};

}

#endif  // OTOMO_SERIAL_KISS_TNC_H