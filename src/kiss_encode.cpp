#include "otomo-serial/kiss_tnc.h"

namespace otomo_serial
{

KissOutputStream::KissOutputStream()
{
  init();
}

void KissOutputStream::init()
{
  buf_.clear();
  buf_.push_back(FEND);
  buf_.push_back(0);
}

void KissOutputStream::addByte(const uint8_t b)
{
  switch (b)
  {
    case (FEND):
      buf_.push_back(FESC);
      buf_.push_back(TFEND);
      break;
    case (FESC):
      buf_.push_back(FESC);
      buf_.push_back(TFESC);
      break;
    default:
      buf_.push_back(b);
      break;
  }
}

std::vector<uint8_t> KissOutputStream::getBuffer()
{
  std::vector<uint8_t> out(buf_);
  out.push_back(FEND);
  return out;
}

}  // namespace otomo_serial
