#pragma once
#include "crc_parameters.h"


#if defined(CRC_CUSTOM_SIZE)
using crc_size_t = CRC_CUSTOM_SIZE;
#elif defined(__AVR__)
using crc_size_t = uint16_t;
#else
using crc_size_t = size_t;
#endif


uint8_t reverse8bits(uint8_t in);
uint16_t reverse16bits(uint16_t in);
uint16_t reverse12bits(uint16_t in);
uint32_t reverse32bits(uint32_t in);
uint64_t reverse64bits(uint64_t in);

uint8_t reverse8bits(uint8_t in)
{
  uint8_t x = in;
  x = (((x & 0xAA) >> 1) | ((x & 0x55) << 1));
  x = (((x & 0xCC) >> 2) | ((x & 0x33) << 2));
  x =          ((x >> 4) | (x << 4));
  return x;
}

uint16_t reverse16bits(uint16_t in)
{
  uint16_t x = in;
  x = (((x & 0XAAAA) >> 1) | ((x & 0X5555) << 1));
  x = (((x & 0xCCCC) >> 2) | ((x & 0X3333) << 2));
  x = (((x & 0xF0F0) >> 4) | ((x & 0X0F0F) << 4));
  x = (( x >> 8) | (x << 8));
  return x;
}

uint16_t reverse12bits(uint16_t in)
{
  return reverse16bits(in) >> 4;
}

uint32_t reverse32bits(uint32_t in)
{
  uint32_t x = in;
  x = (((x & 0xAAAAAAAA) >> 1)  | ((x & 0x55555555) << 1));
  x = (((x & 0xCCCCCCCC) >> 2)  | ((x & 0x33333333) << 2));
  x = (((x & 0xF0F0F0F0) >> 4)  | ((x & 0x0F0F0F0F) << 4));
  x = (((x & 0xFF00FF00) >> 8)  | ((x & 0x00FF00FF) << 8));
  x = (x >> 16) | (x << 16);
  return x;
}

uint64_t reverse64bits(uint64_t in)
{
  uint64_t x = in;
  x = (((x & 0xAAAAAAAAAAAAAAAA) >> 1)  | ((x & 0x5555555555555555) << 1));
  x = (((x & 0xCCCCCCCCCCCCCCCC) >> 2)  | ((x & 0x3333333333333333) << 2));
  x = (((x & 0xF0F0F0F0F0F0F0F0) >> 4)  | ((x & 0x0F0F0F0F0F0F0F0F) << 4));
  x = (((x & 0xFF00FF00FF00FF00) >> 8)  | ((x & 0x00FF00FF00FF00FF) << 8));
  x = (((x & 0xFFFF0000FFFF0000) >> 16) | ((x & 0x0000FFFF0000FFFF) << 16));
  x = (x >> 32) | (x << 32);
  return x;
}


uint8_t reverse8(uint8_t in)
{
  return reverse8bits(in);
}

uint16_t reverse12(uint16_t in)
{
  return reverse12bits(in);
}

uint16_t reverse16(uint16_t in)
{
  return reverse16bits(in);
}

uint32_t reverse32(uint32_t in)
{
  return reverse32bits(in);
}

uint64_t reverse64(uint64_t in)
{
  return reverse64bits(in);
}

class CRC8
{
public:
  CRC8(uint8_t polynome = CRC8_POLYNOME,
       uint8_t initial  = CRC8_INITIAL,
       uint8_t xorOut   = CRC8_XOR_OUT,
       bool reverseIn   = CRC8_REV_IN,
       bool reverseOut  = CRC8_REV_OUT);

  void reset(uint8_t polynome = CRC8_POLYNOME,
             uint8_t initial  = CRC8_INITIAL,
             uint8_t xorOut   = CRC8_XOR_OUT,
             bool reverseIn   = CRC8_REV_IN,
             bool reverseOut  = CRC8_REV_OUT);

  void restart();
  uint8_t calc() const;
  crc_size_t count() const;
  void add(uint8_t value);
  void add(const uint8_t *array, crc_size_t length);
  void add(const uint8_t *array, crc_size_t length, crc_size_t yieldPeriod);

  void setPolynome(uint8_t polynome) { _polynome = polynome; }
  void setInitial(uint8_t initial) { _initial = initial; }
  void setXorOut(uint8_t xorOut) { _xorOut = xorOut; }
  void setReverseIn(bool reverseIn) { _reverseIn = reverseIn; }
  void setReverseOut(bool reverseOut) { _reverseOut = reverseOut; }

  uint8_t getPolynome() const { return _polynome; }
  uint8_t getInitial() const { return _initial; }
  uint8_t getXorOut() const { return _xorOut; }
  bool getReverseIn() const { return _reverseIn; }
  bool getReverseOut() const { return _reverseOut; }

private:
  void _add(uint8_t value);

  uint8_t _polynome;
  uint8_t _initial;
  uint8_t _xorOut;
  bool _reverseIn;
  bool _reverseOut;
  uint8_t _crc;
  crc_size_t _count;
};

CRC8::CRC8(uint8_t polynome,
           uint8_t initial,
           uint8_t xorOut,
           bool reverseIn,
           bool reverseOut) :
  _polynome(polynome),
  _initial(initial),
  _xorOut(xorOut),
  _reverseIn(reverseIn),
  _reverseOut(reverseOut),
  _crc(initial),
  _count(0u)
{}

void CRC8::reset(uint8_t polynome,
                 uint8_t initial,
                 uint8_t xorOut,
                 bool reverseIn,
                 bool reverseOut)
{
  _polynome = polynome;
  _initial = initial;
  _xorOut = xorOut;
  _reverseIn = reverseIn;
  _reverseOut = reverseOut;
  restart();
}

void CRC8::restart()
{
  _crc = _initial;
  _count = 0u;
}

uint8_t CRC8::calc() const
{
  uint8_t rv = _crc;
  if (_reverseOut) rv = reverse8bits(rv);
  rv ^= _xorOut;
  return rv;
}

crc_size_t CRC8::count() const
{
  return _count;
}

void CRC8::add(uint8_t value)
{
  _count++;
  _add(value);
}

void CRC8::add(const uint8_t *array, crc_size_t length)
{
  _count += length;
  while (length--)
  {
    _add(*array++);
  }
}

void CRC8::_add(uint8_t value)
{
  if (_reverseIn) value = reverse8bits(value);
  _crc ^= value;
  for (uint8_t i = 8; i; i--) 
  {
    if (_crc & (1 << 7))
    {
      _crc <<= 1;
      _crc ^= _polynome;
    }
    else
    {
      _crc <<= 1;
    }
  }
}