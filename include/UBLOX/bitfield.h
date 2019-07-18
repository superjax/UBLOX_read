//#pragma once

//#include <stdint.h>

//enum {
//  SwitchByteOrder,
//  KeepByteOrder
//  };

//template<int firstBit, int bitSize, int byteOrder=KeepByteOrder>
//struct BitField
//{
//  typedef BitField<firstBit, bitSize, byteOrder> self_t;
//  typedef unsigned char uchar;

//  enum {
//    lastBit = firstBit + bitSize - 1,
//    mask = bitSize == 64 ? 0xFFFFFFFF : (1ul << bitSize) - 1ul,
//    };

//  uchar *selfArray()
//  {
//    return reinterpret_cast<uchar *>(this);
//  }
//  const uchar *selfArray() const
//  {
//    return reinterpret_cast<const uchar *>(this);
//  }

//  /* used to read data from the field */
//  /* will also work with all the operators that work with integral types */
//  operator unsigned() const
//  {
//    //        const uint8_t *arr = selfArray();
//    const uint8_t *p = selfArray() + (firstBit / 8);
//    if (byteOrder == SwitchByteOrder)
//    {
//      int i = 8 - (firstBit & 7);
//      unsigned ret = *p;
//      while (i < bitSize)
//      {
//        ret <<= 8;
//        ret |= *(++p);
//        i += 8;
//      }
//      return ((ret >> (7 - (lastBit & 7))) & mask);
//    }
//    else
//    {
//      int i = 0;
//      unsigned ret = *p;
//      while (i < bitSize)
//      {
//        ret |= (*(++p) << (i+8));
//        i += 8;
//      }
//      return ((ret >> (firstBit % 8)) & mask);
//    }
//  }

//  /* used to assign a value into the field */
//  inline self_t& operator=(unsigned m)
//  {
//    uchar *arr = selfArray();
//    m &= mask;
//    unsigned wmask = ~(mask << (7 - (lastBit & 7)));
//    m <<= (7 - (lastBit & 7));
//    uchar *p = arr + lastBit / 8;
//    int i = (lastBit & 7) + 1;
//    (*p &= wmask) |= m;
//    while (i < bitSize)
//    {
//      m >>= 8;
//      wmask >>= 8;
//      (*(--p) &= wmask) |= m;
//      i += 8;
//    }
//    return *this;
//  }

//  inline self_t& operator+=(unsigned m)
//  {
//    *this = *this + m;
//    return *this;
//  }

//  inline self_t& operator-=(unsigned m)
//  {
//    *this = *this - m;
//    return *this;
//  }

//  inline self_t& operator*=(unsigned m)
//  {
//    *this = *this * m;
//    return *this;
//  }

//  inline self_t& operator/=(unsigned m)
//  {
//    *this = *this / m;
//    return *this;
//  }

//  inline self_t& operator%=(unsigned m)
//  {
//    *this = *this % m;
//    return *this;
//  }

//  inline self_t& operator<<=(unsigned m)
//  {
//    *this = *this << m;
//    return *this;
//  }

//  inline self_t& operator>>=(unsigned m)
//  {
//    *this = *this >> m;
//    return *this;
//  }

//  inline self_t& operator|=(unsigned m)
//  {
//    *this = *this | m;
//    return *this;
//  }

//  inline self_t& operator&=(unsigned m)
//  {
//    *this = *this & m;
//    return *this;
//  }

//  inline self_t& operator^=(unsigned m)
//  {
//    *this = *this ^ m;
//    return *this;
//  }
//};
