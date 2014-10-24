//
// arch/x86_64/rsp/rsp.c
//
// Declarations for host RSP functions.
//
// This file is subject to the terms and conditions defined in
// 'LICENSE', which is part of this source code package.
//

#include "common.h"
#include "arch/x86_64/rsp/rsp.h"
#include "rsp/cpu.h"

#ifdef __SSSE3__
cen64_align(const uint16_t shuffle_keys[16][8], CACHE_LINE_SIZE)  = {
  /* -- */ {0x0100, 0x0302, 0x0504, 0x0706, 0x0908, 0x0B0A, 0x0D0C, 0x0F0E},
  /* -- */ {0x0100, 0x0302, 0x0504, 0x0706, 0x0908, 0x0B0A, 0x0D0C, 0x0F0E},

  /* 0q */ {0x0100, 0x0100, 0x0504, 0x0504, 0x0908, 0x0908, 0x0D0C, 0x0D0C},
  /* 1q */ {0x0302, 0x0302, 0x0706, 0x0706, 0x0B0A, 0x0B0A, 0x0F0E, 0x0F0E},

  /* 0h */ {0x0100, 0x0100, 0x0100, 0x0100, 0x0908, 0x0908, 0x0908, 0x0908},
  /* 1h */ {0x0302, 0x0302, 0x0302, 0x0302, 0x0B0A, 0x0B0A, 0x0B0A, 0x0B0A},
  /* 2h */ {0x0504, 0x0504, 0x0504, 0x0504, 0x0D0C, 0x0D0C, 0x0D0C, 0x0D0C},
  /* 3h */ {0x0706, 0x0706, 0x0706, 0x0706, 0x0F0E, 0x0F0E, 0x0F0E, 0x0F0E},

  /* 0w */ {0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100},
  /* 1w */ {0x0302, 0x0302, 0x0302, 0x0302, 0x0302, 0x0302, 0x0302, 0x0302},
  /* 2w */ {0x0504, 0x0504, 0x0504, 0x0504, 0x0504, 0x0504, 0x0504, 0x0504},
  /* 3w */ {0x0706, 0x0706, 0x0706, 0x0706, 0x0706, 0x0706, 0x0706, 0x0706},
  /* 4w */ {0x0908, 0x0908, 0x0908, 0x0908, 0x0908, 0x0908, 0x0908, 0x0908},
  /* 5w */ {0x0B0A, 0x0B0A, 0x0B0A, 0x0B0A, 0x0B0A, 0x0B0A, 0x0B0A, 0x0B0A},
  /* 6w */ {0x0D0C, 0x0D0C, 0x0D0C, 0x0D0C, 0x0D0C, 0x0D0C, 0x0D0C, 0x0D0C},
  /* 7w */ {0x0F0E, 0x0F0E, 0x0F0E, 0x0F0E, 0x0F0E, 0x0F0E, 0x0F0E, 0x0F0E},
};
#else
__m128i rsp_vect_load_and_shuffle_operand(
  const uint16_t *src, unsigned element) {
  uint16_t word_lo, word_hi;
  uint64_t dword;

  // element => 0w ... 7w
  if (element >= 8) {
    memcpy(&word_lo, src + (element - 8), sizeof(word_lo));
    dword = word_lo | ((uint32_t) word_lo << 16);

    return _mm_shuffle_epi32(_mm_loadl_epi64((__m128i *) &dword),
      _MM_SHUFFLE(0,0,0,0));
  }

  // element => 0h ... 3h
  else if (element >= 4) {
    __m128i v;

    memcpy(&word_hi, src + element - 0, sizeof(word_hi));
    memcpy(&word_lo, src + element - 4, sizeof(word_lo));
    dword = word_lo | ((uint32_t) word_hi << 16);

    v = _mm_loadl_epi64((__m128i *) &dword);
    v = _mm_shufflelo_epi16(v, _MM_SHUFFLE(1,1,0,0));
    return _mm_shuffle_epi32(v, _MM_SHUFFLE(1,1,0,0));
  }

  // element => 0q ... 1q
  else if (element >= 2) {
    __m128i vlo, vhi;
    int i;

    for (i = -2; i < 6; i += 2)
      dword = (dword << 16) | src[element + i];

    vlo = _mm_loadl_epi64((__m128i *) &dword);
    vhi = _mm_slli_si128(vlo, 8);
    vlo = _mm_shufflelo_epi16(vlo, _MM_SHUFFLE(2,2,3,3));
    vhi = _mm_shufflehi_epi16(vhi, _MM_SHUFFLE(0,0,1,1));
    return _mm_or_si128(vhi, vlo);
  }

  return rsp_vect_load_unshuffled_operand(src);
}
#endif

#ifdef __SSSE3__
//
// The magic, omnipotent shift/rotate table. Since SSE does not
// provide the capability to shift (or rotate, for that matter)
// an entire vector by a variable value, we use pshufb with a LUT
// to achieve the desired effect.
//
// This table also takes into account that DMEM is big-endian
// byte ordering, whereas vectors are 2-byte little-endian.
//
cen64_align(const uint16_t byteswap_key[32][8], CACHE_LINE_SIZE) = {

  // Shift left LUT; shifts in zeros from the right, one byte at a time.
  {0x0001, 0x0203, 0x0405, 0x0607, 0x0809, 0x0A0B, 0x0C0D, 0x0E0F},
  {0x8000, 0x0102, 0x0304, 0x0506, 0x0708, 0x090A, 0x0B0C, 0x0D0E},
  {0x8080, 0x0001, 0x0203, 0x0405, 0x0607, 0x0809, 0x0A0B, 0x0C0D},
  {0x8080, 0x8000, 0x0102, 0x0304, 0x0506, 0x0708, 0x090A, 0x0B0C},

  {0x8080, 0x8080, 0x0001, 0x0203, 0x0405, 0x0607, 0x0809, 0x0A0B},
  {0x8080, 0x8080, 0x8000, 0x0102, 0x0304, 0x0506, 0x0708, 0x090A},
  {0x8080, 0x8080, 0x8080, 0x0001, 0x0203, 0x0405, 0x0607, 0x0809},
  {0x8080, 0x8080, 0x8080, 0x8000, 0x0102, 0x0304, 0x0506, 0x0708},

  {0x8080, 0x8080, 0x8080, 0x8080, 0x0001, 0x0203, 0x0405, 0x0607},
  {0x8080, 0x8080, 0x8080, 0x8080, 0x8000, 0x0102, 0x0304, 0x0506},
  {0x8080, 0x8080, 0x8080, 0x8080, 0x8080, 0x0001, 0x0203, 0x0405},
  {0x8080, 0x8080, 0x8080, 0x8080, 0x8080, 0x8000, 0x0102, 0x0304},

  {0x8080, 0x8080, 0x8080, 0x8080, 0x8080, 0x8080, 0x0001, 0x0203},
  {0x8080, 0x8080, 0x8080, 0x8080, 0x8080, 0x8080, 0x8000, 0x0102},
  {0x8080, 0x8080, 0x8080, 0x8080, 0x8080, 0x8080, 0x8080, 0x0001},
  {0x8080, 0x8080, 0x8080, 0x8080, 0x8080, 0x8080, 0x8080, 0x8000},

  // Shift right LUT; shifts in zeros from the left, one byte at a time.
  {0x0001, 0x0203, 0x0405, 0x0607, 0x0809, 0x0A0B, 0x0C0D, 0x0E0F},
  {0x0102, 0x0304, 0x0506, 0x0708, 0x090A, 0x0B0C, 0x0D0E, 0x0F80},
  {0x0203, 0x0405, 0x0607, 0x0809, 0x0A0B, 0x0C0D, 0x0E0F, 0x8080},
  {0x0304, 0x0506, 0x0708, 0x090A, 0x0B0C, 0x0D0E, 0x0F80, 0x8080},

  {0x0405, 0x0607, 0x0809, 0x0A0B, 0x0C0D, 0x0E0F, 0x8080, 0x8080},
  {0x0506, 0x0708, 0x090A, 0x0B0C, 0x0D0E, 0x0F80, 0x8080, 0x8080},
  {0x0607, 0x0809, 0x0A0B, 0x0C0D, 0x0E0F, 0x8080, 0x8080, 0x8080},
  {0x0708, 0x090A, 0x0B0C, 0x0D0E, 0x0F80, 0x8080, 0x8080, 0x8080},

  {0x0809, 0x0A0B, 0x0C0D, 0x0E0F, 0x8080, 0x8080, 0x8080, 0x8080},
  {0x090A, 0x0B0C, 0x0D0E, 0x0F80, 0x8080, 0x8080, 0x8080, 0x8080},
  {0x0A0B, 0x0C0D, 0x0E0F, 0x8080, 0x8080, 0x8080, 0x8080, 0x8080},
  {0x0B0C, 0x0D0E, 0x0F80, 0x8080, 0x8080, 0x8080, 0x8080, 0x8080},

  {0x0C0D, 0x0E0F, 0x8080, 0x8080, 0x8080, 0x8080, 0x8080, 0x8080},
  {0x0D0E, 0x0F80, 0x8080, 0x8080, 0x8080, 0x8080, 0x8080, 0x8080},
  {0x0E0F, 0x8080, 0x8080, 0x8080, 0x8080, 0x8080, 0x8080, 0x8080},
  {0x0F80, 0x8080, 0x8080, 0x8080, 0x8080, 0x8080, 0x8080, 0x8080},
};
#endif

//
// TODO: Test this.
//
#ifdef __SSSE3__
__m128i rsp_vload_dmem(struct rsp *rsp,
  __m128i reg, __m128i dqm, uint32_t addr, unsigned srselect) {
  __m128i data = _mm_load_si128((__m128i *) (rsp->mem + addr));
  __m128i key = _mm_load_si128((__m128i *) (byteswap_key[srselect]));

  // Byteswap and rotate as needed.
  data = _mm_shuffle_epi8(data, key);
  dqm = _mm_shuffle_epi8(dqm, key);

  // Mask and mux in the data.
#ifdef __SSE4__
  data = _mm_blendv_epi8(reg, data, dqm);
#else
  data = _mm_and_si128(dqm, data);
  reg = _mm_andnot_si128(dqm, reg);
  data = _mm_or_si128(data, reg);
#endif

  return data;
}
#endif

//
// TODO: Test this.
//
void rsp_vstore_dmem(struct rsp *rsp,
  __m128i reg, __m128i dqm, uint32_t addr, unsigned srselect) {
  __m128i data = _mm_load_si128((__m128i *) (rsp->mem + addr));
  __m128i key = _mm_load_si128((__m128i *) (byteswap_key[srselect]));

  // Byteswap and rotate as needed.
  data = _mm_shuffle_epi8(data, key);
  dqm = _mm_shuffle_epi8(dqm, key);

  // Mask and mux in the data, write.
#ifdef __SSE4__
  data = _mm_blendv_epi8(data, reg, dqm);
#else
  data = _mm_and_si128(dqm, reg);
  reg = _mm_andnot_si128(dqm, data);
  data = _mm_or_si128(data, reg);
#endif

  _mm_store_si128((__m128i *) (rsp->mem + addr), data);
}

