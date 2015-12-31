/****************************************************************************
 * WORDOPS
 *
 * Word operations. Read/write DWORD and WORD entries from arrays.
 ****************************************************************************/

#ifndef WORDOPS_H
#define WORDOPS_H

#include <stdint.h>

/// Read a DWORD, Motorola byte order.
static inline uint32_t DWORD_READ(const uint8_t *arr, const size_t addr)
{
	return
		(arr[addr+0] << 24) |
		(arr[addr+1] << 16) |
		(arr[addr+2] << 8) |
		(arr[addr+3]);
}

/// Read a DWORD, Motorola byte order.
static inline uint16_t WORD_READ(const uint8_t *arr, const size_t addr)
{
	return
		(arr[addr+0] << 8) |
		(arr[addr+1]);
}

/// Write a DWORD, Motorola byte order.
static inline void DWORD_WRITE(uint8_t *arr, const size_t addr, const uint32_t val)
{
	arr[addr+0] = (val >> 24) & 0xFF;
	arr[addr+1] = (val >> 16) & 0xFF;
	arr[addr+2] = (val >> 8)  & 0xFF;
	arr[addr+3] =  val        & 0xFF;
}

/// Read a DWORD, Motorola byte order.
static inline void WORD_WRITE(uint8_t *arr, const size_t addr, const uint16_t val)
{
	arr[addr+0] = (val >> 8)  & 0xFF;
	arr[addr+1] =  val        & 0xFF;
}

#endif // WORDOPS_H
