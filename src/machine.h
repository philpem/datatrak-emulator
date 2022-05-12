#ifndef MACHINE_H_INCLUDED
#define MACHINE_H_INCLUDED


/// ROM length
#define ROM_LENGTH (256*1024)

/// RAM base
#define RAM_BASE 0x200000

/// RAM window (limit of address space allocated to RAM)
#define RAM_WINDOW ((256*1024)-1)

/// RAM physical length
#define RAM_LENGTH (256*1024)

// Value to return if the CPU reads from unimplemented memory
#ifdef UNIMPL_READS_AS_FF
#  define UNIMPLEMENTED_VALUE (0xFFFFFFFF)
#else
#  define UNIMPLEMENTED_VALUE (0)
#endif


#endif // MACHINE_H_INCLUDED