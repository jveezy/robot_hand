//*************************************************************************************
/** \file mechutil.h
 *    This file contains a set of enhancements to the GCC-AVR system which support 
 *    some of the more helpful C++ features that aren't present or working well in the
 *    normal GCC-AVR distribution (though I have no idea why). The information came 
 *    from an AVR-Freaks posting at: 
 *    \li http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&p=410870
 *
 *  Revisions
 *    \li  04-12-08  JRR  Original file, material from source above
 */
//*************************************************************************************

#ifndef _ME405_H_
#define _ME405_H_

#include <stdlib.h> 

// ------------------ Stuff to make the new and delete operators work -----------------

/** This is the standard "new" operator, defined here because it's not available in the
 *  standard avr-libc library. 
 *  @param size The number of bytes of memory which need to be allocated
 */
void* operator new (size_t size);

/** This is the standard "delete" operator, defined as a replacement for the one which 
 *  has been left out of the avr-libc library. 
 *  @param ptr A pointer to the object which is to be deleted from memory
 */
void operator delete (void* ptr);

/** This is the array form of the standard "new" operator. 
 *  @param size The total number of bytes of memory which need to be allocated
 */
void* operator new[] (size_t size);

/** This is the array form of the standard "delete" operator. 
 *  @param ptr A pointer to the object which is to be deleted from memory
 */
void operator delete[] (void* ptr);

// ---------------------- Stuff for pure virtual functions (?) ------------------------

// This stuff is supposed to help with templates and virtual methods
__extension__ typedef int __guard __attribute__((mode (__DI__))); 
extern "C" int __cxa_guard_acquire (__guard *); 
extern "C" void __cxa_guard_release (__guard *); 
extern "C" void __cxa_guard_abort (__guard *);

/** This method is said to be required to help make pure virtual methods operate 
 *  correctly. It's not certain that this method works; it needs testing. 
 */
extern "C" void __cxa_pure_virtual (void);

#endif // _ME405_H_
