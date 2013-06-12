//*************************************************************************************
/** \file mechutil.cpp
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
 
#include "mechutil.h"

//-------------------------------------------------------------------------------------
// Stuff to make the new and delete operators work. Doxygen comments in mechutil.h

void* operator new (size_t size) 
    { 
    return malloc (size); 
    } 
 
void operator delete (void* ptr) 
    { 
    if (ptr) free (ptr); 
    } 
      
void* operator new[] (size_t size)
    {
    return malloc (size);
    }
 
void operator delete[] (void* ptr) 
    { 
    if (ptr) free (ptr); 
    }


//-------------------------------------------------------------------------------------
// Stuff needed for templates and virtual methods (?). Doxygen comments in mechutil.h

extern "C"
{
int __cxa_guard_acquire (__guard *g)
    {
    return !*(char *)(g);
    }
}

extern "C"
{
void __cxa_guard_release (__guard *g)
    {
    *(char *)g = 1;
    }
}

extern "C"
{
void __cxa_guard_abort (__guard *g)
    {
    }
}

//-------------------------------------------------------------------------------------
// Stuff needed to make pure virtual methods work properly. Doxygen stuff in mechutil.h

extern "C"
{    
void __cxa_pure_virtual (void)
    {
    }
}
