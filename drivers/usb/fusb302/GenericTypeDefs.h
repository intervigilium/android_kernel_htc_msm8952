
#ifndef _FUSB30X_GENERIC_TYPE_DEFS_H_
#define _FUSB30X_GENERIC_TYPE_DEFS_H_

#define FSC_HOSTCOMM_BUFFER_SIZE    64  

#if defined(PLATFORM_LINUX)

#if defined(__GNUC__)
#define __EXTENSION __extension__
#else
#define __EXTENSION
#endif

#if !defined(__PACKED)
    #define __PACKED
#endif

#include <linux/types.h>

#ifndef U8_MAX
#define U8_MAX  ((__u8)~0U)
#endif

#ifndef VOID
typedef void                VOID;
#endif 

#if !defined(BOOL) && !defined(FALSE) && !defined(TRUE)
typedef enum _BOOL { FALSE = 0, TRUE } BOOL;    
#endif 

#ifndef BIT
typedef enum _BIT { CLEAR = 0, SET } BIT;
#endif 

#ifndef INT
typedef __s32               INT;                                                
#endif 

#ifndef INT8
typedef __s8                INT8;                                               
#endif 

#ifndef INT16
typedef __s16               INT16;                                              
#endif 

#ifndef INT32
typedef __s32               INT32;                                              
#endif 

#ifndef INT64
typedef __s64               INT64;                                              
#endif 


#ifndef UINT
typedef __u32               UINT;                                               
#endif 

#ifndef UINT8
typedef __u8                UINT8;                                              
#endif 

#ifndef UINT16
typedef __u16               UINT16;                                             
#endif 

#ifndef UINT32
typedef __u32               UINT32;                                             
#endif 

#ifndef UINT64
typedef __u64               UINT64;                                             
#endif 


#undef __EXTENSION

#endif

#endif 
