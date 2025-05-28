/*

  Contains code inspired by and derived from code written for the RTEMS project.
  See LICENSE for more information.

*/

#ifndef __PROCESSOR_H__
#define __PROCESSOR_H__

#define __stringify(rn)								#rn
#define ATTRIBUTE_ALIGN(v)							__attribute__((aligned(v)))
// courtesy of Marcan
#define STACK_ALIGN(type, name, cnt, alignment)		u8 _al__##name[((sizeof(type)*(cnt)) + (alignment) + (((sizeof(type)*(cnt))%(alignment)) > 0 ? ((alignment) - ((sizeof(type)*(cnt))%(alignment))) : 0))]; \
													type *name = (type*)(((u32)(_al__##name)) + ((alignment) - (((u32)(_al__##name))&((alignment)-1))))


#define _CPU_ISR_Disable( _isr_cookie ) \
	do { \
        (void) _isr_cookie; /* unused */ \
        enter_critical_section();
	} while (0)

#define _CPU_ISR_Restore( _isr_cookie )  \
	do { \
        (void) _isr_cookie; /* unused */ \
        leave_critical_section();
	} while (0)

#ifdef __cplusplus
   extern "C" {
#endif /* __cplusplus */

static inline u16 bswap16(u16 val)
{
	u16 tmp = val;
	return __lhbrx(&tmp,0);
}

static inline u32 bswap32(u32 val)
{
	u32 tmp = val;
	return __lwbrx(&tmp,0);
}

static inline u64 bswap64(u64 val)
{
	union ullc {
		u64 ull;
		u32 ul[2];
	} outv;
	u64 tmp = val;

	outv.ul[0] = __lwbrx(&tmp,4);
	outv.ul[1] = __lwbrx(&tmp,0);

	return outv.ull;
}

#ifdef __cplusplus
   }
#endif /* __cplusplus */

#endif
