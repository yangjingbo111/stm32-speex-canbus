/* config.h.in.  Generated from configure.ac by autoheader.  */

/* Define if building universal (internal helper macro) */
#undef AC_APPLE_UNIVERSAL_BUILD

/* Make use of ARM4 assembly optimizations */
#undef ARM4_ASM

/* Make use of ARM5E assembly optimizations */
#undef ARM5E_ASM

/* Make use of Blackfin assembly optimizations */
#undef BFIN_ASM

/* Disable all parts of the API that are using floats */
#define DISABLE_FLOAT_API

/* Disable VBR and VAD from the codec */
#define DISABLE_VBR

/* Enable valgrind extra checks */
#undef ENABLE_VALGRIND

/* Symbol visibility prefix */
#define EXPORT

/* Debug fixed-point implementation */
#undef FIXED_DEBUG

/* Compile as fixed-point */
#define FIXED_POINT

/* Compile as floating-point */
//#define FLOATING_POINT




/* Define to 1 if you have the <sys/audioio.h> header file. */
#undef HAVE_SYS_AUDIOIO_H

/* Define to 1 if you have the <sys/soundcard.h> header file. */
#undef HAVE_SYS_SOUNDCARD_H

/* Define to 1 if you have the <sys/stat.h> header file. */
#undef HAVE_SYS_STAT_H

/* Define to 1 if you have the <sys/types.h> header file. */
#undef HAVE_SYS_TYPES_H

/* Define to 1 if you have the <unistd.h> header file. */
#undef HAVE_UNISTD_H

/* Define to the sub-directory where libtool stores uninstalled libraries. */
#undef LT_OBJDIR

/* Define to the address where bug reports for this package should be sent. */
#undef PACKAGE_BUGREPORT

/* Define to the full name of this package. */
#undef PACKAGE_NAME

/* Define to the full name and version of this package. */
#undef PACKAGE_STRING

/* Define to the one symbol short name of this package. */
#undef PACKAGE_TARNAME

/* Define to the home page for this package. */
#undef PACKAGE_URL

/* Define to the version of this package. */
#undef PACKAGE_VERSION

/* The size of `int', as computed by sizeof. */
#undef SIZEOF_INT

/* The size of `int16_t', as computed by sizeof. */
#undef SIZEOF_INT16_T

/* The size of `int32_t', as computed by sizeof. */
#undef SIZEOF_INT32_T

/* The size of `long', as computed by sizeof. */
#undef SIZEOF_LONG

/* The size of `short', as computed by sizeof. */
#undef SIZEOF_SHORT

/* The size of `uint16_t', as computed by sizeof. */
#undef SIZEOF_UINT16_T

/* The size of `uint32_t', as computed by sizeof. */
#undef SIZEOF_UINT32_T

/* The size of `u_int16_t', as computed by sizeof. */
#undef SIZEOF_U_INT16_T

/* The size of `u_int32_t', as computed by sizeof. */
#undef SIZEOF_U_INT32_T



/* Define to 1 if you have the ANSI C header files. */
//#undef STDC_HEADERS



/* Make use of alloca */
#undef USE_ALLOCA

/* Use FFTW3 for FFT */
#undef USE_GPL_FFTW3

/* Use Intel Math Kernel Library for FFT */
#undef USE_INTEL_MKL

/* Use KISS Fast Fourier Transform */
#undef USE_KISS_FFT

/* Use FFT from OggVorbis */
#undef USE_SMALLFT

/* Use SpeexDSP library */
#undef USE_SPEEXDSP

/* Use C99 variable-size arrays */
#undef VAR_ARRAYS

/* Enable support for the Vorbis psy model */
#undef VORBIS_PSYCHO



/* Enable SSE support */
#undef _USE_SSE

/* Define to empty if `const' does not conform to ANSI C. */
#undef const

/* Define to `__inline__' or `__inline' if that's what the C compiler
   calls it, or to nothing if 'inline' is not supported under any name.  */
#ifndef __cplusplus
#undef inline
#endif



#ifdef  __CC_ARM               /* ARM Compiler */
#define inline __inline
#endif

//#define DISABLE_WIDEBAND
#define DISABLE_NOTIFICATIONS
#define DISABLE_WARNINGS
#define OVERRIDE_SPEEX_PUTC
#define OVERRIDE_SPEEX_FATAL

extern void _speex_fatal(const char *str, const char *file, int line);
extern void _speex_putc(int ch, void *file);

