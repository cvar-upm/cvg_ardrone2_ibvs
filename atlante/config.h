/**
 * Description: Framework configuration parameters
 * Author: Ignacio Mellado (CVG)
 * Last revision date: 3/11/2010
 */

/**
 * Required minimum sizes for framework basic types (in bytes). 
 *
 * The appropriate platform types will be chosen to represent framework types 
 * with the required minimum sizes (or bigger). If it cannot be done, an error is output
 * during compilation.
 *
 * Floating point types are always forced to be: 32-bit floats and 64-bit doubles. Otherwise,
 * a compiling error is produced.
 *
 * Please, keep in mind that, as exact type sizes are platform-dependent, serialization 
 * objects should be considered (for instance, for file creation or data transmission).
 *
 */
#define CVG_REQUIRED_MAX_UCHAR			0xFF
#define CVG_REQUIRED_MAX_USHORT			0xFFFF
#define CVG_REQUIRED_MAX_UINT			0xFFFFFFFF
#define CVG_REQUIRED_MAX_ULONG			0xFFFFFFFFFFFFFFFF

/**
 * If CVG_DO_NOT_USE_FLOATING_POINT is defined, methods using floating point math will not be
 * implemented, thus reducing code size on platforms without FPU.
 * Do not define this label if your app needs floating point support.
 */
/* 
#define CVG_DO_NOT_USE_FLOATING_POINT 
*/
