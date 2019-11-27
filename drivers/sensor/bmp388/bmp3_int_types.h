/**
 * Copyright (C) 2019 Wilmers Messtechnik
 *
 * @file    bmp3_int_types.h
 * @date    27.11.2019
 * @version 1.0.0
 * @brief
 *
 */
#ifndef BMP3_INT_TYPES_DEFS_H_
#define BMP3_INT_TYPES_DEFS_H_

/*************************** Common macros   *****************************/
#define S8_C(x) (x)
#define U8_C(x) (x)

#define S16_C(x) (x)
#define UINT16_C(x) (x)

#define S32_C(x) ((x) + (INT32_MAX - INT32_MAX))
#define U32_C(x) ((x) + (UINT32_MAX - UINT32_MAX))

#define S64_C(x) ((x) + (INT64_MAX - INT64_MAX))
#define U64_C(x) ((x) + (UINT64_MAX - UINT64_MAX))

#endif /* BMP3_INT_TYPES_DEFS_H_ */
