/******************************************************************************

  Copyright (C), 2001-2011, DCN Co., Ltd.

 ******************************************************************************
 File Name     : string.h
Version       : Initial Draft
Author        : JackABK
Created       : 2014/1/31
Last Modified :
Description   : string.c header file
Function List :
History       :
1.Date        : 2014/1/31
Author      : JackABK
Modification: Created file

 ******************************************************************************/

#ifndef __STRING_H__
#define __STRING_H__


#ifdef __cplusplus
#if __cplusplus
extern "C"{
#endif
#endif /* __cplusplus */

extern void *memcpy(void *dest, const void *src, int n);
extern void *memset(void *dest, int c, int n);
extern int round(float number);
extern char *strcat(char *dest, char *src);
extern char *strchr(const char *s, int c);
extern int strcmp(const char* s1, const char* s2);
extern char *strcpy(char *dest, const char *src);
extern int strlen(char *str );
extern int strncmp(const char *s1 , const char *s2 , int n);
extern char *strncpy(char *dest, const char *src, int n);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */


#endif /* __STRING_H__ */
