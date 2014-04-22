/*=============================================================================
  *
  * @file     : unit_tests.h
  * @data       : 2014/2/17
  * @brief   : unit_tests.c header file
  *            the unit tests of method is refer to JTN002 - MinUnit -- a minimal unit testing framework for C,
  *            very thanks to the original author, it's helpful me.
  *            for info, see:  
  *            http://www.jera.com/techinfo/jtns/jtn002.html 
  *============================================================================*/
#ifndef __UNIT_TESTS_H__
#define __UNIT_TESTS_H__
#define assert(message, test) do { if (!(test)) return message; } while (0)
#define run_test(test) do { char *message = test(); tests_run_cnt++; \
                                if (message) return message; } while (0)
extern int tests_run_cnt; /*unit tests running count*/

extern int unit_tests_task(void);
#endif /* __UNIT_TESTS_H__ */
