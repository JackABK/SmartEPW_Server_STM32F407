#include "FreeRTOS.h"
#include "task.h"
#include "unit_tests.h"
#include "ultrasound.h"



int tests_run_cnt = 0;

int foo = 7;
int bar = 5;

static char * test_foo() {
     assert("error, foo != 7", foo == 7);
     return 0;
}
 
static char * test_bar() {
     assert("error, bar != 5", bar == 5);
     return 0;
}


static char * all_tests() {
       run_test(test_foo);
       return 0;
}


 
int unit_tests_task(void){
     printf("=============TESTS UNIT STARTING=============\r\n");
     char *result = all_tests();
     printf("Tests run: %d\r\n", tests_run_cnt);
     if (result != 0) {
         printf("%s\r\n", result);
     }
     else {
         printf("============ALL TESTS PASSED============\r\n");
     }
 
     return result != 0;
}




