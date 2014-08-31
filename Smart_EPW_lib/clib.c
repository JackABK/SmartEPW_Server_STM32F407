

#include <stddef.h>
#include <stdint.h>
#include <limits.h>
#include "stdarg.h"  /*need to using the va_list and some var.*/
#include "fat_filelib.h"
#include "fio.h"
#include <stdlib.h>  
// C program for implementation of ftoa()
#include <stdio.h>
#include <math.h>


#define FORMAT_CONVERT_ERROR -1  
#define STDOUT_FROM_UART 1  //Write buffer to fd 1 (stdout, through uart)


#define ALIGN (sizeof(size_t))
#define ONES ((size_t)-1/UCHAR_MAX)                                                                      
#define HIGHS (ONES * (UCHAR_MAX/2+1))
#define HASZERO(x) ((x)-ONES & ~(x) & HIGHS)

/*num_to_str to used flags*/
enum sign_type_T {
	SIGNED_INT,
	UNSIGNED_INT	
};

typedef union {  
    long    L;  
    float   F;  
}       LF_t;


#define SS (sizeof(size_t))
void *memset(void *dest, int c, size_t n)
{
		unsigned char *s = dest;
		c = (unsigned char)c;
		for (; ((uintptr_t)s & ALIGN) && n; n--) *s++ = c;
		if (n) {
				size_t *w, k = ONES * c;
				for (w = (void *)s; n>=SS; n-=SS, w++) *w = k;
				for (s = (void *)w; n; n--, s++) *s = c;
		}
		return dest;
}

void *memcpy(void *dest, const void *src, size_t n)
{
		void *ret = dest;

		//Cut rear
		uint8_t *dst8 = dest;
		const uint8_t *src8 = src;
		switch (n % 4) {
				case 3 : *dst8++ = *src8++;
				case 2 : *dst8++ = *src8++;
				case 1 : *dst8++ = *src8++;
				case 0 : ;
		}

		//stm32 data bus width
		uint32_t *dst32 = (void *)dst8;
		const uint32_t *src32 = (void *)src8;
		n = n / 4;
		while (n--) {
				*dst32++ = *src32++;
		}

		return ret;
}

char *strchr(const char *s, int c)
{
		for (; *s && *s != c; s++);
		return (*s == c) ? (char *)s : NULL;
}

char *strcpy(char *dest, const char *src)
{
		const unsigned char *s = src;
		unsigned char *d = dest;
		while ((*d++ = *s++));
		return dest;
}

char *strncpy(char *dest, const char *src, size_t n)
{
		const unsigned char *s = src;
		unsigned char *d = dest;
		while (n-- && (*d++ = *s++));
		return dest;
}
int strlen(char *str )
{
		int i = 0 ;
		while(str[i]!='\0'){
				i++;
		}
		return i ;
}
int strcmp(const char* s1, const char* s2)
{
		while(*s1 && (*s1==*s2))
				s1++,s2++;
		return *(const unsigned char*)s1-*(const unsigned char*)s2;
}
int strncmp(const char *s1 , const char *s2 , size_t n)
{
		int i ;
		for(i = 0; i < n; i++) {
				if (s1[i] != s2[i]) {
						return s1[i] - s2[i];
				}
		}
		return 0;
} 
char *strcat(char *dest, char *src)
{
		for (;*dest;dest++); 
		while(*dest++ = *src++);
		return dest ;
}  

void my_puts(char *msg)
{
  if (!msg) return;
  fio_write(STDOUT_FROM_UART , msg , strlen(msg));
}

int str2int(char *str)
{
 int i=0,tmp=0;
 while(str[i]!='\0')
 {
  if(str[i]>='0'&&str[i]<='9')  tmp=tmp*10+(str[i]-'0');
  else return FORMAT_CONVERT_ERROR;

  i++;
 }
 return tmp;
}

void int2str(int in , char*out )
{
  int i, number_len=0;
  char out_tmp[10];

  if(in == 0)
  {
    out[0] = '0';
    out[1] = '\0';
    return ;
  }

  while(in > 0){
     out_tmp[number_len] = '0' +  (in % 10);
     in /= 10;
     number_len++;
  }
  
  for(i=0; i<number_len; i++){
     out[i] = out_tmp[number_len-1-i] ;
  }    

  out[number_len] = '\0';
}
char *num_to_str(unsigned int num , char * buf , unsigned int base , int flags)
{
	int i, negative=0;

	if (flags==SIGNED_INT) {
		negative= (int)num<0 ;    
        if(negative) num = -(int)num;
	}
	/*UNSIGNED_INT is dont need to setting anything*/

        /******common part*******/
		if(num==0){
    	    buf[30]='0';
        	return &buf[30];
    	}	
		for(i=30; i>=0&&num; --i, num/=base)
	        buf[i] = "0123456789ABCDEF" [num % base];
		if(negative){
   		     buf[i]='-';
   	    	 --i;
   		}
	return buf+i+1;
}
char *itoa(int num, unsigned int base){
    static char buf[32]={0};
    return num_to_str(num , buf , base ,SIGNED_INT);	
}
char *utoa(unsigned int num, unsigned int base){
    static char buf[32]={0};
    return num_to_str(num , buf , base , UNSIGNED_INT);	
} 
char *addrtoa(long int addr){
	static char buf[32]={0};
	return num_to_str(addr , buf , 16 , UNSIGNED_INT);
}



#if 0
/*this print can common used for printf and sprintf*/
static int print(char * dest , const char *format, va_list args )
{
  int int_tmp , i;
  char ch_tmp[2] = {0 , 0}; /*second byte is as a stop character */
  char *str_tmp=0;
  char str_out_buf[100];

    for(i=0; format[i]!=0; i++){
        if(format[i]=='%'){
            switch(format[i+1]){
                case 'c':
                case 'C':
                    {
                        ch_tmp[0]    = (char) va_arg(args,int);
                        str_tmp = ch_tmp;
                    } 
                    break;
                case 'd':
                case 'D':
                    {
                        int_tmp = va_arg(args, int);
                        str_tmp = itoa(int_tmp,10);
                    }
                    break;  
                case 'x':
                case 'X':
					{
						int_tmp = va_arg(args , int);
						str_tmp = itoa(int_tmp , 16);
					}
					break;
                case 'S':
                case 's':
                    {
                        str_tmp = va_arg(args, char *);
                    }
                    break;
				case 'u':
				case 'U':
					{
						int_tmp = va_arg(args , int);
						str_tmp = utoa((unsigned int)int_tmp , 10);	
					}
					break;
				case 'p':
				case 'P':
					{
						int_tmp = va_arg(args , long int);
						str_tmp = addrtoa((long int)int_tmp);	
						my_puts("0x"); /*pre-print hex format*/ 
					}
					break;
                default:
                    {   
                   		ch_tmp[0] = format[i];
						str_tmp = ch_tmp; 
                    }
            }
          /* next to char */
          i++;
        }
        else{
             ch_tmp[0] = format[i];
             str_tmp = ch_tmp;
        }
        my_puts(str_tmp);
        strcat(dest , str_tmp);
    }
    va_end(args);
	return i ;
}

int sprintf(char *dest , const char *format , ...)
{
    va_list args;
    va_start( args, format );
    return print( &dest, format, args );
} 
#endif

// reverses a string 'str' of length 'len'
void reverse(char *str, int len)
{
    int i=0, j=len-1, temp;
    while (i<j)
    {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++; j--;
    }
}

  

// Converts a given integer x to string str[].  d is the number
 // of digits required in output. If d is more than the number
 // of digits in x, then 0s are added at the beginning.
int intToStr(int x, char str[], int d)
{
    int i = 0;
    while (x)
    {
        str[i++] = (x%10) + '0';
        x = x/10;
    }
 
    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';
 
    reverse(str, i);
    str[i] = '\0';
    return i;
}


// Converts a floating point number to string.
void ftoa(float n, char *res, int afterpoint)
{
  // Extract integer part
  int ipart = (int)n;

  // Extract floating part
  float fpart = n - (float)ipart;

  // convert integer part to string
  int i = intToStr(ipart, res, 0);

  // check for display option after point
  if (afterpoint != 0)
  {
      res[i] = '.';  // add dot

      // Get the value of fraction part upto given no.
      // of points after dot. The third parameter is needed
      // to handle cases like 233.007
      fpart = fpart * pow(10, afterpoint);

      intToStr((int)fpart, res + i + 1, afterpoint);
  }
}
int math_round(float number)
{
	return number > 0 ?  (number + 0.5f) : (number - 0.5f);
}


