#ifndef __EXAMPLE_H__
#define __EXAMPLE_H__

#ifdef __cplusplus
#if __cplusplus
extern "C"{
#endif
#endif /* __cplusplus */

#include "fat_filelib.h"

extern void fat_fs_init(FL_FILE *file);
extern int media_init();
extern int media_read(unsigned long sector, unsigned char *buffer, unsigned long sector_count);
extern int media_write(unsigned long sector, unsigned char *buffer, unsigned long sector_count);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */


#endif /* __EXAMPLE_H__ */
