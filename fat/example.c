#include <stdio.h>
#include "fat_filelib.h"
#include "stm32f4_discovery_sdio_sd.h"
#include "example.h"
int media_init()
{
	SD_Init();
    return 1;
}

int media_read(unsigned long sector, unsigned char *buffer, unsigned long sector_count)
{
    unsigned long i;
	SD_Error error;
    for (i=0;i<sector_count;i++)
    {
        // ...
        // Add platform specific sector (512 bytes) read code here
        //..
        
        while(SD_GetStatus()!=SD_TRANSFER_OK);
	 error=SD_ReadBlock(buffer,sector*512,512);
	while(SD_GetStatus()!=SD_TRANSFER_OK);
        sector ++;
        buffer += 512;
    }

    return 1;
}

int media_write(unsigned long sector, unsigned char *buffer, unsigned long sector_count)
{
    unsigned long i;
	SD_Error error;

    for (i=0;i<sector_count;i++)
    {
        // ...
        // Add platform specific sector (512 bytes) write code here
        //..
        while(SD_GetStatus()!=SD_TRANSFER_OK);
	error=SD_WriteBlock(buffer,sector*512,512);
	while(SD_GetStatus()!=SD_TRANSFER_OK);
        sector ++;
        buffer += 512;
    }

    return 1;
}

void example_fat_fs()
{
    FL_FILE *file;

    // Initialise media
    media_init();

    // Initialise File IO Library
    fl_init();

    // Attach media access functions to library
    if (fl_attach_media(media_read, media_write) != FAT_INIT_OK)
    {
        printf("ERROR: Media attach failed\n");
        return; 
    }

    // List root directory
    fl_listdirectory("/");

    // Create File
    file = fl_fopen("/file.bin", "w");
    if (file)
    {
        // Write some data
        unsigned char data[] = { 1, 2, 3, 4 };
        if (fl_fwrite(data, 1, sizeof(data), file) != sizeof(data))
            printf("ERROR: Write file failed\n");
    }
    else
        printf("ERROR: Create file failed\n");

    // Close file
    fl_fclose(file);

    // Delete File
   // if (fl_remove("/file.bin") < 0)
    //    printf("ERROR: Delete file failed\n");

    // List root directory
    fl_listdirectory("/");

    fl_shutdown();
}
