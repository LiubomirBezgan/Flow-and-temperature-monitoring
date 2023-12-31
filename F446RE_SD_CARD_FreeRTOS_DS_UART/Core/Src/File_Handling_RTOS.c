/*
 * File_Handling_RTOS.c
 *
 *  Created on: 30-April-2020
 *      Author: Controllerstech
 */

#include "File_Handling_RTOS.h"
#include "stm32f4xx_hal.h"




/* =============================>>>>>>>> NO CHANGES AFTER THIS LINE =====================================>>>>>>> */

FATFS fs;  // file system
FIL fil; // File
FILINFO fno;
FRESULT fresult;  // result
UINT br, bw;  // File read/write count

/**** capacity related *****/
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;


FRESULT Mount_SD (const TCHAR* path)
{
	FRESULT mounting_fresult;

	if (FR_DISK_ERR == (mounting_fresult = f_mount(&fs, path, 1)) )
	{
		FATFS_UnLinkDriver(USERPath);
		MX_FATFS_Init();
		if ( FR_OK != (mounting_fresult = f_mount(&fs, "", 1)) )
		{
			f_mount(NULL, "", 1);
			return mounting_fresult;
		}
	}
	return mounting_fresult;
}

void Unmount_SD (const TCHAR* path)
{
	fresult = f_mount(NULL, path, 1);
}

/* Start node to be scanned (***also used as work area***) */
FRESULT Scan_SD (char* pat)
{
    DIR dir;
    UINT i;
    char *path = pvPortMalloc(20*sizeof (char));
    sprintf (path, "%s",pat);

    fresult = f_opendir(&dir, path);                       /* Open the directory */
    if (fresult == FR_OK)
    {
        for (;;)
        {
            fresult = f_readdir(&dir, &fno);                   /* Read a directory item */
            if (fresult != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
            if (fno.fattrib & AM_DIR)     /* It is a directory */
            {
            	if (!(strcmp ("SYSTEM~1", fno.fname))) continue;

                i = strlen(path);
                sprintf(&path[i], "/%s", fno.fname);
                fresult = Scan_SD(path);                     /* Enter the directory */
                if (fresult != FR_OK) break;
                path[i] = 0;
            }
        }
        f_closedir(&dir);
    }
    vPortFree(path);
    return fresult;
}

/* Only supports removing files from home directory */
FRESULT Format_SD (void)
{
    DIR dir;
    char *path = pvPortMalloc(20*sizeof (char));
    sprintf (path, "%s","/");

    fresult = f_opendir(&dir, path);                       /* Open the directory */
    if (fresult == FR_OK)
    {
        for (;;)
        {
            fresult = f_readdir(&dir, &fno);                   /* Read a directory item */
            if (fresult != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
            if (fno.fattrib & AM_DIR)     /* It is a directory */
            {
            	if (!(strcmp ("SYSTEM~1", fno.fname))) continue;
            	fresult = f_unlink(fno.fname);
            	if (fresult == FR_DENIED) continue;
            }
            else
            {   /* It is a file. */
               fresult = f_unlink(fno.fname);
            }
        }
        f_closedir(&dir);
    }
    vPortFree(path);
    return fresult;
}




FRESULT Write_File (char *name, char *data)
{

	/**** check whether the file exists or not ****/
	fresult = f_stat (name, &fno);
	if (fresult != FR_OK)
	{
	    return fresult;
	}

	else
	{
	    /* Create a file with read write access and open it */
	    fresult = f_open(&fil, name, FA_OPEN_EXISTING | FA_WRITE);
	    if (fresult != FR_OK)
	    {
	        return fresult;
	    }

	    else
	    {
	    	fresult = f_write(&fil, data, strlen(data), &bw);

	    	/* Close file */
	    	fresult = f_close(&fil);
	    }
	    return fresult;
	}
}

FRESULT Read_File (char *name)
{
	/**** check whether the file exists or not ****/
	fresult = f_stat (name, &fno);
	if (fresult != FR_OK)
	{
	    return fresult;
	}

	else
	{
		/* Open file to read */
		fresult = f_open(&fil, name, FA_READ);

		if (fresult != FR_OK)
		{
		    return fresult;
		}

		/* Read data from the file
		* see the function details for the arguments */

		char *buffer = pvPortMalloc(sizeof(f_size(&fil)));
		fresult = f_read (&fil, buffer, f_size(&fil), &br);
		if (fresult != FR_OK)
		{
			vPortFree(buffer);
		}

		else
		{
			vPortFree(buffer);

			/* Close file */
			fresult = f_close(&fil);
		}
	    return fresult;
	}
}

FRESULT Create_File (char *name)
{
	fresult = f_stat (name, &fno);
	if (fresult == FR_OK)
	{
	    return fresult;
	}
	else
	{
		fresult = f_open(&fil, name, FA_CREATE_ALWAYS|FA_READ|FA_WRITE);
		if (fresult != FR_OK)
		{
		    return fresult;
		}

		fresult = f_close(&fil);
	}
    return fresult;
}

FRESULT Update_File (const char *f_name, File_counter_t * p_counter, const char * f_extension, char *data)
{
	char file_name_buffer[FILE_NAME_LEN];

	/**** check whether the file exists or not ****/
	if (false == p_counter->isfound)
	{
		p_counter->file_counter = 0;
	do {
		(p_counter->file_counter)++;
		sprintf(file_name_buffer, "%s%03u%s", f_name, p_counter->file_counter, f_extension);
	}
	while ( FR_OK == (fresult = f_stat (file_name_buffer, &fno)) );
	p_counter->isfound = true;
	fresult = f_stat (file_name_buffer, &fno);
	}
	else
	{
		sprintf(file_name_buffer, "%s%03u%s", f_name, p_counter->file_counter, f_extension);
	}

	if (FR_NO_FILE == fresult)
	{
		 /* Create a new file with read write access and open it */
		fresult = f_open(&fil, file_name_buffer, FA_CREATE_ALWAYS|FA_READ|FA_WRITE);
	    if (fresult != FR_OK)
	    {
	        return fresult;
	    }

	    /* Writing the table header */
	    const char * table_header = "Time [s],T1 [C],T2 [C],T3 [C],T4 [C],Flow1 [l/min],Flow2 [l/min],Humid [%]\r\n";
	    fresult = f_write(&fil, table_header, strlen (table_header), &bw);

	    /* Writing text */
	    fresult = f_write(&fil, data, strlen (data), &bw);

	    /* Close file */
	    fresult = f_close(&fil);
	}
	else
	{
		 /* Create a file with read write access and open it */
	    fresult = f_open(&fil, file_name_buffer, FA_OPEN_APPEND | FA_WRITE);
	    if (fresult != FR_OK)
	    {
	        return fresult;
	    }

	    /* Writing text */
	    fresult = f_write(&fil, data, strlen (data), &bw);

	    /* Close file */
	    fresult = f_close(&fil);
	}
    return fresult;
}

FRESULT Remove_File (char *name)
{
	/**** check whether the file exists or not ****/
	fresult = f_stat (name, &fno);
	if (fresult != FR_OK)
	{
		return fresult;
	}

	else
	{
		fresult = f_unlink (name);
	}
	return fresult;
}

FRESULT Create_Dir (char *name)
{
    fresult = f_mkdir(name);

    return fresult;
}

void Check_SD_Space (void)
{
    /* Check free space */
    f_getfree("", &fre_clust, &pfs);

    total = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);

    free_space = (uint32_t)(fre_clust * pfs->csize * 0.5);
}

