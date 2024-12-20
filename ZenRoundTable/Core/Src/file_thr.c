/*
 * THR_Files.c
 *
 *  Created on: Apr 3, 2024
 *      Author: steff
 */

#include <file_thr.h>
#include "main.h"
#include "fatfs.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

FATFS FatFs;
FRESULT fres;
FILINFO fno;
FIL fd_write;
DIR dir;
int nfile,ndir;
char *path;

uint8_t file_mount(){

  HAL_Delay(1000);

  fres = f_mount(&FatFs, "", 1);
  if (fres != FR_OK) {
	  debugPrint(DEBUG_THR,"mountSDCard: f_mount error (%i)\r\n", fres);
	return 1;
  }
//  DWORD free_clusters, free_sectors, total_sectors;
//  FATFS* getFreeFs;
//  fres = f_getfree("", &free_clusters, &getFreeFs);
//  if (fres != FR_OK) {
//	  debugPrint(DEBUG_THR,"mountSDCard: f_getfree error (%i)\r\n", fres);
//	while(1);
//  }
//  total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
//  free_sectors = free_clusters * getFreeFs->csize;
//  debugPrint(DEBUG_THR,"mountSDCard: SD card stats:\r\n%10lu KiB total drive space.\r\n%10lu KiB available.\r\n", total_sectors / 2, free_sectors / 2);
  return 0;
}

void file_umount(){
	  f_mount(NULL, "", 0);
	  debugPrint(DEBUG_THR,"unmount_sd: Done...");
}

files file_list(){

	files ret;
	path = "";
    fres = f_opendir(&dir, path);
    if (fres == FR_OK) {
        ret.nFile = 0;
        for (;;) {
        	fres = f_readdir(&dir, &fno);
	        if (fres != FR_OK || fno.fname[0] == 0) break;
	        if (fno.fattrib & AM_DIR) {
	        } else {
	        	//userMessage("Filename: %s, Filesize: %10u  \r\n", fno.fname,fno.fsize);
	            ret.nFile++;
	          }
	    }
    	f_rewinddir(&dir);
    	ret.nFile=0
    			;
    	for (;;) {
    		fres = f_readdir(&dir, &fno);
            if (fres != FR_OK || fno.fname[0] == 0) break;
            if (fno.fattrib & AM_DIR) {
            } else {
            	strcpy(ret.fileNames[ret.nFile], fno.fname);
	            //userMessage("Write %d . File: %s \r\n ",ret.nFile, fno.fname);
	            ret.nFile++;
            }
	     }
		f_closedir(&dir);
		//for ( i=0 ; i<ret.nFile ; i++ ){
			//userMessage("Array Filename: %d. %s \r\n",i,ret.fileNames[i]);
		//}
    } else {
    	debugPrint(DEBUG_THR,"list_files: Failed to open \"%s\". (%u)\r\n", path, fres);
    }
    return ret;
}

enum stateReadThrFile file_open_thr( TCHAR filename[13],FIL* fileObject){

	FRESULT fres = f_open(fileObject, filename, FA_READ);
	if (fres != FR_OK) {
		debugPrint(DEBUG_THR,"openThrFile error (%i) File:%s \r\n",fres, filename);
		return ERR;
	} else {
		debugPrint(DEBUG_THR,"openThrFile: Opened %s successfully \r\n", filename);
		return OPEN_FILE;
	}

}

enum stateReadThrFile file_thr_header(BYTE* readBuffer, FIL* fileObject, BYTE* textHeader){

	const int startLineHeader=1;
	const int endLineHeader=5;

	debugPrint(DEBUG_THR,"readThrHeader: Starts..\r\n");

	for(int i=startLineHeader; i<=endLineHeader; i++){
		debugPrint(DEBUG_THR,"readThrHeader: For %i \r\n",i);
		TCHAR* readRetVal = f_gets((TCHAR*)readBuffer, 100, fileObject);
		if ( ( readRetVal != 0) ) {
			debugPrint(DEBUG_THR,"readThrHeader: Header Line %d  contents: %s \r\n",i, readBuffer);
			strcat((char*)textHeader,(char*)readBuffer);//TODO
			debugPrint(DEBUG_THR,"readThrHeader: textHeader = %s \r\n",textHeader);
			if (i==endLineHeader){
				debugPrint(DEBUG_THR,"readThrHeader: i==endLindeHeader break \r\n");
				break;
			}
		} else {
			debugPrint(DEBUG_THR,"readThrHeader: Error readRetVal= %s \r\n",readRetVal);
			return ERR;
		}
	}
	return READ_HEADER;
}

thrValues_t readThrValuesPerLine(BYTE* readBuffer, FIL* fileObject){

	int positionSpace,positionLF;
	static int line=5;
	thrValues_t readValues;

	char theta[10]="",rho[10]="";

	TCHAR* readRetVal = f_gets((TCHAR*)readBuffer, 100, fileObject);
	debugPrint(DEBUG_THR,"readThrValuesperLine: readRetVal = %s , readBuffer= %s \r\n", readRetVal,readBuffer );
	line++;
	if( ( readBuffer[0] == 10 ) && ( readRetVal != 0) ) {	// Check if result ok and not last Line

		debugPrint(DEBUG_THR,"readThrValuesPerLine: Reached End of File \r\n");
		readValues.state=END_OF_FILE;
		readValues.theta=0.0;
		readValues.rho=0.0;
		line=5;
		return readValues;

	} else if (readRetVal != 0){

		positionSpace = strcspn((char*)readBuffer," ");	// Search for "space", theta and rho are seperated by it
		positionLF = strcspn((char*)readBuffer, "\n");	// Search for "LF");

		strncpy(theta,(char*)readBuffer,positionSpace);	// theta -> characters until "space"
		strncpy(rho,(char*)readBuffer+positionSpace,(positionLF-positionSpace));	// rho -> characters between "space" and "LF"

		readValues.state=READ_LINE;
		readValues.theta=atof(theta);;
		readValues.rho=atof(rho);
		debugPrint(DEBUG_THR,"readThrValuesPerLine:theta and rho processed \r\n");
		debugPrint(DEBUG_THR,"readThrValuesPerLine:theta=%f, rho=%f, line=%i \r\n",readValues.theta,readValues.rho,line);
		return readValues;

	} else if (readRetVal == 0) {

		readValues.state=ERR;
		readValues.theta=0.0;
		readValues.rho=0.0;
		return readValues;
		debugPrint(DEBUG_THR,"readThrValuesPerLine: readRetVal == 0 ERR \r\n");

	}
	return readValues;
}

enum stateReadThrFile file_close(FIL* fileObject){

	f_close(fileObject);
	debugPrint(DEBUG_THR,"closeThrFile: File closed \r\n");
	return CLOSE_FILE;

}


void file_write_open(char* fname)
{

	  fres = f_open(&fd_write, "EmbeTronicX.txt", FA_WRITE | FA_READ | FA_CREATE_ALWAYS);

	  if(fres != FR_OK)
	  {
		  f_close(&fd_write);
		  userMessage("File creation/open Error : (%i)\r\n", fres);
		  return;
	  }
}

void file_write_line(char* line)
{
	int16_t err;

//	if (fd_write==0)
//	{
//		userMessage("File not Open");
//		return;
//	}
	err = f_puts(line, &fd_write);

	if (err)
	{
		userMessage("Error writing");
	}
}

void file_write_close()
{
	f_close(&fd_write);
}
