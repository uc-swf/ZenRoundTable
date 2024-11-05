/*
 * THR_Files.h
 *
 *  Created on: Apr 3, 2024
 *      Author: steff
 */

#ifndef INC_SD_THR_H_
#define INC_SD_THR_H_

enum stateReadThrFile {INIT,OPEN_FILE,READ_HEADER,READ_LINE,END_OF_FILE,CLOSE_FILE,ERR};
typedef struct { char fileNames [20][13]; int nFile; } files;
typedef struct { float theta; float rho; enum stateReadThrFile state;} thrValues_t;


void main_thr_files();
void  mountSDCard();
void unmount_sd();
void check_dir();
files list_files();
void sd_read_thr();
void readThrFromSD();
void sd_write_testfile();

enum stateReadThrFile openThrFile();
enum stateReadThrFile readThrHeader();
thrValues_t readThrValuesPerLine();
enum stateReadThrFile closeThrFile();

#endif /* INC_SD_THR_H_ */
