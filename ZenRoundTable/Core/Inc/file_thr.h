/*
 * THR_Files.h
 *
 *  Created on: Apr 3, 2024
 *      Author: steff
 */

#ifndef INC_FILE_THR_H_
#define INC_FILE_THR_H_

#include <stdint.h>

enum stateReadThrFile {INIT,OPEN_FILE,READ_HEADER,READ_LINE,END_OF_FILE,CLOSE_FILE,ERR};
typedef struct { char fileNames [20][13]; int nFile; } files;
typedef struct { float theta; float rho; enum stateReadThrFile state;} thrValues_t;


void main_thr_files();
uint8_t  file_mount();
void file_umount();
void file_chkdir();
files file_list();
void file_read_thr();
void file_read_thr_from_sd();
void sd_write_testfile();

enum stateReadThrFile file_open_thr();
enum stateReadThrFile file_thr_header();
thrValues_t readThrValuesPerLine();
enum stateReadThrFile file_close();

void file_write_open(char* fname);
void file_write_line(char* line);
void file_write_close();


#endif /* INC_FILE_THR_H_ */
