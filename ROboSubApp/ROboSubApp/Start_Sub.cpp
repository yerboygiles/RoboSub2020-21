/*************************************************************
* Authors:        Theodor Giles, Tyler Lucas
* Filename:       Start_Sub.cpp
* Date Created:   7/14/21
* Modifications:  5/12/21 - Began translation from Python3 to C++
**************************************************************/
/*************************************************************
* Overview:
*			Gives the sub tasks to perform
* Input:
*			Requires a text file with instructions for the sub
* Output:
*			TODO: this
**************************************************************/

#include <iostream>
#include "TaskIO.h"

using namespace std;

int main()
{
	// INITIALIZE LEAK REPORT AT EXIT AND WRITE IN CONSOLE WINDOW
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
	_CrtSetReportMode(_CRT_WARN, _CRTDBG_MODE_FILE);
	_CrtSetReportFile(_CRT_WARN, _CRTDBG_FILE_STDERR);

	cout << " ===== ROSA v2.0 =====\n";

	TaskIO Mission = TaskIO("mission.txt", 0, 1, 0); //TODO: make task_io_lib_v2 class TaskIO
	Mission.getTasks();
	Mission.terminate();

	return 0;
}