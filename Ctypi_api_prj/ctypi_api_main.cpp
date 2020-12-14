/************************************************************************
*	File: Ctypi_API.cpp                                                 *
*                                                                       *
*	Maor Mutzafi                                                        *
*************************************************************************/

#include <string>
#include <iostream>
#include <stdio.h>
#include <conio.h>
#include <stdlib.h>
#include "KYFGLib.h"
#include "clserkyi.h"
#include <cstdio>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <stdint.h>
#include <math.h>
#include <time.h>
#include <windows.h>
#include <mmsystem.h>
#include <direct.h>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <dirent.h>
#include <tchar.h>

#ifdef __GNUC__ // _aligned_malloc() implementation for gcc
void* _aligned_malloc(size_t size, size_t alignment)
{
	size_t pageAlign = size % 4096;
	if (pageAlign)
	{
		size += 4096 - pageAlign;
	}

#if(GCC_VERSION <= 40407)
	void* memptr = 0;
	posix_memalign(&memptr, alignment, size);
	return memptr;
#else
	return aligned_alloc(alignment, size);
#endif
}
#endif // #ifdef __GNUC__

#pragma comment(lib, "winmm.lib")

using namespace cv;
using namespace std;

#define MaxWidth 4704
#define MaxHeight 3416
#define T_max 100000
#define MaxBuffNum 64

int 
	Width, Height, OffsetX
	, OffsetY, Gain, 
	t = 0, NFr, NFrBTrg, ta_trigger = 0, Nctypi,
	trigger_ind = 0, t_snd_delay = 0;
double 
	FrameRate, ExposureTime, ExposureTimeMax, 
	snd_delay = 5, dur = 1, sig[T_max * 2];
wchar_t w_snd_dir_path[250], w_snd_file[250];
bool
	Bit8_flag = false, FrameReady = false,
	CaptureMode = false, TriggerOn = true;
char 
	snd_dir_path[] = ".\\Sounds\\*", snd_file[250] = "Sweep.wav", 
	dir_name[250], sim_name[250], sim_only_name[250], params_name[250], 
	sig_im_name[250], sim_name_conf[250], 
	mic_or_tzemer_quads_flag = 't';// 'm' for mic., 'q' for quads, 't' for tzemer

uint8_t** Im8, im_config[MaxWidth * MaxHeight], im_config_ff[MaxWidth * MaxHeight];
uint16_t** Im10, im_config10[MaxWidth * MaxHeight];
uint64_t t_fr[T_max] = { 0 }, t_fr0;
FGHANDLE handle;
STREAM_HANDLE cameraStreamHandle;
CAMHANDLE camHandle;
STREAM_BUFFER_HANDLE streamBufferHandle[MaxBuffNum];

void ctypi_v3(double*);
void ctypi_v3_vel(double*);

int SaveDat(char dat_type);
void Stream_callback_func(STREAM_BUFFER_HANDLE, void*);
void Stream_config_callback_func(STREAM_BUFFER_HANDLE, void*);
void TriggerCallBackFunc(KYFG_AUX_DATA* pData, void* context);

void PlotSignal1(int sl, double[2], double* sig, char title[80], char ax_x_title[80], char ax_y_title[80], int wx, int wy);
Mat PlotSignal2(int sl, double t_min_max[2], double* sig, const char title[80], const char ax_x_title[80], const char ax_y_title[80], int wx, int wy);
void set_exposure(int );
void DFT(int m, double* sr, double* si, double* Sr, double* Si, double* Sabs);
void IDFT(int m, double* Sr, double* Si, double* sr, double* si);
void ShowFrame(int Width, int Height, uint8_t* im_config);
void PlotHist(Mat Im2hist);
void save_sound(int sl, int fs_wav, double* sig, char name[80]);
void take_signle_frame(const char fullframeflag[]);
void choose_snd_file();
void MicRecord();

DWORD WINAPI PlaySoundThread(LPVOID th) { PlaySound(w_snd_file, NULL, SND_ASYNC); return 0; }
DWORD WINAPI MicRecordThread(LPVOID th) { MicRecord(); return 0; }

void main(int argc, char* argv[])
{
	

	if (mic_or_tzemer_quads_flag == 't')
		MoveFileExA("..\\x64\\Release\\Ctypi_api.exe", "..\\x64\\Release\\Ctypi_api_tzemer.exe", MOVEFILE_REPLACE_EXISTING);
	else if (mic_or_tzemer_quads_flag == 'q')
		MoveFileExA("..\\x64\\Release\\Ctypi_api.exe", "..\\x64\\Release\\Ctypi_api_quads.exe", MOVEFILE_REPLACE_EXISTING);
	else if (mic_or_tzemer_quads_flag == 'm')
		MoveFileExA("..\\x64\\Release\\Ctypi_api.exe", "..\\x64\\Release\\Ctypi_api_mic.exe", MOVEFILE_REPLACE_EXISTING);	

	size_t p;
	mbstowcs_s(&p, w_snd_dir_path, snd_dir_path, strlen(snd_dir_path) + 1);
	mbstowcs_s(&p, w_snd_file, snd_dir_path, strlen(snd_dir_path) - 1);

	int detectedCameras;
	size_t frameDataSize, frameDataAligment;
	void* pBuffer[MaxBuffNum];
	printf("\nEnter simulation name (\"0\" for \"not saving mode\", or \"1\" for \"calibration mode\"):"); cin >> sim_only_name; (void)getchar();

	if (sim_only_name[0] == 116 & sim_only_name[1] == 122 & sim_only_name[2] == 101 & sim_only_name[3] == 109 & sim_only_name[4] == 101 & sim_only_name[5] == 114 & sim_only_name[6] == 95
		& sim_only_name[7] == 109 & sim_only_name[8] == 111 & sim_only_name[9] == 100 & sim_only_name[10] == 101 & sim_only_name[11] == 0)
		{ mic_or_tzemer_quads_flag = 't'; 
			printf("Mode has changed to \"tzemer\".\nEnter simulation name (\"0\" for \"not saving mode\", or \"1\" for \"calibration mode\"):");
			cin >> sim_only_name; (void)getchar(); }
	else if (sim_only_name[0] == 109 & sim_only_name[1] == 105 & sim_only_name[2] == 99 & sim_only_name[3] == 95
		& sim_only_name[4] == 109 & sim_only_name[5] == 111 & sim_only_name[6] == 100 & sim_only_name[7] == 101 & sim_only_name[8] == 0)
		{ mic_or_tzemer_quads_flag = 'm'; 
			printf("Mode has changed to \"mic\".\nEnter simulation name (\"0\" for \"not saving mode\", or \"1\" for \"calibration mode\"):");
			cin >> sim_only_name; (void)getchar(); }
	else if (sim_only_name[0] == 113 & sim_only_name[1] == 117 & sim_only_name[2] == 97 & sim_only_name[3] == 100 & sim_only_name[4] == 115 & sim_only_name[5] == 95
		& sim_only_name[6] == 109 & sim_only_name[7] == 111 & sim_only_name[8] == 100 & sim_only_name[9] == 101 & sim_only_name[10] == 0)
		{ mic_or_tzemer_quads_flag = 'm'; 
			printf("Mode has changed to \"quads\".\nEnter simulation name (\"0\" for \"not saving mode\", or \"1\" for \"calibration mode\"):");
			cin >> sim_only_name; (void)getchar(); }

	bool clib_only_flag = (sim_only_name[0] == 49);

	time_t rawtime1; tm timeinfo1; char time_str1[80]; time(&rawtime1); localtime_s(&timeinfo1, &rawtime1); strftime(time_str1, 80, "(%H%M_%d%b%Y)", &timeinfo1);
	if (!clib_only_flag) {
		int jj1; for (jj1 = 0; jj1 < 50; jj1++) {
			char jj1_str[10]; _itoa_s(jj1, jj1_str, 10); char tmp[250];
			snprintf(tmp, 250, "%s%s%s%s", "data/", sim_only_name, "/vid", jj1_str); FILE* f; fopen_s(&f, tmp, "r"); if (!f) break;
		}
		if (jj1) {
			printf("Destination folder already exists? \nContinue working on this folder (note that all parameters have to be the same)? (1 for yes and 0 for no):");
			bool cont_flag;  cin >> cont_flag; if (cont_flag) trigger_ind = jj1; else snprintf(sim_only_name, 250, "%s%s", sim_only_name, time_str1);
		}
	}
	
	(void)_mkdir("data");
	if (sim_only_name[0] != 48 && !clib_only_flag) { snprintf(dir_name, 250, "%s%s%s", "data/", sim_only_name, "/"); (void)_mkdir(dir_name); }

	printf("Ctypi run. Starting with grabber scan&open. Camera scan&open\n");
	// First scan for device
	int n_detected_device;
	KY_DeviceScan(&n_detected_device);
	//KYFG_Scan(NULL, 1);
	// is the device virtual
	KY_DEVICE_INFO pInfo; KY_DeviceInfo(0, &pInfo);
	if (pInfo.isVirtual) { printf("Error: didn't find camera."); (void)getchar(); KYFG_Close(handle); return; }
	// Connect to grabber
	handle = KYFG_Open(0);

	// Setting master and slave FG sync
	/*int mstr_slv = 1;
	if (mstr_slv)
	{
		if (FGSTATUS_OK != KYFG_SetGrabberValueEnum_ByValueName(handle, "TimerSelector", "Timer0")) { printf("Error setting TimerSelector"); (void)getchar(); (void)getchar(); KYFG_Close(handle); return; }
		if (FGSTATUS_OK != KYFG_SetGrabberValueFloat(handle, "TimerDelay", 500)) { printf("Error setting TimerDelay"); (void)getchar(); KYFG_Close(handle); return; }
		if (FGSTATUS_OK != KYFG_SetGrabberValueFloat(handle, "TimerDuration", 500)) { printf("Error setting TimerDuration"); (void)getchar(); KYFG_Close(handle); return; }
		if (FGSTATUS_OK != KYFG_SetGrabberValueEnum_ByValueName(handle, "TimerTriggerSource", "KY_CONTINUOUS")) { printf("Error setting TimerTriggerSource"); (void)getchar(); (void)getchar(); KYFG_Close(handle); return; }
		if (FGSTATUS_OK != KYFG_SetGrabberValueEnum_ByValueName(handle, "CameraTriggerMode", "On")) { printf("Error setting CameraTriggerMode"); (void)getchar(); (void)getchar(); KYFG_Close(handle); return; }
		if (FGSTATUS_OK != KYFG_SetGrabberValueEnum_ByValueName(handle, "CameraTriggerActivation", "AnyEdge")) { printf("Error setting CameraTriggerActivation"); (void)getchar(); (void)getchar(); KYFG_Close(handle); return; }
		if (FGSTATUS_OK != KYFG_SetGrabberValueEnum_ByValueName(handle, "CameraTriggerSource", "KY_TTL_0")) { printf("Error setting CameraTriggerSource"); (void)getchar(); (void)getchar(); KYFG_Close(handle); return; }
		if (FGSTATUS_OK != KYFG_SetGrabberValueEnum_ByValueName(handle, "LineSelector", "KY_TTL_0")) { printf("Error setting LineSelector"); (void)getchar(); (void)getchar(); KYFG_Close(handle); return; }
		if (FGSTATUS_OK != KYFG_SetGrabberValueEnum_ByValueName(handle, "LineMode", "Output")) { printf("Error setting LineMode"); (void)getchar(); (void)getchar(); KYFG_Close(handle); return; }
		if (FGSTATUS_OK != KYFG_SetGrabberValueEnum_ByValueName(handle, "LineSource", "KY_TIMER_ACTIVE_0")) { printf("Error setting LineSource"); (void)getchar(); (void)getchar(); KYFG_Close(handle); return; }

	}
	else
	{
		if (FGSTATUS_OK != KYFG_SetGrabberValueEnum_ByValueName(handle, "CameraTriggerMode", "On")) { printf("Error setting CameraTriggerMode"); (void)getchar(); (void)getchar(); KYFG_Close(handle); return; }
		if (FGSTATUS_OK != KYFG_SetGrabberValueEnum_ByValueName(handle, "CameraTriggerActivation", "AnyEdge")) { printf("Error setting CameraTriggerActivation"); (void)getchar(); (void)getchar(); KYFG_Close(handle); return; }
		if (FGSTATUS_OK != KYFG_SetGrabberValueEnum_ByValueName(handle, "CameraTriggerSource", "KY_TTL_0")) { printf("Error setting CameraTriggerSource"); (void)getchar(); (void)getchar(); KYFG_Close(handle); return; }
		if (FGSTATUS_OK != KYFG_SetGrabberValueEnum_ByValueName(handle, "LineSelector", "KY_TTL_0")) { printf("Error setting LineSelector"); (void)getchar(); (void)getchar(); KYFG_Close(handle); return; }
		if (FGSTATUS_OK != KYFG_SetGrabberValueEnum_ByValueName(handle, "LineMode", "Input")) { printf("Error setting LineMode"); (void)getchar(); (void)getchar(); KYFG_Close(handle); return; }
		if (FGSTATUS_OK != KYFG_SetGrabberValueEnum_ByValueName(handle, "LineSource", "KY_DISABLED")) { printf("Error setting LineSource"); (void)getchar(); (void)getchar(); KYFG_Close(handle); return; }
	}*/

	// scan for connected cameras
	if (FGSTATUS_OK != KYFG_UpdateCameraList(handle, &camHandle, &detectedCameras)) { printf("Error scanning camera"); (void)getchar(); KYFG_Close(handle); return; }
	// open a connection to chosen camera
	if (FGSTATUS_OK != KYFG_CameraOpen2(camHandle, 0)) { printf("Error open camera"); (void)getchar(); KYFG_Close(handle); return; }
	KYFG_SetGrabberValueInt(handle, "CameraSelector", 0);

	KYFG_SetCameraValueEnum_ByValueName(camHandle, "PixelFormat", "Mono8");
	KYFG_SetCameraValueInt(camHandle, "Height", 768);
	KYFG_SetCameraValueInt(camHandle, "Width", 2160);
	KYFG_SetCameraValueInt(camHandle, "OffsetX", 1344);
	KYFG_SetCameraValueInt(camHandle, "OffsetY", 1344);	
	if (FGSTATUS_OK != KYFG_SetCameraValueFloat(camHandle, "AcquisitionFrameRate", 1000))
	{
		printf("Error setting the AcquisitionFrameRate"); (void)getchar(); KYFG_Close(handle); return;
	}

	//KYFG_SetCameraValueFloat(camHandle, "ExposureTime", KYFG_GetCameraValueFloat(camHandle, "ExposureTimeMax"));

	Bit8_flag = (uint8_t)KYFG_GetCameraValueEnum(camHandle, "PixelFormat") == 1;
	FrameRate = KYFG_GetCameraValueFloat(camHandle, "AcquisitionFrameRate");
	ExposureTime = KYFG_GetCameraValueFloat(camHandle, "ExposureTime");
	OffsetY = KYFG_GetCameraValueInt(camHandle, "OffsetY");
	OffsetX = KYFG_GetCameraValueInt(camHandle, "OffsetX");
	Height = KYFG_GetCameraValueInt(camHandle, "Height");
	Width = KYFG_GetCameraValueInt(camHandle, "Width");
	Gain = KYFG_GetCameraValueEnum(camHandle, "AnalogGainLevel") + 1;

	printf("\nSet params.");
	printf("\nLast iter. Width = %d, Height = %d, OffsetX = %d, OffsetY = %d, \nFrameRate = %f, ExposureTime = %f, Gain = %d, %dbits\n", Width, Height, OffsetX, OffsetY, FrameRate, ExposureTime, Gain, Bit8_flag ? 8 : 10);
	printf("\nUse the params. from last iter.? (1 for yes and 0 for no):"); cin >> CaptureMode;

	if (!CaptureMode)
	{

		//Setting PixelFormat
		printf("\nEnter bits num (8 or 10):"); int bit_tmp; cin >> bit_tmp; Bit8_flag = bit_tmp == 8 ? true : false;
		if (Bit8_flag)
		{
			if (FGSTATUS_OK != KYFG_SetCameraValueEnum_ByValueName(camHandle, "PixelFormat", "Mono8")) { printf("Error setting the PixelFormat"); (void)getchar(); KYFG_Close(handle); return; }
		}
		else
		{
			if (FGSTATUS_OK != KYFG_SetCameraValueEnum_ByValueName(camHandle, "PixelFormat", "Mono10")) { printf("Error setting the PixelFormat"); (void)getchar(); KYFG_Close(handle); return; }
		}

		KYFG_SetCameraValueInt(camHandle, "Height", 16);
		printf("\nEnter frame rate (max %f):", KYFG_GetCameraValueFloat(camHandle, "AcquisitionFrameRateMax")); cin >> FrameRate;
		if (FGSTATUS_OK != KYFG_SetCameraValueFloat(camHandle, "AcquisitionFrameRate", FrameRate)) { printf("Error setting FrameRate"); (void)getchar(); KYFG_Close(handle); return; }
		ExposureTimeMax = KYFG_GetCameraValueFloat(camHandle, "ExposureTimeMax");
		int HM = (int)floor(((double)2160 * (double)360) / FrameRate / 16 * (Bit8_flag ? .99 : .93)) * 16;
		printf("\nEnter frame height (max %d):", HM > MaxHeight ? MaxHeight : HM); cin >> Height;
		while (FGSTATUS_OK != KYFG_SetCameraValueInt(camHandle, "Height", Height)) { printf("Error setting the Height. Enter again: "); cin >> Height; }
		printf("\nEnter frame width (max %d) :", MaxWidth); cin >> Width;
		while (FGSTATUS_OK != KYFG_SetCameraValueInt(camHandle, "Width", Width)) { printf("Error setting the Width. Enter again: "); cin >> Width; }
		printf("\nEnter frame offset-x:"); cin >> OffsetX;
		while (FGSTATUS_OK != KYFG_SetCameraValueInt(camHandle, "OffsetX", OffsetX)) { printf("Error setting the OffsetX. Enter again: "); cin >> OffsetX; }
		printf("\nEnter frame offset-y:"); cin >> OffsetY;
		while (FGSTATUS_OK != KYFG_SetCameraValueInt(camHandle, "OffsetY", OffsetY)) { printf("Error setting the OffsetY. Enter again: "); cin >> OffsetY; }
	}

	Mat config_image_o(Height, Width, CV_8UC1, im_config);
	Mat config_image1(Height / 4, Width / 4, CV_8UC1);
	Mat config_image2(Height / 2, Width / 2, CV_8UC1);
	Mat config_image3(Height, Width, CV_8UC1);
	take_signle_frame("FullFrame");
	memcpy(&im_config_ff, &im_config, MaxWidth * MaxHeight);
	Mat full_frame(MaxHeight, MaxWidth, CV_8UC1, im_config_ff);
	Mat full_frame_with_frame(MaxHeight / 8, MaxWidth / 8, CV_8UC1);
	resize(full_frame, full_frame_with_frame, Size(MaxWidth / 8, MaxHeight / 8));
	char full_frame_im_name[250]; snprintf(full_frame_im_name, 250, "%s%s", dir_name, "im_ff.jpg");
	if (sim_only_name[0] != 48 && !clib_only_flag) imwrite(full_frame_im_name, full_frame);
	KYFG_SetCameraValueInt(camHandle, "OffsetX", OffsetX);
	KYFG_SetCameraValueInt(camHandle, "OffsetY", OffsetY);
	

	if (!CaptureMode || clib_only_flag)
	{

		/*KYFG_SetCameraValueEnum_ByValueName(camHandle, "DeviceSerialPortSelector", "RS232_0");
		KYFG_SetCameraValueEnum_ByValueName(camHandle, "DeviceSerialPortBaudRate", "Baud115200");

		uint32_t numSerialPorts = 1; clGetNumSerialPortsEx(camHandle, &numSerialPorts);
		uint32_t portNumber = 7; hSerRef serialRef = 0; clSerialComPortInitEx(camHandle, 1, &portNumber, NULL, &serialRef);
		char briger_dest[] = "C:\\Program Files (x86)\\Birger Engineering\\BEI Device Interface\\BEI Device Interface.exe";
		wchar_t w_briger_dest[250];
		mbstowcs_s(&p, w_briger_dest, briger_dest, strlen(briger_dest) + 1);
		STARTUPINFO si = { sizeof(STARTUPINFO) }; PROCESS_INFORMATION pi;
		CreateProcessW(w_briger_dest, 0, 0, 0, 0, 0, 0, 0, &si, &pi);

		take_signle_frame("FullFrameContinuous");*/

		KYFG_SetCameraValueFloat(camHandle, "AcquisitionFrameRate", FrameRate);
		ExposureTimeMax = KYFG_GetCameraValueFloat(camHandle, "ExposureTimeMax");
		ExposureTime = ExposureTimeMax;

		KYFG_SetCameraValueFloat(camHandle, "AcquisitionFrameRate", 20 > FrameRate ? FrameRate : 20);
		if (FGSTATUS_OK != KYFG_SetCameraValueFloat(camHandle, "ExposureTime", ExposureTimeMax)) { printf("Error setting ExposureTime"); (void)getchar(); (void)getchar(); KYFG_Close(handle); return; }

		printf("\nSet the offset using the arrows, gain using numpad2&8,");
		printf("\nexposure using numpad4&6 (sensative numpad7&9, more sensative numpad1&3)");
		printf("\npress numpad5 to see the params. press \"e\" to set optimal exposure. list. And \"q\" to finish.");

		int kk_loop = 0, kk_period = 10;

		while (true)
		{
			if ((++kk_loop %= kk_period) == 0)
			{
				take_signle_frame("FullFrame");
				memcpy(&im_config_ff, &im_config, MaxWidth* MaxHeight);
				
			}
			resize(full_frame, full_frame_with_frame, Size(MaxWidth / 8, MaxHeight / 8));
			//full_frame.copyTo(full_frame_with_frame);
			line(full_frame_with_frame, Point(OffsetX / 8, OffsetY / 8), Point(OffsetX / 8 + Width / 8, OffsetY / 8), Scalar(255), 2, LINE_8);
			line(full_frame_with_frame, Point(OffsetX / 8, OffsetY / 8), Point(OffsetX / 8, OffsetY / 8 + Height / 8), Scalar(255), 2, LINE_8);
			line(full_frame_with_frame, Point(OffsetX / 8 + Width / 8, OffsetY / 8 + Height / 8), Point(OffsetX / 8 + Width / 8, OffsetY / 8), Scalar(255), 2, LINE_8);
			line(full_frame_with_frame, Point(OffsetX / 8 + Width / 8, OffsetY / 8 + Height / 8), Point(OffsetX / 8, OffsetY / 8 + Height / 8), Scalar(255), 2, LINE_8);
			imshow("Full frame", full_frame_with_frame);
			take_signle_frame("");

			if (Width > 2048) { resize(config_image_o, config_image1, Size(Width / 4, Height / 4)); imshow("Live im.", config_image1); }
			else if (Width > 512) { resize(config_image_o, config_image2, Size(Width / 2, Height / 2)); imshow("Live im.", config_image2); }
			else imshow("Live im.", config_image_o);
			PlotHist(config_image_o);

			waitKey(1); Sleep(10);

			if (GetAsyncKeyState(0x51)) { printf("\nFinal Params.: Width = %d, Height = %d, OffsetX = %d, OffsetY = %d, ExposureTime = %f, Gain = %d", Width, Height, OffsetX, OffsetY, ExposureTime, Gain); break; } // press 'q' to break
			//if (GetAsyncKeyState(0x45)) { printf("\nSetting optimal exposure.");  set_exposure(Gain); printf(" fin.\n");}

			if (GetAsyncKeyState(VK_RIGHT))
				if (Width + OffsetX < MaxWidth - 15) KYFG_SetCameraValueInt(camHandle, "OffsetX", OffsetX += 16);

			if (GetAsyncKeyState(0x44))
				if (Width + OffsetX < MaxWidth - 63) KYFG_SetCameraValueInt(camHandle, "OffsetX", OffsetX += 64);

			if (GetAsyncKeyState(VK_LEFT))
				if (Width + OffsetX > 15) KYFG_SetCameraValueInt(camHandle, "OffsetX", OffsetX -= 16);

			if (GetAsyncKeyState(0x41))
				if (Width + OffsetX > 63) KYFG_SetCameraValueInt(camHandle, "OffsetX", OffsetX -= 64);

			if (GetAsyncKeyState(VK_DOWN))
				if (Height + OffsetY < MaxHeight - 15) KYFG_SetCameraValueInt(camHandle, "OffsetY", OffsetY += 16);

			if (GetAsyncKeyState(0x53))
				if (Height + OffsetY < MaxHeight - 63) KYFG_SetCameraValueInt(camHandle, "OffsetY", OffsetY += 64);

			if (GetAsyncKeyState(VK_UP))
				if (Height + OffsetY > 15) KYFG_SetCameraValueInt(camHandle, "OffsetY", OffsetY -= 16);

			if (GetAsyncKeyState(0x57))
				if (Height + OffsetY > 63) KYFG_SetCameraValueInt(camHandle, "OffsetY", OffsetY -= 64);

			if (GetAsyncKeyState(VK_NUMPAD6)) if (ExposureTime + 1000 < ExposureTimeMax) { ExposureTime += 1000; KYFG_SetCameraValueFloat(camHandle, "ExposureTime", ExposureTime); }
			if (GetAsyncKeyState(VK_NUMPAD4)) if (ExposureTime - 1000 > 0) { ExposureTime -= 1000; KYFG_SetCameraValueFloat(camHandle, "ExposureTime", ExposureTime); }
			if (GetAsyncKeyState(VK_NUMPAD9)) if (ExposureTime + 100 < ExposureTimeMax) { ExposureTime += 100; KYFG_SetCameraValueFloat(camHandle, "ExposureTime", ExposureTime); }
			if (GetAsyncKeyState(VK_NUMPAD7)) if (ExposureTime - 100 > 0) { ExposureTime -= 100; KYFG_SetCameraValueFloat(camHandle, "ExposureTime", ExposureTime); }
			if (GetAsyncKeyState(VK_NUMPAD3)) if (ExposureTime + 1 < ExposureTimeMax) { ExposureTime += 1; KYFG_SetCameraValueFloat(camHandle, "ExposureTime", ExposureTime); }
			if (GetAsyncKeyState(VK_NUMPAD1)) if (ExposureTime - 1 > 0) { ExposureTime -= 1; KYFG_SetCameraValueFloat(camHandle, "ExposureTime", ExposureTime); }

			if (GetAsyncKeyState(VK_NUMPAD8) && Gain < 4) { Gain *= 2; KYFG_SetCameraValueEnum(camHandle, "AnalogGainLevel", Gain - 1); }
			if (GetAsyncKeyState(VK_NUMPAD2) && Gain > 1) { Gain /= 2; KYFG_SetCameraValueEnum(camHandle, "AnalogGainLevel", Gain - 1); }

			if (GetAsyncKeyState(VK_NUMPAD5)) printf("\nCurr. Params.: Width = %d, Height = %d, OffsetX = %d, OffsetY = %d, ExposureTime = %f, Gain = %d", Width, Height, OffsetX, OffsetY, ExposureTime, Gain);
		}
	}

	if (FGSTATUS_OK != KYFG_SetCameraValueFloat(camHandle, "AcquisitionFrameRate", FrameRate)) { printf("Error setting FrameRate"); (void)getchar(); KYFG_Close(handle); return; }
	KYFG_SetCameraValueInt(camHandle, "OffsetX", OffsetX);
	KYFG_SetCameraValueInt(camHandle, "OffsetY", OffsetY);

	if (clib_only_flag) { KYFG_Close(handle); return; }
	
	if (mic_or_tzemer_quads_flag == 'm')
	{
		choose_snd_file();
		printf("\nEnter rec. duration in sec. (max %d):", (int)((double)T_max / FrameRate));
		cin >> dur; NFr = (int)(dur * FrameRate);
	}
	else if (mic_or_tzemer_quads_flag == 'q')
	{
		printf("\nEnter rec. duration in sec. (max %d):", (int)((double)T_max / FrameRate));
		cin >> dur; NFr = (int)(dur * FrameRate);
	}
	else if (mic_or_tzemer_quads_flag == 't')
	{
		printf("\nEnter total num. (max %d) of frames:", T_max); cin >> NFr; NFrBTrg = NFr;
		while (NFrBTrg >= NFr) { printf("\nEnter num. of frames before trigger (max %d):", NFr - 1); cin >> NFrBTrg; }
	}

	// set the file name
	time_t rawtime; tm timeinfo; char time_str[80]; time(&rawtime); localtime_s(&timeinfo, &rawtime); strftime(time_str, 80, "tim%H%M_%d%b%Y", &timeinfo);	char NFr_str[80], Width_str[80], Height_str[80], OffsetX_str[80];
	char OffsetY_str[80], FrameRate_str[80], ExposureTime_str[80], Gain_str[80], NFrBTrg_str[80];
	_itoa_s(NFrBTrg, NFrBTrg_str, 10); _itoa_s(NFr, NFr_str, 10); _itoa_s(Width, Width_str, 10); _itoa_s(Height, Height_str, 10); _itoa_s(OffsetX, OffsetX_str, 10); _itoa_s(Gain, Gain_str, 10);
	_itoa_s(OffsetY, OffsetY_str, 10); _itoa_s(FrameRate, FrameRate_str, 10); _itoa_s(ExposureTime, ExposureTime_str, 10);

	if (mic_or_tzemer_quads_flag == 't')
		snprintf(params_name, 250, "%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s", time_str, "_", "NFr_", NFr_str, "_NFrBTrg_", NFrBTrg_str, "_roi_", Width_str, "X", Height_str, "_Offset_", OffsetX_str, "X", OffsetY_str, "_FR_", FrameRate_str, "_Expt_", ExposureTime_str, "_Gain_", Gain_str, "_", Bit8_flag ? "8" : "10", "bits");
	else
		snprintf(params_name, 250, "%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s", time_str, "_", "NFr_", NFr_str, "_roi_", Width_str, "X", Height_str, "_Offset_", OffsetX_str, "X", OffsetY_str, "_FR_", FrameRate_str, "_Expt_", ExposureTime_str, "_Gain_", Gain_str, "_", Bit8_flag ? "8" : "10", "bits");

	if (sim_only_name[0] != 48) { FILE* qfile; char dir_params_name[250]; snprintf(dir_params_name, 250, "%s%s%s", dir_name, "params_", params_name); fopen_s(&qfile, dir_params_name, "wb"); fwrite(&dir_params_name, 1, 1, qfile); (void)fclose(qfile); }

	// NUC setup - capture dark im. and bright one
	int nuc_flag = 0; // if (sim_only_name[0] != 48) { printf("\nDo you like to take images for NUC? (1 for yes and 0 for no)"); cin >> nuc_flag; } else nuc_flag=0;
	for (int jj = 0; jj < 2 && nuc_flag == 1; jj++)
	{
		if (jj == 0)
		{
			snprintf(sim_name_conf, 250, "%s%s", dir_name, "dark_im");
			printf("\nNUC: dark image. Close the camera cover and press enter to continue."); getchar(); getchar();
		}
		else
		{
			snprintf(sim_name_conf, 250, "%s%s", dir_name, "bright_im");
			printf("\nNUC: bright image. Open the camera cover, and point to bright background and press enter to continue."); getchar();
		}
		// create stream
		if (FGSTATUS_OK != KYFG_StreamCreate(camHandle, &cameraStreamHandle, 0)) { printf("Error creating stream"); getchar(); KYFG_Close(handle); return; }
		//Get frame data size and data aligment
		if (FGSTATUS_OK != KYFG_StreamGetInfo(cameraStreamHandle, KY_STREAM_INFO_PAYLOAD_SIZE, &frameDataSize, NULL, NULL)) { printf("Error stream get info. (frame-data-size)"); getchar(); return; }
		if (FGSTATUS_OK != KYFG_StreamGetInfo(cameraStreamHandle, KY_STREAM_INFO_BUF_ALIGNMENT, &frameDataAligment, NULL, NULL)) { printf("Error stream get info. (frame-data-aligment)"); getchar(); return; }

		//Register stream callback
		if (FGSTATUS_OK != KYFG_StreamBufferCallbackRegister(cameraStreamHandle, Stream_config_callback_func, NULL)) { printf("Error register buffer callback"); getchar(); KYFG_Close(handle); return; }
		// Alloc. buff. mem.
		pBuffer[0] = _aligned_malloc(frameDataSize, frameDataAligment);
		//Announce buffer & queue
		if (FGSTATUS_OK != KYFG_BufferAnnounce(cameraStreamHandle, pBuffer[0], frameDataSize, NULL, streamBufferHandle)) { printf("Error buffer announce"); getchar(); return; }
		if (FGSTATUS_OK != KYFG_BufferToQueue(streamBufferHandle[0], KY_ACQ_QUEUE_INPUT)) { printf("Error buffer to queue"); getchar(); KYFG_Close(handle); return; }
		//Start acquisition
		if (FGSTATUS_OK != KYFG_CameraStart(camHandle, cameraStreamHandle, 0)) { printf("Error camera start"); getchar(); KYFG_Close(handle); return; }
		FrameReady = false;
		while (!FrameReady) { if (GetAsyncKeyState(VK_ESCAPE)) { printf("Closing the camera!"); getchar(); KYFG_Close(handle); return; } } // waiting for acquisition to finish 
							   // TODO: set event
		//Un-register stream callback & stop acquisition
		if (FGSTATUS_OK != KYFG_StreamBufferCallbackUnregister(cameraStreamHandle, Stream_callback_func)) { printf("Error un-register buffer callback"); getchar(); KYFG_Close(handle); return; }
		if (FGSTATUS_OK != KYFG_CameraStop(camHandle)) { printf("Error camera stop"); getchar(); KYFG_Close(handle); return; }
		// Stream delete
		KYFG_StreamDelete(cameraStreamHandle);
		// Free buff. mem.
		_aligned_free(pBuffer[0]);
		if (SaveDat('c') == -1) { getchar(); KYFG_Close(handle); return; }
	}


	// alloc. array for the vid.
	if (Bit8_flag)
	{
		Im8 = (uint8_t**)malloc(NFr * sizeof(uint8_t*));
		if (Im8 == NULL) { printf("Error allocating im. 1"); (void)getchar(); KYFG_Close(handle); return; }
		for (int i = 0; i < NFr; i++) { Im8[i] = (uint8_t*)malloc(Width * Height * sizeof(uint8_t)); if (Im8[i] == NULL) { printf("Error allocating im. 2"); (void)getchar(); KYFG_Close(handle); return; } }
	}
	else
	{
		Im10 = (uint16_t**)malloc(NFr * sizeof(uint16_t*));
		if (Im10 == NULL) { printf("Error allocating im. 1"); (void)getchar(); KYFG_Close(handle); return; }
		for (int i = 0; i < NFr; i++) { Im10[i] = (uint16_t*)malloc(Width * Height * sizeof(uint16_t)); if (Im10[i] == NULL) { printf("Error allocating im. 2"); (void)getchar(); KYFG_Close(handle); return; } }
	}

	// Closing the camera
	if (FGSTATUS_OK != KYFG_CameraClose(camHandle)) { printf("Error closing the camera"); (void)getchar(); KYFG_Close(handle); return; }

	//Close frame-grabber
	if (FGSTATUS_OK != KYFG_Close(handle)) { printf("Error closing grabber"); (void)getchar(); return; }

	bool buff_malloc_flag = false;

	while (true) //loop over the num. of triggers (stopping cond. by user input)
	{
		// Connect to grabber
		handle = KYFG_Open(0);

		if (mic_or_tzemer_quads_flag == 't')
		{
			// setting trigger
			if (FGSTATUS_OK != KYFG_SetGrabberValueEnum_ByValueName(handle, "LineSelector", "KY_TTL_1")) { printf("Error setting grabber line-selector"); (void)getchar(); (void)getchar(); KYFG_Close(handle); return; }
			if (FGSTATUS_OK != KYFG_SetGrabberValueEnum_ByValueName(handle, "LineMode", "Input")) { printf("Error setting grabber line-mode"); (void)getchar(); (void)getchar(); KYFG_Close(handle); return; }
			if (FGSTATUS_OK != KYFG_SetGrabberValueEnum_ByValueName(handle, "LineEventMode", "RisingEdge")) { printf("Error setting grabber line-event-mode"); (void)getchar(); (void)getchar(); KYFG_Close(handle); return; }
			if (FGSTATUS_OK != KYFG_AuxDataCallbackRegister(handle, TriggerCallBackFunc, NULL)) { printf("Error setting trigger callback function"); (void)getchar(); (void)getchar(); KYFG_Close(handle); return; }
		}

		// scan for connected cameras
		if (FGSTATUS_OK != KYFG_UpdateCameraList(handle, &camHandle, &detectedCameras)) { printf("Error scanning camera"); (void)getchar(); KYFG_Close(handle); return; }

		// open a connection to chosen camera
		if (FGSTATUS_OK != KYFG_CameraOpen2(camHandle, 0)) { printf("Error open camera"); (void)getchar(); KYFG_Close(handle); return; }
		
		// Take one frame for validation
		take_signle_frame("");
		if (Width > 2048) { resize(config_image_o, config_image1, Size(Width / 4, Height / 4)); imshow("Validation frame", config_image1); }
		else if (Width > 512) { resize(config_image_o, config_image2, Size(Width / 2, Height / 2)); imshow("Validation frame", config_image2); }
		else imshow("Validation frame", config_image_o); PlotHist(config_image_o); waitKey(1); Sleep(10);
		if (sim_only_name[0] != 48) // Save the frame
			{ char trigger_ind_str[10]; _itoa_s(trigger_ind, trigger_ind_str, 10); snprintf(sig_im_name, 250, "%s%s%s%s", dir_name, "validation_frame", trigger_ind_str, ".jpg"); imwrite(sig_im_name, config_image_o); }

		printf("\nPress enter for to start acquisition or q to quit:"); (void)getchar();
		if (getchar() == 113) { KYFG_Close(handle); if (Bit8_flag) { for (int i = 0; i < NFr; i++) free(Im8[i]); free(Im8); } else { for (int i = 0; i < NFr; i++) free(Im10[i]); free(Im10); } return; }

		// open a connection to chosen camera
		if (FGSTATUS_OK != KYFG_CameraOpen2(camHandle, 0)) { printf("Error open camera"); (void)getchar(); KYFG_Close(handle); return; }
		// create stream
		if (FGSTATUS_OK != KYFG_StreamCreate(camHandle, &cameraStreamHandle, 0)) { printf("Error creating stream"); (void)getchar(); KYFG_Close(handle); return; }
		// get stream info
		if (FGSTATUS_OK != KYFG_StreamGetInfo(cameraStreamHandle, KY_STREAM_INFO_PAYLOAD_SIZE, &frameDataSize, NULL, NULL)) { printf("Error stream get info. (frame-data-size)"); (void)getchar(); KYFG_Close(handle); return; }
		if (FGSTATUS_OK != KYFG_StreamGetInfo(cameraStreamHandle, KY_STREAM_INFO_BUF_ALIGNMENT, &frameDataAligment, NULL, NULL)) { printf("Error stream get info. (frame-data-aligment)"); (void)getchar(); KYFG_Close(handle); return; }

		// buff. alloc. (only once)
		if (!buff_malloc_flag)
			for (int jj = 0; jj < MaxBuffNum; jj++)
				pBuffer[jj] = _aligned_malloc(frameDataSize, frameDataAligment);
		buff_malloc_flag = true;

		// Register stream callback
		if (FGSTATUS_OK != KYFG_StreamBufferCallbackRegister(cameraStreamHandle, Stream_callback_func, NULL)) { printf("Error register buffer callback"); (void)getchar(); KYFG_Close(handle); return; }

		for (int jj = 0; jj < MaxBuffNum; jj++)
			if (FGSTATUS_OK != KYFG_BufferAnnounce(cameraStreamHandle, pBuffer[jj], frameDataSize, NULL, streamBufferHandle + jj))
				{ printf("Error buffer announce"); (void)getchar(); KYFG_Close(handle); return; }

		if (FGSTATUS_OK != KYFG_BufferQueueAll(cameraStreamHandle, KY_ACQ_QUEUE_UNQUEUED, KY_ACQ_QUEUE_INPUT)) { printf("Error BufferQueueAll"); (void)getchar(); KYFG_Close(handle); ; }

		// fan auto
		if (FGSTATUS_OK != KYFG_SetCameraValueEnum_ByValueName(camHandle, "DeviceFanControlMode", "Auto")) { printf("Error DeviceFanControlMode"); (void)getchar(); KYFG_Close(handle); return; }

		// Start acquisition
		if (FGSTATUS_OK != KYFG_CameraStart(camHandle, cameraStreamHandle, 0)) { printf("Error camera start"); (void)getchar(); KYFG_Close(handle); return; }

		if (mic_or_tzemer_quads_flag == 'm')
		{
			printf("\nStart acquisition (in %dsec). ", (int)snd_delay);
			CreateThread(NULL, 0, PlaySoundThread, NULL, 0, NULL);
			CreateThread(NULL, 0, MicRecordThread, NULL, 0, NULL);
			TriggerOn = true; t_snd_delay = 0; ta_trigger = 0;

			while (t_snd_delay < snd_delay * FrameRate) { printf(""); }
			printf("Go!");

			t_fr0 = t_fr[(t - 1) % NFr];
			while (ta_trigger < NFr) { printf(""); } // waiting for acquisition to finish after trigger
			TriggerOn = false;

			printf("\nAcquisition finished. \n");
		}
		else if (mic_or_tzemer_quads_flag == 'q')
		{
			snd_delay = 5;
			printf("\nStart acquisition (in %dsec). ", (int)snd_delay);
			TriggerOn = true; t_snd_delay = 0; ta_trigger = 0;

			while (t_snd_delay < snd_delay * FrameRate) { printf(""); }
			printf("Go!");

			t_fr0 = t_fr[(t - 1) % NFr];

			while (ta_trigger < NFr) { printf(""); } // waiting for acquisition to finish after trigger
			TriggerOn = false;
			printf("\nAcquisition finished. \n");

		}
		else if (mic_or_tzemer_quads_flag == 't')
		{

			snd_delay = 5;
			printf("\nStart acquisition (in %dsec). ", (int)snd_delay);
			TriggerOn = false; t_snd_delay = 0; ta_trigger = 0;

			while (t_snd_delay < snd_delay * FrameRate) { printf(""); }

			TriggerOn = false;
			ta_trigger = 0; while (t < NFrBTrg) { printf(""); } // acquisition before enabling trigger
			printf("\nWaiting for trigger (#%d) or press SPACE for stimulating trigger. To exit press escape.", trigger_ind);
			// Note: Stimulated trigger works only once!
			short last_esc = GetKeyState(VK_ESCAPE), last_space = GetKeyState(VK_SPACE);
			while (ta_trigger < NFr - NFrBTrg) {
				if ((GetKeyState(VK_SPACE) != last_space) & !TriggerOn) { TriggerOn = true; printf("\nStimulated trigger has been pushed."); t_fr0 = t_fr[(t - 1) % NFr]; }
				if (GetKeyState(VK_ESCAPE) != last_esc) { printf("Closing the camera!"); getchar(); getchar(); KYFG_Close(handle); return; }
			} // waiting for acquisition to finish after trigger
			printf("\nAcquisition finished. \n");

		}

		// fan on
		if (FGSTATUS_OK != KYFG_SetCameraValueEnum_ByValueName(camHandle, "DeviceFanControlMode", "On")) { printf("Error DeviceFanControlMode"); (void)getchar(); KYFG_Close(handle); return; }

		//Un-register stream callback & stop acquisition
		if (FGSTATUS_OK != KYFG_StreamBufferCallbackUnregister(cameraStreamHandle, Stream_callback_func)) { printf("Error un-register buffer callback"); (void)getchar(); KYFG_Close(handle); return; }
		//Stop acquisition
		if (FGSTATUS_OK != KYFG_CameraStop(camHandle)) { printf("Error camera stop"); (void)getchar(); KYFG_Close(handle); return; }
		// Stream delete
		KYFG_StreamDelete(cameraStreamHandle);
		//Close frame-grabber
		if (FGSTATUS_OK != KYFG_Close(handle)) { printf("Error closing grabber"); (void)getchar(); return; }

		printf("\nRun ctypi? (1 for yes and 0 for no):"); bool run_ctypi_flag; cin >> run_ctypi_flag;
		if (run_ctypi_flag)
		{
			printf("\nRuning ctypi\n");
			ctypi_v3(sig);

			if ((sim_only_name[0] != 48) & (mic_or_tzemer_quads_flag == 'm'))
			{
				printf("\nSave the signal as sound? (1 for yes and 0 for no):"); bool save_snd_flag; cin >> save_snd_flag;
				if (save_snd_flag)
				{
					char trigger_ind_str[10]; _itoa_s(trigger_ind, trigger_ind_str, 10);
					char dir_snd_name[250]; snprintf(dir_snd_name, 250, "%s%s%s%s%s%s", "data\\", sim_only_name, "\\", "snd", trigger_ind_str, ".wav");
					// save the sound
					save_sound(NFr, FrameRate, sig, dir_snd_name);
					// play the sound
					system(dir_snd_name);
				}
			}

			printf("\nPlot the signal? (1 for yes and 0 for no):"); bool plot_sig_flag; cin >> plot_sig_flag;
			if (plot_sig_flag)
			{
				double t_min_max[2] = { 0 ,NFr / FrameRate };
				Mat cvsig_im = PlotSignal2(NFr, t_min_max, sig, params_name, "Time", "Amp.", 800, 500);
				if (sim_only_name[0] != 48)
				{
					printf("\nSave this im.? (1 for yes and 0 for no):"); bool plot_save_im_sig_flag; cin >> plot_save_im_sig_flag;
					if (plot_save_im_sig_flag)
					{
						char trigger_ind_str[10]; _itoa_s(trigger_ind, trigger_ind_str, 10);
						snprintf(sig_im_name, 250, "%s%s%s%s", dir_name, "fig_sig_t", trigger_ind_str, ".jpg");
						imwrite(sig_im_name, cvsig_im);
					}
				}
			}
			printf("\nPlot the DFT if the signal? (1 for yes and 0 for no):"); bool plot_SIG_flag; cin >> plot_SIG_flag;
			if (plot_SIG_flag)
			{
				double sig_i[T_max] = { 0 }, SIG[2 * T_max], SIG_abs[2 * T_max];
				printf("\nWait for DFT to finish.");
				printf("\nDFT x:"); DFT(NFr, sig + NFr + 1, sig_i, &SIG[0], &SIG[NFr], SIG_abs + NFr);
				printf("\nDFT y:"); DFT(NFr, sig, sig_i, &SIG[0], &SIG[NFr], SIG_abs);
				printf("\nDFT finished.");
				double f_min_max[2] = { -FrameRate / 2 , FrameRate / 2 };
				Mat cvsig_im = PlotSignal2(NFr, f_min_max, SIG_abs, params_name, "Freq.", "Amp.", 800, 500);
				cvsig_im = PlotSignal2(NFr, f_min_max, SIG_abs, params_name, "Freq.", "Amp.", 800, 500);
				if (sim_only_name[0] != 48)
				{
					printf("\nSave this im.? (1 for yes and 0 for no):"); bool plot_save_im_sig_flag; cin >> plot_save_im_sig_flag;
					if (plot_save_im_sig_flag)
					{
						char trigger_ind_str[10]; _itoa_s(trigger_ind, trigger_ind_str, 10);
						snprintf(sig_im_name, 250, "%s%s%s%s", dir_name, "fig_sig_f", trigger_ind_str, ".jpg");
						imwrite(sig_im_name, cvsig_im);
					}
				}
			}
			if (sim_only_name[0] != 48)
			{
				printf("\nSave the signal? (1 for yes and 0 for no):"); bool save_sig_flag; cin >> save_sig_flag;
				if (save_sig_flag) {
					if (SaveDat('t') == -1) { (void)getchar(); KYFG_Close(handle); return; }
					if (SaveDat('s') == -1) { (void)getchar(); KYFG_Close(handle); return; }
				}
			}
		}

		if (sim_only_name[0] != 48)
		{
			bool save_flag; printf("\nSave the seq.? (1 for yes and 0 for no):"); cin >> save_flag;
			if (save_flag) {
				if (SaveDat('t') == -1) { (void)getchar(); KYFG_Close(handle); return; }
				if (SaveDat('v') == -1) { (void)getchar(); KYFG_Close(handle); return; }
			}
			else trigger_ind--;
		}


		t = 0; trigger_ind++;

		bool more_triggers_flag; printf("\nMore triggers? (1 for yes and 0 for no):"); cin >> more_triggers_flag;
		if (!more_triggers_flag) break;
	}

	// Free buff. mem.
	// for (int jj = 0; jj < MaxBuffNum; jj++) _aligned_free(pBuffer[jj]);


	// free vid. array
	if (Bit8_flag)
	{
		for (int i = 0; i < NFr; i++) free(Im8[i]); free(Im8);
	}
	else { for (int i = 0; i < NFr; i++) free(Im10[i]); free(Im10); }
	return;
}

void Stream_callback_func(STREAM_BUFFER_HANDLE streamBufferHandle, void* userContext)
{
	uint8_t* pFrameMemory8 = 0;
	uint16_t* pFrameMemory10 = 0;

	if (ta_trigger == NFr ) return;

	if ((mic_or_tzemer_quads_flag == 'm') | (mic_or_tzemer_quads_flag == 'q'))
	{
		if (TriggerOn)
			if (t_snd_delay == snd_delay * FrameRate)
				ta_trigger++;
			else
				t_snd_delay++;
	}
	else if (mic_or_tzemer_quads_flag == 't')
		if (TriggerOn) ta_trigger++;
		else t_snd_delay++;


	if (!streamBufferHandle) { printf("\nstreamBufferHandle throw!\n"); return; }

	KYFG_BufferGetInfo(streamBufferHandle,
		KY_STREAM_BUFFER_INFO_TIMESTAMP, &t_fr[t], NULL, NULL);
	if (Bit8_flag)
	{
		KYFG_BufferGetInfo(streamBufferHandle,
			KY_STREAM_BUFFER_INFO_BASE, &pFrameMemory8, NULL, NULL);
		memcpy(&Im8[t][0], pFrameMemory8, Width * Height);
	}
	else
	{
		KYFG_BufferGetInfo(streamBufferHandle,
			KY_STREAM_BUFFER_INFO_BASE, &pFrameMemory10, NULL, NULL);
		memcpy(&Im10[t][0], pFrameMemory10, 2 * Width * Height);
	}
	++t %= NFr;
	// return stream buffer to input queue
	KYFG_BufferToQueue(streamBufferHandle, KY_ACQ_QUEUE_INPUT);
}

void Stream_config_callback_func(STREAM_BUFFER_HANDLE streamBufferHandle, void* userContext)
{
	uint8_t* pFrameMemory8 = 0;
	uint16_t* pFrameMemory10 = 0;
	if (Bit8_flag)
	{
		KYFG_BufferGetInfo(streamBufferHandle,
			KY_STREAM_BUFFER_INFO_BASE, &pFrameMemory8, NULL, NULL);
		memcpy(&im_config, pFrameMemory8, Width * Height);
	}
	else
	{
		KYFG_BufferGetInfo(streamBufferHandle,
			KY_STREAM_BUFFER_INFO_BASE, &pFrameMemory10, NULL, NULL);
		memcpy(&im_config10, pFrameMemory10, 2 * Width * Height);
		for (int ii = 0; ii < Width * Height; ii++) im_config[ii] = (uint8_t)(im_config10[ii] / 4);
	}

	FrameReady = true;
	// return stream buffer to input queue
	KYFG_BufferToQueue(streamBufferHandle, KY_ACQ_QUEUE_INPUT);
}

void TriggerCallBackFunc(KYFG_AUX_DATA* pData, void* context)
{
	if (!TriggerOn)
	{
		printf("\nTrigger has been pushed.");
		t_fr0 = t_fr[(t - 1) % NFr];
		TriggerOn = true;
	}
}

void ctypi_v3(double* sig)
{
	Nctypi = NFr - 1;
	const int ind_off = 3;
	int ti, i0, i1, j0, j1;
	const double c1[7] = { 0.0116850998497429921230139626686650444753468036651611328125,-0.0279730819380002923568717676516826031729578971862792968750,0.2239007887600356350166208585505955852568149566650390625000,0.5847743866564433234955799889576155692338943481445312500000,0.2239007887600356350166208585505955852568149566650390625000,-0.0279730819380002923568717676516826031729578971862792968750,0.0116850998497429921230139626686650444753468036651611328125 };
	double px2 = 0, py2 = 0, pxy = 0, ABpx = 0, ABpy = 0, tmpABtx, tmpABty, tmp_px, tmp_py;
	double tmpABtx1, tmpABty1, tmp_px1, tmp_py1;
	printf("press \"q\" (& hold) to quite.\n");

	for (ti = 1; ti < NFr; ti++)
	{
		if (GetAsyncKeyState(0x51)) break;
		px2 = 0; py2 = 0; pxy = 0; ABpx = 0; ABpy = 0;
		printf("Ctypi: %d frames out of %d\r", ti + 1, NFr);
		for (i0 = ind_off; i0 < Width - ind_off; ++i0)
		{
			for (i1 = ind_off; i1 < Height - ind_off; ++i1)
			{
				tmpABtx = 0; tmpABty = 0; tmp_px = 0; tmp_py = 0;
				if (Bit8_flag)
				{
					for (j0 = -ind_off; j0 <= ind_off; ++j0) { tmpABtx += ((double)Im8[(ti + t - 1) % NFr][(i0 + j0) * Height + i1] - (double)Im8[(ti + t) % NFr][(i0 + j0) * Height + i1]) * c1[j0 + ind_off]; }
					for (j1 = -ind_off; j1 <= ind_off; ++j1) { tmpABty += ((double)Im8[(ti + t - 1) % NFr][(i0)*Height + i1 + j1] - (double)Im8[(ti + t) % NFr][(i0)*Height + i1 + j1]) * c1[j1 + ind_off]; }
					tmp_py = ((double)Im8[(ti + t - 1) % NFr][(i0)*Height + i1 + 1] + (double)Im8[(ti + t) % NFr][(i0)*Height + i1 + 1]) / 4 - ((double)Im8[(ti + t - 1) % NFr][(i0)*Height + i1 - 1] + (double)Im8[(ti + t) % NFr][(i0)*Height + i1 - 1]) / 4;
					tmp_px = ((double)Im8[(ti + t - 1) % NFr][(i0 + 1) * Height + i1] + (double)Im8[(ti + t) % NFr][(i0 + 1) * Height + i1]) / 4 - ((double)Im8[(ti + t - 1) % NFr][(i0 - 1) * Height + i1] + (double)Im8[(ti + t) % NFr][(i0 - 1) * Height + i1]) / 4;
				}
				else
				{
					for (j0 = -ind_off; j0 <= ind_off; ++j0) { tmpABtx += ((double)Im10[(ti + t - 1) % NFr][(i0 + j0) * Height + i1] - (double)Im10[(ti + t) % NFr][(i0 + j0) * Height + i1]) * c1[j0 + ind_off]; }
					for (j1 = -ind_off; j1 <= ind_off; ++j1) { tmpABty += ((double)Im10[(ti + t - 1) % NFr][(i0)*Height + i1 + j1] - (double)Im10[(ti + t) % NFr][(i0)*Height + i1 + j1]) * c1[j1 + ind_off]; }
					tmp_py = ((double)Im10[(ti + t - 1) % NFr][(i0)*Height + i1 + 1] + (double)Im10[(ti + t) % NFr][(i0)*Height + i1 + 1]) / 4 - ((double)Im10[(ti + t - 1) % NFr][(i0)*Height + i1 - 1] + (double)Im10[(ti + t) % NFr][(i0)*Height + i1 - 1]) / 4;
					tmp_px = ((double)Im10[(ti + t - 1) % NFr][(i0 + 1) * Height + i1] + (double)Im10[(ti + t) % NFr][(i0 + 1) * Height + i1]) / 4 - ((double)Im10[(ti + t - 1) % NFr][(i0 - 1) * Height + i1] + (double)Im10[(ti + t) % NFr][(i0 - 1) * Height + i1]) / 4;
				}
				px2 += tmp_px * tmp_px;
				py2 += tmp_py * tmp_py;
				pxy += tmp_px * tmp_py;
				ABpx += tmpABtx * tmp_px;
				ABpy += tmpABty * tmp_py;
			}
		}
		*(sig + ti - 1) = (py2 * ABpx - pxy * ABpy) / (px2 * py2 - pxy * pxy);
		*(sig + ti + NFr - 2) = (px2 * ABpy - pxy * ABpx) / (px2 * py2 - pxy * pxy);
	}
}

void ctypi_v3_vel(double* sig)
{
	int ti, i0, i1, j0, j1;
	double px2 = 0, py2 = 0, pxy = 0, tmp_impx, tmp_impy, diff_im, px, py;
	printf("press \"q\" (& hold) to quite.\n");

	for (ti = 0; ti < NFr; ti++)
	{

		if (GetAsyncKeyState(0x51)) break;
		printf("Ctypi: %d frames out of %d\r", ti + 1, NFr);
		tmp_impx = 0; tmp_impy = 0; px2 = 0; py2 = 0; pxy = 0;
		for (i0 = 0; i0 < Width - 2; ++i0)
			for (i1 = 0; i1 < Height - 2; ++i1)
			{
				if (Bit8_flag)
				{

					py = (double)Im8[(ti + t) % NFr][(i0 + 1) * Height + i1 + 2] / 2 - (double)Im8[(ti + t) % NFr][(i0 + 1) * Height + i1 + 0] / 2;
					px = (double)Im8[(ti + t) % NFr][(i0 + 2) * Height + i1 + 1] / 2 - (double)Im8[(ti + t) % NFr][(i0 + 0) * Height + i1 + 1] / 2;
					diff_im = (double)Im8[(ti + t + 1) % NFr][(i0 + 1) * Height + i1 + 1] - (double)Im8[(ti + t) % NFr][(i0 + 1) * Height + i1 + 1];
				}
				else
				{
					py = (double)Im10[(ti + t) % NFr][(i0 + 1) * Height + i1 + 2] / 2 - (double)Im10[(ti + t) % NFr][(i0 + 1) * Height + i1 + 0] / 2;
					px = (double)Im10[(ti + t) % NFr][(i0 + 2) * Height + i1 + 1] / 2 - (double)Im10[(ti + t) % NFr][(i0 + 0) * Height + i1 + 1] / 2;
					diff_im = (double)Im10[(ti + t + 1) % NFr][(i0 + 1) * Height + i1 + 1] - (double)Im10[(ti + t) % NFr][(i0 + 1) * Height + i1 + 1];
				}
				px2 += px * px;
				py2 += py * py;
				pxy += px * py;

				tmp_impx += diff_im * px;
				tmp_impy += diff_im * py;

			}
		*(sig + ti) = (py2 * tmp_impx - pxy * tmp_impy) / (px2 * py2 - pxy * pxy);
		*(sig + ti + NFr) = (px2 * tmp_impy - pxy * tmp_impx) / (px2 * py2 - pxy * pxy);
	}
}

int SaveDat(char dat_type) // 'v' for video, 'i' for image, 's' for signal, 'q' for int signal, 't' for frame time
{
	FILE* qfile;
	int dat_cell_sz;
	char file_name[250];
	char trigger_ind_str[10]; _itoa_s(trigger_ind, trigger_ind_str, 10);
	if (dat_type == 'v')
	{
		snprintf(file_name, 250, "%s%s%s", dir_name, "vid", trigger_ind_str);
		fopen_s(&qfile, file_name, "wb");
		if (qfile == NULL) { printf("\nError: cannot open video file. Exiting...\n"); (void)getchar(); return -1; }         /* error! */
		for (int ii = 0; ii < NFr; ii++)
			for (int jj = 0; jj < Width * Height; jj++)
				if (Bit8_flag) { if (fwrite(&Im8[(ii + t) % NFr][jj], sizeof(uint8_t), 1, qfile) != 1) { (void)fclose(qfile); printf("\nError: while writing to video file. Exiting...\n"); (void)getchar(); return -1; } }
				else if (fwrite(&Im10[(ii + t) % NFr][jj], sizeof(uint16_t), 1, qfile) != 1) { (void)fclose(qfile); printf("\nError: while writing to video file. Exiting...\n"); (void)getchar(); return -1; }
	}
	else if (dat_type == 'i')
	{
		snprintf(file_name, 250, "%s%s%s", dir_name, "im", trigger_ind_str);
		fopen_s(&qfile, file_name, "wb");
		if (qfile == NULL) { printf("\nError: cannot open image file. Exiting...\n"); (void)getchar(); return -1; }         /* error! */
		if (Bit8_flag ? fwrite(&Im8[0][0], sizeof(uint8_t), Width * Height, qfile) : fwrite(&Im10[0][0], sizeof(uint16_t), Width * Height, qfile) != Width * Height)
		{
			(void)fclose(qfile); printf("\nError: while writing to image file. Exiting...\n"); (void)getchar(); return -1;
		}
	}
	else if (dat_type == 'f')
	{
		snprintf(file_name, 250, "%s%s", dir_name, "im_ff");
		fopen_s(&qfile, file_name, "wb");
		if (qfile == NULL) { printf("\nError: cannot open image (ff) file. Exiting...\n"); (void)getchar(); return -1; }         /* error! */
		if (fwrite(im_config, sizeof(uint8_t), MaxWidth * MaxHeight, qfile) != MaxWidth * MaxHeight)
		{ (void)fclose(qfile); printf("\nError: while writing to image (ff) file. Exiting...\n"); (void)getchar(); return -1; }
	}
	else if (dat_type == 'c')
	{
		fopen_s(&qfile, sim_name_conf, "wb");
		if (qfile == NULL) { printf("\nError: cannot open image file. Exiting...\n"); (void)getchar(); return -1; }         /* error! */
		if (Bit8_flag) { if (fwrite(im_config, sizeof(uint8_t), Width * Height, qfile) != Width * Height) { (void)fclose(qfile); printf("\nError: while writing to image file. Exiting...\n"); (void)getchar(); return -1; } }
		else if (fwrite(im_config10, sizeof(uint16_t), Width * Height, qfile) != Width * Height) { (void)fclose(qfile); printf("\nError: while writing to image file. Exiting...\n"); (void)getchar(); return -1; }
	}
	else if (dat_type == 's')
	{
		snprintf(file_name, 250, "%s%s%s", dir_name, "sig", trigger_ind_str);
		fopen_s(&qfile, file_name, "wb");
		if (qfile == NULL) { printf("\nError: cannot open signal file. Exiting...\n"); (void)getchar(); return -1; }         /* error! */
		if (fwrite(sig, sizeof(double), 2 * Nctypi, qfile) != 2 * Nctypi)
		{
			(void)fclose(qfile); printf("\nError: while writing to signal file. Exiting...\n"); (void)getchar(); return -1;
		}
	}
	else if (dat_type == 't')
	{
		snprintf(file_name, 250, "%s%s%s", dir_name, "t_fr", trigger_ind_str);
		fopen_s(&qfile, file_name, "wb");
		if (qfile == NULL) { printf("\nError: cannot open frame time file. Exiting...\n"); (void)getchar(); return -1; }         /* error! */
		for (int ii = 0; ii < NFr; ii++)
			if (fwrite(&t_fr[(ii + t) % NFr], sizeof(uint64_t), 1, qfile) != 1)
			{
				(void)fclose(qfile); printf("\nError: while writing to frame time file. Exiting...\n"); (void)getchar(); return -1;
			}
		fwrite(&t_fr0, sizeof(uint64_t), 1, qfile);
	}
	else
	{
		printf("\nError: saving data. unknown data type. Exiting...\n"); (void)getchar(); return -1;
	}

	return fclose(qfile);
}

void PlotHist(Mat Im2hist)
{
	int hist_sz = 256;
	float range[] = { 0 , 256 };
	const float* hist_range = { range };
	bool uniform = true, accumulate = false;
	Mat hist;
	calcHist(&Im2hist, 1, 0, Mat(), hist, 1, &hist_sz, &hist_range, uniform, accumulate);
	int hist_w = 512, hist_h = 400;
	int bin_w = cvRound((double)hist_w / hist_sz);
	Mat hist_im(hist_h, hist_w, CV_8UC1, Scalar(0, 0, 0));
	normalize(hist, hist, 0, hist_im.rows, NORM_MINMAX, -1, Mat());
	for (int ii = 1; ii < hist_sz; ii++)
	{
		line(hist_im,
			Point(bin_w * (ii - 1), hist_h - cvRound(hist.at<float>(ii - 1))),
			Point(bin_w * (ii), hist_h - cvRound(hist.at<float>(ii))),
			Scalar(255, 255, 255), 2, 8, 0);
	}
	imshow("Histogram", hist_im);
	waitKey(1); Sleep(10);

	return;
}

void set_exposure(int gain)
{
	int max_pxl = (gain == 1) ? 225 : ((gain == 2) ? 200 : 180);
	int NmaxVal;
	double ExpStep = 1024;
	bool lst_iter_dec = false;

	KYFG_SetCameraValueFloat(camHandle, "ExposureTime", ExposureTimeMax);

	while (true)
	{
		if (GetAsyncKeyState(0x51)) break;
		take_signle_frame("FullFrame");
		NmaxVal = 0;
		for (int ii = 0; ii < MaxHeight * MaxWidth; ii++)
			if (im_config[ii] > max_pxl) NmaxVal++;
		//printf_s("max val = %d, ExposureTime = %f\n", maxVal, ExposureTime);

		if (NmaxVal > (Height * Width / 500))
		{
			KYFG_SetCameraValueFloat(camHandle, "ExposureTime", ExposureTime -= ExpStep); 
			lst_iter_dec = true;
		}
		else if ((ExposureTime + ExpStep < ExposureTimeMax) & !lst_iter_dec)
		{
			KYFG_SetCameraValueFloat(camHandle, "ExposureTime", ExposureTime += ExpStep);
			lst_iter_dec = false;
		}
		else { ExpStep /= 2; lst_iter_dec = false;}

		if (ExpStep < .5) break;
	}

}
void choose_snd_file()
{
	printf("\n\nSound files list:\n");
	WIN32_FIND_DATA FindFileData;
	HANDLE h = FindFirstFileW(w_snd_dir_path, &FindFileData);
	FindNextFile(h, &FindFileData);

	int indii = 0;

	while (FindNextFile(h, &FindFileData))
	{
		_tprintf(TEXT("%d. %s\n"), ++indii, FindFileData.cFileName);
	}
	FindClose(h);

	printf("Choose sound file from the above list (enter number):");
	int fl_no; cin >> fl_no; (void)getchar();

	h = FindFirstFileW(w_snd_dir_path, &FindFileData);
	FindNextFile(h, &FindFileData);
	for (int ii = 0; ii < fl_no; ii++, FindNextFile(h, &FindFileData)) {}
	wcscat_s(w_snd_file, FindFileData.cFileName);
	FindClose(h);
	_tprintf(TEXT("\nThe choosen sound file is %s\n"), FindFileData.cFileName);
}


void take_signle_frame(const char fullframeflag[])
{
	size_t frameDataSize, frameDataAligment;
	void* pBuffer;
	int Height_tmp = KYFG_GetCameraValueInt(camHandle, "Height");
	int Width_tmp = KYFG_GetCameraValueInt(camHandle, "Width");
	int OffsetX_tmp = KYFG_GetCameraValueInt(camHandle, "OffsetX");
	int OffsetY_tmp = KYFG_GetCameraValueInt(camHandle, "OffsetY");
	if (strlen(fullframeflag) > 1) { Height = MaxHeight; Width = MaxWidth; OffsetX = 0; OffsetY = 0; }

	KYFG_SetCameraValueInt(camHandle, "Height", Height);
	KYFG_SetCameraValueInt(camHandle, "Width", Width);
	KYFG_SetCameraValueInt(camHandle, "OffsetX", OffsetX);
	KYFG_SetCameraValueInt(camHandle, "OffsetY", OffsetY);
	// create stream
	if (FGSTATUS_OK != KYFG_StreamCreate(camHandle, &cameraStreamHandle, 0)) { printf("Error creating stream"); (void)getchar(); KYFG_Close(handle); return; }
	//Get frame data size and data aligment
	if (FGSTATUS_OK != KYFG_StreamGetInfo(cameraStreamHandle, KY_STREAM_INFO_PAYLOAD_SIZE, &frameDataSize, NULL, NULL)) { printf("Error stream get info. (frame-data-size)"); (void)getchar(); return; }
	if (FGSTATUS_OK != KYFG_StreamGetInfo(cameraStreamHandle, KY_STREAM_INFO_BUF_ALIGNMENT, &frameDataAligment, NULL, NULL)) { printf("Error stream get info. (frame-data-aligment)"); (void)getchar(); return; }

	//Register stream callback
	if (FGSTATUS_OK != KYFG_StreamBufferCallbackRegister(cameraStreamHandle, Stream_config_callback_func, NULL)) { printf("Error register buffer callback"); (void)getchar(); KYFG_Close(handle); return; }
	// Alloc. buff. mem.
	pBuffer = _aligned_malloc(frameDataSize, frameDataAligment);
	//Announce buffer & queue
	if (FGSTATUS_OK != KYFG_BufferAnnounce(cameraStreamHandle, pBuffer, frameDataSize, NULL, streamBufferHandle)) { printf("Error buffer announce"); (void)getchar(); return; }
	if (FGSTATUS_OK != KYFG_BufferToQueue(streamBufferHandle[0], KY_ACQ_QUEUE_INPUT)) { printf("Error buffer to queue"); (void)getchar(); KYFG_Close(handle); return; }
	//Start acquisition
	if (FGSTATUS_OK != KYFG_CameraStart(camHandle, cameraStreamHandle, 0)) { printf("Error camera start"); (void)getchar(); KYFG_Close(handle); return; }
	
	if (strlen(fullframeflag) > 10)
	{
		Mat config_image_o(Height, Width, CV_8UC1, im_config);
		Mat config_image1(Height / 4, Width / 4, CV_8UC1);
		printf("\nSet the focus and press \"q\".\n");
		while (true)
		{

			FrameReady = false;
			while (!FrameReady) { if (GetAsyncKeyState(VK_ESCAPE)) { printf("Closing the camera!"); (void)getchar(); KYFG_CameraStop(camHandle);  KYFG_Close(handle); return; } } // waiting for acquisition to finish 

			cv::resize(config_image_o, config_image1, Size(Width / 4, Height / 4));
			cv::imshow("Live im.", config_image1);
			PlotHist(config_image_o);

			cv::waitKey(1); Sleep(5);

			if (GetAsyncKeyState(0x51)) break; // press 'q' to break

		}

	}
	else
	{
		FrameReady = false;
		while (!FrameReady) { if (GetAsyncKeyState(VK_ESCAPE)) { printf("Closing the camera!"); (void)getchar(); KYFG_CameraStop(camHandle);  KYFG_Close(handle); return; } } // waiting for acquisition to finish 
	}

	//Un-register stream callback & stop acquisition
	if (FGSTATUS_OK != KYFG_StreamBufferCallbackUnregister(cameraStreamHandle, Stream_callback_func)) { printf("Error un-register buffer callback"); (void)getchar(); KYFG_Close(handle); return; }
	if (FGSTATUS_OK != KYFG_CameraStop(camHandle)) { printf("Error camera stop"); (void)getchar(); KYFG_Close(handle); return; }
	// Stream delete
	KYFG_StreamDelete(cameraStreamHandle);
	// Free buff. mem.
	_aligned_free(pBuffer);

	Height = Height_tmp; Width = Width_tmp; OffsetX = OffsetX_tmp; OffsetY = OffsetY_tmp;

	KYFG_SetCameraValueInt(camHandle, "Height", Height);
	KYFG_SetCameraValueInt(camHandle, "Width", Width);

	KYFG_SetCameraValueInt(camHandle, "OffsetX", OffsetX);
	KYFG_SetCameraValueInt(camHandle, "OffsetY", OffsetY);
	if (FGSTATUS_OK != KYFG_SetCameraValueFloat(camHandle, "AcquisitionFrameRate", FrameRate)) { printf("Error setting FrameRate"); (void)getchar(); KYFG_Close(handle); return; }

}


void MicRecord()
{
	//printf("\n\nStart rec.\n\n");
	int sampleRate = 44100;
	int dur_sampleRate = (int)((dur + snd_delay) * (double)sampleRate);
	short int* waveIn = (short int*)malloc(dur_sampleRate * sizeof(short int));
	// 'short int' is a 16-bit type; I request 16-bit samples below
							 // for 8-bit capture, you'd use 'unsigned char' or 'BYTE' 8-bit     types

	HWAVEIN      hWaveIn;
	MMRESULT result;

	WAVEFORMATEX pFormat;
	pFormat.wFormatTag = WAVE_FORMAT_PCM;     // simple, uncompressed format
	pFormat.nChannels = 1;                    //  1=mono, 2=stereo
	pFormat.nSamplesPerSec = sampleRate;      // 8.0 kHz, 11.025 kHz, 22.05 kHz, and 44.1 kHz
	pFormat.nAvgBytesPerSec = sampleRate * 2;   // =  nSamplesPerSec × nBlockAlign
	pFormat.nBlockAlign = 2;                  // = (nChannels × wBitsPerSample) / 8
	pFormat.wBitsPerSample = 16;              //  16 for high quality, 8 for telephone-grade
	pFormat.cbSize = 0;

	// Specify recording parameters

	result = waveInOpen(&hWaveIn, WAVE_MAPPER, &pFormat,
		0L, 0L, WAVE_FORMAT_DIRECT);

	WAVEHDR      WaveInHdr;
	// Set up and prepare header for input
	WaveInHdr.lpData = (LPSTR)waveIn;
	WaveInHdr.dwBufferLength = dur_sampleRate * 2;
	WaveInHdr.dwBytesRecorded = 0;
	WaveInHdr.dwUser = 0L;
	WaveInHdr.dwFlags = 0L;
	WaveInHdr.dwLoops = 0L;
	waveInPrepareHeader(hWaveIn, &WaveInHdr, sizeof(WAVEHDR));

	// Insert a wave input buffer
	result = waveInAddBuffer(hWaveIn, &WaveInHdr, sizeof(WAVEHDR));


	// Commence sampling input
	result = waveInStart(hWaveIn);

	do {} while (waveInUnprepareHeader(hWaveIn, &WaveInHdr, sizeof(WAVEHDR)) == WAVERR_STILLPLAYING);
	waveInClose(hWaveIn);

	if (sim_only_name[0] != 48)
	{
		char file_name[250];
		char trigger_ind_str[10]; _itoa_s(trigger_ind, trigger_ind_str, 10);
		snprintf(file_name, 250, "%s%s%s", dir_name, "mic_data", trigger_ind_str);
		FILE* qfile;
		fopen_s(&qfile, file_name, "wb");
		if (qfile == NULL) { printf("\nError: open snd file for mic. data. Exiting...\n"); (void)getchar(); return; }         /* error! */
		if (fwrite(WaveInHdr.lpData, sizeof(short int), dur_sampleRate, qfile) != dur_sampleRate) { printf("\nError: writing mic. data. Exiting...\n"); (void)getchar(); return; }
		fclose(qfile);
	}
	//printf("\n\nfin. rec.\n\n");

	return;

}


void circ_shift(int m, double* s);

namespace little_endian_io
{
	template <typename Word>
	std::ostream& write_word(std::ostream& outs, Word value, unsigned size = sizeof(Word))
	{
		for (; size; --size, value >>= 8)
			outs.put(static_cast <char> (value & 0xFF));
		return outs;
	}
}
using namespace little_endian_io;

void PlotSignal1(int sl, double t_min_max[2], double* sig, char title[80], char ax_x_title[80], char ax_y_title[80], int wx, int wy)
{
	int b = 160; // boudaries sz
	int npt[] = { 4 };
	double sig_max = -1000, sig_min = 1000;
	Mat sig_image = Mat::zeros(wy + b, wx + b, CV_8UC3);
	// draw boundaries
	Point bound0[1][4] = { Point(0, 0) ,Point(wx + b, 0) ,Point(wx + b, b / 2) ,Point(0, b / 2) };
	Point bound1[1][4] = { Point(0, 0) ,Point(0 , wy + b) ,Point(b / 2 , wy + b) ,Point(b / 2 , 0) };
	Point bound2[1][4] = { Point(wx + b, wy + b) ,Point(0, wy + b) ,Point(0, wy + b / 2) ,Point(wx + b, wy + b / 2) };
	Point bound3[1][4] = { Point(wx + b, wy + b) ,Point(wx + b,0) ,Point(wx + b / 2,0) ,Point(wx + b / 2,wy + b) };
	const Point* ppt0[1] = { bound0[0] }, * ppt1[1] = { bound1[0] }, * ppt2[1] = { bound2[0] }, * ppt3[1] = { bound3[0] };
	fillPoly(sig_image, ppt0, npt, 1, Scalar(150, 150, 150), LINE_8);
	fillPoly(sig_image, ppt1, npt, 1, Scalar(150, 150, 150), LINE_8);
	fillPoly(sig_image, ppt2, npt, 1, Scalar(150, 150, 150), LINE_8);
	fillPoly(sig_image, ppt3, npt, 1, Scalar(150, 150, 150), LINE_8);
	// draw legend
	Point legend[1][4] = { Point(wx + b / 2, 0) , Point(wx - b / 2, 0) , Point(wx - b / 2, b / 2) , Point(wx + b / 2, b / 2) };
	const Point* ppt_leg[1] = { legend[0] };
	fillPoly(sig_image, ppt_leg, npt, 1, Scalar(0, 0, 0), LINE_8);
	line(sig_image, Point(wx - .4 * b, b / 4), Point(wx, b / 4), Scalar(255, 255, 255), 2, LINE_8);
	putText(sig_image, "sig.", cvPoint(wx, b / 4),
		FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255, 255, 255), 1, CV_AA);
	// find min/max
	for (int ii = 0; ii < sl; ii++) sig_max = sig[ii] > sig_max ? sig[ii] : sig_max; // find sig. max
	for (int ii = 0; ii < sl; ii++) sig_min = sig[ii] < sig_min ? sig[ii] : sig_min; // find sig. min

	double factor = (double)wy / (sig_max - sig_min) * .975;
	double bias = wy / 2 + factor * sig_max / 2 + factor * sig_min / 2 + b / 2;

	// draw graph & axes & title
	for (int ii = 0; ii < sl - 1; ii++) line(sig_image, Point((double)ii / sl * wx + b / 2, bias - factor * sig[ii]), Point((double)ii / sl * wx + b / 2 + 1, bias - factor * sig[ii + 1]), Scalar(255, 255, 255), 1, LINE_8);
	line(sig_image, Point(b / 2, bias), Point(wx + b / 2, bias), Scalar(0, 0, 255), 2, LINE_8);
	int t0 = (int)b / 2 - wx / (t_min_max[1] - t_min_max[0]) * t_min_max[0];
	line(sig_image, Point(t0, b / 2), Point(t0, wy + b / 2), Scalar(0, 0, 255), 2, LINE_8);
	putText(sig_image, ax_x_title, cvPoint(wx + b / 2, bias + 20),
		FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255, 255, 255), 1, CV_AA);
	putText(sig_image, ax_y_title, cvPoint(20, b / 4),
		FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255, 255, 255), 1, CV_AA);
	putText(sig_image, title, cvPoint(b / 2 + 40, b / 4),
		FONT_HERSHEY_TRIPLEX, 0.8, cvScalar(255, 255, 255), 1, CV_AA);
	// plot x-y lim
	char s_max_str[50], s_min_str[50], t_max_str[50], t_min_str[50];
	snprintf(s_max_str, 50, "%f", sig_max); snprintf(s_min_str, 50, "%f", sig_min);
	snprintf(t_max_str, 50, "%f", t_min_max[1]); snprintf(t_min_str, 50, "%f", t_min_max[0]);
	putText(sig_image, "0", cvPoint(b / 2 - 40, bias + 0),
		FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255, 255, 255), 1, CV_AA);
	putText(sig_image, "0", cvPoint(t0, wy + b / 2 + 20),
		FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255, 255, 255), 1, CV_AA);
	putText(sig_image, s_max_str, cvPoint(0, b / 2),
		FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255, 255, 255), 1, CV_AA);
	putText(sig_image, s_min_str, cvPoint(0, wy + b / 2),
		FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255, 255, 255), 1, CV_AA);
	putText(sig_image, t_min_str, cvPoint(b / 2, wy + b / 2 + 20),
		FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255, 255, 255), 1, CV_AA);
	putText(sig_image, t_max_str, cvPoint(wx + b / 2, wy + b / 2 + 20),
		FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255, 255, 255), 1, CV_AA);

	imshow(title, sig_image);
	moveWindow(title, 0, 0);
}
Mat PlotSignal2(int sl, double t_min_max[2], double* sig, const char title[80], const char ax_x_title[80], const char ax_y_title[80], int wx, int wy)
{
	int b = 160; // boudaries
	int npt[] = { 4 };
	double sig_max = -1000, sig_min = 1000;
	Mat sig_image = Mat::zeros(wy + b, wx + b, CV_8UC3);
	// draw boundaries
	Point bound0[1][4] = { Point(0, 0) ,Point(wx + b, 0) ,Point(wx + b, b / 2) ,Point(0, b / 2) };
	Point bound1[1][4] = { Point(0, 0) ,Point(0 , wy + b) ,Point(b / 2 , wy + b) ,Point(b / 2 , 0) };
	Point bound2[1][4] = { Point(wx + b, wy + b) ,Point(0, wy + b) ,Point(0, wy + b / 2) ,Point(wx + b, wy + b / 2) };
	Point bound3[1][4] = { Point(wx + b, wy + b) ,Point(wx + b,0) ,Point(wx + b / 2,0) ,Point(wx + b / 2,wy + b) };
	const Point* ppt0[1] = { bound0[0] }, * ppt1[1] = { bound1[0] }, * ppt2[1] = { bound2[0] }, * ppt3[1] = { bound3[0] };
	fillPoly(sig_image, ppt0, npt, 1, Scalar(150, 150, 150), LINE_8);
	fillPoly(sig_image, ppt1, npt, 1, Scalar(150, 150, 150), LINE_8);
	fillPoly(sig_image, ppt2, npt, 1, Scalar(150, 150, 150), LINE_8);
	fillPoly(sig_image, ppt3, npt, 1, Scalar(150, 150, 150), LINE_8);
	// draw legend
	Point legend[1][4] = { Point(wx + b / 2, 0) , Point(wx - b / 2, 0) , Point(wx - b / 2, b / 2) , Point(wx + b / 2, b / 2) };
	const Point* ppt_leg[1] = { legend[0] };
	fillPoly(sig_image, ppt_leg, npt, 1, Scalar(0, 0, 0), LINE_8);
	line(sig_image, Point(wx - .4 * b, b / 8), Point(wx, b / 8), Scalar(0, 255, 255), 2, LINE_8);
	line(sig_image, Point(wx - .4 * b, 3 * b / 8), Point(wx, 3 * b / 8), Scalar(255, 255, 0), 2, LINE_8);
	putText(sig_image, "x-sig.", cvPoint(wx, b / 8),
		FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0, 255, 255), 1, CV_AA);
	putText(sig_image, "y-sig.", cvPoint(wx, 3 * b / 8),
		FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255, 255, 0), 1, CV_AA);
	// find min/max
	for (int ii = 0; ii < sl; ii++) for (int jj = 0; jj < 2; jj++) sig_max = *(sig + jj * sl + ii) > sig_max ? *(sig + jj * sl + ii) : sig_max; // find sig. max
	printf("\nZoom in?:"); double zin; std::cin >> zin; sig_max = sig_max / zin;

	for (int ii = 0; ii < sl; ii++) for (int jj = 0; jj < 2; jj++) sig_min = *(sig + jj * sl + ii) < sig_min ? *(sig + jj * sl + ii) : sig_min; // find sig. min

	double factor = (double)wy / (sig_max - sig_min) * .975;
	double bias = wy / 2 + factor * sig_max / 2 + factor * sig_min / 2 + b / 2;

	// draw graph & axes & title
	for (int ii = 0; ii < sl - 1; ii++) line(sig_image, Point((double)ii / sl * wx + b / 2, bias - factor * *(sig + ii)), Point((double)ii / sl * wx + b / 2 + 1, bias - factor * *(sig + ii + 1)), Scalar(0, 255, 255), 1, LINE_8);
	for (int ii = 0; ii < sl - 1; ii++) line(sig_image, Point((double)ii / sl * wx + b / 2, bias - factor * *(sig + sl + ii)), Point((double)ii / sl * wx + b / 2 + 1, bias - factor * *(sig + sl + ii + 1)), Scalar(255, 255, 0), 1, LINE_8);

	line(sig_image, Point(b / 2, bias), Point(wx + b / 2, bias), Scalar(0, 0, 255), 2, LINE_8);
	int t0 = (int)b / 2 - wx / (t_min_max[1] - t_min_max[0]) * t_min_max[0];
	line(sig_image, Point(t0, b / 2), Point(t0, wy + b / 2), Scalar(0, 0, 255), 2, LINE_8);
	putText(sig_image, ax_x_title, cvPoint(wx + b / 2, bias + 20),
		FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255, 255, 255), 1, CV_AA);
	putText(sig_image, ax_y_title, cvPoint(20, b / 4),
		FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255, 255, 255), 1, CV_AA);
	putText(sig_image, title, cvPoint(20, wy + 7 * b / 8),
		FONT_HERSHEY_COMPLEX_SMALL, .9, cvScalar(0, 255, 255), 1, CV_AA);
	// plot x-y lim
	char s_max_str[50], s_min_str[50], t_max_str[50], t_min_str[50];
	snprintf(s_max_str, 50, "%f", sig_max); snprintf(s_min_str, 50, "%f", sig_min);
	snprintf(t_max_str, 50, "%f", t_min_max[1]); snprintf(t_min_str, 50, "%f", t_min_max[0]);
	putText(sig_image, "0", cvPoint(b / 2 - 40, bias + 0),
		FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255, 255, 255), 1, CV_AA);
	putText(sig_image, "0", cvPoint(t0, wy + b / 2 + 20),
		FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255, 255, 255), 1, CV_AA);
	putText(sig_image, s_max_str, cvPoint(0, b / 2),
		FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255, 255, 255), 1, CV_AA);
	putText(sig_image, s_min_str, cvPoint(0, wy + b / 2),
		FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255, 255, 255), 1, CV_AA);
	putText(sig_image, t_min_str, cvPoint(b / 2, wy + b / 2 + 20),
		FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255, 255, 255), 1, CV_AA);
	putText(sig_image, t_max_str, cvPoint(wx + b / 2, wy + b / 2 + 20),
		FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255, 255, 255), 1, CV_AA);

	imshow(ax_x_title, sig_image); cvWaitKey(); (void)getchar(); (void)getchar();
	return sig_image;
	//printf("\nPress Enter."); 
	//moveWindow(title, 0, 0);


	//namedWindow(title, WINDOW_AUTOSIZE);

	/*IplImage* image_arr = cvCreateImage(cvSize(sig_image.cols, sig_image.rows), 8, 3), ipltemp = sig_image;
	cvCopy(&ipltemp, image_arr);

	cvShowImage(title, image_arr);
	cvWaitKey();
	printf("Done1");
	destroyWindow(title);
	printf("Done2");*/
}

void ShowFrame(int Width, int Height, uint8_t* im_config)
{
	int win_max_sz = 800;
	Mat config_image(Width, Height, CV_8UC1, im_config);
	resize(config_image, config_image, Size((Height < Width) ? win_max_sz : round(win_max_sz * Width / Height)
		, (Height > Width) ? win_max_sz : round(win_max_sz * Height / Width)));
	char config_window[200], Wstr[10], Hstr[10]; _itoa_s(Width, Wstr, 10); _itoa_s(Height, Hstr, 10);
	snprintf(config_window, 200, "%s%s%s%s", "Drawing 1: config. image, Width = ", Wstr, ", Height = ", Hstr);
	namedWindow(config_window, WINDOW_AUTOSIZE);

	IplImage* config_image_arr = cvCreateImage(cvSize(config_image.cols, config_image.rows), 8, 1), ipltemp = config_image;
	cvCopy(&ipltemp, config_image_arr);

	cvShowImage(config_window, config_image_arr);
	cvWaitKey();
	destroyWindow(config_window);
}

void DFT(int m, double* sr, double* si, double* Sr, double* Si, double* Sabs)
{
	double arg, cosarg, sinarg;

	circ_shift(m, sr);
	circ_shift(m, si);
	printf("\n");
	for (int i = 0; i < m; i++) {
		printf("%.2f\r", (double)i / (double)m);
		Sr[i] = 0;
		Si[i] = 0;
		arg = -CV_2PI * (double)i / (double)m;
		for (int k = 0; k < m; k++) {
			cosarg = cos(k * arg);
			sinarg = sin(k * arg);
			Sr[i] += (sr[k] * cosarg - si[k] * sinarg) / (double)m;
			Si[i] += (sr[k] * sinarg + si[k] * cosarg) / (double)m;
		}
		Sabs[i] = sqrt(Sr[i] * Sr[i] + Si[i] * Si[i]);
	}

	circ_shift(m, Sabs);
	circ_shift(m, Sr);
	circ_shift(m, Si);
	circ_shift(m, sr);
	circ_shift(m, si);
	return;
}

void IDFT(int m, double* Sr, double* Si, double* sr, double* si)
{
	double arg, cosarg, sinarg;

	circ_shift(m, Sr);
	circ_shift(m, Si);

	for (int i = 0; i < m; i++) {
		sr[i] = 0;
		si[i] = 0;
		arg = CV_2PI * (double)i / (double)m;
		for (int k = 0; k < m; k++) {
			cosarg = cos(k * arg);
			sinarg = sin(k * arg);
			sr[i] += (Sr[k] * cosarg - Si[k] * sinarg);
			si[i] += (Sr[k] * sinarg + Si[k] * cosarg);
		}
	}
	circ_shift(m, Sr);
	circ_shift(m, Si);
	circ_shift(m, sr);
	circ_shift(m, si);

	return;
}

void circ_shift(int m, double* s)
{
	double* stmp = NULL;
	stmp = (double*)malloc(m * sizeof(double));
	for (int ii = 0; ii < m; ii++) stmp[((ii - m / 2) < 0) ? ii + m / 2 : ii - m / 2] = s[ii];
	for (int ii = 0; ii < m; ii++) s[ii] = stmp[ii];
}


void save_sound(int sl, int fs_wav, double* sig, char name[80])
{
	ofstream mic_sound(name, ios::binary);
	int no_channels = 2;
	double max_volume = 32760; // volume

	int bits_per_sample = 16; // 16 bits per sample
	mic_sound << "RIFF----WAVEfmt ";
	write_word(mic_sound, 16, 4);
	write_word(mic_sound, 1, 2);
	write_word(mic_sound, no_channels, 2);
	write_word(mic_sound, fs_wav, 4);
	write_word(mic_sound, fs_wav * bits_per_sample * no_channels / 8, 4);
	write_word(mic_sound, bits_per_sample * no_channels / 2, 2);
	write_word(mic_sound, bits_per_sample, 2);
	mic_sound << "data----";

	double sig1_max = -1000, sig1_min = 1000, sig2_max = -1000, sig2_min = 1000;
	for (int ii = 0; ii < sl; ii++) sig1_max = sig[ii] > sig1_max ? sig[ii] : sig1_max; // find sig. max
	for (int ii = 0; ii < sl; ii++) sig1_min = sig[ii] < sig1_min ? sig[ii] : sig1_min; // find sig. min
	for (int ii = 0; ii < sl; ii++) sig2_max = sig[ii + sl] > sig2_max ? sig[ii + sl] : sig2_max; // find sig. max
	for (int ii = 0; ii < sl; ii++) sig2_min = sig[ii + sl] < sig2_min ? sig[ii + sl] : sig2_min; // find sig. min
	double sig1_mean = (sig1_max + sig1_min) / 2, sig1_p2p = (sig1_max - sig1_min) / 2, sig2_mean = (sig2_max + sig2_min) / 2, sig2_p2p = (sig2_max - sig2_min) / 2;

	for (int n = 0; n < sl; n++)
	{
		write_word(mic_sound, (short)(max_volume * (sig[n] - sig1_mean) / sig1_p2p), bits_per_sample / 8);
		write_word(mic_sound, (short)(max_volume * (sig[n + sl] - sig2_mean) / sig2_p2p), bits_per_sample / 8);
	}
	size_t file_len = mic_sound.tellp();
	mic_sound.seekp(44);
	write_word(mic_sound, file_len - 44, 4);
	mic_sound.seekp(4);
	write_word(mic_sound, file_len - 8, 4);
}