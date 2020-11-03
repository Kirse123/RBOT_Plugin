#include <QApplication>
#include <QThread>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <filesystem>
#include <stdio.h>
#include <io.h>
#include <string>
#include <vector>

#include "dll_controller.h"
#include "unity_interface.h"

using namespace cv;
using namespace std;

int CreatePoseEstimator(int camera_width, int camera_height, float inZNear, float inZFar, float* inK, float* inDistCoeffs)
{
	int resCode = DllController::Instance()->CreatePoseEstimator(camera_width, camera_height, inZNear, inZFar, inK, inDistCoeffs);

	DllController::Instance()->WriteLogLine("PoseEstimator created with result: " + to_string(resCode));

	return resCode;
}

int AddObj3d(char* fullFileName, float tx, float ty, float tz, float alpha, float beta, float gamma, float scale, float qualityThreshold, float* templateDistances) {

	vector<float> dist = { templateDistances[0], templateDistances[1], templateDistances[2] };

	string tmpFileName = string(fullFileName);	

	int resCode = DllController::Instance()->AddObj(tmpFileName, tx, ty, tz, alpha, beta, gamma, scale, qualityThreshold, dist);

	DllController::Instance()->WriteLogLine("Object " + tmpFileName + " added with result: " + to_string(resCode));

	return resCode;
}

int RemoveObj3d(int index)
{
	int resCode = DllController::Instance()->RemoveObj(index);

	DllController::Instance()->WriteLogLine("Object " + to_string(index) + " removed with result: " + to_string(resCode));

	return resCode;
}

int ToggleTracking(int objectIndex, bool undistortFrame) {

	int resCode = DllController::Instance()->ToggleTracking(objectIndex, undistortFrame);

	DllController::Instance()->WriteLogLine("Tracking toggled with result: " + to_string(resCode) + ", undistortFrame= " + to_string(undistortFrame));

	return resCode;
}

int EstimatePoses(float* outPoseData, bool undistortFrame, bool checkForLoss)
{
	int resCode = DllController::Instance()->EstimatePose(outPoseData, undistortFrame, checkForLoss);

	DllController::Instance()->WriteLogLine("Poses estimated with result: " + to_string(resCode) + " undistortFrame=" + to_string(undistortFrame) + " checkForLoss=" + to_string(checkForLoss));

	return resCode;
}

int TextureToCVMat(unsigned char* framePtr, int height, int width)
{
	int resCode = DllController::Instance()->TextureToCVMat(framePtr, height, width);

	DllController::Instance()->WriteLogLine("Frame received with result: " + to_string(resCode));

	return resCode;
}

int Reset()
{
	int resCode = DllController::Instance()->Reset();

	DllController::Instance()->WriteLogLine("Estimation reseted with resul: " + to_string(resCode));

	return resCode;
}

void Close() 
{
	DllController::Instance()->WriteLogLine("Closing dll...");
	DllController::Instance()->CloseLog();
	DllController::Instance()->Close();
}

bool FileExists(const char *fname)
{
	return experimental::filesystem::exists(experimental::filesystem::path(fname));
}