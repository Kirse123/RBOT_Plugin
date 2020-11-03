#include "dll_controller.h"	

using namespace cv;
using namespace std;

DllController::DllController()
{
	// Open log file for writing
	this->logFile = ofstream("log.txt", ios_base::trunc);
	// Redirect std::cout to log.txt
	cout.rdbuf(logFile.rdbuf());
}

int DllController::CreatePoseEstimator(int camera_width, int camera_height, float inZNear, float inZFar, float* inK, float* inDistCoeffs)
{
	// Set up logging in the file
	// Save old cout buf
	streambuf *coutbuf = cout.rdbuf();
	// Redirect std::cout to log.txt
	cout.rdbuf(logFile.rdbuf());
	
	if (qApp == NULL || qApp == nullptr) {
		char *argv[] = { "RBOT_DLL", "arg1", "arg2", NULL };
		int argc = sizeof(argv) / sizeof(char*) - 1;
		QApplication* a = new QApplication(argc, argv);
	}

	if (qApp == NULL || qApp == nullptr)
	{
		// Reset to standard output again
		cout.rdbuf(coutbuf);
		return -6;
	}

	if (DllController::Instance()->objects.size() == 0) {
		// Reset to standard output again
		cout.rdbuf(coutbuf);
		return -2;
	}

	//camera image size (image)
	width = camera_width;
	height = camera_height;

	// near and far plane of the OpenGL view frustum
	zNear = inZNear;
	zFar = inZFar;

	// camera instrinsics
	//K = Matx33f((float)camera_width, 0, (float)camera_width / 2.0, 0, (float)camera_hight, (float)camera_width / 2.0, 0, 0, 1);
	K = Matx33f(inK[0], inK[1], inK[2], inK[3], inK[4], inK[5], inK[6], inK[7], inK[8]);
	distCoeffs = Matx14f(inDistCoeffs[0], inDistCoeffs[1], inDistCoeffs[2], inDistCoeffs[3]);

	// Delete old pose estimator
	if (poseEstimator != NULL || poseEstimator != nullptr)
	{
		delete poseEstimator;
	}

	// create new  pose estimator
	poseEstimator = new PoseEstimator6D(width, height, zNear, zFar, K, distCoeffs, objects);

	//RenderingEngine::Instance()->getContext()->moveToThread(a->thread());

	// active the OpenGL context for the offscreen rendering engine during pose estimation
	RenderingEngine::Instance()->makeCurrent();

	cout << "Estimator created with parametrs:" << endl;
	
	cout << "width= " + to_string(width) + ", height= " + to_string(height) << endl;
	
	cout << "zNear= " + to_string(zNear) + ", zFar= " + to_string(zFar) << endl;
	
	cout << "K = { ";
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			cout << to_string(K(i, j)) + " ";
		}
	}
	cout << "}" << endl;

	cout << "distCoeffs = { ";
	for (int i = 0; i < 4; ++i) {
		cout << to_string(distCoeffs(0, i)) + "";
	}
	cout << "}" << endl;				

	// Reset to standard output again
	cout.rdbuf(coutbuf);
	return 0;
}

int DllController::EstimatePose(float* outData, bool undistortFrame, bool checkForLoss)
{
	// Save old cout buf
	streambuf *coutbuf = cout.rdbuf();
	// Redirect std::cout to log.txt
	cout.rdbuf(logFile.rdbuf());
	
	if (poseEstimator == NULL || poseEstimator == nullptr)
	{
		// Reset to standard output again
		cout.rdbuf(coutbuf);
		return -1;
	}

	if (this->currentFrame.empty())
	{
		// Reset to standard output again
		cout.rdbuf(coutbuf);
		return -3;
	}

	if (objects.size() == 0) {
		// Reset to standard output again
		cout.rdbuf(coutbuf);
		return -2;
	}

	// the main pose update call
	poseEstimator->estimatePoses(this->currentFrame, undistortFrame, checkForLoss);

	for (int k = 0; k < objects.size(); ++k) 
	{
		Matx44f res = objects[k]->getPose();
		for (int i = 0; i < 4; ++i) {
			for (int j = 0; j < 4; ++j) {
				outData[k * 16 + i * 4 + j] = res(i, j);
				WriteLog(to_string(res(i, j)) + " ");
			}
		}
		WriteLogLine("");
	}

	// Reset to standard output again
	cout.rdbuf(coutbuf);
	return 0;
}

int DllController::AddObj(string fullFileName, float tx, float ty, float tz, float alpha, float beta, float gamma, float scale, float qualityThreshold, vector<float> &templateDistances)
{
	// Save old cout buf
	streambuf *coutbuf = cout.rdbuf();
	// Redirect std::cout to log.txt
	cout.rdbuf(logFile.rdbuf());

	if (!FileExists(&fullFileName[0])) {
		// Reset to standard output again
		cout.rdbuf(coutbuf);
		return -5;
	}

	cout << "Adding Object: " + fullFileName << endl;
	cout << "tx= " + to_string(tx) << ", ty= " + to_string(ty) << ", tz= " + to_string(tz) << endl;
	cout << "alpha= " + to_string(alpha) << ", beta= " + to_string(beta) << ", gamma= " + to_string(gamma) << endl;
	cout << "scale= " + to_string(scale) << ", qualityThreshold= " + to_string(qualityThreshold) << endl;
	cout << "templateDistances ={ ";
	for (int i = 0; i < templateDistances.size(); ++i) {
		cout << templateDistances[i] << " ";
	}
	cout << "}" << endl;

	//load 3D-object
	objects.push_back(new Object3D(fullFileName, tx, ty, tz, alpha, beta, gamma, scale, qualityThreshold, templateDistances));

	// Reset to standard output again
	cout.rdbuf(coutbuf);
	return 0;
}

int DllController::RemoveObj(int index)
{	
	// Save old cout buf
	streambuf *coutbuf = cout.rdbuf();
	// Redirect std::cout to log.txt
	cout.rdbuf(logFile.rdbuf());

	if (objects.size() < index + 1)
	{
		// Reset to standard output again
		cout.rdbuf(coutbuf);
		// out of range error
		return -2;
	}

	this->objects.erase(objects.begin() + index);

	// Reset to standard output again
	cout.rdbuf(coutbuf);
	return 0;
}

int DllController::ToggleTracking(int objectIndex, bool undistortFrame)
{
	// Save old cout buf
	streambuf *coutbuf = cout.rdbuf();
	// Redirect std::cout to log.txt
	cout.rdbuf(logFile.rdbuf());

	if (poseEstimator == NULL || poseEstimator == nullptr)
	{
		// Reset to standard output again
		cout.rdbuf(coutbuf);
		return -1;
	}
	// Object not found
	if (objectIndex > objects.size() - 1)
	{
		// Reset to standard output again
		cout.rdbuf(coutbuf);
		return -2;
	}
	if (currentFrame.empty())
	{
		// Reset to standard output again
		cout.rdbuf(coutbuf);
		return -3;
	}

	poseEstimator->toggleTracking(this->currentFrame, objectIndex, undistortFrame);

	// Reset to standard output again
	cout.rdbuf(coutbuf);
	return 0;
}

int DllController::TextureToCVMat(unsigned char* framePtr, int frame_height, int frame_width)
{
	// Save old cout buf
	streambuf *coutbuf = cout.rdbuf();
	// Redirect std::cout to log.txt
	cout.rdbuf(logFile.rdbuf());

	this->width = frame_width;
	this->height = frame_height;

	Mat frame(height, width, CV_8UC4, framePtr);
	flip(frame, currentFrame, 0);
	cvtColor(currentFrame, currentFrame, COLOR_BGRA2RGB);	

	// no frame error
	if (currentFrame.empty())
	{
		// Reset to standard output again
		cout.rdbuf(coutbuf);
		return -3;
	}

	// Reset to standard output again
	cout.rdbuf(coutbuf);
	return 0;
}

int DllController::Reset()
{
	if (poseEstimator == NULL || poseEstimator == nullptr)
		return -1;
	poseEstimator->reset();

	return 0;
}

void DllController::Close() 
{
	// deactivate the offscreen rendering OpenGL context
	RenderingEngine::Instance()->doneCurrent();
	// clean up
	RenderingEngine::Instance()->destroy();

	for (int i = 0; i < objects.size(); i++)
	{
		delete objects[i];
	}
	objects.clear();

	if (poseEstimator != nullptr || poseEstimator != NULL) {
		delete poseEstimator;
		poseEstimator = nullptr;
	}

	QApplication::quit();
}

bool DllController::FileExists(const char *fname)
{
	return experimental::filesystem::exists(experimental::filesystem::path(fname));
}

void  DllController::WriteLogLine(string logLine)
{
	streambuf *coutbuf = cout.rdbuf();		//save old buf
	cout.rdbuf(logFile.rdbuf());			//redirect std::cout to out.txt!

	cout << logLine << endl;

	cout.rdbuf(coutbuf);
}
void DllController::WriteLog(string logTxt) 
{
	// save old buf
	streambuf *coutbuf = cout.rdbuf();
	// redirect std::cout to log.txt!
	cout.rdbuf(logFile.rdbuf());	

	cout << logTxt;

	cout.rdbuf(coutbuf);
}
void DllController::CloseLog() 
{
	logFile.close();
}