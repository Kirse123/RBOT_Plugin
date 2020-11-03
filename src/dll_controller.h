#ifndef DLL_CONTROLLER_H
#define DLL_CONTROLLER_H

#include <QApplication>
#include <QThread>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <filesystem>
#include <stdio.h>
#include <io.h>
#include <string>
#include <fstream>

#include "object3d.h"
#include "pose_estimator6d.h"

class DllController
{
public:
	/*
		*	Method returning reference to an instance field and realizing Singlton pattern
	*/
	static DllController* Instance() {
		static DllController _instance;
		return &_instance;
	}

	/*
		*  Initializer of the pose estimator initializing rectification
		*  maps for image undistorting and the rendering engine, given
		*  the 3x3 float intrinsic camera matrix
		*  inK = [fx 0 cx]
		*        [0 fy cy]
		*        [0  0  1]
		*  and distortion coefficients(as needed by OpenCV).
		*  It also initializes the OpenGL rendering buffers for all
		*  provided 3D objects using the OpenGL context of the engine.
	*/
	int CreatePoseEstimator(int camera_width, int camera_height, float inZNear, float inZFar, float* inK, float* inDistCoeffs);


	/**
		 *  Initializes/starts tracking based on the current camera frame
		 *  for a specified 3D object using its initial pose by building
		 *  the first set of tclc-histograms. If the object was already
		 *  initialized this method will reset/stop tracking for it
		 *  instead.
		 *
		 *  @param  frame The current camera frame  (RGB, uchar).
		 *  @param  objectIndex The index of the object to be initialized.
		 *  @param  undistortFrame A flag indicating whether the image should first be undistorted for initialization (default = true).
	 */
	int ToggleTracking(int objectIndex, bool undistortFrame = true);

	/*
		 *  This method tries to track and detect the 6DOF poses of all
		 *  currently initialized rigid 3D objects by minimizing the
		 *  region-based cost function using tclc-histograms. During
		 *  successful tracking, the pose is estimated frame-to-frame.
		 *  If tracking has been lost for an object, the pose will be
		 *  estimated using a template matching approach for pose detection
		 *  also based on tclc-histograms.
		 *
		 *  @param outData  Pointer to array, containing out data
		 *  @param undistortFrame A flag indicating whether the image should first be undistorted for initialization (default = true).
		 *  @param undistortFrame A flag indicating whether it should be checked for a tracking loss after pose estimation (default = true).

	*/
	int EstimatePose(float* outData, bool undistortFrame = true, bool checkForLoss = true);

	/*
		*  Method creating a 3D object class from a specified initial 6DOF pose, a
		*  scaling factor, a tracking quality threshhold and a set of distances to the
		*  camera for template generation used within pose detection.Here, also the set
		*  of n tclc - histograms is initialized, with n being the total number of 3D model
		*  vertices.
		*
		*  @param objFilename  The relative path to an OBJ / PLY file describing the model.
		*  @param tx  The models initial translation in X - direction relative to the camera.
		*  @param ty  The models initial translation in Y - direction relative to the camera.
		*  @param tz  The models initial translation in Z - direction relative to the camera.
		*  @param alpha  The models initial Euler angle rotation about X - axis of the camera.
		*  @param beta  The models initial Euler angle rotation about Y - axis of the camera.
		*  @param gamma  The models initial Euler angle rotation about Z - axis of the camera.
		*  @param scale  A scaling factor applied to the model in order change its size independent of the original data.
		*  @param qualityThreshold  The individual quality tracking quality threshold used to decide whether tracking and detection have been successful(should be within[0.5, 0.6]).
		*  @param templateDistances  A vector of absolute Z - distance values to be used for template generation(typically 3 values: a close, an intermediate and a far distance)
	*/
	int AddObj(std::string fullFileName, float tx, float ty, float tz, float alpha, float beta, float gamma, float scale, float qualityThreshold, std::vector<float> &templateDistances = DllController::Instance()->distances);

	int RemoveObj(int index);

	/*
		*  Converts Unity texture into CVMat
	*/
	int TextureToCVMat(unsigned char* framePtr, int frame_height, int frame_width);

	/**
		*  Resets/stops pose tracking for all objects by clearing the
		*  respective sets of tclc-histograms.
	*/
	int Reset();

	// Free alloceted memory
	void Close();

	// Write log into log.txt
	void WriteLog(std::string logTxt);
	void WriteLogLine(std::string logLine);
	void CloseLog();

private:
	// Camera image size
	int width, height;

	// Near and far plane of the OpenGL view frustum
	float zNear, zFar;

	// Distances for the pose detection template generation
	std::vector<float> distances;

	// Camera instrinsics
	cv::Matx33f K;
	cv::Matx14f distCoeffs;

	// Collection of 3D-models
	std::vector<Object3D*> objects;

	// Last received frame
	cv::Mat currentFrame;

	// Pose estimator reference, to call object-trackinf methods
	PoseEstimator6D* poseEstimator;

	// Path to the folder, where 3D-models are stored
	std::string models_folder;

	// Check file existence
	bool FileExists(const char *fname);

	std::ofstream logFile;

	DllController();

	DllController(DllController const&) = delete;
	void operator=(DllController const&) = delete;
};
#endif // !DLL_CONTROLLER_H

