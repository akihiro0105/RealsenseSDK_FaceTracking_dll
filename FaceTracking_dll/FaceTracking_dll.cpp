#include <Windows.h>
#include <math.h>
#include <string>
#include "pxcfacedata.h"
#include "pxcfaceconfiguration.h"
#include "pxcmetadata.h"
#include "pxcsensemanager.h"

extern "C" {
	PXCSession* session = NULL;
	pxcCHAR device[256];
	PXCFaceConfiguration::TrackingModeType profile;
	PXCFaceData* m_output;

	bool Stopflag;
	int expression[22];//表情1-100
	int detection[3];//顔の位置x,y,size
	float landmark[156];//マーカー位置
	float Rotation[3];//顔の回転yaw,pitch,roll

	static DWORD WINAPI ProcessingThread(LPVOID arg)
	{
		PXCSenseManager* senseManager = session->CreateSenseManager();
		/* Set Mode & Source */
		PXCCaptureManager* captureManager = senseManager->QueryCaptureManager();
		captureManager->FilterByDeviceInfo(device, 0, 0);//Devise
		/* Set Module */
		senseManager->EnableFace();
		/* Initialize */
		PXCFaceModule* faceModule = senseManager->QueryFace();
		PXCFaceConfiguration* config = faceModule->CreateActiveConfiguration();
		config->SetTrackingMode(profile);//Profile
		config->ApplyChanges();
		if (senseManager->Init() < PXC_STATUS_NO_ERROR)
		{
			captureManager->FilterByStreamProfiles(NULL);
			senseManager->Init();
		}
		config->detection.isEnabled = 1;
		config->landmarks.isEnabled = 1;
		config->pose.isEnabled = 0;
		config->QueryPulse()->Disable();
		config->QueryExpressions()->Enable();
		config->QueryExpressions()->EnableAllExpressions();
		config->ApplyChanges();

		m_output = faceModule->CreateOutput();

		while (true)
		{
			senseManager->AcquireFrame(true);
			if (Stopflag == true)
			{
				senseManager->ReleaseFrame();
				break;
			}
			m_output->Update();
			PXCCapture::Sample* sample = senseManager->QueryFaceSample();
			if (sample != NULL)
			{
				int numFaces = m_output->QueryNumberOfDetectedFaces();
				for (int j = 0; j < numFaces; j++) {
					PXCFaceData::Face* trackedFace = m_output->QueryFaceByIndex(0);
					if (trackedFace != NULL) {

						PXCImage::ImageInfo info = sample->color->QueryInfo();
						int width = info.width;
						int height = info.height;

						//Detection
						if (trackedFace->QueryDetection() != NULL) {
							PXCFaceData::DetectionData* detectionData = trackedFace->QueryDetection();
							PXCRectI32 rectangle;
							detectionData->QueryBoundingRect(&rectangle);
							detection[0] = width / 2 - (rectangle.x + rectangle.w / 2);
							detection[1] = height / 2 - (rectangle.y + rectangle.h / 2);
							detection[2] = rectangle.w;
						}

						//Landmark
						if (trackedFace->QueryLandmarks() != NULL) {
							PXCFaceData::LandmarksData* landmarkData = trackedFace->QueryLandmarks();
							PXCFaceData::LandmarkPoint* m_landmarkPoints = new PXCFaceData::LandmarkPoint[config->landmarks.numLandmarks];
							landmarkData->QueryPoints(m_landmarkPoints);
							for (int i = 0; i < 78; i++)
							{
								landmark[2 * i] = (float)width / 2 - m_landmarkPoints[i].image.x;
								landmark[2 * i + 1] = (float)height / 2 - m_landmarkPoints[i].image.y;
							}
							float bufx = landmark[20] - landmark[36];
							float bufy = landmark[21] - landmark[37];
							//yaw
							float len1 = sqrt(pow((landmark[21] - landmark[53]), 2) + pow((landmark[20] - landmark[52]), 2));
							float len2 = sqrt(pow((landmark[37] - landmark[53]), 2) + pow((landmark[36] - landmark[52]), 2));
							Rotation[0] = -len1 + len2;
							//pitch
							float fx = landmark[52] - (bufx / 2 + landmark[36]);
							float fy = landmark[53] - (bufy / 2 + landmark[37]);
							float pitch = sqrt(pow(fy, 2) + pow(fx, 2));
							if (fy < 0) pitch *= -1;
							Rotation[1] = pitch;
							//roll
							if (bufy != 0)Rotation[2] = -atan2(bufy, bufx);
						}

						//Expression
						if (trackedFace->QueryExpressions() != NULL) {
							PXCFaceData::ExpressionsData* expressionsData = trackedFace->QueryExpressions();
							PXCFaceData::ExpressionsData::FaceExpressionResult expressionResult[21];
							for (int i = 0; i < 22; i++) {
								expressionsData->QueryExpression((PXCFaceData::ExpressionsData::FaceExpression)i, &expressionResult[i]);
								expression[i] = (int)expressionResult[i].intensity;
							}
						}
					}
				}
			}
			senseManager->ReleaseFrame();
		}
		m_output->Release();
		config->Release();
		senseManager->Close();
		senseManager->Release();

		session->Release();
		return 0;
	}

	__declspec(dllexport) void  __stdcall Init(int dev)
	{
		session = PXCSession::CreateInstance();
		//Device
		PXCSession::ImplDesc desc;
		memset(&desc, 0, sizeof(desc));
		desc.group = PXCSession::IMPL_GROUP_SENSOR;
		desc.subgroup = PXCSession::IMPL_SUBGROUP_VIDEO_CAPTURE;
		for (int i = 0; ; ++i)
		{
			PXCSession::ImplDesc desc1;
			if (session->QueryImpl(&desc, i, &desc1) < PXC_STATUS_NO_ERROR)break;
			PXCCapture *capture;
			if (session->CreateImpl<PXCCapture>(&desc1, &capture) < PXC_STATUS_NO_ERROR)continue;
			for (int j = 0; ; ++j) {
				PXCCapture::DeviceInfo deviceInfo;
				if (capture->QueryDeviceInfo(j, &deviceInfo) < PXC_STATUS_NO_ERROR)break;
				wprintf(L"Device:%s\n", deviceInfo.name);
				if (j == dev)wcscpy_s(device, deviceInfo.name);
			}
			capture->Release();
		}

		//Module
		pxcCHAR module[256];
		PXCSession::ImplDesc desc_M, desc1_M;
		memset(&desc_M, 0, sizeof(desc_M));
		desc_M.cuids[0] = PXCFaceModule::CUID;
		session->QueryImpl(&desc_M, 0, &desc1_M);
		wcscpy_s(module, desc1_M.friendlyName);

		//Profile
		PXCSession::ImplDesc desc_P;
		memset(&desc_P, 0, sizeof(desc_P));
		desc_P.cuids[0] = PXCFaceModule::CUID;
		wcscpy_s<sizeof(desc_P.friendlyName) / sizeof(pxcCHAR)>(desc_P.friendlyName, module);//Module
		PXCFaceModule *faceModule;
		session->CreateImpl<PXCFaceModule>(&desc_P, &faceModule);
		profile = PXCFaceConfiguration::TrackingModeType::FACE_MODE_COLOR;

		//start
		Stopflag = false;
		for (int i = 0; i < 22; i++)expression[i] = 0;
		for (int i = 0; i < 3; i++)detection[i] = 0;
		for (int i = 0; i < 156; i++)landmark[i] = 0.0f;
		for (int i = 0; i < 3; i++)Rotation[i] = 0.0f;

		wchar_t cBuff[256] = L"";
		GetConsoleTitle(cBuff, 256);
		HWND dialogWindow = FindWindow(NULL, cBuff);
		CreateThread(0, 0, ProcessingThread, dialogWindow, 0, 0);
		Sleep(0);
	}

	__declspec(dllexport) int  __stdcall GetExpression(int num)
	{
		//EXPRESSION_BROW_RAISER_LEFT = 0,
		//EXPRESSION_BROW_RAISER_RIGHT = 1,
		//EXPRESSION_BROW_LOWERER_LEFT = 2,
		//EXPRESSION_BROW_LOWERER_RIGHT = 3,

		//EXPRESSION_SMILE = 4,
		//EXPRESSION_KISS = 5,
		//EXPRESSION_MOUTH_OPEN = 6,

		//EXPRESSION_EYES_CLOSED_LEFT = 7,
		//EXPRESSION_EYES_CLOSED_RIGHT = 8,

		//EXPRESSION_EYES_TURN_LEFT = 15,
		//EXPRESSION_EYES_TURN_RIGHT = 16,
		//EXPRESSION_EYES_UP = 17,
		//EXPRESSION_EYES_DOWN = 18,
		//EXPRESSION_TONGUE_OUT = 19,
		//EXPRESSION_PUFF_RIGHT = 20,
		//EXPRESSION_PUFF_LEFT = 21
		int buf = expression[num];
		return buf;
	}

	__declspec(dllexport) int  __stdcall GetDetection(int num)
	{
		//0:x
		//1:y
		//2:size
		int buf = detection[num];
		return buf;
	}

	__declspec(dllexport) float  __stdcall GetLandmark(int num)
	{
		//78point
		//x,y
		float buf = landmark[num];
		return buf;
	}

	__declspec(dllexport) float  __stdcall GetRotation(int num)
	{
		//0:yaw
		//1:pitch
		//2:roll
		float buf = Rotation[num];
		return buf;
	}

	__declspec(dllexport) void  __stdcall Stop()
	{
		Stopflag = true;
	}
}