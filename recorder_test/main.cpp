/****************************************************************************
 *                                                                           *
 *   Recorder Test                                                           *
 *                                                                           *
 *  This project is a sample of recording and playing.                       *
 *                                                                           *
 *  this is free software: you can redistribute it and/or modify             *
 *  it under the terms of the GNU Lesser General Public License as published *
 *  by the Free Software Foundation, either version 3 of the License, or     *
 *  (at your option) any later version.                                      *
 *                                                                           *
 *  You should have received a copy of the GNU Lesser General Public License *
 *  along with OpenNI. If not, see <http://www.gnu.org/licenses/>.           *
 *                                                                           *
 ****************************************************************************/
//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include <iostream>
#include <XnCppWrapper.h>
#include <stdexcept>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------
#define CONFIG_XML_PATH "./config.xml"
#define RECORD_FILE_PATH "../../skeletonrec.oni"
//#define RECORD_FILE_PATH "../../mydata.oni"
#define MAX_NUM_USERS 1

#define USE_RECORED_DATA TRUE
#define DO_RECORED FALSE

//---------------------------------------------------------------------------
// Globals
//---------------------------------------------------------------------------
xn::Context g_Context;
xn::ScriptNode g_scriptNode;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator g_UserGenerator;
xn::ImageGenerator g_ImageGenerator;
xn::SkeletonCapability g_SkeletonCap = NULL;
IplImage *g_rgbImage = NULL;


XnBool g_bNeedPose = FALSE;
XnChar g_strPose[20] = "";

CvScalar g_Colors[] =
{
	CV_RGB(255,255,255),
	CV_RGB(255,0,0),
	CV_RGB(0,255,0),
	CV_RGB(0,0,255),
	CV_RGB(0,255,255),
	CV_RGB(255,0,255),
	CV_RGB(255,255,0),
};


//---------------------------------------------------------------------------
// Macro
//---------------------------------------------------------------------------
#define CHECK_RC(nRetVal, what) \
if (nRetVal != XN_STATUS_OK)    \
{   \
printf("%s failed: %s\n", what, xnGetStatusString(nRetVal));    \
return nRetVal; \
}

#if defined(DEBUG)
#define LOG_D(fmt, ...) { \
std::string format("[DEBUG] "); \
format.append(fmt); \
printf(format.c_str(), __VA_ARGS__); \
printf("\n"); \
}
#else
#define LOG_D(fmt, ...)
#endif

#define LOG_I(fmt, ...) { \
std::string format("[INFO] "); \
format.append(fmt); \
printf(format.c_str(), __VA_ARGS__); \
printf("\n"); \
}

#define LOG_E(fmt, ...) { \
std::string format("[ERROR] %dn:%d "); \
format.append(fmt); \
printf(format.c_str(), __FILE__, __LINE__, __VA_ARGS__); \
printf("\n"); \
}


//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------
using namespace xn;

XnBool fileExists(const char *fn)
{
	XnBool exists;
	xnOSDoesFileExist(fn, &exists);
	return exists;
}

// Callback: New user was detected
void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
    XnUInt32 epochTime = 0;
    xnOSGetEpochTime(&epochTime);
    LOG_D("%d New User %d", epochTime, nId);
    // New user found
    if (g_bNeedPose)
    {
        g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
    }
    else
    {
        g_SkeletonCap.RequestCalibration(nId, TRUE);
    }
}
// Callback: An existing user was lost
void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
    XnUInt32 epochTime = 0;
    xnOSGetEpochTime(&epochTime);
    LOG_D("%d Lost user %d", epochTime, nId);	
}
// Callback: Detected a pose
void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, const XnChar* strPose, XnUserID nId, void* pCookie)
{
    XnUInt32 epochTime = 0;
    xnOSGetEpochTime(&epochTime);
    LOG_D("%d Pose %s detected for user %d", epochTime, strPose, nId);
    g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
    g_SkeletonCap.RequestCalibration(nId, TRUE);
}
// Callback: Started calibration
void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie)
{
    XnUInt32 epochTime = 0;
    xnOSGetEpochTime(&epochTime);
    LOG_D("%d Calibration started for user %d", epochTime, nId);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationComplete(xn::SkeletonCapability& capability, XnUserID nId, XnCalibrationStatus eStatus, void* pCookie)
{
    XnUInt32 epochTime = 0;
    xnOSGetEpochTime(&epochTime);
    if (eStatus == XN_CALIBRATION_STATUS_OK)
    {
        // Calibration succeeded
        LOG_D("%d Calibration complete, start tracking user %d", epochTime, nId);		
        g_SkeletonCap.StartTracking(nId);
    }
    else
    {
        // Calibration failed
        LOG_D("%d Calibration failed for user %d\n", epochTime, nId);
        if(eStatus==XN_CALIBRATION_STATUS_MANUAL_ABORT)
        {
            LOG_D("%s", "Manual abort occured, stop attempting to calibrate!");
            return;
        }
        if (g_bNeedPose)
        {
            g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
        }
        else
        {
            g_SkeletonCap.RequestCalibration(nId, TRUE);
        }
    }
}

// スケルトンを描画する
void DrawSkelton(XnUserID player, int idx)
{

    // 線を引く開始と終了のJointの定義
    XnSkeletonJoint joints[][2] = {
        {XN_SKEL_HEAD, XN_SKEL_NECK},
        {XN_SKEL_NECK, XN_SKEL_LEFT_SHOULDER},
        {XN_SKEL_LEFT_SHOULDER, XN_SKEL_LEFT_ELBOW},
        {XN_SKEL_LEFT_ELBOW, XN_SKEL_LEFT_HAND},
        {XN_SKEL_NECK, XN_SKEL_RIGHT_SHOULDER},
        {XN_SKEL_RIGHT_SHOULDER, XN_SKEL_RIGHT_ELBOW},
        {XN_SKEL_RIGHT_ELBOW, XN_SKEL_RIGHT_HAND},
        {XN_SKEL_LEFT_SHOULDER, XN_SKEL_TORSO},
        {XN_SKEL_RIGHT_SHOULDER, XN_SKEL_TORSO},
        {XN_SKEL_TORSO, XN_SKEL_LEFT_HIP},
        {XN_SKEL_LEFT_HIP, XN_SKEL_LEFT_KNEE},
        {XN_SKEL_LEFT_KNEE, XN_SKEL_LEFT_FOOT},
        {XN_SKEL_TORSO, XN_SKEL_RIGHT_HIP},
        {XN_SKEL_RIGHT_HIP, XN_SKEL_RIGHT_KNEE},
        {XN_SKEL_RIGHT_KNEE, XN_SKEL_RIGHT_FOOT},
        {XN_SKEL_LEFT_HIP, XN_SKEL_RIGHT_HIP}
    };

	XnSkeletonJointPosition joint1, joint2;
    int nJointsCount = sizeof(joints) / sizeof(joints[0]);
    int color_idx = idx;
    if( color_idx > (sizeof(g_Colors) / sizeof(g_Colors[0])) ){
        color_idx = (sizeof(g_Colors) / sizeof(g_Colors[0])) - 1;
    }
       
    for(int i = 0; i < nJointsCount;i++){
        g_SkeletonCap.GetSkeletonJointPosition(player, joints[i][0], joint1);
        g_SkeletonCap.GetSkeletonJointPosition(player, joints[i][1], joint2);        
        if (joint1.fConfidence < 0.2 || joint2.fConfidence < 0.2)
        {
            return;
        }
        
        XnPoint3D pt[2];
        pt[0] = joint1.position;
        pt[1] = joint2.position;
        
        g_DepthGenerator.ConvertRealWorldToProjective(2, pt, pt);
        
        // 線で結んで
        cvLine( g_rgbImage, cvPoint(pt[0].X, pt[0].Y), cvPoint(pt[1].X, pt[1].Y), g_Colors[color_idx], 1, CV_AA);
        // それぞれの点を塗りつぶす
        cvCircle(g_rgbImage, cvPoint(pt[0].X, pt[0].Y), 2, g_Colors[color_idx], -1, CV_AA, 0);
        cvCircle(g_rgbImage, cvPoint(pt[1].X, pt[1].Y), 2, g_Colors[color_idx], -1, CV_AA, 0);
    }
    
}


int main(int argc, char **argv)
{
    XnStatus nRetVal = XN_STATUS_OK;
    xn::EnumerationErrors errors;
    
    if( USE_RECORED_DATA ){
        g_Context.Init();
        g_Context.OpenFileRecording(RECORD_FILE_PATH);
        xn::Player player;
        
        // Player nodeの取得
        nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_PLAYER, player);
        CHECK_RC(nRetVal, "Find player");
        
        LOG_D("PlaybackSpeed: %d", player.GetPlaybackSpeed());
        
        xn:NodeInfoList nodeList;
        player.EnumerateNodes(nodeList);
        for( xn::NodeInfoList::Iterator it = nodeList.Begin();
            it != nodeList.End(); ++it){
            
            if( (*it).GetDescription().Type == XN_NODE_TYPE_IMAGE ){
                nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_IMAGE, g_ImageGenerator);
                CHECK_RC(nRetVal, "Find image node");
                LOG_D("%s", "ImageGenerator created.");
            }
            else if( (*it).GetDescription().Type == XN_NODE_TYPE_DEPTH ){
                nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
                CHECK_RC(nRetVal, "Find depth node");
                LOG_D("%s", "DepthGenerator created.");            
            }
            else{
                LOG_D("%s %s %s", ::xnProductionNodeTypeToString((*it).GetDescription().Type ),
                      (*it).GetInstanceName(),
                      (*it).GetDescription().strName);
            }
        }
        // UserGeneratorの取得
        nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
        if(nRetVal!=XN_STATUS_OK){
            nRetVal = g_UserGenerator.Create(g_Context); 
            CHECK_RC(nRetVal, "Create user generator");
        }
    }
    else{
        LOG_I("Reading config from: '%s'", CONFIG_XML_PATH);
        
        nRetVal = g_Context.InitFromXmlFile(CONFIG_XML_PATH, g_scriptNode, &errors);
        if (nRetVal == XN_STATUS_NO_NODE_PRESENT)
        {
            XnChar strError[1024];
            errors.ToString(strError, 1024);
            LOG_E("%s\n", strError);
            return (nRetVal);
        }
        else if (nRetVal != XN_STATUS_OK)
        {
            LOG_E("Open failed: %s", xnGetStatusString(nRetVal));
            return (nRetVal);
        }
        
        nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
        CHECK_RC(nRetVal,"No depth");
        
        // ImageGeneratorの作成
        nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_IMAGE, g_ImageGenerator);
        CHECK_RC(nRetVal, "Find image generator");
        
        nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
        if (nRetVal != XN_STATUS_OK)
        {
            nRetVal = g_UserGenerator.Create(g_Context);
            CHECK_RC(nRetVal, "Find user generator");
        }
        
    }

    
    XnCallbackHandle hUserCallbacks, hCalibrationStart, hCalibrationComplete, hPoseDetected;
    if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON))
    {
        LOG_E("%s", "Supplied user generator doesn't support skeleton");
        return 1;
    }
    nRetVal = g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);
    CHECK_RC(nRetVal, "Register to user callbacks");

    g_SkeletonCap = g_UserGenerator.GetSkeletonCap();
    nRetVal = g_SkeletonCap.RegisterToCalibrationStart(UserCalibration_CalibrationStart, NULL, hCalibrationStart);
    CHECK_RC(nRetVal, "Register to calibration start");

    nRetVal = g_SkeletonCap.RegisterToCalibrationComplete(UserCalibration_CalibrationComplete, NULL, hCalibrationComplete);
    CHECK_RC(nRetVal, "Register to calibration complete");
    
    if (g_SkeletonCap.NeedPoseForCalibration())
    {
        g_bNeedPose = TRUE;
        if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
        {
            LOG_E("%s", "Pose required, but not supported");
            return 1;
        }
        nRetVal = g_UserGenerator.GetPoseDetectionCap().RegisterToPoseDetected(UserPose_PoseDetected, NULL, hPoseDetected);
        CHECK_RC(nRetVal, "Register to Pose Detected");
        g_SkeletonCap.GetCalibrationPose(g_strPose);
    }
    
    g_SkeletonCap.SetSkeletonProfile(XN_SKEL_PROFILE_ALL);
    
    nRetVal = g_Context.StartGeneratingAll();
    CHECK_RC(nRetVal, "StartGenerating");
    
    // 表示用の画像データの作成
    XnMapOutputMode mapMode;
    g_ImageGenerator.GetMapOutputMode(mapMode);
    g_rgbImage = cvCreateImage(cvSize(mapMode.nXRes, mapMode.nYRes), IPL_DEPTH_8U, 3);

    LOG_I("%s", "Starting to run");
    if(g_bNeedPose)
    {
        LOG_I("%s", "Assume calibration pose");
    }

    xn::Recorder recorder;
    if( DO_RECORED && !USE_RECORED_DATA ){
        // レコーダーの作成
        LOG_I("%s", "Setup Recorder");
        nRetVal = recorder.Create(g_Context);
        CHECK_RC(nRetVal, "Create recorder");
        
        // 保存設定
        nRetVal = recorder.SetDestination(XN_RECORD_MEDIUM_FILE, RECORD_FILE_PATH);
        CHECK_RC(nRetVal, "Set recorder destination file");
        
        // 深度、ビデオカメラ入力を保存対象として記録開始
        nRetVal = recorder.AddNodeToRecording(g_DepthGenerator, XN_CODEC_NULL);
        CHECK_RC(nRetVal, "Add depth node to recording");
        nRetVal = recorder.AddNodeToRecording(g_ImageGenerator, XN_CODEC_NULL);
        CHECK_RC(nRetVal, "Add image node to recording");
        
        LOG_I("%s", "Recorder setup done.");
    }

    while (!xnOSWasKeyboardHit())
    {
        g_Context.WaitOneUpdateAll(g_UserGenerator);
        if( DO_RECORED  && !USE_RECORED_DATA ){
            nRetVal = recorder.Record();
            CHECK_RC(nRetVal, "Record");
        }

        // ビデオカメラ画像の生データを取得
        xn::ImageMetaData imageMetaData;
        g_ImageGenerator.GetMetaData(imageMetaData);
        // メモリコピー
        xnOSMemCopy(g_rgbImage->imageData, imageMetaData.RGB24Data(), g_rgbImage->imageSize);
        // BGRからRGBに変換して表示
        cvCvtColor(g_rgbImage, g_rgbImage, CV_RGB2BGR);

        // UserGeneratorからユーザー識別ピクセルを取得
        xn::SceneMetaData sceneMetaData;
        g_UserGenerator.GetUserPixels(0, sceneMetaData);
        
        XnUserID allUsers[MAX_NUM_USERS];
        XnUInt16 nUsers = MAX_NUM_USERS;
        g_UserGenerator.GetUsers(allUsers, nUsers);
        for (int i = 0; i < nUsers; i++) {
            
            // キャリブレーションに成功しているかどうか
            if (g_SkeletonCap.IsTracking(allUsers[i])) {
                // スケルトンを描画
                DrawSkelton(allUsers[i], i);
            }
        }
        
        // 表示
        cvShowImage("User View", g_rgbImage);

        // ESCもしくはqが押されたら終了させる
        if (cvWaitKey(10) == 27) {
            break;
        }
    }

    if( !USE_RECORED_DATA ){
        g_scriptNode.Release();
    }
    g_DepthGenerator.Release();
    g_UserGenerator.Release();
    g_Context.Release();

	if (g_rgbImage != NULL) {
		cvReleaseImage(&g_rgbImage);	
	}
	g_Context.Shutdown();

    
}
