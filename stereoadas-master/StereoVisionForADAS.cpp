/**
@file StereoVisionForADAS.cpp
@date 2017/09/03
@author tkwoo(wtk1101@gmail.com)
@brief matching, stixel creation, objectness wrapper
*/
#include "StereoVisionForADAS.h"
//cout<< "1";


StereoCamParam_t CStereoVisionForADAS::InitStereoParam(int nDatasetName)
{
	StereoCamParam_t objStereoCamParam;

	if (nDatasetName == Daimler)
	{
		objStereoCamParam.m_dBaseLine = 0.25;
		objStereoCamParam.m_dMaxDist = 70;
		objStereoCamParam.m_nNumberOfDisp = 48;
		objStereoCamParam.m_nWindowSize = 9;
		objStereoCamParam.objCamParam.m_dCameraHeight = 1.17;
		objStereoCamParam.objCamParam.m_dFocalLength = 1200.;
		objStereoCamParam.objCamParam.m_dPitchDeg = -1.89;
		objStereoCamParam.objCamParam.m_sizeSrc = cv::Size(640, 480);
	}
	else if (nDatasetName == KITTI)
	{
		objStereoCamParam.m_dBaseLine = 0.54;
		objStereoCamParam.m_dMaxDist = 60.;
		objStereoCamParam.m_nNumberOfDisp = 80;
		objStereoCamParam.m_nWindowSize = 11;
		objStereoCamParam.objCamParam.m_dCameraHeight = 1.65;			//TBD..
		objStereoCamParam.objCamParam.m_dFocalLength = 722;
		objStereoCamParam.objCamParam.m_dPitchDeg = -1.;	///< ������ ���׷��� ����� ������ �ݴ�� ��. ���׷��� ����
		objStereoCamParam.objCamParam.m_dYawDeg = -1.2;
		objStereoCamParam.objCamParam.m_dOx = 610;
		objStereoCamParam.objCamParam.m_dOy = 173;
		objStereoCamParam.objCamParam.m_sizeSrc = cv::Size(1242, 375);
	}
	else if (nDatasetName == CityScape)
	{
		//printf("CityScape DB is not availabe yet. sorry");
		objStereoCamParam.m_dBaseLine = 0.21;
		objStereoCamParam.m_dMaxDist = 70.;
		objStereoCamParam.m_nNumberOfDisp = 80;
		objStereoCamParam.m_nWindowSize = 11;
		objStereoCamParam.objCamParam.m_dCameraHeight = 1.22;
		objStereoCamParam.objCamParam.m_dFocalLength = 2263.5/2;
		objStereoCamParam.objCamParam.m_dPitchDeg = -2.18;
		objStereoCamParam.objCamParam.m_dYawDeg = -1.2;
		objStereoCamParam.objCamParam.m_dOx = 610;
		objStereoCamParam.objCamParam.m_dOy = 173;
		//objStereoCamParam.objCamParam.m_sizeSrc = cv::Size(2048 * fScale, 1024 * fScale);
		objStereoCamParam.objCamParam.m_sizeSrc = cv::Size(1024, 512);
	}
	else
		printf("This DB is not availabe. sorry");
	PitchDegToVanishingLine(objStereoCamParam);
	//cout << objStereoCamParam.objCamParam.m_sizeSrc << endl;

	return objStereoCamParam;
}
int CStereoVisionForADAS::PitchDegToVanishingLine(StereoCamParam_t& objStereoParam)
{
	objStereoParam.objCamParam.m_nVanishingY = (int)(objStereoParam.objCamParam.m_dFocalLength*tan(objStereoParam.objCamParam.m_dPitchDeg*PI / 180)) + objStereoParam.objCamParam.m_sizeSrc.height / 2;
	return 0;
}
void CStereoVisionForADAS::MakePseudoColorLUT()
{
	int b = 125;
	int g = 0;
	int r = 0;

	int idx = 0;

	int mode = 0;
	// mode = 0 : increasing 'b'
	// mode = 1 : increasing 'g'
	// mode = 2 : decreasing 'b'
	// mode = 3 : increasing 'r'
	// mode = 4 : decreasing 'g'
	// mode = 5 : decreasing 'r'

	while (1)
	{
		m_pseudoColorLUT[idx][0] = b;
		m_pseudoColorLUT[idx][1] = g;
		m_pseudoColorLUT[idx][2] = r;

		if (b == 255 && g == 0 && r == 0)
			mode = 1;
		else if (b == 255 && g == 255 && r == 0)
			mode = 2;
		else if (b == 0 && g == 255 && r == 0)
			mode = 3;
		else if (b == 0 && g == 255 && r == 255)
			mode = 4;
		else if (b == 0 && g == 0 && r == 255)
			mode = 5;

		switch (mode)
		{
		case 0: b += 5; break;
		case 1: g += 5; break;
		case 2: b -= 5; break;
		case 3: r += 5; break;
		case 4: g -= 5; break;
		case 5: r -= 5; break;
		default: break;
		}

		if (idx == 255)
			break;

		idx++;
	}
}
void CStereoVisionForADAS::cvtPseudoColorImage(Mat& srcGray, Mat& dstColor)
{
  //cout << "1" << endl;
	for (int i = 0; i<srcGray.rows; i++)
	{
		for (int j = 0; j<srcGray.cols; j++)
		{
			unsigned char val = srcGray.data[i*srcGray.cols + j];
			if (val == 0) continue;
			dstColor.data[(i*srcGray.cols + j) * 3 + 0] = m_pseudoColorLUT[val][0];
			dstColor.data[(i*srcGray.cols + j) * 3 + 1] = m_pseudoColorLUT[val][1];
			dstColor.data[(i*srcGray.cols + j) * 3 + 2] = m_pseudoColorLUT[val][2];
		}
	}

}
CStereoVisionForADAS::CStereoVisionForADAS(StereoCamParam_t& objStereoParam)
	:m_objStereoMatching(objStereoParam), m_objStixelEstimation(objStereoParam), m_objStixelSegmentation(objStereoParam)
{
	//cout << objStereoParam.objCamParam.m_sizeSrc << endl;
	m_objStereoParam = objStereoParam;
	MakePseudoColorLUT();
	m_imgColorDisp = Mat(objStereoParam.objCamParam.m_sizeSrc, CV_8UC3);
	m_imgColorDisp = Scalar(0);
	m_imgGround = Mat(objStereoParam.objCamParam.m_sizeSrc, CV_8UC1);
	m_imgStixelGray = Mat(objStereoParam.objCamParam.m_sizeSrc, CV_8UC1);
	m_imgStixelGray = Scalar(0);
}
int CStereoVisionForADAS::Objectness(Mat& imgLeft, Mat& imgRight)
{
	m_vecobjBB.clear();
	m_vecobjStixelInROI.clear();
	m_vecobjStixels.clear();

	if (imgLeft.channels() == 3){
		cvtColor(imgLeft, m_imgLeftInput, CV_BGR2GRAY);
		cvtColor(imgRight, m_imgRightInput, CV_BGR2GRAY);
		//return NO_PROB;
	}
	else
	{
		m_imgLeftInput = imgLeft;
		m_imgRightInput = imgRight;
	}

#if CV_MAJOR_VERSION==2
	m_objStereoMatching.MakeDisparity(m_imgLeftInput, m_imgRightInput,false);
#else
	m_objStereoMatching.MakeDisparity(m_imgLeftInput, m_imgRightInput,false); // wls filter
#endif
	m_matDisp16 = m_objStereoMatching.m_matDisp16;
	m_imgDisp8 = m_objStereoMatching.m_imgDisp8;

	//imshow("disparity", m_imgDisp8);

	m_imgGround = Scalar(0);
	m_objStixelEstimation.EstimateStixels(m_matDisp16, m_imgDisp8);//, false);
	m_imgGround = m_objStixelEstimation.m_imgGround;
	m_vecobjStixelInROI = m_objStixelEstimation.m_vecobjStixelInROI;
	m_vecobjStixels = m_objStixelEstimation.m_vecobjStixels;

	m_objStixelSegmentation.SegmentStixel(m_vecobjStixels);
	m_vecobjBB = m_objStixelSegmentation.m_vecobjBB;

	return 0;
}
void CStereoVisionForADAS::Display(Mat& imgDisplay, Mat& imgStixelResult)
{
  //cout << "4" << endl;
	//if (imgDisplay.channels() == 3) cvtColor(imgDisplay, imgDisplay, CV_BGR2GRAY);
	if (imgStixelResult.channels() == 3) cvtColor(imgStixelResult, imgStixelResult, CV_BGR2GRAY);

	float fBrightness = 70;
  float fBrightness1 = 0;
  //float fBrightness = 70;

	//Display(imgDisplay);
	for (unsigned int i = 0; i < m_vecobjBB.size(); i++){
		rectangle(imgDisplay, m_vecobjBB[i].rectBB, Scalar::all(255 - m_vecobjBB[i].dZ / fBrightness1 * 255), 2, 8);
    //rectangle(imgDisplay, m_vecobjBB[i].rectBB, Scalar::all(255 - m_vecobjBB[i].dZ / fBrightness * 0), 0, 0);
    //cout << "5" << endl;
	}

	/*Mat imgStixelTemp = m_imgColorDisp.clone();
	DrawStixel(imgStixelTemp, m_vecobjStixelInROI);
	imshow("gg", imgStixelTemp);*/
	m_imgStixelGray = Scalar(0);

	DrawGround(m_imgColorDisp, m_imgGround);
	line(m_imgColorDisp, Point(0, m_objStereoParam.objCamParam.m_nVanishingY), Point(m_imgColorDisp.cols, m_objStereoParam.objCamParam.m_nVanishingY), Scalar(255, 255, 255), 3);
	DrawStixel(m_imgColorDisp, m_vecobjStixels);
	//DrawLane(m_imgColorDisp, m_objStereoParam);
	DrawStixel(m_imgColorDisp, m_vecobjStixelInROI);

	cvtColor(m_imgLeftInput, imgStixelResult, CV_GRAY2BGR);
	addWeighted(imgStixelResult, 0.4, m_imgColorDisp, 0.6, 0., imgStixelResult);
	for (unsigned int i = 0; i < m_vecobjBB.size(); i++){
		rectangle(imgStixelResult, m_vecobjBB[i].rectBB, Scalar::all(255 - m_vecobjBB[i].dZ / fBrightness * 250), 2, 8);
		char temp[20];
		sprintf_s(temp, sizeof(temp), "%d: %.2fm", i, m_vecobjBB[i].dZ);
		putText(imgStixelResult, temp,m_vecobjBB[i].rectBB.br() - Point(m_vecobjBB[i].rectBB.width, 0), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255 - m_vecobjBB[i].dZ / fBrightness1 * 250), 2);
	}


}
void CStereoVisionForADAS::DrawStixel(Mat& imgResult, vector<stixel_t>& vecobjStixels)
{
  //cout << "7" << endl;
	for (unsigned int u = 0; u < vecobjStixels.size(); u++){
		line(m_imgStixelGray,
			Point(vecobjStixels[u].nCol, vecobjStixels[u].nGround),
			Point(vecobjStixels[u].nCol, vecobjStixels[u].nHeight),
			Scalar(vecobjStixels[u].chDisparity));
	}
	cvtPseudoColorImage(m_imgStixelGray, imgResult);
	m_imgStixelGray.setTo(0);
}
void CStereoVisionForADAS::DrawGround(Mat& imgResult, Mat& imgGround)
{
  //cout << "6" << endl;
	threshold(imgGround, imgGround, 0, 255, CV_THRESH_BINARY);
	cvtColor(imgGround, imgResult, CV_GRAY2BGR);
	Mat imgTemp = imgResult.clone();
	imgTemp = Scalar(0, 75, 150);
	imgResult = imgResult&imgTemp;
	/*imshow("temp", imgResult);
	waitKey();*/
}
