#pragma once

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class CCfg
{
public:
	//! full constructor
	CCfg();
	//! default destructor
	~CCfg();

	//! loads configuration file from directory
	void ldCfgFl(char* acCfgFlPth);

	inline char* getInFrmPth(void) { return m_acInFrmPth; }
	inline char* getOutCamMatPth(void) { return m_acOutCamMatPth; }
	inline char* getOutCalDispPth(void) { return m_acOutCalDispPth; }
	inline bool getOutCalDispFlg(void) { return m_bOutCalDispFlg; }
	inline int getRszFrmHei(void) { return m_nRszFrmHei; }
	inline bool getCalSel2dPtFlg(void) { return m_bCalSel2dPtFlg; }
	inline std::vector<cv::Point2f> getCal2dPtLs(void) { return m_voCal2dPt; }
	inline std::vector<cv::Point2f> getCal3dPtLs(void) { return m_voCal3dPt; }
	inline int getCalTyp(void) { return m_nCalTyp; }
	inline double getCalRansacReprojThld(void) { return m_fCalRansacReprojThld; }
	inline cv::Size getCalDispGrdDim(void) { return m_oCalDispGrdDim; }
	inline bool getCalDistFlg(void) { return m_bCalDistFlg; }
	inline std::vector<float> getCalDistCoeff(void) { return m_vfCalDistCoeff; }
	inline std::vector<float> getCalFocLen(void) { return m_vfCalFocLen; }
	inline std::vector<float> getCalPrinPt(void) { return m_vfCalPrinPt; }
	inline cv::Mat getCalIntMat(void) { return m_oCalIntMat; }
	inline cv::Mat getCalDistCoeffMat(void) { return m_oCalDistCoeffMat; }

private:
	//! reads char array
	std::string rdCharArr(std::string strCfg, int nParamPos);
	//! reads integer number
	int rdInt(std::string strCfg, int nParamPos);
	//! reads double number
	double rdDbl(std::string strCfg, int nParamPos);
	//! reads bool value
	bool rdBool(std::string strCfg, int nParamPos);
	//! reads size
	cv::Size rdSz(std::string strCfg, int nParamPos);
	//! reads vector of float number
	std::vector<float> rdFltVec(std::string strCfg, int nParamPos);
	//! reads vector of 2D points
	std::vector<cv::Point2f> rdVec2dPt(std::string strCfg, int nParamPos);

	//! path of input video frame
	char m_acInFrmPth[256];
	//! path of output text file of camera matrix
	char m_acOutCamMatPth[256];
	//! path of output display image of camera calibration, necessary when m_bOutCalDispFlg == true
	char m_acOutCalDispPth[256];
	//! flag of output display image of camera calibration
	bool m_bOutCalDispFlg;
	//! resized video frame height (negative: original size)
	int m_nRszFrmHei;
	//! flag of selecting 2D points on the frame image
	bool m_bCalSel2dPtFlg;
	//! input sequence of 2D points on the ground plane, necessary when m_bCalSel2dPtFlg == false
	std::vector<cv::Point2f> m_voCal2dPt;
	//! input sequence of 3D points on the ground plane
	std::vector<cv::Point2f> m_voCal3dPt;
	//! method used to computed the camera matrix: 0 - a regular method using all the points; 4 - Least-Median robust method; 8 - RANSAC-based robust method; -1 - Optimum method with minimum reprojection error
	int m_nCalTyp;
	//! maximum allowed reprojection error to treat a point pair as an inlier, necessary when m_nCalTyp == 8
	double m_fCalRansacReprojThld;
	//! dimension of the grid on the ground plane to display
	cv::Size m_oCalDispGrdDim;
	//! flag of camera undistortion (if 1, require initial intrinsic camera parameters)
	bool m_bCalDistFlg;
	//! input distortion coefficients, necessary when m_bCalDistFlg == true
	std::vector<float> m_vfCalDistCoeff;
	//! focal length(s) of the camera intrinsic matrix, necessary when m_bCalDistFlg == true
	std::vector<float> m_vfCalFocLen;
	//! principal point of the camera intrinsic matrix, necessary when m_bCalDistFlg == true
	std::vector<float> m_vfCalPrinPt;

	//! camera intrinsic matrix
	cv::Mat m_oCalIntMat;
	//! distortion coefficients matrix
	cv::Mat m_oCalDistCoeffMat;
};
