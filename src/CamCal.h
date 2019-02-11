#pragma once

#include <opencv2/calib3d/calib3d.hpp>
#include "Cfg.h"

// camera calibrator
class CCamCal
{
public:
	CCamCal(void);
	~CCamCal(void);

	//! initializes the calibrator
	void initialize(CCfg oCfg, cv::Mat oImgFrm);
	//! perform camera calibration
	void process(void);
	//! output homography matrix and display image
	void output(void);

private:
	//! runs all calibration types
	void runAllCalTyp(void);
	//! calculates reprojection error
	double calcReprojErr(cv::Mat oHomoMat, int nCalTyp, double fCalRansacReprojThld);
	//! outputs text file of homography matrix
	void outTxt(void);
	//! plots a display grid on the ground plane
	void pltDispGrd(void);

	//! configuration parameters
	CCfg m_oCfg;
	//! frame image
	cv::Mat m_oImgFrm;
	//! list of 3D points for PnP
	std::vector<cv::Point2f> m_vo3dPt;
	//! list of 2D points for PnP
	std::vector<cv::Point2f> m_vo2dPt;
	//! homography matrix
	cv::Mat m_oHomoMat;
	//! reprojection error
	double m_fReprojErr;
};

// selector of 2D points for PnP
class C2dPtSel
{
public:
	C2dPtSel(void);
	~C2dPtSel(void);

	//! initializes the 2D point selector
	void initialize(CCfg oCfg, cv::Mat oImgFrm);
	//! selects 2D points
	std::vector<cv::Point> process(void);
	//! pushes a node to the list and draw the circle
	void addNd(int nX, int nY);
	//! checks if the background image is loaded
	inline bool chkImgLd(void)
	{
		if (!m_oImgFrm.empty())
			return true;
		else
			return false;
	}

private:
	//! configuration parameters
	CCfg m_oCfg;
	//! frame image for plotting results
	cv::Mat m_oImgFrm;
	//! list of nodes
	std::vector<cv::Point> m_voNd;
};

extern C2dPtSel o2dPtSel;
