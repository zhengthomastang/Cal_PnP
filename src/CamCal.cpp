#include "CamCal.h"

C2dPtSel o2dPtSel;

void on_mouse(int event, int x, int y, int flags, void*)  // mouse event
{
	if (!o2dPtSel.chkImgLd())
	{
		std::cout << "Error: on_mouse(): frame image is unloaded" << std::endl;
		return;
	}

	if (event == CV_EVENT_FLAG_LBUTTON)
		o2dPtSel.addNd(x, y);

	return;
}

CCamCal::CCamCal(void)
{
	// list of 3D points for PnP
	std::vector<cv::Point2f>().swap(m_vo3dPt);

	// list of 2D points for PnP
	std::vector<cv::Point2f>().swap(m_vo2dPt);
}

CCamCal::~CCamCal(void)
{
	// list of 3D points for PnP
	std::vector<cv::Point2f>().swap(m_vo3dPt);

	// list of 2D points for PnP
	std::vector<cv::Point2f>().swap(m_vo2dPt);
}

void CCamCal::initialize(CCfg oCfg, cv::Mat oImgFrm)
{
	// configuration parameters
	m_oCfg = oCfg;

	// frame image
	m_oImgFrm = oImgFrm.clone();

	// list of 3D points for PnP
	m_vo3dPt = m_oCfg.getCal3dPtLs();

	// list of 2D points for PnP
	if (!m_oCfg.getCalSel2dPtFlg())
		m_vo2dPt = m_oCfg.getCal2dPtLs();

	// homography matrix
	m_oHomoMat = cv::Mat(3, 3, CV_64F);
}

void CCamCal::process(void)
{
	// select 2D points if they are not provided in the configuration file
	if (m_oCfg.getCalSel2dPtFlg())
	{
		std::vector<cv::Point2f>().swap(m_vo2dPt);
		o2dPtSel.initialize(m_oCfg, m_oImgFrm);
		std::vector<cv::Point> vo2dPt = o2dPtSel.process();
		std::cout << "Selected 2D points on the frame image: " << std::endl;
		for (int i = 0; i < vo2dPt.size(); i++)
		{
			m_vo2dPt.push_back(vo2dPt[i]);
			if ((vo2dPt.size() - 1) > i)
				std::cout << "[ " << vo2dPt[i].x << ", " << vo2dPt[i].y << " ]," << std::endl;
			else
				std::cout << "[ " << vo2dPt[i].x << ", " << vo2dPt[i].y << " ]" << std::endl;
		}
	}
	
	// compute homography matrix
	m_oHomoMat = cv::findHomography(m_vo3dPt, m_vo2dPt, m_oCfg.getCalTyp(), m_oCfg.getCalRansacReprojThld());

	// calculate reprojection error
	calcReprojErr(m_oHomoMat, m_oCfg.getCalTyp(), m_oCfg.getCalRansacReprojThld());
	std::cout << std::endl;

	//// run all calibration types
	//runAllCalTyp();
}

void CCamCal::output(void)
{
	// output text file of homography matrix
	outTxt();

	// plot a display grid on the ground plane
	pltDispGrd();
}

void CCamCal::runAllCalTyp(void)
{
	cv::Mat oHomoMat;
	
	// a regular method using all the points
	oHomoMat = cv::findHomography(m_vo3dPt, m_vo2dPt, 0, 0);
	calcReprojErr(oHomoMat, 0, 0);

	// Least-Median robust method
	oHomoMat = cv::findHomography(m_vo3dPt, m_vo2dPt, 4, 0);
	calcReprojErr(oHomoMat, 4, 0);

	// RANSAC-based robust method
	for (double t = 100; t >= 10; t -= 5)
	{
		oHomoMat = cv::findHomography(m_vo3dPt, m_vo2dPt, 8, t);
		calcReprojErr(oHomoMat, 8, t);
	}
}

void CCamCal::calcReprojErr(cv::Mat oHomoMat, int nCalTyp, double fCalRansacReprojThld)
{
	double fReprojErr = 0;

	for (int i = 0; i < m_vo3dPt.size(); i++)
	{
		cv::Mat o3dPtMat(3, 1, CV_64F);
		cv::Mat o2dPtMat(3, 1, CV_64F);
		cv::Point2f o2dPt;

		o3dPtMat.at<double>(0, 0) = m_vo3dPt[i].x;
		o3dPtMat.at<double>(1, 0) = m_vo3dPt[i].y;
		o3dPtMat.at<double>(2, 0) = 1;
		o2dPtMat = oHomoMat * o3dPtMat;
		o2dPt = cv::Point2f((o2dPtMat.at<double>(0, 0) / o2dPtMat.at<double>(2, 0)), (o2dPtMat.at<double>(1, 0) / o2dPtMat.at<double>(2, 0)));

		fReprojErr += cv::norm(m_vo2dPt[i] - o2dPt);
	}
	
	fReprojErr /= m_vo3dPt.size();

	if (8 == nCalTyp)
		std::cout << "Average reprojection error of method #" << nCalTyp << " (threshold: " << fCalRansacReprojThld << "): " << fReprojErr << std::endl;
	else
		std::cout << "Average reprojection error of method #" << nCalTyp << ": " << fReprojErr << std::endl;
}

void CCamCal::outTxt(void)
{
	FILE* pfHomoMat = std::fopen(m_oCfg.getOutCamMatPth(), "w");

	std::fprintf(pfHomoMat, "%.7f %.7f %.7f;%.7f %.7f %.7f;%.7f %.7f %.7f\n",
		m_oHomoMat.at<double>(0, 0), m_oHomoMat.at<double>(0, 1), m_oHomoMat.at<double>(0, 2), 
		m_oHomoMat.at<double>(1, 0), m_oHomoMat.at<double>(1, 1), m_oHomoMat.at<double>(1, 2), 
		m_oHomoMat.at<double>(2, 0), m_oHomoMat.at<double>(2, 1), m_oHomoMat.at<double>(2, 2));

	if (m_oCfg.getCalDistFlg())
	{
		cv::Mat oCalDistCoeffMat = m_oCfg.getCalDistCoeffMat();
		std::fprintf(pfHomoMat, "%.7f %.7f %.7f %.7f\n",
			oCalDistCoeffMat.at<double>(0), oCalDistCoeffMat.at<double>(1), 
			oCalDistCoeffMat.at<double>(2), oCalDistCoeffMat.at<double>(3));
	}

	std::fclose(pfHomoMat);
}

void CCamCal::pltDispGrd(void)
{
	cv::Mat oImgPlt = m_oImgFrm.clone();
	cv::Size oDispGrdDim = m_oCfg.getCalDispGrdDim();

	// find the limits of the 3D grid on the ground plane
	double fXMin = DBL_MAX;
	double fYMin = DBL_MAX;
	double fXMax = -DBL_MAX;
	double fYMax = -DBL_MAX;

	for (int i = 0; i < m_vo3dPt.size(); i++)
	{
		if (fXMin > m_vo3dPt[i].x)
			fXMin = m_vo3dPt[i].x;

		if (fYMin > m_vo3dPt[i].y)
			fYMin = m_vo3dPt[i].y;

		if (fXMax < m_vo3dPt[i].x)
			fXMax = m_vo3dPt[i].x;

		if (fYMax < m_vo3dPt[i].y)
			fYMax = m_vo3dPt[i].y;
	}

	// compute the endpoints for the 3D grid on the ground plane
	std::vector<cv::Point2f> vo3dGrdPtTop, vo3dGrdPtBtm, vo3dGrdPtLft, vo3dGrdPtRgt;

	for (int x = 0; x < oDispGrdDim.width; x++)
	{
		vo3dGrdPtTop.push_back(cv::Point2f((fXMin + (x * ((fXMax - fXMin) / (oDispGrdDim.width - 1)))), fYMin));
		vo3dGrdPtBtm.push_back(cv::Point2f((fXMin + (x * ((fXMax - fXMin) / (oDispGrdDim.width - 1)))), fYMax));
	}

	for (int y = 0; y < oDispGrdDim.height; y++)
	{
		vo3dGrdPtLft.push_back(cv::Point2f(fXMin, (fYMin + (y * ((fYMax - fYMin) / (oDispGrdDim.height - 1))))));
		vo3dGrdPtRgt.push_back(cv::Point2f(fXMax, (fYMin + (y * ((fYMax - fYMin) / (oDispGrdDim.height - 1))))));
	}

	// compute the endpoints for the projected 2D grid
	std::vector<cv::Point2f> vo2dGrdPtTop, vo2dGrdPtBtm, vo2dGrdPtLft, vo2dGrdPtRgt;

	for (int i = 0; i < oDispGrdDim.width; i++)
	{
		cv::Mat o3dPtMat(3, 1, CV_64F);
		cv::Mat o2dPtMat(3, 1, CV_64F);

		o3dPtMat.at<double>(0, 0) = vo3dGrdPtTop[i].x;
		o3dPtMat.at<double>(1, 0) = vo3dGrdPtTop[i].y;
		o3dPtMat.at<double>(2, 0) = 1;
		o2dPtMat = m_oHomoMat * o3dPtMat;
		vo2dGrdPtTop.push_back(cv::Point2f((o2dPtMat.at<double>(0, 0) / o2dPtMat.at<double>(2, 0)), (o2dPtMat.at<double>(1, 0) / o2dPtMat.at<double>(2, 0))));

		o3dPtMat.at<double>(0, 0) = vo3dGrdPtBtm[i].x;
		o3dPtMat.at<double>(1, 0) = vo3dGrdPtBtm[i].y;
		o3dPtMat.at<double>(2, 0) = 1;
		o2dPtMat = m_oHomoMat * o3dPtMat;
		vo2dGrdPtBtm.push_back(cv::Point2f((o2dPtMat.at<double>(0, 0) / o2dPtMat.at<double>(2, 0)), (o2dPtMat.at<double>(1, 0) / o2dPtMat.at<double>(2, 0))));
	}

	for (int i = 0; i < oDispGrdDim.height; i++)
	{
		cv::Mat o3dPtMat(3, 1, CV_64F);
		cv::Mat o2dPtMat(3, 1, CV_64F);

		o3dPtMat.at<double>(0, 0) = vo3dGrdPtLft[i].x;
		o3dPtMat.at<double>(1, 0) = vo3dGrdPtLft[i].y;
		o3dPtMat.at<double>(2, 0) = 1;
		o2dPtMat = m_oHomoMat * o3dPtMat;
		vo2dGrdPtLft.push_back(cv::Point2f((o2dPtMat.at<double>(0, 0) / o2dPtMat.at<double>(2, 0)), (o2dPtMat.at<double>(1, 0) / o2dPtMat.at<double>(2, 0))));

		o3dPtMat.at<double>(0, 0) = vo3dGrdPtRgt[i].x;
		o3dPtMat.at<double>(1, 0) = vo3dGrdPtRgt[i].y;
		o3dPtMat.at<double>(2, 0) = 1;
		o2dPtMat = m_oHomoMat * o3dPtMat;
		vo2dGrdPtRgt.push_back(cv::Point2f((o2dPtMat.at<double>(0, 0) / o2dPtMat.at<double>(2, 0)), (o2dPtMat.at<double>(1, 0) / o2dPtMat.at<double>(2, 0))));
	}

	// draw grid lines on the frame image
	for (int i = 0; i < oDispGrdDim.width; i++)
		cv::line(oImgPlt, vo2dGrdPtTop[i], vo2dGrdPtBtm[i], cv::Scalar(int(255.0 * ((double)i / (double)oDispGrdDim.width)), 127, 127), 2, CV_AA);

	for (int i = 0; i < oDispGrdDim.width; i++)
		cv::line(oImgPlt, vo2dGrdPtLft[i], vo2dGrdPtRgt[i], cv::Scalar(127, 127, int(255.0 * ((double)i / (double)oDispGrdDim.width))), 2, CV_AA);

	// plot the 2D points
	for (int i = 0; i < m_vo2dPt.size(); i++)
	{
		char acPtIdx[32];
		cv::circle(oImgPlt, m_vo2dPt[i], 6, cv::Scalar(255, 0, 0), 1, CV_AA);  // draw the circle
		std::sprintf(acPtIdx, "%d", i);
		cv::putText(oImgPlt, acPtIdx, m_vo2dPt[i], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
	}

	// plot the projected 2D points
		// plot the 2D points
	for (int i = 0; i < m_vo2dPt.size(); i++)
	{
		cv::Mat o3dPtMat(3, 1, CV_64F);
		cv::Mat o2dPtMat(3, 1, CV_64F);

		o3dPtMat.at<double>(0, 0) = m_vo3dPt[i].x;
		o3dPtMat.at<double>(1, 0) = m_vo3dPt[i].y;
		o3dPtMat.at<double>(2, 0) = 1;
		o2dPtMat = m_oHomoMat * o3dPtMat;

		cv::circle(oImgPlt, cv::Point2f((o2dPtMat.at<double>(0, 0) / o2dPtMat.at<double>(2, 0)), (o2dPtMat.at<double>(1, 0) / o2dPtMat.at<double>(2, 0))), 
			12, cv::Scalar(0, 0, 255), 1, CV_AA);  // draw the circle
	}

	// display plotted image
	cv::namedWindow("3D grid on the ground plane", CV_WINDOW_NORMAL);
	cv::imshow("3D grid on the ground plane", oImgPlt);
	cv::waitKey(0);
	cv::destroyAllWindows();

	// save plotted image
	if (m_oCfg.getOutCalDispFlg())
		cv::imwrite(m_oCfg.getOutCalDispPth(), oImgPlt);
}


C2dPtSel::C2dPtSel(void)
{

}

C2dPtSel::~C2dPtSel(void)
{

}

void C2dPtSel::initialize(CCfg oCfg, cv::Mat oImgFrm)
{
	// configuration parameters
	m_oCfg = oCfg;

	// frame image for plotting results
	m_oImgFrm = oImgFrm.clone();
}

std::vector<cv::Point> C2dPtSel::process(void)
{
	std::vector<cv::Point> voVanPt;

	if (m_oCfg.getCalSel2dPtFlg())
	{
		std::cout << "Hot keys: \n"
			<< "\tESC - exit\n"
			<< "\tr - re-select a set of 2D points\n"
			<< "\to - finish selecting a set of 2D points\n"
			<< std::endl;

		cv::Mat oImgFrm = m_oImgFrm.clone();

		cv::namedWindow("selector of 2D points", CV_WINDOW_NORMAL);
		cv::imshow("selector of 2D points", m_oImgFrm);
		cv::setMouseCallback("selector of 2D points", on_mouse);  // register for mouse event

		while (1)
		{
			int nKey = cv::waitKey(0);	// read keyboard event

			if (nKey == 27)
				break;

			if (nKey == 'r')  // reset the nodes
			{
				std::vector<cv::Point>().swap(m_voNd);
				m_oImgFrm = oImgFrm.clone();
				cv::imshow("selector of 2D points", m_oImgFrm);
			}

			if (nKey == 'o')	// finish selection of pairs of test points
			{
				cv::destroyWindow("selector of 2D points");
				std::vector<cv::Point2f> vo3dPt = m_oCfg.getCal3dPtLs();
				if (vo3dPt.size() == m_voNd.size())
					return m_voNd;
				else
				{
					std::cout << "Error: Mis-match between the number of selected 2D points and the number of 3D points." << std::endl;
					break;
				}
			}
		}
	}
}

void C2dPtSel::addNd(int nX, int nY)
{
	char acNdIdx[32];
	cv::Point oCurrNd;
	oCurrNd.x = nX;
	oCurrNd.y = nY;

	m_voNd.push_back(oCurrNd);
	// std::cout << "current node(" << oCurrNd.x << "," << oCurrNd.y << ")" << std::endl;	// for debug
	cv::circle(m_oImgFrm, oCurrNd, 6, cv::Scalar(255, 0, 0), 1, CV_AA);  // draw the circle
	std::sprintf(acNdIdx, "%d", (m_voNd.size() - 1));
	cv::putText(m_oImgFrm, acNdIdx, oCurrNd, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
	cv::imshow("selector of 2D points", m_oImgFrm);
}
