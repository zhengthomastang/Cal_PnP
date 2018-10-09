#include "Cfg.h"

CCfg::CCfg()
{
	//std::strcpy(m_acInFrmPth, ".\\data\\frm.jpg");	// in Windows
	std::strcpy(m_acInFrmPth, "./data/frm.jpg");	// in Linux
	//std::strcpy(m_acOutCamMatPth, ".\\data\\calibration.txt");	// in Windows
	std::strcpy(m_acOutCamMatPth, "./data/calibration.txt");	// in Linux
	//std::strcpy(m_acOutCalDispPth, ".\\data\\calibration.jpg");	// in Windows
	std::strcpy(m_acOutCalDispPth, "./data/calibration.jpg");	// in Linux
	m_bOutCalDispFlg = false;
	m_nRszFrmHei = -1;
	m_bCalSel2dPtFlg = true;
	std::vector<cv::Point2f>().swap(m_voCal2dPt);
	std::vector<cv::Point2f>().swap(m_voCal3dPt);
	m_nCalTyp = 0;
	m_fCalRansacReprojThld = 3.0;
	m_oCalDispGrdDim = cv::Size(10, 10);
	m_bCalDistFlg = false;
	std::vector<float>().swap(m_vfCalDistCoeff);
	std::vector<float>().swap(m_vfCalFocLen);
	std::vector<float>().swap(m_vfCalPrinPt);
	m_oCalIntMat = cv::Mat::eye(3, 3, CV_64F);
	m_oCalDistCoeffMat = cv::Mat::zeros(4, 1, CV_64F);
}

CCfg::~CCfg()
{

}

void CCfg::ldCfgFl(char* acCfgFlPth)
{
	FILE * poCfgFl;
	long nlFlSz, nlRdRst;
	char * pcBuf;

	if (NULL == acCfgFlPth)
		//poCfgFl = std::fopen(".\\data\\cfg.json", "r");	// in Windows
		poCfgFl = std::fopen("./data/cfg.json", "r");	// in Linux
	else
		poCfgFl = std::fopen(acCfgFlPth, "r");

	if (NULL == poCfgFl) { std::fputs("Error: configuration file not opened\n", stderr); exit(1); }

	// obtain file size:
	fseek(poCfgFl, 0, SEEK_END);
	nlFlSz = ftell(poCfgFl);
	rewind(poCfgFl);

	// allocate memory to contain the whole file:
	pcBuf = (char*)malloc(sizeof(char)*nlFlSz);
	if (NULL == poCfgFl) { fputs("Memory error", stderr); exit(2); }

	// copy the file into the buffer:
	nlRdRst = fread(pcBuf, 1, nlFlSz, poCfgFl);
	//if (nlRdRst != nlFlSz) { fputs("Reading error", stderr); exit(3); }

	std::string strCfg(pcBuf);
	//strCfg.erase(std::remove_if(strCfg.begin(), strCfg.end(), [](char c) { return c >= 0 && isspace(c); }), strCfg.end());	// in Windows
    strCfg.erase(std::remove_if(strCfg.begin(), strCfg.end(), ::isspace), strCfg.end());	// in Linux

	int nParamPos = strCfg.find("\"inFrmPth\"");
	if (std::string::npos != nParamPos)
		std::strcpy(m_acInFrmPth, rdCharArr(strCfg, nParamPos).c_str());

	nParamPos = strCfg.find("\"outCamMatPth\"");
	if (std::string::npos != nParamPos)
		std::strcpy(m_acOutCamMatPth, rdCharArr(strCfg, nParamPos).c_str());

	nParamPos = strCfg.find("\"outCalDispPth\"");
	if (std::string::npos != nParamPos)
		std::strcpy(m_acOutCalDispPth, rdCharArr(strCfg, nParamPos).c_str());

	nParamPos = strCfg.find("\"outCalDispFlg\"");
	if (std::string::npos != nParamPos)
		m_bOutCalDispFlg = rdBool(strCfg, nParamPos);

	nParamPos = strCfg.find("\"rszFrmHei\"");
	if (std::string::npos != nParamPos)
		m_nRszFrmHei = rdInt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"calSel2dPtFlg\"");
	if (std::string::npos != nParamPos)
		m_bCalSel2dPtFlg = rdBool(strCfg, nParamPos);

	nParamPos = strCfg.find("\"cal2dPtLs\"");
	if (std::string::npos != nParamPos)
		m_voCal2dPt = rdVec2dPt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"cal3dPtLs\"");
	if (std::string::npos != nParamPos)
		m_voCal3dPt = rdVec2dPt(strCfg, nParamPos);

    nParamPos = strCfg.find("\"calTyp\"");
	if (std::string::npos != nParamPos)
		m_nCalTyp = rdInt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"calRansacReprojThld\"");
	if (std::string::npos != nParamPos)
		m_fCalRansacReprojThld = rdDbl(strCfg, nParamPos);

	nParamPos = strCfg.find("\"calDispGrdDim\"");
	if (std::string::npos != nParamPos)
		m_oCalDispGrdDim = rdSz(strCfg, nParamPos);

	nParamPos = strCfg.find("\"calDistFlg\"");
	if (std::string::npos != nParamPos)
		m_bCalDistFlg = rdBool(strCfg, nParamPos);

	nParamPos = strCfg.find("\"calDistCoeff\"");
	if (std::string::npos != nParamPos)
		m_vfCalDistCoeff = rdFltVec(strCfg, nParamPos);

	nParamPos = strCfg.find("\"calFocLen\"");
	if (std::string::npos != nParamPos)
		m_vfCalFocLen = rdFltVec(strCfg, nParamPos);

	nParamPos = strCfg.find("\"calPrinPt\"");
	if (std::string::npos != nParamPos)
		m_vfCalPrinPt = rdFltVec(strCfg, nParamPos);

	// assertion
	CV_Assert(4 <= m_voCal3dPt.size());
	if (!m_bCalSel2dPtFlg)
		CV_Assert(m_voCal2dPt.size() == m_voCal3dPt.size());
	CV_Assert((0 == m_nCalTyp) || (4 == m_nCalTyp) || (8 == m_nCalTyp));
	CV_Assert(1 <= m_fCalRansacReprojThld);
	CV_Assert((1 <= m_oCalDispGrdDim.width) && (1 <= m_oCalDispGrdDim.height));

	if (m_bCalDistFlg)
	{
		CV_Assert(4 == m_vfCalDistCoeff.size());
		CV_Assert((1 == m_vfCalFocLen.size()) || (2 == m_vfCalFocLen.size()));
		CV_Assert(2 == m_vfCalPrinPt.size());

		m_oCalDistCoeffMat.at<double>(0) = m_vfCalDistCoeff[0];
		m_oCalDistCoeffMat.at<double>(1) = m_vfCalDistCoeff[1];
		m_oCalDistCoeffMat.at<double>(2) = m_vfCalDistCoeff[2];
		m_oCalDistCoeffMat.at<double>(3) = m_vfCalDistCoeff[3];

		if (1 == m_vfCalFocLen.size())
		{
			m_oCalIntMat.at<double>(0, 0) = m_vfCalFocLen[0];
			m_oCalIntMat.at<double>(1, 1) = m_vfCalFocLen[0];
		}
		else if (2 == m_vfCalFocLen.size())
		{
			m_oCalIntMat.at<double>(0, 0) = m_vfCalFocLen[0];
			m_oCalIntMat.at<double>(1, 1) = m_vfCalFocLen[1];
		}

		m_oCalIntMat.at<double>(0, 2) = m_vfCalPrinPt[0];
		m_oCalIntMat.at<double>(1, 2) = m_vfCalPrinPt[1];
	}

	// terminate
	fclose(poCfgFl);
	free(pcBuf);
}

std::string CCfg::rdCharArr(std::string strCfg, int nParamPos)
{
	int nValPos, nValLen;

	nValPos = strCfg.find(":", (nParamPos + 1)) + 2;
	nValLen = strCfg.find("\"", (nValPos + 1)) - nValPos;

	return strCfg.substr(nValPos, nValLen);
}

int CCfg::rdInt(std::string strCfg, int nParamPos)
{
	int nValPos, nValLen, nValEnd1, nValEnd2;

	nValPos = strCfg.find(":", (nParamPos + 1)) + 1;
	nValEnd1 = strCfg.find(",", (nValPos + 1));
	nValEnd2 = strCfg.find("}", (nValPos + 1));
	nValLen = (nValEnd1 <= nValEnd2) ? (nValEnd1 - nValPos) : (nValEnd2 - nValPos);

	return std::atoi(strCfg.substr(nValPos, nValLen).c_str());
}

double CCfg::rdDbl(std::string strCfg, int nParamPos)
{
	int nValPos, nValLen, nValEnd1, nValEnd2;

	nValPos = strCfg.find(":", (nParamPos + 1)) + 1;
	nValEnd1 = strCfg.find(",", (nValPos + 1));
	nValEnd2 = strCfg.find("}", (nValPos + 1));
	nValLen = (nValEnd1 <= nValEnd2) ? (nValEnd1 - nValPos) : (nValEnd2 - nValPos);

	return std::atof(strCfg.substr(nValPos, nValLen).c_str());
}

bool CCfg::rdBool(std::string strCfg, int nParamPos)
{
	int nBoolVal, nValPos, nValLen, nValEnd1, nValEnd2;

	nValPos = strCfg.find(":", (nParamPos + 1)) + 1;
	nValEnd1 = strCfg.find(",", (nValPos + 1));
	nValEnd2 = strCfg.find("}", (nValPos + 1));
	nValLen = (nValEnd1 <= nValEnd2) ? (nValEnd1 - nValPos) : (nValEnd2 - nValPos);

	nBoolVal = std::atoi(strCfg.substr(nValPos, nValLen).c_str());
	if (nBoolVal > 0)
		return true;
	else if (nBoolVal <= 0)
		return false;
}

cv::Size CCfg::rdSz(std::string strCfg, int nParamPos)
{
	int nWidPos = nParamPos, nWidLen, nHeiPos, nHeiLen;

	nWidPos = strCfg.find(":", (nParamPos + 1)) + 2;
	nWidLen = strCfg.find(",", (nWidPos + 1)) - nWidPos;
	nHeiPos = nWidPos + nWidLen + 1;
	nHeiLen = strCfg.find("]", (nHeiPos + 1)) - nHeiPos;
	cv::Size oSz(std::atof(strCfg.substr(nWidPos, nWidLen).c_str()), std::atof(strCfg.substr(nHeiPos, nHeiLen).c_str()));

	return oSz;
}

std::vector<cv::Point2f> CCfg::rdVec2dPt(std::string strCfg, int nParamPos)
{
	int nValPos, nValLen, nValEnd, nXPos = nParamPos, nXLen, nYPos, nYLen;
	std::vector<cv::Point2f> vo2dPt;

	nValPos = strCfg.find(":", (nParamPos + 1)) + 1;
	nValEnd = strCfg.find("]]", (nValPos + 1));

	while (nValPos < nValEnd)
	{
		nXPos = strCfg.find("[", (nValPos + 1)) + 1;
		nXLen = strCfg.find(",", (nXPos + 1)) - nXPos;
		nYPos = nXPos + nXLen + 1;
		nYLen = strCfg.find("]", (nYPos + 1)) - nYPos;
		vo2dPt.push_back(cv::Point2f(std::atof(strCfg.substr(nXPos, nXLen).c_str()), std::atof(strCfg.substr(nYPos, nYLen).c_str())));
		nValPos = nYPos + nYLen + 1;
	}

	return vo2dPt;
}

std::vector<float> CCfg::rdFltVec(std::string strCfg, int nParamPos)
{
	int nValPos, nValLen, nValEnd;
	std::vector<float> vfVal;

	nValPos = strCfg.find(":", (nParamPos + 1)) + 2;
	nValEnd = strCfg.find("]", (nValPos + 1));

	while (nValPos < nValEnd)
	{
		nValLen = strCfg.find(",", (nValPos + 1)) - nValPos;
		if (0 > nValLen)
			nValLen = nValEnd - nValPos;
		vfVal.push_back(std::atof(strCfg.substr(nValPos, nValLen).c_str()));
		nValPos = nValPos + nValLen + 1;
	}

	return vfVal;
}
