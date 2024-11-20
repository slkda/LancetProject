/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/


// Blueberry
#include <berryISelectionService.h>
#include <berryIWorkbenchWindow.h>

// Qmitk
#include "HTOrobot.h"


// mitk image
#include <mitkImage.h>

AimHandle aimHandle = NULL;
E_Interface EI;
T_MarkerInfo markerSt;
T_AimPosStatusInfo statusSt;
const double PI = 3.1415926;

using namespace Eigen;
using namespace std;

const std::string HTOrobot::VIEW_ID = "org.mitk.views.htorobot";

void HTOrobot::SetFocus()
{
  //m_Controls.buttonPerformImageProcessing->setFocus();
}

void HTOrobot::CreateQtPartControl(QWidget *parent)
{
  // create GUI widgets from the Qt Designer's .ui file
  m_Controls.setupUi(parent);
  //connect(m_Controls.buttonPerformImageProcessing, &QPushButton::clicked, this, &HTOrobot::DoImageProcessing);
  //Hans Robot
  connect(m_Controls.pushButton_connectHans_2, &QPushButton::clicked, this, &HTOrobot::connectHans);
  connect(m_Controls.pushButton_powerOn, &QPushButton::clicked, this, &HTOrobot::HansPowerOn);
  connect(m_Controls.pushButton_powerOff, &QPushButton::clicked, this, &HTOrobot::HansPowerOFF);
  connect(m_Controls.pushButton_openHandGuiding, &QPushButton::clicked, this, &HTOrobot::HandGuiding);
  connect(m_Controls.pushButton_closeHandGuiding, &QPushButton::clicked, this, &HTOrobot::closeHandGuiding);
  connect(m_Controls.pushButton_suddenStop, &QPushButton::clicked, this, &HTOrobot::suddenStop);
  //set tcp to flange
  connect(m_Controls.pushButton_setTcpInFlange, &QPushButton::clicked, this, &HTOrobot::SetTcpToFlange);//table1
  connect(m_Controls.pushButton_setTcpToflange, &QPushButton::clicked, this, &HTOrobot::SetTcpToFlange);//table2

  connect(m_Controls.pushButton_connectAim, &QPushButton::clicked, this, &HTOrobot::connectAimooe);
  connect(m_Controls.pushButton_updateData, &QPushButton::clicked, this, &HTOrobot::upDateData);
  //Robot Registration
  connect(m_Controls.pushButton_captureRobot, &QPushButton::clicked, this, &HTOrobot::captureRobot);
  connect(m_Controls.pushButton_replaceMatrix, &QPushButton::clicked, this, &HTOrobot::ReplaceRegistration);
  connect(m_Controls.pushButton_saveMatrix, &QPushButton::clicked, this, &HTOrobot::saveRegistration);
  connect(m_Controls.pushButton_reuseMatrix, &QPushButton::clicked, this, &HTOrobot::reuseRegistration);
  connect(m_Controls.pushButton_printMatrix, &QPushButton::clicked, this, &HTOrobot::PrintToolMatrix);
  
  connect(m_Controls.pushButton_setinitial, &QPushButton::clicked, this, &HTOrobot::setInitializationPoint);
  connect(m_Controls.pushButton_gotoinitial, &QPushButton::clicked, this, &HTOrobot::gotoInitialization);
  connect(m_Controls.pushButton_setInitialPoint, &QPushButton::clicked, this, &HTOrobot::setInitializationPoint);
  connect(m_Controls.pushButton_GotoInitialPoint, &QPushButton::clicked, this, &HTOrobot::gotoInitialization);


  //Robot move
  connect(m_Controls.pushButton_xp_2, &QPushButton::clicked, this, &HTOrobot::xp);
  connect(m_Controls.pushButton_yp_2, &QPushButton::clicked, this, &HTOrobot::yp);
  connect(m_Controls.pushButton_zp_2, &QPushButton::clicked, this, &HTOrobot::zp);
  connect(m_Controls.pushButton_xm_2, &QPushButton::clicked, this, &HTOrobot::xm);
  connect(m_Controls.pushButton_ym_2, &QPushButton::clicked, this, &HTOrobot::ym);
  connect(m_Controls.pushButton_zm_2, &QPushButton::clicked, this, &HTOrobot::zm);
  connect(m_Controls.pushButton_rxp_2, &QPushButton::clicked, this, &HTOrobot::rxp);
  connect(m_Controls.pushButton_ryp_2, &QPushButton::clicked, this, &HTOrobot::ryp);
  connect(m_Controls.pushButton_rzp_2, &QPushButton::clicked, this, &HTOrobot::rzp);
  connect(m_Controls.pushButton_rxm_2, &QPushButton::clicked, this, &HTOrobot::rxm);
  connect(m_Controls.pushButton_rym_2, &QPushButton::clicked, this, &HTOrobot::rym);
  connect(m_Controls.pushButton_rzm_2, &QPushButton::clicked, this, &HTOrobot::rzm);

 

  //Osteotomy guide calibration
  connect(m_Controls.pushButton_cailbration, &QPushButton::clicked, this, &HTOrobot::OstGuidCalibration);
  
  //脊柱项目探针标定TCP
  //connect(m_Controls.CalculateGuideTCP, &QPushButton::clicked, this, &HTOrobot::CalculateGuideTCP);
  connect(m_Controls.GetProbeEndPosOneBtn, &QPushButton::clicked, this, [=]() {
	  GetProbeEndPosBtnClicked(1);
	});//获取探针末端位置

  connect(m_Controls.GetGuiderOriginPosBtn, &QPushButton::clicked, this, &HTOrobot::GetGuiderOriginPosBtnClicked);
  //获取原点位置

  connect(m_Controls.ResetProbeEndPosOneBtn, &QPushButton::clicked, this, [=]() {
	  ResetProbeEndPosBtnClicked(1);
	  });//重置

  connect(m_Controls.CalculateGuiderTCPOneBtn, &QPushButton::clicked, this, &HTOrobot::CalculateGuideTCP);

  connect(m_Controls.pushButton_StartDisplayTCPAxesActor, &QPushButton::clicked, this, &HTOrobot::StartDisplayTCPAxesActor);

  probeEndOneVector.resize(4);


}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
void HTOrobot::OnSelectionChanged(berry::IWorkbenchPart::Pointer /*source*/,
                                                const QList<mitk::DataNode::Pointer> &nodes)
{
  //// iterate all selected objects, adjust warning visibility
  //foreach (mitk::DataNode::Pointer node, nodes)
  //{
  //  if (node.IsNotNull() && dynamic_cast<mitk::Image *>(node->GetData()))
  //  {
  //    m_Controls.labelWarning->setVisible(false);
  //    m_Controls.buttonPerformImageProcessing->setEnabled(true);
  //    return;
  //  }
  //}

  //m_Controls.labelWarning->setVisible(true);
  //m_Controls.buttonPerformImageProcessing->setEnabled(false);
}
void HTOrobot::DoImageProcessing()
{
  //QList<mitk::DataNode::Pointer> nodes = this->GetDataManagerSelection();
  //if (nodes.empty())
  //  return;

  //mitk::DataNode *node = nodes.front();

  //if (!node)
  //{
  //  // Nothing selected. Inform the user and return
  //  QMessageBox::information(nullptr, "Template", "Please load and select an image before starting image processing.");
  //  return;
  //}

  //// here we have a valid mitk::DataNode

  //// a node itself is not very useful, we need its data item (the image)
  //mitk::BaseData *data = node->GetData();
  //if (data)
  //{
  //  // test if this data item is an image or not (could also be a surface or something totally different)
  //  mitk::Image *image = dynamic_cast<mitk::Image *>(data);
  //  if (image)
  //  {
  //    std::stringstream message;
  //    std::string name;
  //    message << "Performing image processing for image ";
  //    if (node->GetName(name))
  //    {
  //      // a property called "name" was found for this DataNode
  //      message << "'" << name << "'";
  //    }
  //    message << ".";
  //    MITK_INFO << message.str();

  //    // actually do something here...
  //  }
  //}
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void HTOrobot::connectHans()
{
	int nRet = HRIF_Connect(boxID, hostname, nPort);
	if (HRIF_IsConnected(0))
	{
		m_Controls.textBrowser_HTO->append("Robotic arm Hans connection successfully!");
	}
	else
	{
		m_Controls.textBrowser_HTO->append("disconnect!");
	}
}

void HTOrobot::HansPowerOn()
{
	if (!HRIF_GrpEnable(boxID, rbtID))
	{
		m_Controls.textBrowser_HTO->append("power on successfully");
	}
	else
	{
		m_Controls.textBrowser_HTO->append("failed");
	}
}

void HTOrobot::HansPowerOFF()
{
	if (HRIF_GrpDisable(boxID, rbtID) == 0)
	{
		m_Controls.textBrowser_HTO->append("power off true");
	}
	else
	{
		m_Controls.textBrowser_HTO->append("failed");
	}
}

void HTOrobot::HandGuiding()
{
	HRIF_GrpOpenFreeDriver(boxID, rbtID);
	m_Controls.textBrowser_HTO->append("Open handguiding");
}

void HTOrobot::closeHandGuiding()
{
	HRIF_GrpCloseFreeDriver(boxID, rbtID);
	m_Controls.textBrowser_HTO->append("Turn OFF handguiding");
}

void HTOrobot::SetTcpToFlange()
{
	m_Controls.textBrowser_HTO->append("TCP initialization");
	dTcp_Rx = 0; dTcp_Ry = 0; dTcp_Rz = 0; dTcp_X = 0; dTcp_Y = 0; dTcp_Z = 0;
	int nRet = HRIF_SetTCP(boxID, rbtID, dTcp_X, dTcp_Y, dTcp_Z, dTcp_Rx, dTcp_Ry, dTcp_Rz);
	if (nRet == 0)
	{
		MITK_INFO << "TCP initialization successfully";
	}
	else
	{
		MITK_INFO << "failed";
	}
}

void HTOrobot::connectAimooe()
{
	T_AIMPOS_DATAPARA mPosDataPara;
	Aim_API_Initial(aimHandle);
	Aim_SetEthernetConnectIP(aimHandle, 192, 168, 31, 10);
	rlt = Aim_ConnectDevice(aimHandle, I_ETHERNET, mPosDataPara);


	if (rlt == AIMOOE_OK)
	{

		qDebug() << "connect success";
	}
	else {

		qDebug() << "connect failed";
	}

	QString filename = QFileDialog::getExistingDirectory(nullptr, "Select the Tools store folder", "");
	if (filename.isNull()) return;
	filename.append("/");
	qDebug() << "The selected folder address :" << filename;
	rlt = Aim_SetToolInfoFilePath(aimHandle, filename.toLatin1().data());

	if (rlt == AIMOOE_OK)
	{

		qDebug() << "set filenemae success";
	}
	else {

		qDebug() << "set filenemae failed";
	}

	int size = 0;
	Aim_GetCountOfToolInfo(aimHandle, size);

	if (size != 0)
	{
		t_ToolBaseInfo* toolarr = new t_ToolBaseInfo[size];

		rlt = Aim_GetAllToolFilesBaseInfo(aimHandle, toolarr);

		if (rlt == AIMOOE_OK)
		{
			for (int i = 0; i < size; i++)
			{
				/*		char* ptool = toolarr[i].name;
						QString toolInfo = QString("Tool Name：") + QString::fromLocal8Bit(ptool);
						m_Controls.textBrowser_HTO->append(toolInfo);*/
			}
		}
		delete[] toolarr;
	}
	else {
		std::cout << "There are no tool identification files in the current directory:";
	}

	std::cout << "End of connection";
	rlt = AIMOOE_OK;
}

void HTOrobot::upDateData()
{
	if (m_AimoeVisualizeTimer == nullptr)
	{
		m_AimoeVisualizeTimer = new QTimer(this);
	}
	/*connect(m_AimoeVisualizeTimer, SIGNAL(timeout()), this, SLOT(getMatrix()));*/
	connect(m_AimoeVisualizeTimer, SIGNAL(timeout()), this, SLOT(GetMatrix_1()));


	m_AimoeVisualizeTimer->start(100);
}

void HTOrobot::setInitializationPoint()
{
	m_Controls.textBrowser_HTO->append("Set Initial Point");
	int initializationPoints = HRIF_ReadActPos(boxID, rbtID, dX, dY, dZ, dRx, dRy, dRz, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, dTcp_X, dTcp_Y,
		dTcp_Z, dTcp_Rx, dTcp_Ry, dTcp_Rz, dUcs_X, dUcs_Y, dUcs_Z, dUcs_Rx, dUcs_Ry, dUcs_Rz);
	if (initializationPoints == 0)
	{
		std::cout << "Initial Position:X=" << dX << "Initial Position:Y=" << dY << "Initial Position:Z=" << dZ << "Initial Position:Rx=" << dRx << "Initial Position:Ry=" << dRy << "Initial Position:Rz=" << dRz << std::endl;
	}
	else
	{
		std::cerr << "Failed to read initial Position.Error code:" << initializationPoints << std::endl;
	}
}

void HTOrobot::gotoInitialization()
{
	m_Controls.textBrowser_HTO->append("Goto Initial Points");
	int nRet = HRIF_MoveJ(0, 0, dX, dY, dZ, dRx, dRy, dRz, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, sTcpName, sUcsName,
		dVelocity, dAcc, dRadius, nIsUseJoint, nIsSeek, nIOBit, nIOState, strCmdID);
}

void HTOrobot::getToolInfor()
{
	rlt = Aim_GetMarkerAndStatusFromHardware(aimHandle, I_ETHERNET, markerSt, statusSt);
	if (rlt == AIMOOE_NOT_REFLASH)
	{
		std::cout << "camera get data failed";
	}

	std::vector<int> residualIndex;
	for (int i = 0; i < markerSt.MarkerNumber; i++)
	{
		residualIndex.push_back(i);
	}

	T_AimToolDataResult* mtoolsrlt = new T_AimToolDataResult;
	mtoolsrlt->next = NULL;
	mtoolsrlt->validflag = false;
	rlt = Aim_FindToolInfo(aimHandle, markerSt, mtoolsrlt, 0);
	T_AimToolDataResult* prlt = mtoolsrlt;

	if (rlt == AIMOOE_OK)
	{
		do
		{
			if (prlt->validflag)
			{
				float Pi = 3.141592;
				std::cout << "找到工具: " << prlt->toolname << "  平均误差" << prlt->MeanError << " RMS误差" << prlt->Rms;
				std::cout << "工具原点:" << prlt->OriginCoor[0] << "," << prlt->OriginCoor[1] << "," << prlt->OriginCoor[2];
				std::cout << "工具角度:" << prlt->rotationvector[0] * 180 / PI << "," << prlt->rotationvector[1] * 180 / PI << "," << prlt->rotationvector[2] * 180 / PI;
				std::cout << "标记点坐标:";
				for (int i = 0; i < prlt->toolptidx.size(); i++)
				{
					int idx = prlt->toolptidx[i];
					std::vector<int>::iterator iter = residualIndex.begin();
					iter = find(residualIndex.begin(), residualIndex.end(), idx);
					if (iter != residualIndex.end())
					{
						residualIndex.erase(iter);
					}
					if (idx < 0)
						cout << "0 0 0";
					else
						cout << markerSt.MarkerCoordinate[idx * 3 + 0] << " " << markerSt.MarkerCoordinate[idx * 3 + 1] << " " << markerSt.MarkerCoordinate[idx * 3 + 2] << " ";
				}
			}
			T_AimToolDataResult* pnext = prlt->next;
			delete prlt;
			prlt = pnext;
		} while (prlt != NULL);
		/*cout << endl;*/
		if (residualIndex.size() > 0)
		{
			/*		cout << "in all" << residualIndex.size() << "point：";
					for (int i = 0; i < residualIndex.size(); i++)
					{
						int j = residualIndex[i];
						cout << i << ":" << markerSt.MarkerCoordinate[j * 3 + 0] << "," << markerSt.MarkerCoordinate[j * 3 + 1] << "," << markerSt.MarkerCoordinate[j * 3 + 2];
					}*/
		}
	}
	else
	{
		delete prlt;
	}
	std::cout << "查找结束";
	rlt = AIMOOE_OK;
}

void HTOrobot::GetMatrix()//Print Matrix
{
	QString position_text;
	std::vector<std::string> toolidarr;
	toolidarr.push_back("Spine_RobotBaseRF");
	toolidarr.push_back("Spine_RobotEndRF");
	toolidarr.push_back("Spine_Probe");
	toolidarr.push_back("Spine_PatientRF");
	toolidarr.push_back("Spine_MetalBallRF");

	rlt = Aim_GetMarkerAndStatusFromHardware(aimHandle, I_ETHERNET, markerSt, statusSt);
	T_AimToolDataResult* mtoolsrlt = new T_AimToolDataResult;
	mtoolsrlt->next = nullptr;
	mtoolsrlt->validflag = false;

	rlt = Aim_FindToolInfo(aimHandle, markerSt, mtoolsrlt, 0);
	T_AimToolDataResult* prlt = mtoolsrlt;

	if (rlt == AIMOOE_OK)
	{
		do
		{
			////获取HTO_RobotBaseRF
			if (strcmp(prlt->toolname, "HTO_RobotBaseRF") == 0)
			{
				if (prlt->validflag)
				{
					t_tran[0] = prlt->Tto[0];
					t_tran[1] = prlt->Tto[1];
					t_tran[2] = prlt->Tto[2];
					for (int i = 0; i < 3; i++)
					{
						for (int j = 0; j < 3; j++)
						{
							R_tran[i][j] = prlt->Rto[i][j];
						}
					}
					//拼接矩阵
					vtkNew<vtkMatrix4x4> m_T_temp1;
					CombineRotationTranslation(R_tran, t_tran, m_T_temp1);
					memcpy_s(T_CamToBaseRF, sizeof(double) * 16, m_T_temp1->GetData(), sizeof(double) * 16);

				}
				else
				{

				}
			}

			//获取HTO_Probe数据
			if (strcmp(prlt->toolname, "HTO_Probe") == 0)
			{
				if (prlt->validflag)
				{
					t_tran[0] = prlt->Tto[0];
					t_tran[1] = prlt->Tto[1];
					t_tran[2] = prlt->Tto[2];
					for (int i = 0; i < 3; i++)
					{
						for (int j = 0; j < 3; j++)
						{
							R_tran[i][j] = prlt->Rto[i][j];
						}
					}
					//拼接矩阵
					vtkNew<vtkMatrix4x4> m_T_temp2;
					CombineRotationTranslation(R_tran, t_tran, m_T_temp2);
					memcpy_s(T_CamToBaseRF, sizeof(double) * 16, m_T_temp2->GetData(), sizeof(double) * 16);
					ProbeTop[0] = prlt->tooltip[0];
					ProbeTop[1] = prlt->tooltip[1];
					ProbeTop[2] = prlt->tooltip[2];
				}
				else
				{

				}
			}
			//获取HTO_RobotEndRF数据
			if (strcmp(prlt->toolname, "HTO_RobotEndRF") == 0)
			{
				if (prlt->validflag)
				{
					t_tran[0] = prlt->Tto[0];
					t_tran[1] = prlt->Tto[1];
					t_tran[2] = prlt->Tto[2];
					for (int i = 0; i < 3; i++)
					{
						for (int j = 0; j < 3; j++)
						{
							R_tran[i][j] = prlt->Rto[i][j];
						}
					}
					//拼接矩阵
					vtkNew<vtkMatrix4x4> m_T_temp3;
					CombineRotationTranslation(R_tran, t_tran, m_T_temp3);
					memcpy_s(T_CamToEndRF, sizeof(double) * 16, m_T_temp3->GetData(), sizeof(double) * 16);

				}
				else
				{

				}
			}
			//获取HTO_PatientRF数据
			if (strcmp(prlt->toolname, "HTO_PatientRF") == 0)
			{
				if (prlt->validflag)
				{
					t_tran[0] = prlt->Tto[0];
					t_tran[1] = prlt->Tto[1];
					t_tran[2] = prlt->Tto[2];
					for (int i = 0; i < 3; i++)
					{
						for (int j = 0; j < 3; j++)
						{
							R_tran[i][j] = prlt->Rto[i][j];
						}
					}
					//拼接矩阵
					vtkNew<vtkMatrix4x4> m_T_temp4;
					CombineRotationTranslation(R_tran, t_tran, m_T_temp4);
					memcpy_s(T_CamToPatientRF, sizeof(double) * 16, m_T_temp4->GetData(), sizeof(double) * 16);

				}
				else
				{

				}
			}
			T_AimToolDataResult* pnext = prlt->next;
			delete prlt;
			prlt = pnext;
		} while (prlt != NULL);
	}
	else
	{
		delete prlt;
	}
}

void HTOrobot::GetMatrix_1()
{
	QString position_text;
	std::vector<std::string> toolidarr;
	toolidarr.push_back("HTO_RobotBaseRF");
	toolidarr.push_back("HTO_RobotEndRF");
	toolidarr.push_back("HTO_Probe");
	toolidarr.push_back("HTO_PatientRF");
	toolidarr.push_back("HTO_MetalBallRF");

	rlt = Aim_GetMarkerAndStatusFromHardware(aimHandle, I_ETHERNET, markerSt, statusSt);

	T_AimToolDataResult* mtoolsrlt = new T_AimToolDataResult;//新建一个值指，将指针清空用于存数据
	mtoolsrlt->next = NULL;
	mtoolsrlt->validflag = false;

	rlt = Aim_FindToolInfo(aimHandle, markerSt, mtoolsrlt, 0);//获取数据
	T_AimToolDataResult* prlt = mtoolsrlt;//将获取完数据的从mtoolsrlt给prlt指针


	if (rlt == AIMOOE_OK)//判断是否采集成功
	{
		do
		{
			//获取Spine_RobotBaseRF
			if (strcmp(prlt->toolname, "HTO_RobotBaseRF") == 0)
			{
				if (prlt->validflag)
				{
					//获取相机数据
					t_tran[0] = prlt->Tto[0];
					t_tran[1] = prlt->Tto[1];
					t_tran[2] = prlt->Tto[2];
					for (int i = 0; i < 3; ++i)
					{
						for (int j = 0; j < 3; ++j)
						{
							R_tran[i][j] = prlt->Rto[i][j];
						}
					}

					//拼接矩阵
					vtkNew<vtkMatrix4x4> m_T_temp1;
					CombineRotationTranslation(R_tran, t_tran, m_T_temp1);
					memcpy_s(T_CamToBaseRF, sizeof(double) * 16, m_T_temp1->GetData(), sizeof(double) * 16);

					//改边QT界面（变色+显示坐标）
					//m_Controls.textBrowser->append(QString("Spine_RobotBaseRF: %1, %2, %3").arg(prlt->Tto[0]).arg(prlt->Tto[1]).arg(prlt->Tto[2]));


				}
				else
				{
					//  QT界面变红
					/*m_Controls.textBrowser->append("Spine_RobotBaseRF:Failed");*/
				}

			}

			//获取Spine_Probe数据
			if (strcmp(prlt->toolname, "HTO_Probe") == 0)
			{
				if (prlt->validflag)
				{
					//获取相机数据
					t_tran[0] = prlt->Tto[0];
					t_tran[1] = prlt->Tto[1];
					t_tran[2] = prlt->Tto[2];
					for (int i = 0; i < 3; ++i)
					{
						for (int j = 0; j < 3; ++j)
						{
							R_tran[i][j] = prlt->Rto[i][j];
						}
					}

					//拼接矩阵
					vtkNew<vtkMatrix4x4> m_T_temp2;
					CombineRotationTranslation(R_tran, t_tran, m_T_temp2);
					memcpy_s(T_CamToProbe, sizeof(double) * 16, m_T_temp2->GetData(), sizeof(double) * 16);
					ProbeTop[0] = prlt->tooltip[0];
					ProbeTop[1] = prlt->tooltip[1];
					ProbeTop[2] = prlt->tooltip[2];

					//改边QT界面（变色+显示坐标）
					//m_Controls.textBrowser->append(QString("Spine_Probe: %1, %2, %3").arg(prlt->Tto[0]).arg(prlt->Tto[1]).arg(prlt->Tto[2]));

				}
				else
				{
					//  QT界面变红
					/*m_Controls.textBrowser->append("Spine_Probe:Failed");*/

				}

			}

			//获取Spine_RobotEndRF数据
			if (strcmp(prlt->toolname, "HTO_RobotEndRF") == 0)
			{
				if (prlt->validflag)
				{
					//获取相机数据
					t_tran[0] = prlt->Tto[0];
					t_tran[1] = prlt->Tto[1];
					t_tran[2] = prlt->Tto[2];
					for (int i = 0; i < 3; ++i)
					{
						for (int j = 0; j < 3; ++j)
						{
							R_tran[i][j] = prlt->Rto[i][j];
						}
					}

					//拼接矩阵
					vtkNew<vtkMatrix4x4> m_T_temp3;
					CombineRotationTranslation(R_tran, t_tran, m_T_temp3);
					memcpy_s(T_CamToEndRF, sizeof(double) * 16, m_T_temp3->GetData(), sizeof(double) * 16);

					//改边QT界面（变色+显示坐标）
					//m_Controls.textBrowser->append(QString("Spine_RobotEndRF: %1, %2, %3").arg(prlt->Tto[0]).arg(prlt->Tto[1]).arg(prlt->Tto[2]));


				}
				else
				{
					//  QT界面变红
					/*m_Controls.textBrowser->append("Spine_RobotEndRF:Failed");*/

				}

			}

			//获取Spine_PatientRF数据
			if (strcmp(prlt->toolname, "HTO_PatientRF") == 0)
			{
				if (prlt->validflag)
				{
					//获取相机数据
					t_tran[0] = prlt->Tto[0];
					t_tran[1] = prlt->Tto[1];
					t_tran[2] = prlt->Tto[2];
					for (int i = 0; i < 3; ++i)
					{
						for (int j = 0; j < 3; ++j)
						{
							R_tran[i][j] = prlt->Rto[i][j];
						}
					}

					//拼接矩阵
					vtkNew<vtkMatrix4x4> m_T_temp4;
					CombineRotationTranslation(R_tran, t_tran, m_T_temp4);
					memcpy_s(T_CamToPatientRF, sizeof(double) * 16, m_T_temp4->GetData(), sizeof(double) * 16);

					//改边QT界面（变色+显示坐标）
					//m_Controls.textBrowser->append(QString("Spine_PatientRF: %1, %2, %3").arg(prlt->Tto[0]).arg(prlt->Tto[1]).arg(prlt->Tto[2]));

				}
				else
				{
					//  QT界面变红
					/*m_Controls.textBrowser->append("Spine_PatientRF:Failed");*/

				}

			}


			//获取Spine_Probe数据
			if (strcmp(prlt->toolname, "HTO_Probe") == 0)
			{
				if (prlt->validflag)
				{
					ProbeTop[0] = prlt->tooltip[0];
					ProbeTop[1] = prlt->tooltip[1];
					ProbeTop[2] = prlt->tooltip[2];
				}
			}
			////获取Spine_MetalBallRF数据
			//if (strcmp(prlt->toolname, "Spine_MetalBallRF") == 0)
			//{
			//	if (prlt->validflag)
			//	{
			//		//获取相机数据
			//		t_tran[0] = prlt->Tto[0];
			//		t_tran[1] = prlt->Tto[1];
			//		t_tran[2] = prlt->Tto[2];
			//		for (int i = 0; i < 3; ++i)
			//		{
			//			for (int j = 0; j < 3; ++j)
			//			{
			//				R_tran[i][j] = prlt->Rto[i][j];
			//			}
			//		}

			//		//拼接矩阵
			//		vtkNew<vtkMatrix4x4> m_T_temp4;
			//		CombineRotationTranslation(R_tran, t_tran, m_T_temp4);
			//		memcpy_s(T_CamToMetalBallRF, sizeof(double) * 16, m_T_temp4->GetData(), sizeof(double) * 16);

			//		//改边QT界面（变色+显示坐标）
			//		//m_Controls.textBrowser->append(QString("Spine_PatientRF: %1, %2, %3").arg(prlt->Tto[0]).arg(prlt->Tto[1]).arg(prlt->Tto[2]));

			//	}
			//	else
			//	{
			//		//  QT界面变红
			//		/*m_Controls.textBrowser->append("Spine_PatientRF:Failed");*/

			//	}

			//}

			T_AimToolDataResult* pnext = prlt->next;
			delete prlt;
			prlt = pnext;
		} while (prlt != NULL);
		/*cout << endl;*/

	}
	else
	{
		delete prlt;
	}

}

void HTOrobot::getBaseToFlangeMatrix()
{
	//Read TCP get T_BaseToFlanger
	double dX = 0; double dY = 0; double dZ = 0;
	double dRx = 0; double dRy = 0; double dRz = 0;
	int nRet = HRIF_ReadActTcpPos(0, 0, dX, dY, dZ, dRx, dRy, dRz);
	if (nRet == 0)
	{
		qDebug() << "get BaseToFlange Matrix";
		m_Controls.textBrowser_HTO->append("get BaseToFlange Matrix");

		std::cout << "Current TCP Position and Orientation:" << std::endl;
		std::cout << "Position - X: " << dX << ", Y: " << dY << ", Z: " << dZ << std::endl;
		std::cout << "Orientation - Rx: " << dRx << ", Ry: " << dRy << ", Rz: " << dRz << std::endl;

		auto tmpTrans = vtkTransform::New();
		tmpTrans->PostMultiply();
		tmpTrans->RotateZ(dRz);
		tmpTrans->RotateY(dRy);
		tmpTrans->RotateX(dRx);
		tmpTrans->Translate(dX, dY, dZ);
		tmpTrans->Update();

		//VTKT_BaseToFlanger
		vtkSmartPointer<vtkMatrix4x4> VTKT_BaseToFlanger = tmpTrans->GetMatrix();
		QVector<double> _vtkMatrix4x4;
		_vtkMatrix4x4 = { VTKT_BaseToFlanger->GetElement(0,0), VTKT_BaseToFlanger->GetElement(0, 1), VTKT_BaseToFlanger->GetElement(0, 2), VTKT_BaseToFlanger->GetElement(0,3),
						  VTKT_BaseToFlanger->GetElement(1, 0),VTKT_BaseToFlanger->GetElement(1, 1), VTKT_BaseToFlanger->GetElement(1, 2), VTKT_BaseToFlanger->GetElement(1,3),
						  VTKT_BaseToFlanger->GetElement(2, 0), VTKT_BaseToFlanger->GetElement(2, 1), VTKT_BaseToFlanger->GetElement(2, 2), VTKT_BaseToFlanger->GetElement(2,3),
						  VTKT_BaseToFlanger->GetElement(3, 0), VTKT_BaseToFlanger->GetElement(3, 1), VTKT_BaseToFlanger->GetElement(3, 2), VTKT_BaseToFlanger->GetElement(3,3)
		};
		std::cout << "VTKT_BaseToFlanger Matrix:" << std::endl << std::endl;
		for (int i = 0; i < 4; ++i)
		{
			for (int j = 0; j < 4; ++j)
			{
				std::cout << VTKT_BaseToFlanger->GetElement(i, j) << " ";
			}
			std::cout << std::endl;
		}
	}
	else {
		//MITK_INFO << "Failed to read TCP position and orientation.";
		m_Controls.textBrowser_HTO->append("Failed to read TCP position and orientation.");
	}

}

void HTOrobot::CombineRotationTranslation(float Rto[3][3], float Tto[3], vtkMatrix4x4* resultMatrix)
{
	// Set rotation part
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			resultMatrix->SetElement(i, j, Rto[i][j]);
		}
	}
	for (int i = 0; i < 3; ++i)
	{
		resultMatrix->SetElement(i, 3, Tto[i]);
	}
}

void HTOrobot::suddenStop()
{
	HRIF_GrpStop(boxID, rbtID);
	MITK_INFO << "HRIF_GrpStop(boxID, rbtID);" << HRIF_GrpStop(boxID, rbtID);;
}
//
//void HTOrobot::captureRobot()
//{
//	m_Controls.textBrowser_HTO->append("captureRobot");
//	if (hto_IndexOfRobotCapture < 5)
//	{
//		hto_IndexOfRobotCapture++;
//		m_Controls.lineEdit_HansCapture->setText(QString::number(hto_IndexOfRobotCapture));
//		std::cout << "hto_IndexOfRobotCapture:" << hto_IndexOfRobotCapture << std::endl;
//		HansCapturePose(true);
//	}
//	else if (hto_IndexOfRobotCapture < 10)
//	{
//		hto_IndexOfRobotCapture++;
//		m_Controls.lineEdit_HansCapture->setText(QString::number(hto_IndexOfRobotCapture));
//		std::cout << "hto_IndexOfRobotCapture:" << hto_IndexOfRobotCapture << std::endl;
//		HansCapturePose(false);
//	}
//	else
//	{
//		vtkNew<vtkMatrix4x4> robotEndToFlangeMatrix;
//		hto_RobotRegistration.GetTCPmatrix(robotEndToFlangeMatrix);
//
//		vtkMatrix4x4* matrix4x4 = vtkMatrix4x4::New();
//		hto_RobotRegistration.GetRegistraionMatrix(matrix4x4);
//
//		double x = robotEndToFlangeMatrix->GetElement(0, 3);
//		double y = robotEndToFlangeMatrix->GetElement(1, 3);
//		double z = robotEndToFlangeMatrix->GetElement(2, 3);
//
//		std::cout << "X: " << x << std::endl;
//		std::cout << "Y: " << y << std::endl;
//		std::cout << "Z: " << z << std::endl;
//
//		robotEndToFlangeMatrix->Invert();
//		m_Controls.textBrowser_HTO->append("Registration RMS:" + QString::number(hto_RobotRegistration.RMS()));
//		std::cout << "Registration RMS:" << hto_RobotRegistration.RMS() << std::endl;
//
//		vtkMatrix4x4* VtkT_BaseToBaseRF = vtkMatrix4x4::New();
//		hto_RobotRegistration.GetRegistraionMatrix(VtkT_BaseToBaseRF);
//		VtkT_BaseToBaseRF->Invert();
//		memcpy_s(T_BaseToBaseRF, sizeof(double) * 16, VtkT_BaseToBaseRF->GetData(), sizeof(double) * 16);
//
//		vtkMatrix4x4* vtkT_FlangeToEdnRF = vtkMatrix4x4::New();
//		hto_RobotRegistration.GetTCPmatrix(vtkT_FlangeToEdnRF);
//		memcpy_s(T_FlangeToEdnRF, sizeof(double) * 16, vtkT_FlangeToEdnRF->GetData(), sizeof(double) * 16);
//
//	}
//
//}
//
//void HTOrobot::HansCapturePose(bool translationOnly)
//{
//	double dX = 0; double dY = 0; double dZ = 0;
//	double dRx = 0; double dRy = 0; double dRz = 0;
//	int nRet = HRIF_ReadActTcpPos(boxID, rbtID, dX, dY, dZ, dRx, dRy, dRz);
//
//	auto tempTrans = vtkTransform::New();
//	tempTrans->PostMultiply();
//	tempTrans->RotateX(dRx);
//	tempTrans->RotateY(dRy);
//	tempTrans->RotateZ(dRz);
//	tempTrans->Translate(dX, dY, dZ);
//	tempTrans->Update();
//
//	vtkSmartPointer<vtkMatrix4x4> VTK_BaseToFlange = tempTrans->GetMatrix();//从机械臂系统中获取BaseToFlange
//	QVector<double>  z_vtkMatrix4x4;
//	z_vtkMatrix4x4 = { VTK_BaseToFlange->GetElement(0,0), VTK_BaseToFlange->GetElement(0, 1), VTK_BaseToFlange->GetElement(0, 2), VTK_BaseToFlange->GetElement(0,3),
//				  VTK_BaseToFlange->GetElement(1, 0),VTK_BaseToFlange->GetElement(1, 1), VTK_BaseToFlange->GetElement(1, 2), VTK_BaseToFlange->GetElement(1,3),
//				  VTK_BaseToFlange->GetElement(2, 0), VTK_BaseToFlange->GetElement(2, 1), VTK_BaseToFlange->GetElement(2, 2), VTK_BaseToFlange->GetElement(2,3),
//				  VTK_BaseToFlange->GetElement(3, 0), VTK_BaseToFlange->GetElement(3, 1), VTK_BaseToFlange->GetElement(3, 2), VTK_BaseToFlange->GetElement(3,3) };
//
//
//	auto VtkT_CameraToEndRF = vtkMatrix4x4::New();
//	VtkT_CameraToEndRF->DeepCopy(T_CamToEndRF);//从相机系统中获取CameraToEndRF
//
//	auto VtkT_BaseRFToCamera = vtkMatrix4x4::New();
//	VtkT_BaseRFToCamera->DeepCopy(T_CamToBaseRF);//从相机系统中获取BaseRFToCamera
//	VtkT_BaseRFToCamera->Invert();//将其进行转置,获得想要的CameraToBaseRF
//
//
//	vtkNew<vtkTransform> tmpTransform;
//	tmpTransform->PostMultiply();
//	tmpTransform->Identity();
//	tmpTransform->SetMatrix(VtkT_CameraToEndRF);
//	tmpTransform->Concatenate(VtkT_BaseRFToCamera);
//	tmpTransform->Update();
//	auto vtkBaseRFtoRobotEndRFMatrix = tmpTransform->GetMatrix();
//
//	hto_RobotRegistration.AddPoseWithVtkMatrix(VTK_BaseToFlange, vtkBaseRFtoRobotEndRFMatrix, translationOnly);
//}


void HTOrobot::captureRobot()
{
	m_Controls.textBrowser_HTO->append("captureRobot");
	if (hto_IndexOfRobotCapture < 5) //The first five translations, 
	{
		hto_IndexOfRobotCapture++;
		std::cout << "m_IndexOfRobotCapture: " << hto_IndexOfRobotCapture << std::endl;
		m_Controls.lineEdit_HansCapture->setText(QString::number(hto_IndexOfRobotCapture));
		CapturePose(true);


	}
	else if (hto_IndexOfRobotCapture < 10) //the last five rotations
	{


		hto_IndexOfRobotCapture++;
		m_Controls.lineEdit_HansCapture->setText(QString::number(hto_IndexOfRobotCapture));
		std::cout << "m_IndexOfRobotCapture: " << hto_IndexOfRobotCapture << std::endl;

		CapturePose(false);
	}
	else
	{
		//MITK_INFO << "OnRobotCapture finish: " << m_IndexOfRobotCapture;
		vtkNew<vtkMatrix4x4> robotEndToFlangeMatrix;
		hto_RobotRegistration.GetTCPmatrix(robotEndToFlangeMatrix);

		vtkMatrix4x4* matrix4x4 = vtkMatrix4x4::New();
		hto_RobotRegistration.GetRegistraionMatrix(matrix4x4);


		double x = robotEndToFlangeMatrix->GetElement(0, 3);
		double y = robotEndToFlangeMatrix->GetElement(1, 3);
		double z = robotEndToFlangeMatrix->GetElement(2, 3);
		std::cout << "X: " << x << std::endl;
		std::cout << "Y: " << y << std::endl;
		std::cout << "Z: " << z << std::endl;

		robotEndToFlangeMatrix->Invert();

		m_Controls.textBrowser_HTO->append("Registration RMS: " + QString::number(hto_RobotRegistration.RMS()));
		std::cout << "Registration RMS: " << hto_RobotRegistration.RMS() << std::endl;


		vtkMatrix4x4* vtkT_BaseToBaseRF = vtkMatrix4x4::New();
		hto_RobotRegistration.GetRegistraionMatrix(vtkT_BaseToBaseRF);
		vtkT_BaseToBaseRF->Invert();
		memcpy_s(T_BaseToBaseRF, sizeof(double) * 16, vtkT_BaseToBaseRF->GetData(), sizeof(double) * 16);


		vtkMatrix4x4* vtkT_FlangeToEdnRF = vtkMatrix4x4::New();
		hto_RobotRegistration.GetTCPmatrix(vtkT_FlangeToEdnRF);
		memcpy_s(T_FlangeToEdnRF, sizeof(double) * 16, vtkT_FlangeToEdnRF->GetData(), sizeof(double) * 16);

		//打印T_BaseToBaseRF
		//std::cout << "----------------------------------------" << std::endl;
		//std::cout << "T_BaseRFToBase:" << std::endl;
		//vtkMatrix4x4* vtkT_BaseRFToBase = vtkMatrix4x4::New();
		//m_RobotRegistration.GetRegistraionMatrix(vtkT_BaseRFToBase);
		//for (int i = 0; i < 4; ++i) {
		//	for (int j = 0; j < 4; ++j) {
		//		std::cout << std::fixed << std::setprecision(6) << vtkT_BaseRFToBase->GetElement(i, j) << " ";
		//	}
		//	std::cout << std::endl;
		//}

	}

}

void HTOrobot::CapturePose(bool translationOnly)
{


	double dX = 0; double dY = 0; double dZ = 0;
	double dRx = 0; double dRy = 0; double dRz = 0;
	int nRet = HRIF_ReadActTcpPos(0, 0, dX, dY, dZ, dRx, dRy, dRz);

	auto tmpTrans = vtkTransform::New();
	tmpTrans->PostMultiply();
	tmpTrans->RotateX(dRx);
	tmpTrans->RotateY(dRy);
	tmpTrans->RotateZ(dRz);


	tmpTrans->Translate(dX, dY, dZ);
	tmpTrans->Update();


	vtkSmartPointer<vtkMatrix4x4> VTKT_BaseToFlanger = tmpTrans->GetMatrix();
	QVector<double> _vtkMatrix4x4;
	_vtkMatrix4x4 = { VTKT_BaseToFlanger->GetElement(0,0), VTKT_BaseToFlanger->GetElement(0, 1), VTKT_BaseToFlanger->GetElement(0, 2), VTKT_BaseToFlanger->GetElement(0,3),
					  VTKT_BaseToFlanger->GetElement(1, 0),VTKT_BaseToFlanger->GetElement(1, 1), VTKT_BaseToFlanger->GetElement(1, 2), VTKT_BaseToFlanger->GetElement(1,3),
					  VTKT_BaseToFlanger->GetElement(2, 0), VTKT_BaseToFlanger->GetElement(2, 1), VTKT_BaseToFlanger->GetElement(2, 2), VTKT_BaseToFlanger->GetElement(2,3),
					  VTKT_BaseToFlanger->GetElement(3, 0), VTKT_BaseToFlanger->GetElement(3, 1), VTKT_BaseToFlanger->GetElement(3, 2), VTKT_BaseToFlanger->GetElement(3,3)
	};

	//std::cout << "VTKT_BaseToFlanger Matrix Contents:" << std::endl;
	//for (int i = 0; i < 4; i++) {
	//	for (int j = 0; j < 4; j++) {
	//		std::cout << _vtkMatrix4x4[i * 4 + j] << "\t";
	//	}
	//	std::cout << std::endl;
	//}

	auto VTKT_CameratoEndRF = vtkMatrix4x4::New();
	VTKT_CameratoEndRF->DeepCopy(T_CamToEndRF);


	auto VTKT_BaseRFToCamera = vtkMatrix4x4::New();
	VTKT_BaseRFToCamera->DeepCopy(T_CamToBaseRF);
	VTKT_BaseRFToCamera->Invert();


	vtkNew<vtkTransform> tmpTransform;
	tmpTransform->PostMultiply();
	tmpTransform->Identity();
	tmpTransform->SetMatrix(VTKT_CameratoEndRF);
	tmpTransform->Concatenate(VTKT_BaseRFToCamera);
	tmpTransform->Update();
	auto vtkBaseRFtoRoboEndRFMatrix = tmpTransform->GetMatrix();


	//vtkMatrix4x4* matrix = tmpTransform->GetMatrix();
	//std::cout << "vtkBaseRFtoRoboEndRFMatrix:" << std::endl;
	//for (int i = 0; i < 4; i++) {
	//	for (int j = 0; j < 4; j++) {
	//		std::cout << matrix->GetElement(i, j) << "\t";
	//	}
	//	std::cout << std::endl;
	//}

	//Robotic arm registration
	hto_RobotRegistration.AddPoseWithVtkMatrix(VTKT_BaseToFlanger, vtkBaseRFtoRoboEndRFMatrix, translationOnly);



}

void HTOrobot::ReplaceRegistration()
{
	m_Controls.textBrowser_HTO->append("replace Registration");
	hto_RobotRegistration.RemoveAllPose();
	hto_IndexOfRobotCapture = 0;
	m_Controls.lineEdit_HansCapture->setText(QString::number(0));
}

void HTOrobot::saveRegistration()
{
	std::ofstream robotMatrixFile(std::string(getenv("USERPROFILE")) + "\\Desktop\\save\\T_BaseToBaseRF.txt");
	for (int i = 0; i < 16; i++) {
		robotMatrixFile << T_BaseToBaseRF[i];
		if (i != 15) {
			robotMatrixFile << ",";
		}
		else {
			robotMatrixFile << ";";
		}
	}
	robotMatrixFile << std::endl;
	robotMatrixFile.close();


	std::ofstream robotMatrixFile1(std::string(getenv("USERPROFILE")) + "\\Desktop\\save\\T_FlangeToEdnRF.txt");
	for (int i = 0; i < 16; i++) {
		robotMatrixFile1 << T_FlangeToEdnRF[i];
		if (i != 15) {
			robotMatrixFile1 << ",";
		}
		else {
			robotMatrixFile1 << ";";
		}
	}
	robotMatrixFile1 << std::endl;
	robotMatrixFile1.close();
	m_Controls.textBrowser_HTO->append("saveArmMatrix");
}

void HTOrobot::reuseRegistration()
{
	std::ifstream inputFile(std::string(getenv("USERPROFILE")) + "\\Desktop\\save\\T_BaseToBaseRF.txt");
	if (inputFile.is_open()) {
		std::string line;
		if (std::getline(inputFile, line)) {
			std::stringstream ss(line);
			std::string token;
			int index = 0;
			while (std::getline(ss, token, ',')) {
				T_BaseToBaseRF[index] = std::stod(token);
				index++;
			}
		}
		inputFile.close();
	}
	else {
		m_Controls.textBrowser_HTO->append("无法打开文件:T_BaseToBaseRF.txt");
	}
	QString output;
	for (int i = 0; i < 16; i++) {
		output += "T_BaseToBaseRF[" + QString::number(i) + "]: " + QString::number(T_BaseToBaseRF[i]) + " ";
	}
	m_Controls.textBrowser_HTO->append(output);
	std::ifstream inputFile2(std::string(getenv("USERPROFILE")) + "\\Desktop\\save\\T_FlangeToEdnRF.txt");
	if (inputFile2.is_open()) {
		std::string line2;
		if (std::getline(inputFile2, line2)) {
			std::stringstream ss2(line2);
			std::string token2;
			int index2 = 0;
			while (std::getline(ss2, token2, ',')) {
				T_FlangeToEdnRF[index2] = std::stod(token2);
				index2++;
			}
		}
		inputFile2.close();
	}
	else {
		m_Controls.textBrowser_HTO->append("无法打开文件：T_FlangeToEdnRF.txt");
	}
	QString output1;
	for (int i = 0; i < 16; i++) {
		output1 += "T_FlangeToEdnRF[" + QString::number(i) + "]: " + QString::number(T_FlangeToEdnRF[i]) + " ";
	}
	m_Controls.textBrowser_HTO->append(output1);
}


void HTOrobot::xp()
{
	nAxisID = 0;
	nDirection = 1;
	nToolMotion = 1;
	m_Controls.textBrowser_HTO->append("xp");
	double inputValue;
	inputValue = m_Controls.lineEdit_intuitiveValue_2->text().toDouble();
	double dDistance = inputValue;
	std::string valueString = std::to_string(inputValue);
	m_Controls.textBrowser_HTO->append(QString::fromStdString(valueString));
	int nMoveRelL = HRIF_MoveRelL(boxID, rbtID, nAxisID, nDirection, dDistance, nToolMotion);
	m_Controls.textBrowser_HTO->append(QString::number(nMoveRelL));
	m_Controls.textBrowser_HTO->append("xp finish");
}

void HTOrobot::yp()
{
	nAxisID = 1;
	nDirection = 1;
	nToolMotion = 1;
	m_Controls.textBrowser_HTO->append("yp");
	double inputValue;
	inputValue = m_Controls.lineEdit_intuitiveValue_2->text().toDouble();
	double dDistance = inputValue;
	std::string valueString = std::to_string(inputValue);
	m_Controls.textBrowser_HTO->append(QString::fromStdString(valueString));
	int nMoveRelL = HRIF_MoveRelL(boxID, rbtID, nAxisID, nDirection, dDistance, nToolMotion);
	m_Controls.textBrowser_HTO->append("yp finish");
}

void HTOrobot::zp()
{
	nAxisID = 2;
	nDirection = 1;
	nToolMotion = 1;
	m_Controls.textBrowser_HTO->append("zp");
	double inputValue;
	inputValue = m_Controls.lineEdit_intuitiveValue_2->text().toDouble();
	double dDistance = inputValue;
	std::string valueString = std::to_string(inputValue);
	m_Controls.textBrowser_HTO->append(QString::fromStdString(valueString));
	int nMoveRelL = HRIF_MoveRelL(boxID, rbtID, nAxisID, nDirection, dDistance, nToolMotion);
	m_Controls.textBrowser_HTO->append("zp finish");
}

void HTOrobot::xm()
{
	nAxisID = 0;
	nDirection = 0;
	nToolMotion = 1;
	m_Controls.textBrowser_HTO->append("xm");
	double inputValue;
	inputValue = m_Controls.lineEdit_intuitiveValue_2->text().toDouble();
	double dDistance = inputValue;
	std::string valueString = std::to_string(inputValue);
	m_Controls.textBrowser_HTO->append(QString::fromStdString(valueString));
	int nMoveRelL = HRIF_MoveRelL(boxID, rbtID, nAxisID, nDirection, dDistance, nToolMotion);
	m_Controls.textBrowser_HTO->append("xm finish");
}

void HTOrobot::ym()
{
	nAxisID = 1;
	nDirection = 0;
	nToolMotion = 1;
	m_Controls.textBrowser_HTO->append("ym");
	double inputValue;
	inputValue = m_Controls.lineEdit_intuitiveValue_2->text().toDouble();
	double dDistance = inputValue;
	std::string valueString = std::to_string(inputValue);
	m_Controls.textBrowser_HTO->append(QString::fromStdString(valueString));
	int nMoveRelL = HRIF_MoveRelL(boxID, rbtID, nAxisID, nDirection, dDistance, nToolMotion);
	m_Controls.textBrowser_HTO->append("ym finish");
}

void HTOrobot::zm()//HRIF_MoveRelL
{
	nAxisID = 2;
	nDirection = 0;
	nToolMotion = 1;
	m_Controls.textBrowser_HTO->append("zm");
	double inputValue;
	inputValue = m_Controls.lineEdit_intuitiveValue_2->text().toDouble();
	double dDistance = inputValue;
	std::string valueString = std::to_string(inputValue);
	m_Controls.textBrowser_HTO->append(QString::fromStdString(valueString));
	int nMoveRelL = HRIF_MoveRelL(boxID, rbtID, nAxisID, nDirection, dDistance, nToolMotion);
	m_Controls.textBrowser_HTO->append("zm finish");
}

void HTOrobot::rxp()
{
	nAxisID = 3;
	nDirection = 1;
	nToolMotion = 1;
	m_Controls.textBrowser_HTO->append("rxp");
	double inputValue;
	inputValue = m_Controls.lineEdit_intuitiveValue_5->text().toDouble();
	double dDistance = inputValue;
	std::string valueString = std::to_string(inputValue);
	m_Controls.textBrowser_HTO->append(QString::fromStdString(valueString));
	int nMoveRelL = HRIF_MoveRelL(boxID, rbtID, nAxisID, nDirection, dDistance, nToolMotion);
	m_Controls.textBrowser_HTO->append("rxp finish");
}

void HTOrobot::ryp()
{
	nAxisID = 4;
	nDirection = 1;
	nToolMotion = 1;
	m_Controls.textBrowser_HTO->append("ryp");
	double inputValue;
	inputValue = m_Controls.lineEdit_intuitiveValue_5->text().toDouble();
	double dDistance = inputValue;
	std::string valueString = std::to_string(inputValue);
	m_Controls.textBrowser_HTO->append(QString::fromStdString(valueString));
	int nMoveRelL = HRIF_MoveRelL(boxID, rbtID, nAxisID, nDirection, dDistance, nToolMotion);
	m_Controls.textBrowser_HTO->append("ryp finish");
}

void HTOrobot::rzp()
{
	nAxisID = 5;
	nDirection = 1;
	nToolMotion = 1;
	m_Controls.textBrowser_HTO->append("rzp");
	double inputValue;
	inputValue = m_Controls.lineEdit_intuitiveValue_5->text().toDouble();
	double dDistance = inputValue;
	std::string valueString = std::to_string(inputValue);
	m_Controls.textBrowser_HTO->append(QString::fromStdString(valueString));
	int nMoveRelL = HRIF_MoveRelL(boxID, rbtID, nAxisID, nDirection, dDistance, nToolMotion);
	m_Controls.textBrowser_HTO->append("rzp finish");
}

void HTOrobot::rxm()
{
	nAxisID = 3;
	nDirection = 0;
	nToolMotion = 1;
	m_Controls.textBrowser_HTO->append("rxm");
	double inputValue;
	inputValue = m_Controls.lineEdit_intuitiveValue_5->text().toDouble();
	double dDistance = inputValue;
	std::string valueString = std::to_string(inputValue);
	m_Controls.textBrowser_HTO->append(QString::fromStdString(valueString));
	int nMoveRelL = HRIF_MoveRelL(boxID, rbtID, nAxisID, nDirection, dDistance, nToolMotion);
	m_Controls.textBrowser_HTO->append("rxm finish");
}

void HTOrobot::rym()
{
	nAxisID = 4;
	nDirection = 0;
	nToolMotion = 1;
	m_Controls.textBrowser_HTO->append("rym");
	double inputValue;
	inputValue = m_Controls.lineEdit_intuitiveValue_5->text().toDouble();
	double dDistance = inputValue;
	std::string valueString = std::to_string(inputValue);
	m_Controls.textBrowser_HTO->append(QString::fromStdString(valueString));
	int nMoveRelL = HRIF_MoveRelL(boxID, rbtID, nAxisID, nDirection, dDistance, nToolMotion);
	m_Controls.textBrowser_HTO->append("rym finish");
}

void HTOrobot::rzm()
{
	nAxisID = 5;
	nDirection = 0;
	nToolMotion = 1;
	m_Controls.textBrowser_HTO->append("rzm");
	double inputValue;
	inputValue = m_Controls.lineEdit_intuitiveValue_5->text().toDouble();
	double dDistance = inputValue;
	std::string valueString = std::to_string(inputValue);
	m_Controls.textBrowser_HTO->append(QString::fromStdString(valueString));
	int nMoveRelL = HRIF_MoveRelL(boxID, rbtID, nAxisID, nDirection, dDistance, nToolMotion);
	m_Controls.textBrowser_HTO->append("rzm finish");
}

void HTOrobot::OstGuidCalibration()
{
	m_Controls.textBrowser_HTO->append("TCP Osteotomy guide calibration");
	//dTcp_X = 99.5545;
	//dTcp_Y = 34.7644;
	//dTcp_Z = 103.064;
	//dTcp_Rx = 162.281;
	//dTcp_Ry = -175.986;
	//dTcp_Rz = 20.0053;
	dTcp_X = 107.701;
	dTcp_Y = 42.3173;
	dTcp_Z = 104.644;
	dTcp_Rx = 84.6308;
	dTcp_Ry = 168.053;
	dTcp_Rz = 24.5419;
	int nOstGuidCal = HRIF_SetTCP(boxID, rbtID, dTcp_X, dTcp_Y, dTcp_Z, dTcp_Rx, dTcp_Ry, dTcp_Rz);
	if (nOstGuidCal == 0) {

		m_Controls.textBrowser_HTO->append("OstGuidCalibration  TCP Set succeed");
	}
	else {

		m_Controls.textBrowser_HTO->append("OstGuidCalibration TCP Set failed");
	}
}


void HTOrobot::PrintToolMatrix()
{

	callCount++;
	std::cout << "Function has been called " << callCount << " times." << std::endl;

	//打印T_CamToEndRF
	std::cout << "----------------------------------------" << std::endl;
	std::cout << "T_CamToEndRF:" << std::endl;
	for (int i = 0; i < 4; ++i)
	{
		std::string row;
		for (int j = 0; j < 4; ++j)
		{
			row += std::to_string(T_CamToEndRF[i * 4 + j]) + " ";
		}
		std::cout << row << std::endl;
	}

	//打印T_CamToBaseRF
	std::cout << "----------------------------------------" << std::endl;
	std::cout << "T_CamToBaseRF:" << std::endl;
	for (int i = 0; i < 4; ++i)
	{
		std::string row;
		for (int j = 0; j < 4; ++j)
		{
			row += std::to_string(T_CamToBaseRF[i * 4 + j]) + " ";
		}
		std::cout << row << std::endl;
	}


	//打印T_CamToProbe
	std::cout << "----------------------------------------" << std::endl;
	std::cout << "T_CamToProbe:" << std::endl;
	for (int i = 0; i < 4; ++i)
	{
		std::string row;
		for (int j = 0; j < 4; ++j)
		{
			row += std::to_string(T_CamToProbe[i * 4 + j]) + " ";
		}
		std::cout << row << std::endl;
	}
	std::cout << "----------------------------------------" << std::endl;
	//打印数据
	std::cout << "ProbeTop: " << ProbeTop[0] << "/" << ProbeTop[1] << "/" << ProbeTop[2] << std::endl;


	//打印T_CamToPatientRF
	std::cout << "----------------------------------------" << std::endl;
	std::cout << "T_CamToPatientRF:" << std::endl;
	for (int i = 0; i < 4; ++i)
	{
		std::string row;
		for (int j = 0; j < 4; ++j)
		{
			row += std::to_string(T_CamToPatientRF[i * 4 + j]) + " ";
		}
		std::cout << row << std::endl;
	}

	//打印机械臂的相关矩阵
	double dX = 0; double dY = 0; double dZ = 0;
	double dRx = 0; double dRy = 0; double dRz = 0;
	int nRet = HRIF_ReadActTcpPos(0, 0, dX, dY, dZ, dRx, dRy, dRz);

	auto tmpTrans = vtkTransform::New();
	tmpTrans->PostMultiply();
	tmpTrans->RotateX(dRx);
	tmpTrans->RotateY(dRy);
	tmpTrans->RotateZ(dRz);


	tmpTrans->Translate(dX, dY, dZ);
	tmpTrans->Update();

	//VTKT_BaseToFlanger
	vtkSmartPointer<vtkMatrix4x4> VTKT_BaseToFlanger = tmpTrans->GetMatrix();
	QVector<double> _vtkMatrix4x4;
	_vtkMatrix4x4 = { VTKT_BaseToFlanger->GetElement(0,0), VTKT_BaseToFlanger->GetElement(0, 1), VTKT_BaseToFlanger->GetElement(0, 2), VTKT_BaseToFlanger->GetElement(0,3),
					  VTKT_BaseToFlanger->GetElement(1, 0),VTKT_BaseToFlanger->GetElement(1, 1), VTKT_BaseToFlanger->GetElement(1, 2), VTKT_BaseToFlanger->GetElement(1,3),
					  VTKT_BaseToFlanger->GetElement(2, 0), VTKT_BaseToFlanger->GetElement(2, 1), VTKT_BaseToFlanger->GetElement(2, 2), VTKT_BaseToFlanger->GetElement(2,3),
					  VTKT_BaseToFlanger->GetElement(3, 0), VTKT_BaseToFlanger->GetElement(3, 1), VTKT_BaseToFlanger->GetElement(3, 2), VTKT_BaseToFlanger->GetElement(3,3)
	};
	std::cout << "----------------------------------------" << std::endl;
	std::cout << "T_BaseToFlanger:" << std::endl;
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			//std::cout << VTKT_BaseToFlanger->GetElement(i, j) << " ";
			std::cout << std::fixed << std::setprecision(6) << VTKT_BaseToFlanger->GetElement(i, j) << " ";
		}
		std::cout << std::endl;
	}

	//打印T_FlangeToEdnRF
	std::cout << "----------------------------------------" << std::endl;


}


/// <summary>
/// 计算法兰到相机的转换矩阵
/// 法兰到相机 == 法兰到末端工具 * 相机到末端工具的逆
/// </summary>
/// <param name="TF2ENDRF">法兰到末端工具位置关系</param>
/// <param name="TCamera2EndRF">相机到末端工具的位置关系</param>
/// <returns></returns>
//功能：计算法兰到相机的转换矩阵。
//输入参数：TF2ENDRF（法兰到末端工具的变换矩阵）和 TCamera2EndRF（相机到末端工具的变换矩阵）。
//过程：首先，将输入数组转换为 vtkMatrix4x4 类型。然后，计算 TCamera2EndRF 的逆矩阵，并与 TF2ENDRF 相乘，得到最终的法兰到相机的转换矩阵。
vtkSmartPointer<vtkMatrix4x4> HTOrobot::CalculateTFlange2Camera(double* TF2ENDRF, double* TCamera2EndRF)
{
	auto matrixF2ENDRF = GetArray2vtkMatrix(TF2ENDRF);
	PrintDataHelper::AppendTextBrowserMatrix(m_Controls.textBrowser_HTO, "matrixF2ENDRF: ", matrixF2ENDRF);

	vtkSmartPointer<vtkMatrix4x4> matrixCamera2EndRF = GetArray2vtkMatrix(TCamera2EndRF);
	PrintDataHelper::AppendTextBrowserMatrix(m_Controls.textBrowser_HTO, "matrixCamera2EndRF: ", matrixCamera2EndRF);

	vtkSmartPointer<vtkMatrix4x4> matrixEndRF2Camera = vtkSmartPointer<vtkMatrix4x4>::New();
	matrixEndRF2Camera->DeepCopy(matrixCamera2EndRF);//将相机到配准工具在转换矩阵复制到matrixEndRF2Camera
	matrixEndRF2Camera->Invert();//将矩阵进行转置
	PrintDataHelper::AppendTextBrowserMatrix(m_Controls.textBrowser_HTO, "matrixEndRF2Camera: ", matrixEndRF2Camera);
	vtkSmartPointer<vtkMatrix4x4> result = vtkSmartPointer<vtkMatrix4x4>::New();

	// 执行矩阵乘法
	vtkMatrix4x4::Multiply4x4(matrixF2ENDRF, matrixEndRF2Camera, result);
	//result = T_flange2EndRf  *  T_EndR2Camera   

	PrintDataHelper::AppendTextBrowserMatrix(m_Controls.textBrowser_HTO, "result: ", result);	//T_flange2Camera
	PrintDataHelper::CoutMatrix("CalculateTFlange2Camera: ", result);
	return result;	//T_flange2Camera
}




//已知TA2B 计算B坐标系下点在A坐标下点的位置   则Pa = (TA2B) * Pb
//功能：计算在法兰坐标系下，相机坐标系中给定点的位置。
//输入参数：matrixFlange2Camera（法兰到相机的变换矩阵）和 posInCamera（相机坐标系中的点）。
//过程：使用 vtkMatrix4x4 的 MultiplyPoint 方法将相机坐标系中的点转换为法兰坐标系。
Eigen::Vector3d HTOrobot::CalculatePointInFlangePos(vtkMatrix4x4* matrixFlange2Camera, Eigen::Vector3d posInCamera)
{
	vtkSmartPointer<vtkMatrix4x4> Flange2Camera = vtkSmartPointer<vtkMatrix4x4>::New();
	Flange2Camera->DeepCopy(matrixFlange2Camera);
	PrintDataHelper::AppendTextBrowserMatrix(m_Controls.textBrowser_HTO, "Flange2Camera: ", Flange2Camera);


	double point[4] = { posInCamera[0], posInCamera[1], posInCamera[2], 1 };
	double m[4];
	Flange2Camera->MultiplyPoint(point, m);
	PrintDataHelper::AppendTextBrowserArray(m_Controls.textBrowser_HTO, "Pos in Flange is", 3, m );

	return Eigen::Vector3d(m[0], m[1], m[2]);
}



//功能：计算一点在线上投影的向量。
//输入参数：点 P 和线段的两个端点 A 和 B。
//过程：计算向量 AP 和 AB，然后找到 AP 在 AB 上的投影。
Eigen::Vector3d HTOrobot::CalculatePointProjectInLine(Eigen::Vector3d P, Eigen::Vector3d A, Eigen::Vector3d B)
{
	// 计算向量AP和AB
	Eigen::Vector3d AP = P - A;
	Eigen::Vector3d AB = B - A;

	// 计算向量AB的单位向量
	Eigen::Vector3d AB_normalized = AB.normalized();

	// 计算向量AP在向量AB上的投影
	double projection_length = AP.dot(AB_normalized);
	Eigen::Vector3d projection = projection_length * AB_normalized;

	// 计算垂线的终点
	//Eigen::Vector3d perpendicular_end = A + projection;
	return projection;
}


// ****p4********
// ************p3
// 
//********p*****
// p1******p2****
//功能：计算工具末端中心点（TCP）的位置。
//过程：使用 CalculateTFlange2Camera 方法计算法兰到相机的转换矩阵，然后将一组点（m_GuideBoardVector 数组中的点）从相机坐标系转换到
//法兰坐标系。接着，使用这些点计算出一个新的坐标系，该坐标系的Z轴垂直于引导板，X轴和Y轴在引导板平面上。最后，将这个新坐标系的原点和Z轴
//旋转角度设置为TCP。
void HTOrobot::CalculateGuideTCP()
{
	auto Tflange2camera = CalculateTFlange2Camera(T_FlangeToEdnRF, T_CamToEndRF);
	//T法兰到相机的转换矩阵
	std::vector<Eigen::Vector3d> posInGuide;
	for (int i = 0; i < 4; ++i)
	{
		Eigen::Vector3d point = CalculatePointInFlangePos(Tflange2camera, probeEndOneVector[i]);//将探针采集的四个点从相机坐标系转换到法兰坐标系
		std::string str = "Point" + std::to_string(i);
		CoutTextBrowerArray("CalculateGuideTCP" + i, point.data(), 3);
		//PrintDataHelper::AppendTextBrowserArray(m_Controls.textBrowser_HTO,  point,str.c_str());

		posInGuide.push_back(point);
	}

	Eigen::Vector3d p1 = posInGuide[0];
	Eigen::Vector3d p2 = posInGuide[1];
	Eigen::Vector3d p3 = posInGuide[2];
	Eigen::Vector3d p4 = posInGuide[3];


	Eigen::Vector3d RX = Eigen::Vector3d(p1[0] - p3[0], p1[1] - p3[1], p1[2] - p3[2]);
	RX.normalize();
	//计算p2 到线段p1 p3的投影点


	auto projectPoint = CalculatePointProjectInLine(p2, p1, p3);
	//使用p2到project 作为rx

	Eigen::Vector3d RY = Eigen::Vector3d(projectPoint[0] - p2[0], projectPoint[1] - p2[1], projectPoint[2] - p2[2]);
	RY.normalize();



	Eigen::Vector3d RZ = RX.cross(RY);
	RZ.normalize();



	Eigen::Matrix3d Re;

	Re << RX[0], RY[0], RZ[0],
		RX[1], RY[1], RZ[1],
		RX[2], RY[2], RZ[2];

	Eigen::Vector3d eulerAngle = Re.eulerAngles(2, 1, 0);
	std::cout << "eulerAngle" << std::endl;
	//------------------------------------------------
	double tcp[6];
	tcp[0] = p4[0];
	tcp[1] = p4[1];
	tcp[2] = p4[2];
	tcp[3] =  vtkMath::DegreesFromRadians(eulerAngle(2)) ;
	tcp[4] =  vtkMath::DegreesFromRadians(eulerAngle(1)) ;
	tcp[5] =  vtkMath::DegreesFromRadians(eulerAngle(0)) ;

	m_Controls.textBrowser_HTO->append("tcp rx ry rz is:");
	m_Controls.textBrowser_HTO->append(QString::number(tcp[3]) + " /" + QString::number(tcp[4]) + " /" + QString::number(tcp[5]));
	m_Controls.textBrowser_HTO->append("tcp x y z is:");
	m_Controls.textBrowser_HTO->append(QString::number(tcp[0]) + " /" + QString::number(tcp[1]) + "/ " + QString::number(tcp[2]));
	std::cout << "HRIF_SetTCP" << std::endl;
	int nRet = HRIF_SetTCP(boxID ,rbtID , tcp[0], tcp[1], tcp[2], tcp[3], tcp[4], tcp[5]);
	if (nRet != 0)
	{
		m_Controls.textBrowser_HTO->append("set tcp failed , Please find The Reason!");
	}
}



//功能：将 double 类型的数组转换为 vtkMatrix4x4 类型。
vtkSmartPointer<vtkMatrix4x4>HTOrobot::GetArray2vtkMatrix(double* array16)
{
	vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();
	// 将元素赋值给vtkMatrix4x4
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			matrix->SetElement(i, j, array16[i * 4 + j]);
		}
	}
	return matrix;
}


void HTOrobot::GetGuiderOriginPosBtnClicked()
{
	//auto newData = GetNewToolData();
	////获取Spine_Probe数据
	//UpdateCameraToToolMatrix(newData, "Spine_Probe", T_CamToProbe, nullptr);
	//Eigen::Vector3d v(newData->tooltip[0], newData->tooltip[1], newData->tooltip[2]);
	//AppendTextBrowerArray("Origin Pos: ", v);
	//probeEndOneVector[3] = v;
	probeEndOneVector[3] = Eigen::Vector3d(ProbeTop);
	AppendTextBrowerArray("Origin is: ", probeEndOneVector[3]);
}

void HTOrobot::GetProbeEndPosBtnClicked(int type)
{

	std::cout << "111" << std::endl;
	// ReadSaveFileData();
	int& currentCount = (type == 1) ? ProbEndCountOne : ProbEndCountTwo;
	int maxCount = (type == 1) ? 3 : 2;
	QLineEdit* countLineEdit = (type == 1) ? m_Controls.ProbeEndOneCountLineEdit : m_Controls.ProbeEndTwoCountLineEdit;
	std::vector<Eigen::Vector3d>& probeEndVector = (type == 1) ? probeEndOneVector : probeEndTwoVector;



	if (currentCount == maxCount)
	{
		QMessageBox msgBox;
		msgBox.setText(QString::fromLocal8Bit("waring"));
		msgBox.setInformativeText(QString::fromLocal8Bit("The collection quantity has reached the upper limit, please reset"));
		msgBox.setStandardButtons(QMessageBox::Ok);
		msgBox.setDefaultButton(QMessageBox::Ok);
		msgBox.exec();
		return;
	}

	currentCount++;
	countLineEdit->setText(QString::number(currentCount));

	//auto newData = GetNewToolData();
	// 获取Spine_Probe数据
	//UpdateCameraToToolMatrix(newData, "Spine_Probe", T_CamToProbe, nullptr);

	//Eigen::Vector3d v(newData->tooltip[0], newData->tooltip[1], newData->tooltip[2]);
	std::string str = "Probe End Pos " + std::to_string(currentCount) + ": ";
	AppendTextBrowerArray(str.c_str(), ProbeTop, 3);

	probeEndVector[currentCount - 1] = Eigen::Vector3d(ProbeTop);
}

//重置末端点
void HTOrobot::ResetProbeEndPosBtnClicked(int type)
{
	if (type == 1)
	{
		for (int i = 0; i < probeEndOneVector.size(); ++i)
		{
			this->probeEndOneVector[i].setZero();
		}
		ProbEndCountOne = 0;
		m_Controls.ProbeEndOneCountLineEdit->setText(QString::number(ProbEndCountOne));
		return;
	}
	for (int i = 0; i < probeEndTwoVector.size(); ++i)
	{
		this->probeEndTwoVector[i].setZero();
	}
	ProbEndCountTwo = 0;
	m_Controls.ProbeEndTwoCountLineEdit->setText(QString::number(ProbEndCountTwo));
}






//坐标系显示
void HTOrobot::StartDisplayTCPAxesActor()
{
	if (!m_RobotTCPAxesTimer)
	{
		m_RobotTCPAxesTimer = new QTimer();
		endMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
	}
	if (!m_IsDisplayRobotTCP)
	{
		//添加一个axesactor
		m_TCPAxesActor = vtkSmartPointer<vtkAxesActor>::New();
		m_TCPAxesActor->SetTotalLength(30, 30, 30);
		m_TCPAxesActor->SetAxisLabels(false);
		m_TCPAxesActor->Modified();
		QmitkRenderWindow* mitkRenderWindow = this->GetRenderWindowPart()->GetQmitkRenderWindow("3d");
		vtkRenderWindow* renderWindow = mitkRenderWindow->GetVtkRenderWindow();
		vtkSmartPointer<vtkRenderer> renderer;
		renderer = renderWindow->GetRenderers()->GetFirstRenderer();
		renderer->AddActor(m_TCPAxesActor);

		connect(m_RobotTCPAxesTimer, SIGNAL(timeout()), this, SLOT(UpdateDisplayRobotTCP()));
		m_RobotTCPAxesTimer->setInterval(150);
		m_IsDisplayRobotTCP = !m_IsDisplayRobotTCP;
	}
	else
	{
		disconnect(m_RobotTCPAxesTimer, SIGNAL(timeout()), this, SLOT(UpdateDisplayRobotTCP()));
		QmitkRenderWindow* mitkRenderWindow = this->GetRenderWindowPart()->GetQmitkRenderWindow("3d");

		vtkRenderWindow* renderWindow = mitkRenderWindow->GetVtkRenderWindow();

		vtkSmartPointer<vtkRenderer> renderer;

		renderer = renderWindow->GetRenderers()->GetFirstRenderer();
		renderer->RemoveActor(m_TCPAxesActor);
		m_IsDisplayRobotTCP = !m_IsDisplayRobotTCP;
	}
}

void HTOrobot::UpdateDisplayRobotTCP()
{
	// 定义空间位置变量
	double dX = 0; double dY = 0; double dZ = 0;
	double dRx = 0; double dRy = 0; double dRz = 0;
	// 定义关节位置变量
	double dJ1 = 0; double dJ2 = 0; double dJ3 = 0;
	double dJ4 = 0; double dJ5 = 0; double dJ6 = 0;
	// 定义工具坐标变量
	double dTcp_X = 0; double dTcp_Y = 0; double dTcp_Z = 0;
	double dTcp_Rx = 0; double dTcp_Ry = 0; double dTcp_Rz = 0;
	// 定义用户坐标变量
	double dUcs_X = 0; double dUcs_Y = 0; double dUcs_Z = 0;
	double dUcs_Rx = 0; double dUcs_Ry = 0; double dUcs_Rz = 0;
	//HRIF_UcsTcp2Base
	int nRet = HRIF_ReadActPos(0, 0, dX, dY, dZ, dRx, dRy, dRz, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, dTcp_X, dTcp_Y,
		dTcp_Z, dTcp_Rx, dTcp_Ry, dTcp_Rz, dUcs_X, dUcs_Y, dUcs_Z, dUcs_Rx, dUcs_Ry, dUcs_Rz);
	PrintTCP("space position: ", dX, dY, dZ, dRx, dRy, dRz);
	Eigen::Matrix3d spaceRotation = EulerAnglesToRotationMatrix(dRx, dRy, dRz);
	Eigen::Vector3d spaceTranslation = Eigen::Vector3d(dX, dY, dZ);

	endMatrix->DeepCopy(GetMatrixByRotationAndTranslation(spaceRotation, spaceTranslation));
	m_TCPAxesActor->SetUserMatrix(endMatrix);
}

void HTOrobot::PrintTCP(std::string tcpName, double x, double y, double z, double rx, double ry, double rz)
{
	std::cout << tcpName << std::endl;
	std::cout << x << " " << y << " " << z << " " << rx << " " << ry << " " << rz << std::endl;
}

Eigen::Matrix3d HTOrobot::EulerAnglesToRotationMatrix(double alpha, double beta, double gamma)
{
	// 计算绕X轴的旋转矩阵
	Eigen::Matrix3d R_x;
	R_x << 1, 0, 0,
		0, cos(alpha), -sin(alpha),
		0, sin(alpha), cos(alpha);

	// 计算绕Y轴的旋转矩阵
	Eigen::Matrix3d R_y;
	R_y << cos(beta), 0, sin(beta),
		0, 1, 0,
		-sin(beta), 0, cos(beta);

	// 计算绕Z轴的旋转矩阵
	Eigen::Matrix3d R_z;
	R_z << cos(gamma), -sin(gamma), 0,
		sin(gamma), cos(gamma), 0,
		0, 0, 1;

	// 总的旋转矩阵 R = R_z * R_y * R_x
	Eigen::Matrix3d R = R_z * R_y * R_x;

	return R;
}

vtkSmartPointer<vtkMatrix4x4> HTOrobot::GetMatrixByRotationAndTranslation(Eigen::Matrix3d rotation, Eigen::Vector3d translation)
{
	vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();
	matrix->Identity();
	for (int row = 0; row < 3; row++)
	{
		for (int col = 0; col < 3; col++)
		{
			matrix->SetElement(row, col, rotation(row, col));
		}
	}
	matrix->SetElement(0, 3, translation[0]);
	matrix->SetElement(1, 3, translation[1]);
	matrix->SetElement(2, 3, translation[2]);
	return matrix;
}



void HTOrobot::AppendTextBrowerArray(const char* text, std::vector<double> array)
{
	QString str;
	for (int i = 0; i < array.size(); ++i)
	{
		str = str + QString::number(array[i]) + " ";
	}
	str = QString(text) + " " + str;
	m_Controls.textBrowser_HTO->append(str);
}

void HTOrobot::AppendTextBrowerArray(const char* text, double* array, int size)
{
	QString str;
	for (int i = 0; i < size; ++i)
	{
		str = str + QString::number(array[i]) + " ";
	}
	str = QString(text) + " " + str;
	m_Controls.textBrowser_HTO->append(str);
}

void HTOrobot::CoutTextBrowerArray(const char* text, double* array, int size)
{
	QString str;
	for (int i = 0; i < size; ++i)
	{
		str = str + QString::number(array[i]) + " ";
	}
	str = QString(text) + " " + str;
	std::cout << str << std::endl;
}

void HTOrobot::AppendTextBrowerArray(const char* text, Eigen::Vector3d array)
{
	QString str;
	for (int i = 0; i < array.size(); ++i)
	{
		str = str + QString::number(array[i]) + " ";
	}
	str = QString(text) + " " + str;
	m_Controls.textBrowser_HTO->append(str);
}