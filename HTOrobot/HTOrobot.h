/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/


#ifndef HTOrobot_h
#define HTOrobot_h

#include <berryISelectionListener.h>

//Hans
#include <HR_Pro.h>
//Aimooe
#include "AimPositionAPI.h"
#include "AimPositionDef.h"
//Robot Registration
#include "robotRegistration.h"
//QT
#include <qfiledialog.h>
#include <qtimer.h>
#include <QmitkAbstractView.h>
#include <mutex>
#include <QMessageBox>

//vkt
#include <vtkMath.h>
#include "PrintDataHelper.h"
//Eigen
#include <eigen3/Eigen/Dense>

#include <vtkRendererCollection.h>
#include <vtkAxesActor.h>
#include <vtkRenderWindow.h>

#include "QmitkRenderWindow.h"


#include "ui_HTOrobotControls.h"

/**
  \brief HTOrobot

  \warning  This class is not yet documented. Use "git blame" and ask the author to provide basic documentation.

  \sa QmitkAbstractView
  \ingroup ${plugin_target}_internal
*/
class HTOrobot : public QmitkAbstractView
{
  // this is needed for all Qt objects that should have a Qt meta-object
  // (everything that derives from QObject and wants to have signal/slots)
  Q_OBJECT

public slots:
	void GetMatrix();
	void GetMatrix_1();

public:
  static const std::string VIEW_ID;

  //Hans initialization
  void connectHans();
  void HansPowerOn();
  void HansPowerOFF();

  void HandGuiding();
  void closeHandGuiding();
  void SetTcpToFlange();

  void suddenStop();
  //Aimooe  initialization
  void connectAimooe();
  void upDateData();
  void getToolInfor();

  void getBaseToFlangeMatrix();
  //robot Registration
  void setInitializationPoint();
  void gotoInitialization();
  void CombineRotationTranslation(float Rto[3][3], float Tto[3], vtkMatrix4x4* resultMatrix);

  void captureRobot();
  void CapturePose(bool translationOnly);
  void ReplaceRegistration();
  void reuseRegistration();
  void saveRegistration();

  void PrintToolMatrix();

  
  //Robot Move
  void xp();
  void yp();
  void zp();
  void xm();
  void ym();
  void zm();
  void rxp();
  void ryp();
  void rzp();
  void rxm();
  void rym();
  void rzm();

  //�������ֵ����ֱ�Ӹ���
  void OstGuidCalibration();
  
  //HTO ����궨
  vtkSmartPointer<vtkMatrix4x4> CalculateTFlange2Camera(double* TF2ENDRF, double* TCamera2EndRF);
  Eigen::Vector3d CalculatePointInFlangePos(vtkMatrix4x4* matrixFlange2Camera, Eigen::Vector3d posInCamera);
  Eigen::Vector3d CalculatePointProjectInLine(Eigen::Vector3d P, Eigen::Vector3d A, Eigen::Vector3d B);
  vtkSmartPointer<vtkMatrix4x4> GetArray2vtkMatrix(double* array16);

  void GetProbeEndPosBtnClicked(int type);//�ɼ�̽��λ��
  void ResetProbeEndPosBtnClicked(int type);//���òɼ�����ֵ
  void GetGuiderOriginPosBtnClicked();//�ɼ�̽��ԭ��λ��
  void CalculateGuideTCP();//����TCP������������ΪTCP��
  std::vector<Eigen::Vector3d> probeEndOneVector;
  std::vector<Eigen::Vector3d> probeEndTwoVector;


  //��ʱ����
  void StartDisplayTCPAxesActor();
  void UpdateDisplayRobotTCP();
  void PrintTCP(std::string tcpName, double x, double y, double z, double rx, double ry, double rz);
  Eigen::Matrix3d EulerAnglesToRotationMatrix(double alpha, double beta, double gamma);
  vtkSmartPointer<vtkMatrix4x4> GetMatrixByRotationAndTranslation(Eigen::Matrix3d rotation, Eigen::Vector3d translation);
  vtkSmartPointer<vtkMatrix4x4> endMatrix;
  QTimer* m_RobotTCPAxesTimer = nullptr;
  bool m_IsDisplayRobotTCP = false;
  int ProbEndCountOne = 0;
  int ProbEndCountTwo = 0;
  vtkSmartPointer<vtkAxesActor> m_TCPAxesActor;
  //��ʱ����


  //���������������
  void AppendTextBrowerArray(const char* text, std::vector<double> array);
  void AppendTextBrowerArray(const char* text, double* array, int size);
  void CoutTextBrowerArray(const char* text, double* array, int size);
  void AppendTextBrowerArray(const char* text, Eigen::Vector3d array);

protected:
  virtual void CreateQtPartControl(QWidget *parent) override;

  virtual void SetFocus() override;

  /// \brief called by QmitkFunctionality when DataManager's selection has changed
  virtual void OnSelectionChanged(berry::IWorkbenchPart::Pointer source,
                                  const QList<mitk::DataNode::Pointer> &nodes) override;

  /// \brief Called when the user clicks the GUI button
  void DoImageProcessing();

  Ui::HTOrobotControls m_Controls;
public:
	//��ʼ����е����׼����
	RobotRegistration hto_RobotRegistration;

	//Hans Declare variables
	unsigned int boxID = 0;
	unsigned int rbtID = 0;
	unsigned short nPort = 10003;
	char* hostname = "192.168.0.10";
	int nIsUseJoint = 1;
	int nIsSeek = 0;
	int nIOBit = 0;
	int nIOState = 0;
	string strCmdID = "0";
	string sTcpName = "TCP_1";
	string sUcsName = "Base";
	double dVelocity = 30;
	double dAcc = 30;
	double dRadius = 50;
	int nToolMotion = 0;
	int nAxisID = 0;
	int nDirection = 0;

	// ����ռ�λ�ñ���
	double dX = 0; double dY = 0; double dZ = 0;
	double dRx = 0; double dRy = 0; double dRz = 0;
	// ����ؽ�λ�ñ���
	double dJ1 = 0; double dJ2 = 0; double dJ3 = 0;
	double dJ4 = 0; double dJ5 = 0; double dJ6 = 0;
	// ���幤���������
	double dTcp_X = 0; double dTcp_Y = 0; double dTcp_Z = 0;
	double dTcp_Rx = 0; double dTcp_Ry = 0; double dTcp_Rz = 0;
	// �����û��������
	double dUcs_X = 0; double dUcs_Y = 0; double dUcs_Z = 0;
	double dUcs_Rx = 0; double dUcs_Ry = 0; double dUcs_Rz = 0;

	int callCount = 0; // ���һ����̬��������¼�������ô���


	//��е����׼����
	double T_BaseToBaseRF[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
	double T_FlangeToEdnRF[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
	//�òɼ����ݵ�ʱ��תVTK����ʹ��
	float R_tran[3][3] = { {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f} };
	float t_tran[3] = { 0.0f, 0.0f, 0.0f };
	//�������е����Mark
	float R_CamToBaseRF[3][3] = { {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f} };
	float t_CamToBaseRF[3] = { 0.0f, 0.0f, 0.0f };
	double T_CamToBaseRF[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
	//�������е��ĩ��Mark
	float R_CamToEndRF[3][3] = { {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f} };
	float t_CamToEndRF[3] = { 0.0f, 0.0f, 0.0f };
	double T_CamToEndRF[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
	//��е��ִ��
	double T_BaseToTarget[16]{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };

	//���������Mark
	float R_CamToPatientRF[3][3] = { {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f} };
	float t_CamToPatientRF[3] = { 0.0f, 0.0f, 0.0f };
	double T_CamToPatientRF[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };

	//�����̽���Mark
	float R_CamToProbe[3][3] = { {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f} };
	float t_CamToProbe[3] = { 0.0f, 0.0f, 0.0f };
	double T_CamToProbe[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
	double ProbeTop[4] = { 0.0f, 0.0f, 0.0f, 1.0f };//���������ϵ�µ�̽����
	double nd_tip_FpatientRF[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
	double nd_tip_FImage_icp[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
	double T_imageToProbe[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 }; // consider the actual length of the selected drill

	std::vector<Eigen::Vector3d> m_GuideBoardVector;

protected: 
	E_ReturnValue rlt;
	QTimer* m_AimoeVisualizeTimer{ nullptr };

	unsigned int hto_IndexOfRobotCapture{ 0 };



	

};

#endif // HTOrobot_h
