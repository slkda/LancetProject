# 胫骨高位截骨（HTO）

## 核心部件：大族机械臂HansRobot   Elfin05   +   艾目易相机  Aimooe

## 核心功能：

1.机械臂配准  （手眼标定）

2.器械标定 （末端夹持截骨导板）

3.机械臂导航定位（运动控制）

## 代码实现：

### 机械臂配准

```c++
void HTOrobot::SetTcpToFlange()//将机械臂TCP设置到机械臂末端法兰Flange
{
	m_Controls.textBrowser_HTO->append("TCP initialization");
	dTcp_Rx = 0; dTcp_Ry = 0; dTcp_Rz = 0; dTcp_X = 0; dTcp_Y = 0; dTcp_Z = 0;
	HRIF_SetTCP(boxID, rbtID, dTcp_X, dTcp_Y, dTcp_Z, dTcp_Rx, dTcp_Ry, dTcp_Rz);
}
```

```c++
void HTOrobot::getBaseToFlangeMatrix()
{
	//Read TCP get T_BaseToFlanger
	double dX = 0; double dY = 0; double dZ = 0;
	double dRx = 0; double dRy = 0; double dRz = 0;
	int nRet = HRIF_ReadActTcpPos(0, 0, dX, dY, dZ, dRx, dRy, dRz);//读取TCP的位置和方向
	if (nRet == 0)
	{
		qDebug() << "get BaseToFlange Matrix";
		m_Controls.textBrowser_HTO->append("get BaseToFlange Matrix");
		std::cout << "Current TCP Position and Orientation:" << std::endl;
		std::cout << "Position - X: " << dX << ", Y: " << dY << ", Z: " << dZ << std::endl;
		std::cout << "Orientation - Rx: " << dRx << ", Ry: " << dRy << ", Rz: " << dRz << std::endl;

		//上述得到的TCP数值是基于Base坐标系
        //利用vtkTransform，将按照机械臂的旋转方式，将TCP从Base旋转到Flange
		auto tmpTrans = vtkTransform::New();
		tmpTrans->PostMultiply();
		tmpTrans->RotateZ(dRz);
		tmpTrans->RotateY(dRy);
		tmpTrans->RotateX(dRx);
		tmpTrans->Translate(dX, dY, dZ);
		tmpTrans->Update();

		//VTKT_BaseToFlanger
        //按照上述的旋转方式，将获得的base到Flange的TCP转换成vtk矩阵
		vtkSmartPointer<vtkMatrix4x4> VTKT_BaseToFlanger = tmpTrans->GetMatrix();
        
        //将vkt矩阵转换成double类型
		QVector<double> _vtkMatrix4x4;
		_vtkMatrix4x4 = { VTKT_BaseToFlanger->GetElement(0,0), VTKT_BaseToFlanger->GetElement(0, 1), VTKT_BaseToFlanger->GetElement(0, 2), VTKT_BaseToFlanger->GetElement(0,3),
						  VTKT_BaseToFlanger->GetElement(1, 0),VTKT_BaseToFlanger->GetElement(1, 1), VTKT_BaseToFlanger->GetElement(1, 2), VTKT_BaseToFlanger->GetElement(1,3),
						  VTKT_BaseToFlanger->GetElement(2, 0), VTKT_BaseToFlanger->GetElement(2, 1), VTKT_BaseToFlanger->GetElement(2, 2), VTKT_BaseToFlanger->GetElement(2,3),
						  VTKT_BaseToFlanger->GetElement(3, 0), VTKT_BaseToFlanger->GetElement(3, 1), VTKT_BaseToFlanger->GetElement(3, 2), VTKT_BaseToFlanger->GetElement(3,3)
		};
        
        //打印log信息，便于调试
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
		m_Controls.textBrowser_HTO->append("Failed to read TCP position and orientation.");
	}
}
```

```c++
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
        //新建一个vkt类型的矩阵，用于存放末端配准工具EndRF到法兰Flange的矩阵
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
        //输出xyz ，其为配准工具EndRF在法兰坐标系下的笛卡尔位置

		robotEndToFlangeMatrix->Invert();

		m_Controls.textBrowser_HTO->append("Registration RMS: " + QString::number(hto_RobotRegistration.RMS()));
		std::cout << "Registration RMS: " << hto_RobotRegistration.RMS() << std::endl;
		//计算RMS误差

		vtkMatrix4x4* vtkT_BaseToBaseRF = vtkMatrix4x4::New();
		hto_RobotRegistration.GetRegistraionMatrix(vtkT_BaseToBaseRF);
		vtkT_BaseToBaseRF->Invert();
		memcpy_s(T_BaseToBaseRF, sizeof(double) * 16, vtkT_BaseToBaseRF->GetData(), sizeof(double) * 16);


		vtkMatrix4x4* vtkT_FlangeToEdnRF = vtkMatrix4x4::New();
		hto_RobotRegistration.GetTCPmatrix(vtkT_FlangeToEdnRF);
		memcpy_s(T_FlangeToEdnRF, sizeof(double) * 16, vtkT_FlangeToEdnRF->GetData(), sizeof(double) * 16);
	}
}
```

```c++
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
    //获取BaseToFlange的齐次变换矩阵
    
    
	QVector<double> _vtkMatrix4x4;
	_vtkMatrix4x4 = { VTKT_BaseToFlanger->GetElement(0,0), VTKT_BaseToFlanger->GetElement(0, 1), VTKT_BaseToFlanger->GetElement(0, 2), VTKT_BaseToFlanger->GetElement(0,3),
					  VTKT_BaseToFlanger->GetElement(1, 0),VTKT_BaseToFlanger->GetElement(1, 1), VTKT_BaseToFlanger->GetElement(1, 2), VTKT_BaseToFlanger->GetElement(1,3),
					  VTKT_BaseToFlanger->GetElement(2, 0), VTKT_BaseToFlanger->GetElement(2, 1), VTKT_BaseToFlanger->GetElement(2, 2), VTKT_BaseToFlanger->GetElement(2,3),
					  VTKT_BaseToFlanger->GetElement(3, 0), VTKT_BaseToFlanger->GetElement(3, 1), VTKT_BaseToFlanger->GetElement(3, 2), VTKT_BaseToFlanger->GetElement(3,3)
	};//将vtk类型的矩阵赋值到double类型的矩阵中

    
	auto VTKT_CameratoEndRF = vtkMatrix4x4::New();
	VTKT_CameratoEndRF->DeepCopy(T_CamToEndRF);
	//获取相机到配准工具的转换矩阵

	auto VTKT_BaseRFToCamera = vtkMatrix4x4::New();
	VTKT_BaseRFToCamera->DeepCopy(T_CamToBaseRF);
	VTKT_BaseRFToCamera->Invert();
	//获取相机到台车mark的转换矩阵

	vtkNew<vtkTransform> tmpTransform;
	tmpTransform->PostMultiply();
	tmpTransform->Identity();
	tmpTransform->SetMatrix(VTKT_CameratoEndRF);
	tmpTransform->Concatenate(VTKT_BaseRFToCamera);
	tmpTransform->Update();
	auto vtkBaseRFtoRoboEndRFMatrix = tmpTransform->GetMatrix();
	
    

    
    
    
	//Robotic arm registration
	hto_RobotRegistration.AddPoseWithVtkMatrix(VTKT_BaseToFlanger, vtkBaseRFtoRoboEndRFMatrix, translationOnly);

}
```

### 截骨导板标定

#### 计算法兰到相机的转换矩阵

```c++
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

```

```c++
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
```

```c++
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
```

```c++
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
```

```c++
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
```

