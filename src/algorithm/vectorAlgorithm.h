#pragma once
#include <math.h>
#include<vtkMath.h>
#include<vector>
#include<vtkSmartPointer.h>
#include<vtkPolyData.h>
//!
//! \file vectorAlgorithm.h
//! \brief Vector API data types
//!
//! \author Xukun Zhang
//! \e-mail schwarsolomon@gmail.com
//! \date 13.08.2022
//! \version 1.0
//	VMesh Library



//! \struct VMdpoint3D
//! \brief Contains 3D double-precision coordinates.
struct double3 {
	double data[3]; //!< point coordinates

	inline double& x() { return data[0]; }
	inline double& y() { return data[1]; }
	inline double& z() { return data[2]; }
	inline double x() const { return data[0]; }
	inline double y() const { return data[1]; }
	inline double z() const { return data[2]; }


	double3()
	{
		data[0] = 0;
		data[1] = 0;
		data[2] = 0;
	}

	double3(double x, double y, double z)
	{
		data[0] = x;
		data[1] = y;
		data[2] = z;
	}

	double3(double* pt)
	{
		data[0] = pt[0];
		data[1] = pt[1];
		data[2] = pt[2];
	}

	//////////////////////////////////////////////////////////////////
	inline const double& operator[] (int Index) const
	{
		return (data[Index]);
	};



	inline double3& operator() (double* P)
	{
		data[0] = P[0];
		data[1] = P[1];
		data[2] = P[2];
		return (*this);
	};

	inline double3& operator() (double x, double y, double z)
	{
		data[0] = x;
		data[1] = y;
		data[2] = z;
		return (*this);
	};

	////////////////////////////////////////////////////////////////
	inline double3& operator= (const double3& P)
	{
		data[0] = P.data[0];
		data[1] = P.data[1];
		data[2] = P.data[2];
		return (*this);
	};

	inline double3& operator= (const double* P)
	{
		data[0] = P[0];
		data[1] = P[1];
		data[2] = P[2];
		return (*this);
	};

	/////////////////////////////////////////////////////////////////
	inline double3 operator+ (const double3 P) const
	{
		double3 res;
		res.data[0] = data[0] + P.data[0];
		res.data[1] = data[1] + P.data[1];
		res.data[2] = data[2] + P.data[2];
		return (res);
	};

	/////////////////////////////////////////////////////////////////
	inline double3 operator- (const double3 P) const
	{
		double3 res;
		res.data[0] = data[0] - P.data[0];
		res.data[1] = data[1] - P.data[1];
		res.data[2] = data[2] - P.data[2];
		return (res);
	};


	inline double3 operator- () const
	{
		double3 res;
		res.data[0] = -data[0];
		res.data[1] = -data[1];
		res.data[2] = -data[2];
		return (res);
	};
	/////////////////////////////////////////////////////////////////
	inline double3 operator* (double s) const
	{
		double3 res;
		res.data[0] = data[0] * s;
		res.data[1] = data[1] * s;
		res.data[2] = data[2] * s;
		return (res);
	};

	inline double3 operator* (double3 s) const
	{
		double3 res;
		res.data[0] = data[0] * s[0];
		res.data[1] = data[1] * s[1];
		res.data[2] = data[2] * s[2];
		return (res);
	};


	inline double3 operator/ (double s) const
	{
		double3 res;
		res.data[0] = data[0] / s;
		res.data[1] = data[1] / s;
		res.data[2] = data[2] / s;
		return (res);
	};

	inline double operator ^ (double3 q)
	{
		return data[0] * q.data[1] - data[1] * q.data[0];
	}

	inline double3& operator+= (const double3& P)
	{
		data[0] += P.data[0];
		data[1] += P.data[1];
		data[2] += P.data[2];
		return (*this);
	}

	inline double3& operator-= (const double3& P)
	{
		data[0] -= P.data[0];
		data[1] -= P.data[1];
		data[2] -= P.data[2];
		return (*this);
	}

	inline double getSquaredlength() const
	{
		return(data[0] * data[0] + data[1] * data[1] + data[2] * data[2]);
	};

	/////////////////////////////////////////////////////////////////
	inline double getLength() const
	{
		return (double)sqrt(getSquaredlength());
	};

	////////////////////////////////////////////////////////////////
	inline double3 normalize(void)
	{
		double length = getLength();
		if (length > -1E-8 && length < 1E-8)
			return 0;

		double rezLength = 1.0f / length;
		data[0] *= rezLength;
		data[1] *= rezLength;
		data[2] *= rezLength;
		double3 norm(data);
		return norm;
	};

	////////////////////////////////////////////////////////////////
	static inline double3 crossProduct(const double3& a, const double3& b)
	{
		double3 result;

		result.data[0] = a.data[1] * b.data[2] - a.data[2] * b.data[1];
		result.data[1] = a.data[2] * b.data[0] - a.data[0] * b.data[2];
		result.data[2] = a.data[0] * b.data[1] - a.data[1] * b.data[0];

		return(result);
	}

	////////////////////////////////////////////////////////////////
	static inline double dotProduct(const double3& a, const double3& b)
	{
		return(a.data[0] * b.data[0] + a.data[1] * b.data[1] + a.data[2] * b.data[2]);
	}

	friend ostream& operator<<(ostream& output, const double3& pt)
	{
		//output << "double3: " << pt.data[0] << " , " << pt.data[1] << " , " << pt.data[2] << endl;
		output << pt.data[0] << " " << pt.data[1] << " " << pt.data[2] << endl;
		return output;
	}
	//not used
	static inline double3 projectPointOntoLine(const double3 point, const double3 point1, const double3 point2, double& t)
	{
		double m = (point1 - point).getSquaredlength();
		double n = (point2.data[0] - point1.data[0]) * (point.data[0] - point1.data[0]) +
			(point2.data[1] - point1.data[1]) * (point.data[1] - point1.data[1]) +
			(point2.data[2] - point1.data[2]) * (point.data[2] - point1.data[2]);
		t = n / m;
		return (point * t + point1 * (1 - t));
	}

	/*
点绕任意向量旋转(右手系)
参数pt为旋转前空间点的坐标,axis为旋转轴向量
Theta为旋转角度(角度制,范围在-180到180)
返回旋转后点的坐标
*/
	static inline double3 RotateByVector(double3& pt, double3& Axis, double& Theta)
	{
		double3 pt1;
		double Radian = Theta * vtkMath::Pi() / static_cast<double>(180.0);
		double c = cos(Radian);
		double s = sin(Radian);
		pt1.data[0] = (Axis.data[0] * Axis.data[0] * (1 - c) + c) * pt.data[0] + (Axis.data[0] * Axis.data[1] * (1 - c) - Axis.data[2] * s) * pt.data[1] + (Axis.data[0] * Axis.data[2] * (1 - c) + Axis.data[1] * s) * pt.data[2];
		pt1.data[1] = (Axis.data[1] * Axis.data[0] * (1 - c) + Axis.data[2] * s) * pt.data[0] + (Axis.data[1] * Axis.data[1] * (1 - c) + c) * pt.data[1] + (Axis.data[1] * Axis.data[2] * (1 - c) - Axis.data[0] * s) * pt.data[2];
		pt1.data[2] = (Axis.data[0] * Axis.data[2] * (1 - c) - Axis.data[1] * s) * pt.data[0] + (Axis.data[1] * Axis.data[2] * (1 - c) + Axis.data[0] * s) * pt.data[1] + (Axis.data[2] * Axis.data[2] * (1 - c) + c) * pt.data[2];
		return pt1;

	}
};

//! \struct VMindexTriplet
//! \brief Usually describes a mesh triangle by 3 vertex indices.
struct VMindexTriplet {
	bool operator==(const VMindexTriplet& triplet) const {
		return ref[0] == triplet.ref[0] && ref[1] == triplet.ref[1] && ref[2] == triplet.ref[2];
	}
	bool operator<(const VMindexTriplet& triplet) const {
		if (ref[0] == triplet.ref[0]) {
			return ref[1] == triplet.ref[1] ? ref[2] < triplet.ref[2] : ref[1] < triplet.ref[1];
		}
		else {
			return ref[0] < triplet.ref[0];
		}
	}
	int ref[3];	//!< vertex indices
};

//! \struct VMindexPair
//! \brief Vertex index pair.
//!
//! VMindexPair is used to describe a triangle in the vertex node:
//! for the i-th node, it corresponds to a triangle (i,ref0,ref1),
//! with ref0 > ref1.
//! VMindexPair also may be used for edge description by its end indices.
struct VMindexPair {
	bool operator==(const VMindexPair& pair) const {
		return ref0 == pair.ref0 && ref1 == pair.ref1;
	}
	bool operator<(const VMindexPair& pair) const {
		return ref0 == pair.ref0 ? ref1 < pair.ref1 : ref0 < pair.ref0;
	}
	inline VMindexPair& operator= (const VMindexPair& vP)
	{
		ref0 = vP.ref0;
		ref1 = vP.ref1;
		triID = vP.triID;
		return (*this);
	};

	int ref0;	//!< index of the second triangle corner
	int ref1;	//!< index of the third triangle corner, \a ref1 > \a ref0
	int triID;
};

//! \struct LMplaneEquation
//! \brief Describes plane equation.
//!
//! The LMplaneEquation structure describes a plane by an equation:
//! \code
//! x*coef[0]+y*coef[1]+z*coef[2]=coef[3].
//! \endcode
struct VMplaneEquation {
	double coef[4];	//!< plane equation coefficients
};

struct Plane
{
	double3 point;
	double3 normal;
	double data[4];// Ax+By+Cz+D=0
	Plane()
	{
		data[0] = 0;
		data[1] = 0;
		data[2] = 0;
		data[3] = 0;
	}


	Plane(const double3 pt, const double3 norm)
	{
		point = pt;
		normal = norm;
		data[0] = norm.data[0];
		data[1] = norm.data[1];
		data[2] = norm.data[2];
		data[3] = -pt.data[0] * norm.data[0] - pt.data[1] * norm.data[1] - pt.data[2] * norm.data[2];
	}
	inline Plane& operator= (const Plane& P)
	{
		point = P.point;
		normal = P.normal;
		data[0] = P.data[0];
		data[1] = P.data[1];
		data[2] = P.data[2];
		data[3] = P.data[3];
		return (*this);
	};
	//Plane(const double A, const  double B, const  double C, const  double D)
	//{
	//	data[0] = A;
	//	data[1] = B;
	//	data[2] = C;
	//	data[3] = D;
	//}
};

struct Line
{
	//x=pt[0]+vec[0]*t
	//y=pt[1]+vec[1]*t
	//z=pt[2]+vec[2]*t
	double3 pt;
	double3 vec;
	Line()
	{

	}
	Line(const double3 point, const double3 vector)
	{
		pt.data[0] = point.data[0];
		pt.data[1] = point.data[1];
		pt.data[2] = point.data[2];
		vec.data[0] = vector.data[0];
		vec.data[1] = vector.data[1];
		vec.data[2] = vector.data[2];
	}
	inline Line& operator= (const Line& P)
	{
		pt.data[0] = P.pt.data[0];
		pt.data[1] = P.pt.data[1];
		pt.data[2] = P.pt.data[2];
		vec.data[0] = P.vec.data[0];
		vec.data[1] = P.vec.data[1];
		vec.data[2] = P.vec.data[2];
		return (*this);
	};
};

struct Line_Cross_Plane
{
	Line line;
	Plane plane;
	double3 cross_pt;
	Line_Cross_Plane(const Line l, const Plane p)
	{
		line = l;
		plane = p;
	}
	double3 compute()
	{
		double t = (plane.normal.data[0] * (plane.point.data[0] - line.pt.data[0]) + plane.normal.data[1] * (plane.point.data[1] - line.pt.data[1]) + plane.normal.data[2] * (plane.point.data[2] - line.pt.data[2])) / (plane.normal.data[0] * line.vec.data[0] + plane.normal.data[1] * line.vec.data[1] + plane.normal.data[2] * line.vec.data[2]);
		cross_pt.data[0] = line.pt.data[0] + line.vec.data[0] * t;
		cross_pt.data[1] = line.pt.data[1] + line.vec.data[1] * t;
		cross_pt.data[2] = line.pt.data[2] + line.vec.data[2] * t;
		return (cross_pt);
	}
	double3 project_point_on_line(double3 crossPt, double3 moveVec)
	{
		// crosspt为交点
		// moveVec为移动方向
		// 平面中心点
		double3 point = plane.point;
		// 交点与平面中心点的向量
		double3 vector1 = crossPt - point;
		// 移动的方向
		 //moveVec;
		// 点乘
		double result = double3::dotProduct(vector1, moveVec);
		// 向量的模长
		double len_crossPt_to_planepoint = (crossPt - plane.point).getLength();
		double len_moveVec = moveVec.getLength();
		// 计算theta的夹角
		double theta = acos(result / (len_moveVec * len_crossPt_to_planepoint));
		// 计算投影
		double output = cos(theta) * len_crossPt_to_planepoint;
		// 移动方向的正交化

		moveVec.normalize();
		return moveVec * output;


		//return moveVec * double3::dotProduct(crossPt - plane.point, moveVec) / pow((crossPt - plane.point).getLength(), 2);
	}
};

//! \struct LM_RGBA
//! \brief Describes vertex color in the RGBA format.
struct VM_RGBA {
	unsigned char r;		//!< red color component
	unsigned char g;		//!< green color component
	unsigned char b;		//!< blue color component
	unsigned char alpha;	//!< alpha color component
};

//! \union LMvertColor
//! \brief Describes vertex color in the RGBA format.
union VMvertColor {
	VM_RGBA rgba;		//!< color in LM_RGBA format
	unsigned int color;	//!< color as a single 32-bit unsigned integer
};

//! \class LMlinearTransform
//! \brief Describes a general 3D linear transformation.
//!
//! The transformation data are used to transform 3D points in the following way:
//! \code
//! x1[i]=(T.mat[i][0]*x[0]+T.mat[i][1]*x[1]+T.mat[i][2]*x[2])+T.shift[i];
//! \endcode
struct VMlinearTransform {
	double mat[3][3];	//!< rotation matrix part
	double shift[3];	//!< shift part
};

//! \struct LMglTransform
//! \brief Describes the OpenGL-style linear transformation as a 4x4 matrix
struct VMglTransform {
	double matrix[16];	//!< matrix data
};



//! \struct VMuv
////纹理
struct VMuv
{
	double u;
	double v;
};

//! \struct VMRender
////渲染数据
struct VMRender
{
	int verNum;   ///点的个数
	double* posList;    ///点坐标verNum*3
	double* norList;    ///点法矢verNum*3
	double* posUVList;   ///点纹理verNum*2
	double* posRGBAList;  ///点颜色verNum*4

	int triNum;    ///三角面片数量
	int* faceindex;   ///点索引triNum*3
};

struct SPoint
{
	double3 controlPt;
	int triId;
};

struct spline
{
	std::vector<SPoint> ctrlPt;
	std::vector<std::vector<double3>> segLine;//分段样条线
	std::vector<double3> allPt;
	std::vector<vtkSmartPointer<vtkPolyData>> paraPD;//参数化polydata
};