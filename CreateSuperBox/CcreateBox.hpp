#pragma once
#include <iostream>
#include <sstream>

#include <NXOpen/NXopen_h.hxx>
#include <uf_h.h>
#include<uf_vec.h>

using namespace NXOpen;
using std::string;
using std::exception;
using std::stringstream;
using std::endl;
using std::cout;
using std::cerr;


class CcreateBox
{
public:
	static Session *theSession;
	static UI *theUI;
	CcreateBox();
	~CcreateBox();
	

private:
	Part *workPart, *displayPart;
	NXMessageBox *mb;
	ListingWindow *lw;
	LogFile *lf;
	NXOpen::Session::UndoMarkId markId1;
	std::vector < NXOpen::TaggedObject *> m_objects; //ѡ��
	int m_seletype;  //ѡ������
	bool m_offsetbool;  //����ƫ��
	
	bool m_isFeatureCreated; //���ܴ���
	NXOpen::Features::Feature* m_feature;

	
	double m_radial;


public:
	void print(const NXString &);
	void print(const string &);
	void print(const char*);
	
	void getseletype(int seletype);
	void getoffsetbool(bool offsetbool);
	

	Point3d getBounding_box(std::vector<NXOpen::TaggedObject *>tagObject); //�Զ�̬�����ȡ������γߴ�

	std::vector< NXOpen::Body * > toolingBox(std::vector<NXOpen::TaggedObject *>tagObject, Point3d pointOrigin,NXOpen::Matrix3x3 matrix, double offset[6]);//���������
	
	std::vector< NXOpen::Body * > redefinetoolingBox(std::vector<NXOpen::TaggedObject *>tagObject, Point3d pointOrigin,NXOpen::Matrix3x3 matrix, double offset[6]);//���������
	
	NXOpen::Matrix3x3 XYtoMatrix(NXOpen::Vector3d XAxis, NXOpen::Vector3d YAxis);
	
	NXOpen::Point3d AbsPointTOCoordinate(Point3d origin, NXOpen::Vector3d XAxis, NXOpen::Vector3d YAxis, Point3d pointAbs);
	
	std::vector< NXOpen::Body * > toolingCylinder(std::vector<NXOpen::TaggedObject *>tagObject,Point3d pointOrigin, NXOpen::Vector3d ZAxis, double offsetCyl[3]);// ����Բ��
	
	std::vector< NXOpen::Body * > redefinetoolingCylinder(std::vector<NXOpen::TaggedObject *>tagObject, Point3d pointOrigin, NXOpen::Vector3d ZAxis, double offsetCyl[3]);
	
	std::vector<NXOpen::SelectionIntentRule *> Rule(std::vector<NXOpen::TaggedObject *>tagObject);
	
	tag_t  subtract(Body* target, std::vector<NXOpen::TaggedObject *>tagObject);//���
	
	tag_t trimBody(tag_t bodyTag, std::vector<NXOpen::TaggedObject *>tagObject); //ƽ���޼�
	
	Vector3d getfaceDir(Face *face);  //�����÷�����

	double getfaceRadius(Face *face);
	
	Point3d  getfacecentre(Face *face); //��ȡ������ĵ�
	
	void  Point_WcstoAbs(double wcs_point[3], double abs_point[3]);//����ת������
	
	void Point_AbstoWcs(double abs_point[3], double wcs_point[3]); //����ת��Ϊ����
	
	void Vec_WcstoAbs(double wcs_Vec[3], double abs_Vec[3]);
	
	void Vec_AbstoWcs(double abs_Vec[3], double wcs_Vec[3]);

};


