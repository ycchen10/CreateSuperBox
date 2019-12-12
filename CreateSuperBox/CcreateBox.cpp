
#include "CcreateBox.hpp"
#include <algorithm>


Session *(CcreateBox::theSession) = NULL;
UI *(CcreateBox::theUI) = NULL;



CcreateBox::CcreateBox()
{



	CcreateBox::theSession = NXOpen::Session::GetSession();
	CcreateBox::theUI = UI::GetUI();
	mb = theUI->NXMessageBox();
	lw = theSession->ListingWindow();
	lf = theSession->LogFile();

	workPart = theSession->Parts()->Work();
	displayPart = theSession->Parts()->Display();


}


CcreateBox::~CcreateBox()
{
}
void CcreateBox::print(const NXString &msg)
{
	if (!lw->IsOpen()) lw->Open();
	lw->WriteLine(msg);
}
void CcreateBox::print(const string &msg)
{
	if (!lw->IsOpen()) lw->Open();
	lw->WriteLine(msg);
}
void CcreateBox::print(const char * msg)
{
	if (!lw->IsOpen()) lw->Open();
	lw->WriteLine(msg);
}

void CcreateBox::getseletype(int seletype)
{
	m_seletype = seletype;

}
void CcreateBox::getoffsetbool(bool offsetbool)
{
	m_offsetbool = offsetbool;
}




Point3d CcreateBox::getBounding_box(std::vector<NXOpen::TaggedObject *>tagObject) //获取body以工作坐标系最大外形点
{



	tag_t objects_t = NULL_TAG;

	double min_corner[3];
	double directions[3][3];
	double distances[3];
	double output_point[3];
	double box[6];
	double max[6];
	double origin[3];
	objects_t = tagObject[0]->Tag();

	UF_MODL_ask_bounding_box_exact(objects_t, NULL_TAG, min_corner, directions, distances);



	UF_CSYS_map_point(UF_CSYS_WORK_COORDS, min_corner, UF_CSYS_ROOT_WCS_COORDS, output_point);//把绝对坐标转换为工作坐标
	box[0] = output_point[0];
	box[1] = output_point[1];
	box[2] = output_point[2];
	box[3] = output_point[0] + distances[0];
	box[4] = output_point[1] + distances[1];
	box[5] = output_point[2] + distances[2];


	if (tagObject.size() != 1)
	{
		for (int i = 0; i < tagObject.size(); i++) //比较两个body获取最小点和最大点
		{
			tag_t temp = NULL_TAG;
			temp = tagObject[i]->Tag();
			UF_MODL_ask_bounding_box_exact(temp, NULL_TAG, min_corner, directions, distances);
			UF_CSYS_map_point(UF_CSYS_WORK_COORDS, min_corner, UF_CSYS_ROOT_WCS_COORDS, output_point);  //把绝对坐标转换为工作坐标
			max[0] = output_point[0];
			max[1] = output_point[1];
			max[2] = output_point[2];
			max[3] = output_point[0] + distances[0];
			max[4] = output_point[1] + distances[1];
			max[5] = output_point[2] + distances[2];
			if (max[0] < box[0])
			{
				box[0] = max[0];
			}
			if (max[1] < box[1])
			{
				box[1] = max[1];
			}
			if (max[2] < box[2])
			{
				box[2] = max[2];
			}
			if (max[3] > box[3])
			{
				box[3] = max[3];
			}
			if (max[4] > box[4])
			{
				box[4] = max[4];
			}
			if (max[5] > box[5])
			{
				box[5] = max[5];
			}

		}
	}
	origin[0] = (box[0] + box[3]) / 2;
	origin[1] = (box[1] + box[4]) / 2;
	origin[2] = (box[2] + box[5]) / 2;
	double originAbs[3] = {};
	UF_CSYS_map_point(UF_CSYS_ROOT_WCS_COORDS, origin, UF_CSYS_WORK_COORDS, originAbs);  //把工作坐标转换为绝对坐标

	Point3d pointOrigin(originAbs[0], originAbs[1], originAbs[2]);

	return pointOrigin;


}


std::vector< NXOpen::Body * > CcreateBox::toolingBox(std::vector<NXOpen::TaggedObject *>tagObject, Point3d pointOrigin,
	NXOpen::Matrix3x3 matrix, double offset[6])
{
	if (tagObject.size() > 0)
	{
		if (m_isFeatureCreated)
		{
			std::vector< NXOpen::Body * > bodies;
			bodies = redefinetoolingBox(tagObject, pointOrigin, matrix, offset);
			return bodies;
		}
	
	
		NXOpen::Features::ToolingBox *nullNXOpen_Features_ToolingBox = NULL;
		NXOpen::Features::ToolingBoxBuilder *toolingBoxBuilder1;
		toolingBoxBuilder1 = workPart->Features()->ToolingFeatureCollection()->CreateToolingBoxBuilder(nullNXOpen_Features_ToolingBox);

		toolingBoxBuilder1->SetType(NXOpen::Features::ToolingBoxBuilder::TypesBoundedBlock); //创建有界边界盒

		
		toolingBoxBuilder1->OffsetPositiveX()->SetValue(offset[0]);

		toolingBoxBuilder1->OffsetNegativeX()->SetValue(offset[1]);

		toolingBoxBuilder1->OffsetPositiveY()->SetValue(offset[2]);

		toolingBoxBuilder1->OffsetNegativeY()->SetValue(offset[3]);

		toolingBoxBuilder1->OffsetPositiveZ()->SetValue(offset[4]);

		toolingBoxBuilder1->OffsetNegativeZ()->SetValue(offset[5]);


		toolingBoxBuilder1->SetBoxMatrixAndPosition(matrix, pointOrigin); //设置坐标
	

		std::vector<NXOpen::SelectionIntentRule *> rules1 = Rule(tagObject);

		NXOpen::ScCollector *scCollector1;
		scCollector1 = toolingBoxBuilder1->BoundedObject();



		scCollector1->ReplaceRules(rules1, false);



		std::vector<NXOpen::NXObject *> selections1(1);
		selections1[0] = dynamic_cast<NXOpen::NXObject*>(tagObject[0]);
		std::vector<NXOpen::NXObject *> deselections1(0);
		toolingBoxBuilder1->SetSelectedOccurrences(selections1, deselections1);

		toolingBoxBuilder1->CalculateBoxSize();


		m_feature = toolingBoxBuilder1->CommitFeature();

		toolingBoxBuilder1->Destroy();

		workPart->FacetedBodies()->DeleteTemporaryFacesAndEdges(); //删除临时面和边
		m_isFeatureCreated = true;   //功能创建


		std::vector< NXOpen::Body * > bodies = dynamic_cast<Features::ToolingBox *>(m_feature)->GetBodies();

		return bodies;


	}


}
std::vector< NXOpen::Body * > CcreateBox::redefinetoolingBox(std::vector<NXOpen::TaggedObject *>tagObject,
	Point3d pointOrigin, NXOpen::Matrix3x3 matrix, double offset[6])
{



	NXOpen::Features::ToolingBoxBuilder *toolingBoxBuilder1;
	toolingBoxBuilder1 = workPart->Features()->ToolingFeatureCollection()->CreateToolingBoxBuilder(dynamic_cast<NXOpen::Features::ToolingBox*>(m_feature));

	toolingBoxBuilder1->SetType(NXOpen::Features::ToolingBoxBuilder::TypesBoundedBlock); //创建有界边界盒
	


	toolingBoxBuilder1->OffsetPositiveX()->SetValue(offset[0]);

	toolingBoxBuilder1->OffsetNegativeX()->SetValue(offset[1]);

	toolingBoxBuilder1->OffsetPositiveY()->SetValue(offset[2]);

	toolingBoxBuilder1->OffsetNegativeY()->SetValue(offset[3]);

	toolingBoxBuilder1->OffsetPositiveZ()->SetValue(offset[4]);

	toolingBoxBuilder1->OffsetNegativeZ()->SetValue(offset[5]);
	if (!m_offsetbool)
	{
		toolingBoxBuilder1->OffsetPositiveX()->SetValue(offset[0]);

		toolingBoxBuilder1->OffsetNegativeX()->SetValue(offset[1]);

		toolingBoxBuilder1->OffsetPositiveY()->SetValue(offset[2]);

		toolingBoxBuilder1->OffsetNegativeY()->SetValue(offset[3]);

		toolingBoxBuilder1->OffsetPositiveZ()->SetValue(offset[4]);

		toolingBoxBuilder1->OffsetNegativeZ()->SetValue(offset[5]);

		toolingBoxBuilder1->SetSingleOffset(false);  //设置是否单项偏置

		toolingBoxBuilder1->Clearance()->SetValue(offset[0]);

	}


	std::vector<NXOpen::SelectionIntentRule *> rules1 = Rule(tagObject);


	NXOpen::ScCollector *scCollector1;
	scCollector1 = toolingBoxBuilder1->BoundedObject();


	scCollector1->ReplaceRules(rules1, false);



	std::vector<NXOpen::NXObject *> selections1(1);
	selections1[0] = dynamic_cast<NXOpen::NXObject*>(tagObject[0]);
	std::vector<NXOpen::NXObject *> deselections1(0);
	toolingBoxBuilder1->SetSelectedOccurrences(selections1, deselections1);

	toolingBoxBuilder1->CalculateBoxSize();
	markId1 = CcreateBox::theSession->SetUndoMark(NXOpen::Session::MarkVisibilityVisible, "Start Extrude");
	m_feature = toolingBoxBuilder1->CommitFeature();

	toolingBoxBuilder1->Destroy();
	workPart->FacetedBodies()->DeleteTemporaryFacesAndEdges(); //删除临时面和边
	NXOpen::Features::ToolingBox*  boxobj = dynamic_cast<NXOpen::Features::ToolingBox*>(m_feature);



	CcreateBox::theSession->UpdateManager()->DoUpdate(markId1);
	CcreateBox::theSession->DeleteUndoMark(markId1, NULL);

	std::vector< NXOpen::Body * > bodies = dynamic_cast<Features::ToolingBox *>(m_feature)->GetBodies();

	return bodies;

}

std::vector< NXOpen::Body * > CcreateBox::toolingCylinder(std::vector<NXOpen::TaggedObject *>tagObject,
	Point3d pointOrigin, NXOpen::Vector3d ZAxis, double offsetCyl[3])
{
	if (tagObject.size() > 0)
	{
		if (m_isFeatureCreated)
		{
			std::vector< NXOpen::Body * > bodies;
			bodies=redefinetoolingCylinder(tagObject, pointOrigin, ZAxis, offsetCyl);
			return bodies;
		}

		
		NXOpen::Features::ToolingBox *nullNXOpen_Features_ToolingBox = NULL;
		NXOpen::Features::ToolingBoxBuilder *toolingBoxBuilder1;
		toolingBoxBuilder1 = workPart->Features()->ToolingFeatureCollection()->CreateToolingBoxBuilder(nullNXOpen_Features_ToolingBox);

		toolingBoxBuilder1->SetType(NXOpen::Features::ToolingBoxBuilder::TypesBoundedCylinder);
		
		NXOpen::Direction *direction1 = workPart->Directions()->CreateDirection(pointOrigin, ZAxis, NXOpen::SmartObject::UpdateOptionWithinModeling);
		


		toolingBoxBuilder1->SetAxisVector(direction1); //设置轴
		
		toolingBoxBuilder1->RadialOffset()->SetValue(offsetCyl[0]);
		toolingBoxBuilder1->OffsetPositiveZ()->SetValue(offsetCyl[1]);
		toolingBoxBuilder1->OffsetNegativeZ()->SetValue(offsetCyl[2]);


		std::vector<NXOpen::SelectionIntentRule *> rules1=Rule(tagObject);

	
		NXOpen::ScCollector *scCollector1;
		scCollector1 = toolingBoxBuilder1->BoundedObject();



		scCollector1->ReplaceRules(rules1, false);



		std::vector<NXOpen::NXObject *> selections1(1);
		selections1[0] = dynamic_cast<NXOpen::NXObject*>(tagObject[0]);
		std::vector<NXOpen::NXObject *> deselections1(0);
		toolingBoxBuilder1->SetSelectedOccurrences(selections1, deselections1);

		toolingBoxBuilder1->CalculateBoxSize();


		m_feature = toolingBoxBuilder1->CommitFeature();

		toolingBoxBuilder1->Destroy();

		workPart->FacetedBodies()->DeleteTemporaryFacesAndEdges(); //删除临时面和边
		m_isFeatureCreated = true;   //功能创建


		std::vector< NXOpen::Body * > bodies = dynamic_cast<Features::ToolingBox *>(m_feature)->GetBodies();

		return bodies;


	}


}

std::vector< NXOpen::Body * > CcreateBox::redefinetoolingCylinder(std::vector<NXOpen::TaggedObject *>tagObject,
	Point3d pointOrigin, NXOpen::Vector3d ZAxis, double offsetCyl[3])
{
	if (tagObject.size() > 0)
	{
		

		NXOpen::Features::ToolingBox *nullNXOpen_Features_ToolingBox = NULL;
		NXOpen::Features::ToolingBoxBuilder *toolingBoxBuilder1;
		toolingBoxBuilder1 = workPart->Features()->ToolingFeatureCollection()->CreateToolingBoxBuilder(dynamic_cast<NXOpen::Features::ToolingBox*>(m_feature));

		toolingBoxBuilder1->SetType(NXOpen::Features::ToolingBoxBuilder::TypesBoundedCylinder);

		NXOpen::Direction *direction1 = workPart->Directions()->CreateDirection(pointOrigin, ZAxis, NXOpen::SmartObject::UpdateOptionWithinModeling);



		toolingBoxBuilder1->SetAxisVector(direction1); //设置轴
	
		toolingBoxBuilder1->RadialOffset()->SetValue(offsetCyl[0]);
		toolingBoxBuilder1->OffsetPositiveZ()->SetValue(offsetCyl[1]);
		toolingBoxBuilder1->OffsetNegativeZ()->SetValue(offsetCyl[2]);
		if (!m_offsetbool)
		{
			
			toolingBoxBuilder1->RadialOffset()->SetValue(offsetCyl[0]);
			toolingBoxBuilder1->OffsetPositiveZ()->SetValue(offsetCyl[1]);
			toolingBoxBuilder1->OffsetNegativeZ()->SetValue(offsetCyl[2]);
			
			toolingBoxBuilder1->SetSingleOffset(false);  //设置是否单项偏置

			toolingBoxBuilder1->Clearance()->SetValue(offsetCyl[0]);

		}



		std::vector<NXOpen::SelectionIntentRule *> rules1 = Rule(tagObject);


		NXOpen::ScCollector *scCollector1;
		scCollector1 = toolingBoxBuilder1->BoundedObject();



		scCollector1->ReplaceRules(rules1, false);



		std::vector<NXOpen::NXObject *> selections1(1);
		selections1[0] = dynamic_cast<NXOpen::NXObject*>(tagObject[0]);
		std::vector<NXOpen::NXObject *> deselections1(0);
		toolingBoxBuilder1->SetSelectedOccurrences(selections1, deselections1);
		
		toolingBoxBuilder1->CalculateBoxSize();

		markId1 = CcreateBox::theSession->SetUndoMark(NXOpen::Session::MarkVisibilityVisible, "Start Extrude");
		m_feature = toolingBoxBuilder1->CommitFeature();

		toolingBoxBuilder1->Destroy();

		workPart->FacetedBodies()->DeleteTemporaryFacesAndEdges(); //删除临时面和边
		m_isFeatureCreated = true;   //功能创建
		CcreateBox::theSession->UpdateManager()->DoUpdate(markId1);
		CcreateBox::theSession->DeleteUndoMark(markId1, NULL);

		std::vector< NXOpen::Body * > bodies = dynamic_cast<Features::ToolingBox *>(m_feature)->GetBodies();

		return bodies;


	}

	
}

tag_t  CcreateBox::subtract(Body* target, std::vector<NXOpen::TaggedObject *>tagObject)
{
	Body *toolBody = NULL;
	if (m_seletype == 0)
	{
		tag_t body;
		UF_MODL_ask_face_body(tagObject[0]->Tag(), &body);
		toolBody = dynamic_cast<Body*>(NXObjectManager::Get(body));
	}

	if (m_seletype == 1)
	{
		tag_t body;
		UF_MODL_ask_edge_body(tagObject[0]->Tag(), &body);
		toolBody = dynamic_cast<Body*>(NXObjectManager::Get(body));
	}

	if (m_seletype == 2)
	{
		int  n_parents = 0;
		tag_t  *parents = NULL_TAG;
		tag_t  body = NULL_TAG;
		int type;
		int subtype;
		for (int j = 0; j < tagObject.size(); j++)
		{
			UF_SO_ask_parents(tagObject[0]->Tag(), UF_SO_ASK_ALL_PARENTS, &n_parents, &parents);
			for (int i = 0; i < n_parents; i++)
			{
				UF_OBJ_ask_type_and_subtype(parents[i], &type, &subtype);
				if (type == UF_solid_type)
				{
					UF_MODL_ask_edge_body(parents[i], &body);
					toolBody = dynamic_cast<Body*>(NXObjectManager::Get(body));
				}
			}
			if (body == NULL_TAG)
			{
				uc1601("选择点不在体上", 1);
			}
		}
		
	}


	NXOpen::Features::BooleanFeature *nullNXOpen_Features_BooleanFeature = NULL;
	NXOpen::Features::BooleanBuilder *booleanBuilder1 = workPart->Features()->CreateBooleanBuilderUsingCollector(nullNXOpen_Features_BooleanFeature);

	NXOpen::ScCollector *scCollector1 = booleanBuilder1->ToolBodyCollector();

	NXOpen::GeometricUtilities::BooleanRegionSelect *booleanRegionSelect1 = booleanBuilder1->BooleanRegionSelect();

	booleanBuilder1->SetTolerance(0.001); //设置公差

	booleanBuilder1->SetCopyTools(true);  //设置保持工具体

	booleanBuilder1->SetOperation(NXOpen::Features::Feature::BooleanTypeSubtract); //设置相减

	bool added1 = booleanBuilder1->Targets()->Add(target); //设置目标体
	std::vector<NXOpen::TaggedObject *> targets1(1);
	targets1[0] = target;
	booleanRegionSelect1->AssignTargets(targets1);

	NXOpen::ScCollector *scCollector2 = workPart->ScCollectors()->CreateCollector();

	std::vector<NXOpen::Body *> bodies1(1);

	bodies1[0] = toolBody;

	NXOpen::BodyDumbRule *bodyDumbRule1 = workPart->ScRuleFactory()->CreateRuleBodyDumb(bodies1, true);
	std::vector<NXOpen::SelectionIntentRule *> rules1(1);
	rules1[0] = bodyDumbRule1;
	scCollector2->ReplaceRules(rules1, false);

	booleanBuilder1->SetToolBodyCollector(scCollector2);

	std::vector<NXOpen::TaggedObject *> targets2(1);
	targets2[0] = target;
	booleanRegionSelect1->AssignTargets(targets2);

	NXOpen::NXObject *nXObject1;
	nXObject1 = booleanBuilder1->Commit();
	
   
	
	booleanBuilder1->Destroy();
	tag_t bodyfeatTag= nXObject1->Tag();
	tag_t bodyTag = NULL_TAG;
	UF_MODL_ask_feat_body(bodyfeatTag, &bodyTag);
	return bodyTag;
}


/*void CcreateBox::trimBody(Body* bodys, Face* face)
{
	NXOpen::Features::TrimBody2 *nullNXOpen_Features_TrimBody2 = NULL;
	NXOpen::Features::TrimBody2Builder *trimBody2Builder1 = workPart->Features()->CreateTrimBody2Builder(nullNXOpen_Features_TrimBody2);

	NXOpen::Plane *plane1;

	std::vector<NXOpen::NXObject*> obj(1);
	obj[0] = dynamic_cast<NXOpen::NXObject*>(face);

	plane1->SetGeometry(obj);

	plane1->SetFlip(true);

	plane1->SetReverseSide(false);
	plane1->SetAlternate(NXOpen::PlaneTypes::AlternateTypeOne);

	plane1->Evaluate();

	plane1->SetUpdateOption(NXOpen::SmartObject::UpdateOptionWithinModeling);
	trimBody2Builder1->BooleanTool()->FacePlaneTool()->SetToolPlane(plane1);

	trimBody2Builder1->SetTolerance(0.001);

	trimBody2Builder1->BooleanTool()->SetToolOption(NXOpen::GeometricUtilities::BooleanToolBuilder::BooleanToolTypeNewPlane);

	trimBody2Builder1->BooleanTool()->ExtrudeRevolveTool()->ToolSection()->SetDistanceTolerance(0.001);

	trimBody2Builder1->BooleanTool()->ExtrudeRevolveTool()->ToolSection()->SetChainingTolerance(0.00095);

	std::vector<NXOpen::Body *> bodies1(1);
	bodies1[0] = bodys;
	NXOpen::BodyDumbRule *bodyDumbRule1 = workPart->ScRuleFactory()->CreateRuleBodyDumb(bodies1, true);

	std::vector<NXOpen::SelectionIntentRule *> rules1(1);
	rules1[0] = bodyDumbRule1;
	NXOpen::ScCollector *scCollector1 = workPart->ScCollectors()->CreateCollector();
	scCollector1->ReplaceRules(rules1, false);

	trimBody2Builder1->SetTargetBodyCollector(scCollector1);  //设定要修建得体

	trimBody2Builder1->BooleanTool()->SetReverseDirection(true);

	NXOpen::NXObject *nXObject1;
	nXObject1 = trimBody2Builder1->Commit();

	trimBody2Builder1->Destroy();



}*/


std::vector<NXOpen::SelectionIntentRule *>CcreateBox::Rule(std::vector<NXOpen::TaggedObject *>tagObject)
{
	std::vector<NXOpen::SelectionIntentRule *> rules1(1);

	if (m_seletype == 0)
	{
		NXOpen::FaceDumbRule *faceDumbRule1;
		std::vector<NXOpen::Face *> faces;
		for (int i = 0;i < tagObject.size();i++)
		{
			faces.push_back(dynamic_cast<Face*>(tagObject[i]));
		}
		faceDumbRule1 = workPart->ScRuleFactory()->CreateRuleFaceDumb(faces);  //设定选定面
		rules1[0] = faceDumbRule1;

	}

	if (m_seletype == 1)
	{
		NXOpen::EdgeDumbRule *edgeDumbRule1;
		std::vector<NXOpen::Edge *> edges;
		for (int i = 0;i < tagObject.size();i++)
		{
			edges.push_back(dynamic_cast<Edge*>(tagObject[i]));
		}
		edgeDumbRule1 = workPart->ScRuleFactory()->CreateRuleEdgeDumb(edges);  //设定选定线
		rules1[0] = edgeDumbRule1;
	}


	if (m_seletype == 2)
	{
		NXOpen::CurveDumbRule  *curveDumbRule;
		std::vector<NXOpen::Point *> points;
		for (int i = 0;i < tagObject.size();i++)
		{
			points.push_back(dynamic_cast<Point*>(tagObject[i]));
		}
		curveDumbRule = workPart->ScRuleFactory()->CreateRuleCurveDumbFromPoints(points);  //设定选定点
		rules1[0] = curveDumbRule;
	}
	return rules1;
}

NXOpen::Matrix3x3 CcreateBox::XYtoMatrix(NXOpen::Vector3d XAxis, NXOpen::Vector3d YAxis)
{
	double x[3] = { XAxis.X,XAxis.Y,XAxis.Z };
	double y[3] = { YAxis.X,YAxis.Y,YAxis.Z };
	double mtx[9];

	UF_MTX3_initialize(x, y, mtx);

	NXOpen::Matrix3x3 matrix(mtx[0], mtx[1], mtx[2], mtx[3], mtx[4], mtx[5], mtx[6], mtx[7], mtx[8]);

	return matrix;
}

NXOpen::Point3d CcreateBox::AbsPointTOCoordinate(Point3d origin, NXOpen::Vector3d XAxis, NXOpen::Vector3d YAxis, Point3d pointAbs)
{
	double originAbs[3] = { 0.0,0.0,0.0 }; //绝对坐标
	double xAxisAbs[3] = { 1,0,0 };
	double yAxisAbs[3] = { 0,1,0 };
	double mtxto[16] = {};
	double originPointAbs[3] = { origin.X, origin.Y , origin.Z };
	double x_vec[3] = { XAxis.X,XAxis.Y,XAxis.Z };
	double y_vec[3] = { YAxis.X,YAxis.Y,YAxis.Z };
	double pointAbsTmpe[3] = { pointAbs.X ,pointAbs.Y,pointAbs.Z };
	double pointCoorTmpe[3] = {};
	UF_MTX4_csys_to_csys(originPointAbs, x_vec, y_vec, originAbs, xAxisAbs, yAxisAbs, mtxto);

	UF_MTX4_vec3_multiply(pointAbsTmpe, mtxto, pointCoorTmpe);
	Point3d pointCoor(pointCoorTmpe[0], pointCoorTmpe[1], pointCoorTmpe[2]);
	for (int i = 0; i < 16; i++)
	{
		print(std::to_string(mtxto[i]));
	}
	
	return pointCoor;

}

Vector3d CcreateBox::getfaceDir(Face *face)
{

	tag_t face_tg = face->Tag();
	int  type;
	double point[3];
	double dir[3];

	double box[6];
	double  radius;
	double  rad_data;
	int  norm_dir;
	UF_MODL_ask_face_data(face_tg, &type, point, dir, box, &radius, &rad_data, &norm_dir);
	Vector3d vec(dir[0], dir[1], dir[2]);

	return vec;

}

double CcreateBox::getfaceRadius(Face *face)
{

	tag_t face_tg = face->Tag();
	int  type;
	double point[3];
	double dir[3];

	double box[6];
	double  radius;
	double  rad_data;
	int  norm_dir;
	UF_MODL_ask_face_data(face_tg, &type, point, dir, box, &radius, &rad_data, &norm_dir);
	Vector3d vec(dir[0], dir[1], dir[2]);
	if (type==16)
	{
		return radius;
	}
	else
	{
		return 0;
	}

}

Point3d CcreateBox::getfacecentre(Face *face)
{
	tag_t face_tg = face->Tag();

	int  type;
	double point[3];
	double out_point[3];
	double dir[3];
	double box[6];
	double  radius;
	double  rad_data;
	int  norm_dir;
	UF_MODL_ask_face_data(face_tg, &type, point, dir, box, &radius, &rad_data, &norm_dir);
	//	UF_CSYS_map_point(UF_CSYS_WORK_COORDS, point, UF_CSYS_ROOT_WCS_COORDS, out_point);

	Point3d point3d(point[0], point[1], point[2]);

	return point3d;
}

void CcreateBox::Point_WcstoAbs(double wcs_point[3], double abs_point[3])
{

	UF_CSYS_map_point(UF_CSYS_ROOT_WCS_COORDS, wcs_point, UF_CSYS_WORK_COORDS, abs_point);
}

void CcreateBox::Point_AbstoWcs(double abs_point[3], double wcs_point[3])
{

	UF_CSYS_map_point(UF_CSYS_WORK_COORDS, abs_point, UF_CSYS_ROOT_WCS_COORDS, wcs_point);

}

void CcreateBox::Vec_WcstoAbs(double wcs_Vec[3], double abs_Vec[3])
{
	double tol, magnitude, wcs_origin[3] = { 0.0,0.0,0.0 }, abs_origin[3];
	UF_MODL_ask_distance_tolerance(&tol);
	UF_CSYS_map_point(UF_CSYS_ROOT_WCS_COORDS, wcs_origin, UF_CSYS_WORK_COORDS, abs_origin);
	UF_CSYS_map_point(UF_CSYS_ROOT_WCS_COORDS, wcs_Vec, UF_CSYS_WORK_COORDS, abs_Vec);
	UF_VEC3_sub(abs_Vec, abs_origin, abs_Vec);
	UF_VEC3_unitize(abs_Vec, tol, &magnitude, abs_Vec);
}

void CcreateBox::Vec_AbstoWcs(double abs_Vec[3], double wcs_Vec[3])
{
	double tol, magnitude, abs_origin[3] = { 0.0,0.0,0.0 }, wcs_origin[3];
	UF_MODL_ask_distance_tolerance(&tol);
	UF_CSYS_map_point(UF_CSYS_WORK_COORDS, abs_origin, UF_CSYS_ROOT_WCS_COORDS, wcs_origin);
	UF_CSYS_map_point(UF_CSYS_WORK_COORDS, abs_Vec, UF_CSYS_ROOT_WCS_COORDS, wcs_Vec);
	UF_VEC3_sub(wcs_Vec, wcs_origin, wcs_Vec);
	UF_VEC3_unitize(wcs_Vec, tol, &magnitude, wcs_Vec);
}

tag_t CcreateBox::trimBody(tag_t bodyTag, std::vector<NXOpen::TaggedObject *>tagObject)
{
	tag_t trim_feature = NULL_TAG;
	tag_t  dplane_tag = NULL_TAG;
	tag_t trim_body= NULL_TAG;
	int  type=0;
	double point[3] = {};
	double dir[3] = {};
	double box[6] = {};
	double radius=0.0;
	double rad_data=0.0;
	int  norm_dir=0.0;
//	uf_list_p_t body_list;
	


	if (m_seletype==0)
	{
		

		for (int i = 0; i < tagObject.size(); i++)
		{
			
			UF_MODL_ask_face_data(tagObject[i]->Tag(), &type, point, dir, box, &radius, &rad_data, &norm_dir);
			if (trim_feature== NULL_TAG)
			{
				if (type == 22)
				{
			//		UF_MODL_create_list(&body_list);
					UF_MODL_create_fixed_dplane(point, dir, &dplane_tag);

					
				    UF_MODL_trim_body(bodyTag, dplane_tag, 1, &trim_feature);
					
					UF_MODL_ask_face_body(trim_feature, &trim_body);
					
				//	UF_MODL_put_list_item(body_list, trim_feature);
					//UF_MODL_delete_body_parms(body_list);
				//	UF_OBJ_delete_object(dplane_tag);
			//		UF_MODL_delete_list(&body_list);
					trim_feature = NULL_TAG;
				}
			}
			else
			{
				if (type == 22)
				{

					UF_MODL_create_fixed_dplane(point, dir, &dplane_tag);
					
					
					UF_MODL_trim_body(trim_body, dplane_tag, 1, &trim_feature);
					trim_body = NULL_TAG;
					UF_MODL_ask_face_body(trim_feature, &trim_body);
					trim_feature = NULL_TAG;
			//		UF_OBJ_delete_object(dplane_tag);
				}
			}
			



		}
	}
	
	return trim_body;
}