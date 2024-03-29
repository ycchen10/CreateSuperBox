﻿//==============================================================================
//  WARNING!!  This file is overwritten by the Block UI Styler while generating
//  the automation code. Any modifications to this file will be lost after
//  generating the code again.
//
//       Filename:  C:\Users\ycchen10\OneDrive - kochind.com\Desktop\NX8.5Dev\UI\CreateSuperBox.cpp
//
//        This file was generated by the NX Block UI Styler
//        Created by: ycchen10
//              Version: NX 11
//              Date: 11-02-2018  (Format: mm-dd-yyyy)
//              Time: 08:14 (Format: hh-mm)
//
//==============================================================================

//==============================================================================
//  Purpose:  This TEMPLATE file contains C++ source to guide you in the
//  construction of your Block application dialog. The generation of your
//  dialog file (.dlx extension) is the first step towards dialog construction
//  within NX.  You must now create a NX Open application that
//  utilizes this file (.dlx).
//
//  The information in this file provides you with the following:
//
//  1.  Help on how to load and display your Block UI Styler dialog in NX
//      using APIs provided in NXOpen.BlockStyler namespace
//  2.  The empty callback methods (stubs) associated with your dialog items
//      have also been placed in this file. These empty methods have been
//      created simply to start you along with your coding requirements.
//      The method name, argument list and possible return values have already
//      been provided for you.
//==============================================================================

//------------------------------------------------------------------------------
//These includes are needed for the following template code
//------------------------------------------------------------------------------
#include "CreateSuperBox.hpp"
using namespace NXOpen;
using namespace NXOpen::BlockStyler;

//------------------------------------------------------------------------------
// Initialize static variables
//------------------------------------------------------------------------------
Session *(CreateSuperBox::theSession) = NULL;
UI *(CreateSuperBox::theUI) = NULL;
//------------------------------------------------------------------------------
// Constructor for NX Styler class
//------------------------------------------------------------------------------
CreateSuperBox::CreateSuperBox()
{
	try
	{
		// Initialize the NX Open C++ API environment
		CreateSuperBox::theSession = NXOpen::Session::GetSession();
		CreateSuperBox::theUI = UI::GetUI();
		theDlxFileName = "CreateSuperBox.dlx";
		theDialog = CreateSuperBox::theUI->CreateDialog(theDlxFileName);
		// Registration of callback functions
		theDialog->AddApplyHandler(make_callback(this, &CreateSuperBox::apply_cb));
		theDialog->AddOkHandler(make_callback(this, &CreateSuperBox::ok_cb));
		theDialog->AddUpdateHandler(make_callback(this, &CreateSuperBox::update_cb));
		theDialog->AddFilterHandler(make_callback(this, &CreateSuperBox::filter_cb));
		theDialog->AddInitializeHandler(make_callback(this, &CreateSuperBox::initialize_cb));
		theDialog->AddFocusNotifyHandler(make_callback(this, &CreateSuperBox::focusNotify_cb));
		theDialog->AddDialogShownHandler(make_callback(this, &CreateSuperBox::dialogShown_cb));
	}
	catch (exception& ex)
	{
		//---- Enter your exception handling code here -----
		throw;
	}
}

//------------------------------------------------------------------------------
// Destructor for NX Styler class
//------------------------------------------------------------------------------
CreateSuperBox::~CreateSuperBox()
{
	if (theDialog != NULL)
	{
		delete theDialog;
		theDialog = NULL;
	}
}
//------------------------------- DIALOG LAUNCHING ---------------------------------
//
//    Before invoking this application one needs to open any part/empty part in NX
//    because of the behavior of the blocks.
//
//    Make sure the dlx file is in one of the following locations:
//        1.) From where NX session is launched
//        2.) $UGII_USER_DIR/application
//        3.) For released applications, using UGII_CUSTOM_DIRECTORY_FILE is highly
//            recommended. This variable is set to a full directory path to a file 
//            containing a list of root directories for all custom applications.
//            e.g., UGII_CUSTOM_DIRECTORY_FILE=$UGII_BASE_DIR\ugii\menus\custom_dirs.dat
//
//    You can create the dialog using one of the following way:
//
//    1. USER EXIT
//
//        1) Create the Shared Library -- Refer "Block UI Styler programmer's guide"
//        2) Invoke the Shared Library through File->Execute->NX Open menu.
//
//------------------------------------------------------------------------------
extern "C" DllExport void  ufusr(char *param, int *retcod, int param_len)
{
	CreateSuperBox *theCreateSuperBox = NULL;
	try
	{
		theCreateSuperBox = new CreateSuperBox();
		// The following method shows the dialog immediately
		theCreateSuperBox->Show();
	}
	catch (exception& ex)
	{
		//---- Enter your exception handling code here -----
		CreateSuperBox::theUI->NXMessageBox()->Show("Block Styler", NXOpen::NXMessageBox::DialogTypeError, ex.what());
	}
	if (theCreateSuperBox != NULL)
	{
		delete theCreateSuperBox;
		theCreateSuperBox = NULL;
	}
}

//------------------------------------------------------------------------------
// This method specifies how a shared image is unloaded from memory
// within NX. This method gives you the capability to unload an
// internal NX Open application or user  exit from NX. Specify any
// one of the three constants as a return value to determine the type
// of unload to perform:
//
//
//    Immediately : unload the library as soon as the automation program has completed
//    Explicitly  : unload the library from the "Unload Shared Image" dialog
//    AtTermination : unload the library when the NX session terminates
//
//
// NOTE:  A program which associates NX Open applications with the menubar
// MUST NOT use this option since it will UNLOAD your NX Open application image
// from the menubar.
//------------------------------------------------------------------------------
extern "C" DllExport int ufusr_ask_unload()
{
	//return (int)Session::LibraryUnloadOptionExplicitly;
	return (int)Session::LibraryUnloadOptionImmediately;
	//return (int)Session::LibraryUnloadOptionAtTermination;
}

//------------------------------------------------------------------------------
// Following method cleanup any housekeeping chores that may be needed.
// This method is automatically called by NX.
//------------------------------------------------------------------------------
extern "C" DllExport void ufusr_cleanup(void)
{
	try
	{
		//---- Enter your callback code here -----
	}
	catch (exception& ex)
	{
		//---- Enter your exception handling code here -----
		CreateSuperBox::theUI->NXMessageBox()->Show("Block Styler", NXOpen::NXMessageBox::DialogTypeError, ex.what());
	}
}

int CreateSuperBox::Show()
{
	try
	{
		theDialog->Show();
	}
	catch (exception& ex)
	{
		//---- Enter your exception handling code here -----
		CreateSuperBox::theUI->NXMessageBox()->Show("Block Styler", NXOpen::NXMessageBox::DialogTypeError, ex.what());
	}
	return 0;
}

//------------------------------------------------------------------------------
//---------------------Block UI Styler Callback Functions--------------------------
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//Callback Name: initialize_cb
//------------------------------------------------------------------------------
void CreateSuperBox::initialize_cb()
{
	try
	{
		group0 = dynamic_cast<NXOpen::BlockStyler::Group*>(theDialog->TopBlock()->FindBlock("group0"));
		en_type = dynamic_cast<NXOpen::BlockStyler::Enumeration*>(theDialog->TopBlock()->FindBlock("en_type"));
		group = dynamic_cast<NXOpen::BlockStyler::Group*>(theDialog->TopBlock()->FindBlock("group"));
		m_seleType_enum = dynamic_cast<NXOpen::BlockStyler::Enumeration*>(theDialog->TopBlock()->FindBlock("m_seleType_enum"));
		m_sele = dynamic_cast<NXOpen::BlockStyler::SelectObject*>(theDialog->TopBlock()->FindBlock("m_sele"));
		m_manip = dynamic_cast<NXOpen::BlockStyler::SpecifyOrientation*>(theDialog->TopBlock()->FindBlock("m_manip"));
		m_vector = dynamic_cast<NXOpen::BlockStyler::SpecifyVector*>(theDialog->TopBlock()->FindBlock("m_vector"));
		group1 = dynamic_cast<NXOpen::BlockStyler::Group*>(theDialog->TopBlock()->FindBlock("group1"));
		m_offset_bool = dynamic_cast<NXOpen::BlockStyler::Toggle*>(theDialog->TopBlock()->FindBlock("m_offset_bool"));
		m_offset_dim = dynamic_cast<NXOpen::BlockStyler::LinearDimension*>(theDialog->TopBlock()->FindBlock("m_offset_dim"));
		group5 = dynamic_cast<NXOpen::BlockStyler::Group*>(theDialog->TopBlock()->FindBlock("group5"));
		m_PositiveX = dynamic_cast<NXOpen::BlockStyler::LinearDimension*>(theDialog->TopBlock()->FindBlock("m_PositiveX"));
		m_NegativeX = dynamic_cast<NXOpen::BlockStyler::LinearDimension*>(theDialog->TopBlock()->FindBlock("m_NegativeX"));
		m_PositiveY = dynamic_cast<NXOpen::BlockStyler::LinearDimension*>(theDialog->TopBlock()->FindBlock("m_PositiveY"));
		m_NegativeY = dynamic_cast<NXOpen::BlockStyler::LinearDimension*>(theDialog->TopBlock()->FindBlock("m_NegativeY"));
		m_NegativeZ = dynamic_cast<NXOpen::BlockStyler::LinearDimension*>(theDialog->TopBlock()->FindBlock("m_NegativeZ"));
		m_PositiveZ = dynamic_cast<NXOpen::BlockStyler::LinearDimension*>(theDialog->TopBlock()->FindBlock("m_PositiveZ"));
		m_Radial = dynamic_cast<NXOpen::BlockStyler::LinearDimension*>(theDialog->TopBlock()->FindBlock("m_Radial"));
		group2 = dynamic_cast<NXOpen::BlockStyler::Group*>(theDialog->TopBlock()->FindBlock("group2"));
		m_hile_bool = dynamic_cast<NXOpen::BlockStyler::Toggle*>(theDialog->TopBlock()->FindBlock("m_hile_bool"));
		m_unite_bool = dynamic_cast<NXOpen::BlockStyler::Toggle*>(theDialog->TopBlock()->FindBlock("m_unite_bool"));
		m_plane_bool = dynamic_cast<NXOpen::BlockStyler::Toggle*>(theDialog->TopBlock()->FindBlock("m_plane_bool"));
		m_surface_bool = dynamic_cast<NXOpen::BlockStyler::Toggle*>(theDialog->TopBlock()->FindBlock("m_surface_bool"));
		group3 = dynamic_cast<NXOpen::BlockStyler::Group*>(theDialog->TopBlock()->FindBlock("group3"));
		m_translucency_int = dynamic_cast<NXOpen::BlockStyler::IntegerBlock*>(theDialog->TopBlock()->FindBlock("m_translucency_int"));
		m_layer_int = dynamic_cast<NXOpen::BlockStyler::IntegerBlock*>(theDialog->TopBlock()->FindBlock("m_layer_int"));
		m_color_int = dynamic_cast<NXOpen::BlockStyler::ObjectColorPicker*>(theDialog->TopBlock()->FindBlock("m_color_int"));
		group4 = dynamic_cast<NXOpen::BlockStyler::Group*>(theDialog->TopBlock()->FindBlock("group4"));
		button0 = dynamic_cast<NXOpen::BlockStyler::Button*>(theDialog->TopBlock()->FindBlock("button0"));
		m_preview_str = dynamic_cast<NXOpen::BlockStyler::Label*>(theDialog->TopBlock()->FindBlock("m_preview_str"));
		
		m_preview_str->SetShow(false);

		NXOpen::BlockStyler::PropertyList* ResultButtonproplist = button0->GetProperties();
		ResultButtonproplist->SetString("Bitmap", "general_preview");
		ResultButtonproplist->SetString("Label", "Show Result");
		workPart = theSession->Parts()->Work();
		m_PositiveX->SetShow(false);
		m_PositiveY->SetShow(false);
		m_PositiveZ->SetShow(false);
		m_NegativeX->SetShow(false);
		m_NegativeY->SetShow(false);
		m_NegativeZ->SetShow(false);
		m_Radial->SetShow(false);
//		m_sele->SetAutomaticProgression(false);
		m_vector->SetStepStatusAsString("Optional");


		m_objects.clear();

		if (m_offset_bool->Value())
		{
			m_offset_dim->SetEnable(true);
			if (m_offset_dim->Value() == 0)
			{
				m_offset_dim->SetValue(1.0);

			}

			for (int i = 0; i < 6; i++)
			{
				m_offset[i] = m_offset_dim->Value();
			}
			for (int i = 0; i < 3; i++)
			{
				m_offsetCyl[i] = m_offset_dim->Value();
			}
		}
		else
		{
			m_offset_dim->SetEnable(false);

			for (int i = 0; i < 6; i++)
			{
				m_offset[i] = 1.0;
			}
			for (int i = 0; i < 3; i++)
			{
				m_offsetCyl[i] = 1.0;
			}
		}
		
		
		m_box = new CcreateBox();

		if (en_type->GetProperties()->GetEnum("Value") == 0)
		{
			m_vector->SetShow(false);
			m_manip->SetShow(true);
		}
		else
		{
			m_vector->SetShow(true);
			m_manip->SetShow(false);
		}
		m_seleType_enum->GetProperties()->SetEnum("Value",0);
		m_seleType_enum->SetNumberOfColumns(0);
		m_seletype = m_seleType_enum->GetProperties()->GetEnum("Value");
		m_box->print(std::to_string(m_seletype));
		if (m_seletype == 0)
		{
			m_sele->GetProperties()->SetString("LabelString", "选择面");


			m_sele->AddFilter(UF_solid_type, UF_all_subtype, UF_UI_SEL_FEATURE_ANY_FACE);  //  面  
			
		}

		if (m_seletype == 1)
		{
			m_sele->GetProperties()->SetString("LabelString", "选择边缘");//
			m_sele->AddFilter(NXOpen::BlockStyler::SelectObject::FilterTypeCurvesAndEdges);  //边

		}


		if (m_seletype == 2)
		{
			m_sele->GetProperties()->SetString("LabelString", "选择点");
			m_sele->AddFilter(UF_point_type, 0, 0); // 点
			
		}
		m_box->getseletype(m_seleType_enum->GetProperties()->GetEnum("Value"));
		m_box->getoffsetbool(m_offset_bool->Value());
		

		value = true;


	}
	catch (exception& ex)
	{
		//---- Enter your exception handling code here -----
		CreateSuperBox::theUI->NXMessageBox()->Show("Block Styler", NXOpen::NXMessageBox::DialogTypeError, ex.what());
	}
}

//------------------------------------------------------------------------------
//Callback Name: dialogShown_cb
//This callback is executed just before the dialog launch. Thus any value set 
//here will take precedence and dialog will be launched showing that value. 
//------------------------------------------------------------------------------
void CreateSuperBox::dialogShown_cb()
{
	try
	{
		//---- Enter your callback code here -----
	}
	catch (exception& ex)
	{
		//---- Enter your exception handling code here -----
		CreateSuperBox::theUI->NXMessageBox()->Show("Block Styler", NXOpen::NXMessageBox::DialogTypeError, ex.what());
	}
}

//------------------------------------------------------------------------------
//Callback Name: apply_cb
//------------------------------------------------------------------------------
int CreateSuperBox::apply_cb()
{
	int errorCode = 0;
	try
	{
		//---- Enter your callback code here -----
		

	}
	catch (exception& ex)
	{
		//---- Enter your exception handling code here -----
		errorCode = 1;
		CreateSuperBox::theUI->NXMessageBox()->Show("Block Styler", NXOpen::NXMessageBox::DialogTypeError, ex.what());
	}
	return errorCode;
}

//------------------------------------------------------------------------------
//Callback Name: update_cb
//------------------------------------------------------------------------------
int CreateSuperBox::update_cb(NXOpen::BlockStyler::UIBlock* block)
{
	try
	{

		
		Matrix3x3 matx;
		Matrix3x3 matr = workPart->WCS()->CoordinateSystem()->Orientation()->Element();
		
		if (block == en_type)
		{
			//---------Enter your code here-----------


			if (en_type->GetProperties()->GetEnum("Value") == 0)
			{
				m_vector->SetShow(false);
				m_manip->SetShow(true);
				if (bodies.size()>0)
				{
					UF_OBJ_delete_object(bodies[0]->Tag());
					m_objects.clear();
					m_sele->SetSelectedObjects(m_objects);
					m_PositiveX->SetShow(false);
					m_PositiveY->SetShow(false);
					m_PositiveZ->SetShow(false);
					m_NegativeX->SetShow(false);
					m_NegativeY->SetShow(false);
					m_NegativeZ->SetShow(false);
					m_Radial->SetShow(false);
				}
			}
			else
			{
				m_vector->SetShow(true);
				m_manip->SetShow(false);
				if (bodies.size()>0)
				{
					UF_OBJ_delete_object(bodies[0]->Tag());
					m_objects.clear();
					m_sele->SetSelectedObjects(m_objects);
					m_PositiveX->SetShow(false);
					m_PositiveY->SetShow(false);
					m_PositiveZ->SetShow(false);
					m_NegativeX->SetShow(false);
					m_NegativeY->SetShow(false);
					m_NegativeZ->SetShow(false);
					m_Radial->SetShow(false);
				}
			}
		
			
		}
		else if (block == m_seleType_enum)
		{
			//---------Enter your code here-----------
			m_objects.clear();
			m_sele->SetSelectedObjects(m_objects);
			m_seletype = m_seleType_enum->GetProperties()->GetEnum("Value");
			m_box->getseletype(m_seletype);
			
			if (m_seletype == 0)
			{
				m_sele->GetProperties()->SetString("LabelString", "选择面");
				m_sele->AddFilter(UF_solid_type, UF_all_subtype, UF_UI_SEL_FEATURE_ANY_FACE);  //  Faces  
			}

			if (m_seletype == 1)
			{
				m_sele->GetProperties()->SetString("LabelString", "选择边缘");//只选择 实体边缘 直线和圆弧
				m_sele->AddFilter(NXOpen::BlockStyler::SelectObject::FilterTypeCurvesAndEdges);

			}


			if (m_seletype == 2)
			{
				m_sele->GetProperties()->SetString("LabelString", "选择点");
				m_sele->AddFilter(UF_point_type, 0, 0);
				
				
			}

		}
		else if (block == m_sele)
		{
			//---------Enter your code here-----------
			bodies.clear();
		
			m_objects = m_sele->GetSelectedObjects();
			
			pointOrigin = m_box->getBounding_box(m_objects);

			Point3d originaAbs;
			Vector3d vec;
			Vector3d xvec;
			Vector3d yvec;
			workPart->WCS()->CoordinateSystem()->GetDirections(&xvec, &yvec);
			matx = m_box->XYtoMatrix(xvec, yvec);
			if (en_type->GetProperties()->GetEnum("Value") == 0) //创建方块盒
			{
				
			

				m_PositiveX->SetShow(true);
				m_PositiveY->SetShow(true);
				m_PositiveZ->SetShow(true);
				m_NegativeX->SetShow(true);
				m_NegativeY->SetShow(true);
				m_NegativeZ->SetShow(true);

     			m_manip->SetOrigin(pointOrigin);

				m_manip->SetXAxis(xvec);
				m_manip->SetYAxis(yvec);
				m_manip->GetProperties()->SetLogical("Enable", true); //设置可以动态拖拉
				
				bodies = m_box->toolingBox(m_objects, pointOrigin, matx, m_offset);
			
				
				std::vector<Face*> faces1 = bodies[0]->GetFaces();
				
				for (int i = 0; i < faces1.size(); i++)
				{
					
					originaAbs = m_box->getfacecentre(faces1[i]);
					vec = m_box->getfaceDir(faces1[i]);
					Point3d originaWcs = m_box->AbsPointTOCoordinate(m_manip->Origin(), m_manip->XAxis(), m_manip->YAxis(), originaAbs);

					int tmp = 0;

					if (originaWcs.X > 0.01)
					{
						m_PositiveX->SetHandleOrientation(vec);
						m_PositiveX->SetHandleOrigin(originaAbs);

					}
					if (originaWcs.X  < -0.01)
					{
						m_NegativeX->SetHandleOrientation(vec);
						m_NegativeX->SetHandleOrigin(originaAbs);

					}
					if (originaWcs.Y > 0.01)
					{
						m_PositiveY->SetHandleOrientation(vec);
						m_PositiveY->SetHandleOrigin(originaAbs);

					}
					if (originaWcs.Y < -0.01)
					{
						m_NegativeY->SetHandleOrientation(vec);
						m_NegativeY->SetHandleOrigin(originaAbs);

					}
					if (originaWcs.Z > 0.01)
					{
						m_PositiveZ->SetHandleOrientation(vec);
						m_PositiveZ->SetHandleOrigin(originaAbs);

					}
					if (originaWcs.Z < -0.01)
					{
						m_NegativeZ->SetHandleOrientation(vec);
						m_NegativeZ->SetHandleOrigin(originaAbs);

					}

				}




			}
			
			else 
			{
				m_PositiveX->SetShow(false);
				m_PositiveY->SetShow(false);
				m_PositiveZ->SetShow(true);
				m_NegativeX->SetShow(false);
				m_NegativeY->SetShow(false);
				m_NegativeZ->SetShow(true);
				m_Radial->SetShow(true);
				double xVecWcs[3];
				double zVecWcs[3];
				double mtx[9] = { matr.Xx,matr.Xy,matr.Xz,matr.Yx,matr.Yy,matr.Yz,matr.Zx,matr.Zy,matr.Zz, };
				UF_MTX3_x_vec(mtx, xVecWcs);
				UF_MTX3_z_vec(mtx, zVecWcs);

				Vector3d xVec(xVecWcs[0], xVecWcs[1], xVecWcs[2]);
				
				Vector3d zVec(zVecWcs[0], zVecWcs[1], zVecWcs[2]);
				vecAxis = zVec;
				bodies = m_box->toolingCylinder(m_objects, pointOrigin, vecAxis, m_offsetCyl);
				std::vector<Face*> faces1 = bodies[0]->GetFaces();
				for (int i = 0; i < faces1.size(); i++)
				{

					double radius = m_box->getfaceRadius(faces1[i]);
					
					if (radius==0)
					{
						originaAbs = m_box->getfacecentre(faces1[i]);
						vec = m_box->getfaceDir(faces1[i]);
						Point3d originaWcs = m_box->AbsPointTOCoordinate(pointOrigin, xvec, yvec, originaAbs);
						

						
						if (originaWcs.Z > 0.01)
						{
							m_PositiveZ->SetHandleOrientation(vec);
							m_PositiveZ->SetHandleOrigin(originaAbs);

						}
						if (originaWcs.Z < -0.01)
						{
							m_NegativeZ->SetHandleOrientation(vec);
							m_NegativeZ->SetHandleOrigin(originaAbs);

						}
					}
					else
					{
						
						Point3d radial(pointOrigin.X + radius, pointOrigin.Y, pointOrigin.Z);
						m_Radial->SetHandleOrientation(xVec);
						m_Radial->SetHandleOrigin(radial);
					}
					

				}
			}
			
		}
		else if (block == m_manip)
		{
			//---------Enter your code here-----------
			Point3d originaAbs;
			Vector3d vec;
			matx = m_box->XYtoMatrix(m_manip->XAxis(), m_manip->YAxis());
			bodies = m_box->toolingBox(m_objects, pointOrigin, matx, m_offset);
			std::vector<Face*> faces1 = bodies[0]->GetFaces();

			for (int i = 0; i < faces1.size(); i++)
			{

				originaAbs = m_box->getfacecentre(faces1[i]);
				vec = m_box->getfaceDir(faces1[i]);
				Point3d originaWcs = m_box->AbsPointTOCoordinate(m_manip->Origin(), m_manip->XAxis(), m_manip->YAxis(), originaAbs);

				int tmp = 0;

				if (originaWcs.X > 0.01)
				{
					m_PositiveX->SetHandleOrientation(vec);
					m_PositiveX->SetHandleOrigin(originaAbs);

				}
				if (originaWcs.X  < -0.01)
				{
					m_NegativeX->SetHandleOrientation(vec);
					m_NegativeX->SetHandleOrigin(originaAbs);

				}
				if (originaWcs.Y > 0.01)
				{
					m_PositiveY->SetHandleOrientation(vec);
					m_PositiveY->SetHandleOrigin(originaAbs);

				}
				if (originaWcs.Y < -0.01)
				{
					m_NegativeY->SetHandleOrientation(vec);
					m_NegativeY->SetHandleOrigin(originaAbs);

				}
				if (originaWcs.Z > 0.01)
				{
					m_PositiveZ->SetHandleOrientation(vec);
					m_PositiveZ->SetHandleOrigin(originaAbs);

				}
				if (originaWcs.Z < -0.01)
				{
					m_NegativeZ->SetHandleOrientation(vec);
					m_NegativeZ->SetHandleOrigin(originaAbs);

				}

			}

		}
		else if (block == m_vector)
		{
			//---------Enter your code here-----------
			vecAxis = m_vector->Vector();
			double vec1[3] = { vecAxis.X,vecAxis.Y,vecAxis.Z };
			double vec_perp[3];
			double negated_vec[3];
			double zVecWcs[3];
			double mtx[9] = { matr.Xx,matr.Xy,matr.Xz,matr.Yx,matr.Yy,matr.Yz,matr.Zx,matr.Zy,matr.Zz, };
			UF_MTX3_z_vec(mtx, zVecWcs);
			Vector3d zVec(zVecWcs[0], zVecWcs[1], zVecWcs[2]);
			UF_VEC3_ask_perpendicular(vec1, vec_perp);
			UF_VEC3_negate(vec1, negated_vec);
			Vector3d perpenVec(vec_perp[0], vec_perp[1], vec_perp[2]);
			Vector3d negatedVec(negated_vec[0], negated_vec[1], negated_vec[2]);
			bodies = m_box->toolingCylinder(m_objects, pointOrigin, vecAxis, m_offsetCyl);
			std::vector<Face*> faces1 = bodies[0]->GetFaces();
			for (int i = 0; i < faces1.size(); i++)
			{

				double radius = m_box->getfaceRadius(faces1[i]);
				Vector3d vec1 = m_box->getfaceDir(faces1[i]);
				Point3d originaAbs = m_box->getfacecentre(faces1[i]);
				if (radius == 0)
				{
					
					
				    
					double vecface[3] = { vec1.X,vec1.Y,vec1.Z };
					double vec_Axis[3] = { vecAxis.X,vecAxis.Y,vecAxis.Z };
					double  dot_product;
					UF_VEC3_dot(vecface, vec_Axis, &dot_product);
					if (dot_product>0)
					{
						m_PositiveZ->SetHandleOrientation(vec1);
						m_PositiveZ->SetHandleOrigin(originaAbs);
					}
					
					else
					{
						m_NegativeZ->SetHandleOrientation(vec1);
						m_NegativeZ->SetHandleOrigin(originaAbs);

					}
				}
				else
				{

					
					double origina_Abs[3] = { originaAbs.X , originaAbs.Y, originaAbs.Z };
					double origina_Wcs[3];
					m_box->Point_AbstoWcs(origina_Abs, origina_Wcs);
					origina_Wcs[2] = origina_Wcs[2] + radius;
					m_box->Point_WcstoAbs(origina_Wcs, origina_Abs);
					Point3d radial(origina_Abs[0], origina_Abs[1], origina_Abs[2]);
					m_Radial->SetHandleOrientation(zVec);
					m_Radial->SetHandleOrigin(radial);
				}


			}

		}
		else if (block == m_offset_bool)
		{
			//---------Enter your code here-----------
			if (m_offset_bool->Value())
			{
				m_offset_dim->SetEnable(true);

				
			}
			else
			{
				m_offset_dim->SetEnable(false);

			}
			m_box->getoffsetbool(m_offset_bool->Value());


		}
		else if (block == m_offset_dim)
		{
			//---------Enter your code here-----------
			if (en_type->GetProperties()->GetEnum("Value") == 0)
			{
				for (int i = 0; i < 6; i++)
				{
					m_offset[i] = m_offset_dim->Value();
				}

				bodies = m_box->toolingBox(m_objects, pointOrigin, matx, m_offset);
			}
			else
			{
				for (int i = 0; i < 3; i++)
				{
					m_offsetCyl[i] = m_offset_dim->Value();
				}
				bodies = m_box->toolingCylinder(m_objects, pointOrigin, vecAxis, m_offsetCyl);
			}

		

		}
		else if (block == m_PositiveX)
		{
			//---------Enter your code here-----------
			if (!m_offset_bool->Value())
			{
				m_offset[0] = m_PositiveX->Value();
				bodies = m_box->toolingBox(m_objects, pointOrigin, matx, m_offset);

			}
			else
			{


				for (int i = 0; i < 6; i++)
				{
					m_offset[i] = m_PositiveX->Value();
				}

				m_NegativeX->SetValue(m_offset[1]);
				m_PositiveY->SetValue(m_offset[2]);
				m_NegativeY->SetValue(m_offset[3]);
				m_PositiveZ->SetValue(m_offset[4]);
				m_NegativeZ->SetValue(m_offset[5]);
				bodies = m_box->toolingBox(m_objects, pointOrigin, matx, m_offset);
			}


		
		}
		else if (block == m_NegativeX)
		{
			//---------Enter your code here-----------
			if (!m_offset_bool->Value())
			{
				m_offset[1] = m_NegativeX->Value();
				bodies = m_box->toolingBox(m_objects, pointOrigin, matx, m_offset);
			}

			else
			{
				for (int i = 0; i < 6; i++)
				{
					m_offset[i] = m_NegativeX->Value();
				}
				m_PositiveX->SetValue(m_offset[0]);
				m_PositiveY->SetValue(m_offset[0]);
				m_NegativeY->SetValue(m_offset[0]);
				m_PositiveZ->SetValue(m_offset[0]);
				m_NegativeZ->SetValue(m_offset[0]);
				bodies = m_box->toolingBox(m_objects, pointOrigin, matx, m_offset);

			}

		}
		else if (block == m_PositiveY)
		{
			//---------Enter your code here-----------
			
			if (!m_offset_bool->Value())
			{
				m_offset[2] = m_PositiveY->Value();
				bodies = m_box->toolingBox(m_objects, pointOrigin, matx, m_offset);
			}
			else
			{
				for (int i = 0; i < 6; i++)
				{
					m_offset[i] = m_PositiveY->Value();
				}
				m_PositiveX->SetValue(m_offset[0]);
				m_NegativeX->SetValue(m_offset[0]);
				m_NegativeY->SetValue(m_offset[0]);
				m_PositiveZ->SetValue(m_offset[0]);
				m_NegativeZ->SetValue(m_offset[0]);
				bodies = m_box->toolingBox(m_objects, pointOrigin, matx, m_offset);
			}

		}
		else if (block == m_NegativeY)
		{
			//---------Enter your code here-----------
			if (!m_offset_bool->Value())
			{
				m_offset[3] = m_NegativeY->Value();
				bodies = m_box->toolingBox(m_objects, pointOrigin, matx, m_offset);
			}
			else
			{
				for (int i = 0; i < 6; i++)
				{
					m_offset[i] = m_NegativeY->Value();
				}

				m_PositiveX->SetValue(m_offset[0]);
				m_NegativeX->SetValue(m_offset[0]);
				m_PositiveY->SetValue(m_offset[0]);
				m_PositiveZ->SetValue(m_offset[0]);
				m_NegativeZ->SetValue(m_offset[0]);
				bodies = m_box->toolingBox(m_objects, pointOrigin, matx, m_offset);
			}

		}
		else if (block == m_NegativeZ)
		{
			//---------Enter your code here-----------
			if (en_type->GetProperties()->GetEnum("Value") == 0)
			{
				if (!m_offset_bool->Value())
				{
					m_offset[5] = m_NegativeZ->Value();
					bodies = m_box->toolingBox(m_objects, pointOrigin, matx, m_offset);
				}

				else
				{
					for (int i = 0; i < 6; i++)
					{
						m_offset[i] = m_NegativeZ->Value();
					}
					m_offset_dim->SetValue(m_offset[0]);
					m_PositiveX->SetValue(m_offset[0]);
					m_NegativeX->SetValue(m_offset[0]);
					m_PositiveY->SetValue(m_offset[0]);
					m_NegativeY->SetValue(m_offset[0]);
					m_PositiveZ->SetValue(m_offset[0]);
					
					bodies = m_box->toolingBox(m_objects, pointOrigin, matx, m_offset);
				}

			}
			else
			{
				if (!m_offset_bool->Value())
				{
					m_offsetCyl[2] = m_NegativeZ->Value();
					bodies = m_box->toolingCylinder(m_objects, pointOrigin, vecAxis, m_offsetCyl);
				}
				else
				{
					for (int i = 0; i < 3; i++)
					{
						m_offsetCyl[i] = m_NegativeZ->Value();
					}
					
					m_PositiveZ->SetValue(m_offsetCyl[0]);
					m_Radial->SetValue(m_offsetCyl[0]);
					bodies = m_box->toolingCylinder(m_objects, pointOrigin, vecAxis, m_offsetCyl);
				}

			}

		}
		else if (block == m_PositiveZ)
		{
			//---------Enter your code here-----------
			if (en_type->GetProperties()->GetEnum("Value") == 0)
			{
				if (!m_offset_bool->Value())
				{
					m_offset[5] = m_PositiveZ->Value();
					
					bodies = m_box->toolingBox(m_objects, pointOrigin, matx, m_offset);
				}
				else
				{
					for (int i = 0; i < 6; i++)
					{
						m_offset[i] = m_PositiveZ->Value();
					}
					m_offset_dim->SetValue(m_offset[0]);
					m_PositiveX->SetValue(m_offset[0]);
					m_NegativeX->SetValue(m_offset[0]);
					m_PositiveY->SetValue(m_offset[0]);
					m_NegativeY->SetValue(m_offset[0]);
					m_NegativeZ->SetValue(m_offset[0]);
					
					bodies = m_box->toolingBox(m_objects, pointOrigin, matx, m_offset);
				}


			}
			else
			{
				if (!m_offset_bool->Value())
				{
					m_offsetCyl[1] = m_PositiveZ->Value();
					bodies = m_box->toolingCylinder(m_objects, pointOrigin, vecAxis, m_offsetCyl);
				}
				else
				{
					for (int i = 0; i < 3; i++)
					{
						m_offsetCyl[i] = m_PositiveZ->Value();
					}

					m_NegativeZ->SetValue(m_offsetCyl[0]);
					m_Radial->SetValue(m_offsetCyl[0]);
					bodies = m_box->toolingCylinder(m_objects, pointOrigin, vecAxis, m_offsetCyl);
				}

			}


		}
		else if (block == m_Radial)
		{
			//---------Enter your code here-----------

			if (!m_offset_bool->Value())
			{
				m_offsetCyl[0] = m_Radial->Value();
				bodies = m_box->toolingCylinder(m_objects, pointOrigin, vecAxis, m_offsetCyl);
			}
			else
			{
				for (int i = 0; i < 3; i++)
				{
					m_offsetCyl[i] = m_Radial->Value();
				}

				m_PositiveZ->SetValue(m_offsetCyl[0]);
				m_NegativeZ->SetValue(m_offsetCyl[0]);
				bodies = m_box->toolingCylinder(m_objects, pointOrigin, vecAxis, m_offsetCyl);
			}
		}
		else if (block == m_hile_bool)
		{
			//---------Enter your code here-----------
		}
		else if (block == m_unite_bool)
		{
			//---------Enter your code here-----------

		}
		else if (block == m_plane_bool)
		{
			//---------Enter your code here-----------
		}
		else if (block == m_surface_bool)
		{
			//---------Enter your code here-----------
		}
		else if (block == m_translucency_int)
		{
			//---------Enter your code here-----------
			UF_OBJ_set_translucency(bodies[0]->Tag(), m_translucency_int->Value());


		}
		else if (block == m_layer_int)
		{
			//---------Enter your code here-----------
			UF_OBJ_set_layer(bodies[0]->Tag(), m_layer_int->Value());
			
		}
		else if (block == m_color_int)
		{
			//---------Enter your code here-----------
			m_box->print(std::to_string(m_color_int->GetValue().size()));
			UF_OBJ_set_color(bodies[0]->Tag(), m_color_int->GetValue()[0]);

		}
		else if (block == button0)
		{
			//---------Enter your code here-----------
			
			if (!value)
			{
				NXOpen::BlockStyler::PropertyList* ResultButtonproplist = button0->GetProperties();
				ResultButtonproplist->SetString("Bitmap", "undo");
				ResultButtonproplist->SetString("Label", "Undo Result");
				delete ResultButtonproplist;
			}
			else
			{
				NXOpen::BlockStyler::PropertyList* ResultButtonproplist = button0->GetProperties();
				ResultButtonproplist->SetString("Bitmap", "general_preview");
				ResultButtonproplist->SetString("Label", "Show Result");
				if (m_unite_bool->Value())
				{
					tag_t bodySubTag;
					tag_t bodyTriTag;
					bodySubTag = m_box->subtract(bodies[0], m_objects);
					bodyTriTag = m_box->trimBody(bodySubTag, m_objects);
	
				}
					
				
			}
			
			value = !value;
		}
		else if (block == m_preview_str)
		{
			//---------Enter your code here-----------
		}
	}
	catch (exception& ex)
	{
		//---- Enter your exception handling code here -----
		CreateSuperBox::theUI->NXMessageBox()->Show("Block Styler", NXOpen::NXMessageBox::DialogTypeError, ex.what());
	}
	return 0;
}

//------------------------------------------------------------------------------
//Callback Name: ok_cb
//------------------------------------------------------------------------------
int CreateSuperBox::ok_cb()
{
	int errorCode = 0;
	try
	{
		errorCode = apply_cb();
	}
	catch (exception& ex)
	{
		//---- Enter your exception handling code here -----
		errorCode = 1;
		CreateSuperBox::theUI->NXMessageBox()->Show("Block Styler", NXOpen::NXMessageBox::DialogTypeError, ex.what());
	}
	return errorCode;
}

//------------------------------------------------------------------------------
//Callback Name: filter_cb
//------------------------------------------------------------------------------
int CreateSuperBox::filter_cb(NXOpen::BlockStyler::UIBlock* block, NXOpen::TaggedObject* selectObject)
{
	if (block == m_sele)
	{
		if (bodies.size() > 0)
		{
			if (m_seletype == 0)
			{
				if (dynamic_cast<Face*>(selectObject)->GetBody()->Tag() == bodies[0]->Tag())
				{
					return(UF_UI_SEL_REJECT);
				}
			}

			if (m_seletype == 1)
			{
				if (dynamic_cast<Edge*>(selectObject)->GetBody()->Tag() == bodies[0]->Tag())
				{
					return(UF_UI_SEL_REJECT);
				}
			}
			if (m_seletype == 2)
			{
				int  n_parents = 0;
				tag_t  *parents = NULL_TAG;
				tag_t  bodyTag = NULL_TAG;
				int type;
				int subtype;
				UF_SO_ask_parents(selectObject->Tag(), UF_SO_ASK_ALL_PARENTS, &n_parents, &parents);
				for (int i = 0; i < n_parents; i++)
				{
					UF_OBJ_ask_type_and_subtype(parents[i], &type, &subtype);
					if (type== UF_solid_type)
					{
						UF_MODL_ask_edge_body(parents[i], &bodyTag);
						if (bodyTag == bodies[0]->Tag())
						{
							return(UF_UI_SEL_REJECT);
						}
					}
				}
				
				UF_free(parents);
			}
		}

	}

	return(UF_UI_SEL_ACCEPT);
}

//------------------------------------------------------------------------------
//Callback Name: focusNotify_cb
//This callback is executed when any block (except the ones which receive keyboard entry such as Integer block) receives focus.
//------------------------------------------------------------------------------
void CreateSuperBox::focusNotify_cb(NXOpen::BlockStyler::UIBlock* block, bool focus)
{
	try
	{
		//---- Enter your callback code here -----
	}
	catch (exception& ex)
	{
		//---- Enter your exception handling code here -----
		CreateSuperBox::theUI->NXMessageBox()->Show("Block Styler", NXOpen::NXMessageBox::DialogTypeError, ex.what());
	}
}

//------------------------------------------------------------------------------
//Function Name: GetBlockProperties
//Description: Returns the propertylist of the specified BlockID
//------------------------------------------------------------------------------
PropertyList* CreateSuperBox::GetBlockProperties(const char *blockID)
{
	return theDialog->GetBlockProperties(blockID);
}
