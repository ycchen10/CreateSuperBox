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
//              Date: 10-18-2018  (Format: mm-dd-yyyy)
//              Time: 12:10 (Format: hh-mm)
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
    catch(exception& ex)
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
    catch(exception& ex)
    {
        //---- Enter your exception handling code here -----
        CreateSuperBox::theUI->NXMessageBox()->Show("Block Styler", NXOpen::NXMessageBox::DialogTypeError, ex.what());
    }
    if(theCreateSuperBox != NULL)
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
    catch(exception& ex)
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
    catch(exception& ex)
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
        group1 = dynamic_cast<NXOpen::BlockStyler::Group*>(theDialog->TopBlock()->FindBlock("group1"));
        m_offset_bool = dynamic_cast<NXOpen::BlockStyler::Toggle*>(theDialog->TopBlock()->FindBlock("m_offset_bool"));
        m_offset_dim = dynamic_cast<NXOpen::BlockStyler::LinearDimension*>(theDialog->TopBlock()->FindBlock("m_offset_dim"));
        m_PositiveX = dynamic_cast<NXOpen::BlockStyler::LinearDimension*>(theDialog->TopBlock()->FindBlock("m_PositiveX"));
        m_NegativeX = dynamic_cast<NXOpen::BlockStyler::LinearDimension*>(theDialog->TopBlock()->FindBlock("m_NegativeX"));
        m_PositiveY = dynamic_cast<NXOpen::BlockStyler::LinearDimension*>(theDialog->TopBlock()->FindBlock("m_PositiveY"));
        m_NegativeY = dynamic_cast<NXOpen::BlockStyler::LinearDimension*>(theDialog->TopBlock()->FindBlock("m_NegativeY"));
        m_PositiveZ = dynamic_cast<NXOpen::BlockStyler::LinearDimension*>(theDialog->TopBlock()->FindBlock("m_PositiveZ"));
        m_NegativeZ = dynamic_cast<NXOpen::BlockStyler::LinearDimension*>(theDialog->TopBlock()->FindBlock("m_NegativeZ"));
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

		m_objects.clear();

		m_box = new CcreateBox();

		m_seletype = m_seleType_enum->GetProperties()->GetEnum("Value");

		m_offset_dim->SetValue(1.0);


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

		m_box->getoffset(m_offset_dim->Value());

		m_box->getoffsetbool(m_offset_bool->Value());

		m_box->gettranslucency(m_translucency_int->Value());

		m_box->getlayer(m_layer_int->Value());

		m_box->getcolor(m_color_int->GetValue());


		m_offset_dim->GetProperties()->SetLogical("ShowHandle", true);
		m_offset_dim->GetProperties()->SetLogical("ShowFocusHandle", true);


    }
    catch(exception& ex)
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
    catch(exception& ex)
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
    catch(exception& ex)
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
        if(block == en_type)
        {
        //---------Enter your code here-----------
        }
        else if(block == m_seleType_enum)
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
        else if(block == m_sele)
        {
        //---------Enter your code here-----------
			double *box;
			Point3d pointorigin;
			m_objects = m_sele->GetSelectedObjects();
			m_box->getObjects(m_objects);
			box = m_box->getBounding_box();
	        if (en_type->GetProperties()->GetEnum("Value") ==0)
	        {
				NXOpen::Vector3d x(1, 0, 0);
				NXOpen::Vector3d y(0, 1, 0);
				Point3d	pointorigin1(100, 100, 100);
				pointorigin.X = (box[0] + box[3]) / 2;
				pointorigin.Y = (box[1] + box[4]) / 2;
				pointorigin.Z = (box[2] + box[5]) / 2;
				m_manip->SetOrigin(pointorigin);
				m_manip->SetWCSCoordinates(true);
				m_manip->GetProperties()->SetLogical("Enable", true); //设置可以动态拖拉
				m_offset_dim->SetHandleOrigin(pointorigin);
				m_offset_dim->SetHandleOrientation(x);
				m_offset_dim->SetHandleOrigin(pointorigin1);
				m_offset_dim->SetHandleOrientation(y);
				m_box->getmanip(m_manip->Origin(), m_manip->XAxis(), m_manip->YAxis());
				bodies = m_box->toolingBox();
	        }
			
        }
        else if(block == m_manip)
        {
        //---------Enter your code here-----------
			
			m_box->getmanip(m_manip->Origin(), m_manip->XAxis(), m_manip->YAxis());
			bodies = m_box->toolingBox();
		
        }
        else if(block == m_offset_bool)
        {
        //---------Enter your code here-----------
			m_box->getoffsetbool(m_offset_bool->Value());
        }
        else if(block == m_offset_dim)
        {
        //---------Enter your code here-----------
			
			m_box->getoffset(m_offset_dim->Value());
			bodies = m_box->toolingBox();
        }
        else if(block == m_PositiveX)
        {
        //---------Enter your code here-----------


        }
        else if(block == m_NegativeX)
        {
        //---------Enter your code here-----------
        }
        else if(block == m_PositiveY)
        {
        //---------Enter your code here-----------
        }
        else if(block == m_NegativeY)
        {
        //---------Enter your code here-----------
        }
        else if(block == m_PositiveZ)
        {
        //---------Enter your code here-----------
        }
        else if(block == m_NegativeZ)
        {
        //---------Enter your code here-----------
        }
        else if(block == m_Radial)
        {
        //---------Enter your code here-----------
        }
        else if(block == m_hile_bool)
        {
        //---------Enter your code here-----------
        }
        else if(block == m_unite_bool)
        {
        //---------Enter your code here-----------
        }
        else if(block == m_plane_bool)
        {
        //---------Enter your code here-----------
        }
        else if(block == m_surface_bool)
        {
        //---------Enter your code here-----------
        }
        else if(block == m_translucency_int)
        {
        //---------Enter your code here-----------
        }
        else if(block == m_layer_int)
        {
        //---------Enter your code here-----------
        }
        else if(block == m_color_int)
        {
        //---------Enter your code here-----------
        }
        else if(block == button0)
        {
        //---------Enter your code here-----------
        }
        else if(block == m_preview_str)
        {
        //---------Enter your code here-----------
        }
    }
    catch(exception& ex)
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
    catch(exception& ex)
    {
        //---- Enter your exception handling code here -----
        errorCode = 1;
        CreateSuperBox::theUI->NXMessageBox()->Show("Block Styler", NXOpen::NXMessageBox::DialogTypeError, ex.what());
    }
    return errorCode;
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
    catch(exception& ex)
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
int CreateSuperBox::filter_cb(NXOpen::BlockStyler::UIBlock*  block, TaggedObject * m_objects)
{
	
	if (block == m_sele)
	{
		if (bodies.size()>0)
		{
			if (m_seletype==0)
			{
				if (dynamic_cast<Face*>(m_objects)->GetBody()->Tag() == bodies[0]->Tag())
				{
					return(UF_UI_SEL_REJECT);
				}
			}
			
			if (m_seletype == 1)
			{
				if (dynamic_cast<Edge*>(m_objects)->GetBody()->Tag() == bodies[0]->Tag())
				{
                   return(UF_UI_SEL_REJECT);
				}
			}
			
		}
		
		
		
	}
	
	return(UF_UI_SEL_ACCEPT);
}
