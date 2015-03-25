//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qDummy                      #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#                             COPYRIGHT: XXX                             #
//#                                                                        #
//##########################################################################

#ifndef Q_DATAFRAMEWORK_PLUGIN_HEADER
#define Q_DATAFRAMEWORK_PLUGIN_HEADER

//qCC
#include "../ccStdPluginInterface.h"
#include "data_model.hpp"
#include <ccHObject.h>
#include <ccPointCloud.h>

//Qt
#include <QObject>



#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

typedef pcl::PointXYZ PointT;

//! Dummy qCC plugin
/** Replace the 'qDataFramework' string by your own plugin class name
	and then check 'qDataFramework.cpp' for more directions (you
	have to fill-in the blank methods. The most important one is the
	'getActions' method.  This method should return all actions
	(QAction objects). CloudCompare will automatically add them to an
	icon in the plugin toolbar and to an entry in the plugin menu
	(if your plugin returns several actions, CC will create a dedicated
	toolbar and sub-menu). 
	You are responsible to connect these actions to custom slots of your
	plugin.
	Look at the ccStdPluginInterface::m_app attribute to get access to
	most of CC components (database, 3D views, console, etc.).
**/
class qDataFramework : public QObject, public ccStdPluginInterface
{
	Q_OBJECT
	Q_INTERFACES(ccStdPluginInterface)
#ifdef CC_QT5
	//replace qDummy by the plugin name (IID should be unique - let's hope your plugin name is unique ;)
	Q_PLUGIN_METADATA(IID "imm.cloudcompare.plugin.qDataFramework")
#endif

public:

	//! Default constructor
	qDataFramework(QObject* parent=0);

	//inherited from ccPluginInterface
    virtual QString getName() const { return "qDataFramework "; }
	virtual QString getDescription() const { return "qDataFramework (add description here)"; }
	virtual QIcon getIcon() const;

	//inherited from ccStdPluginInterface
	void onNewSelection(const ccHObject::Container& selectedEntities);
	virtual void getActions(QActionGroup& group);

protected slots:

	/*** ADD YOUR CUSTOM ACTIONS' SLOTS HERE ***/
    void doActionImport();
    void doActionExport();
protected:

	//! Default action
	/** You can add as many actions as you want in a plugin.
		All actions will correspond to an icon in the dedicated
		toolbar and an entry in the plugin menu.
	**/
    QAction* actionImport;
    QAction* actionExportOnlyXML;
    QAction* actionExportDATAXML;


    void fromPCLtoCC (pcl::PointCloud<PointT> pcl_pc, ccPointCloud* cc_pc);
    void fromCCtoPCL (std::string fn, ccPointCloud* cc_pc);
    
    void iterateChildren(ccHObject * object, std::vector<ccPointCloud*> &pointcloudCollection);
    data_model workingModel;
};

#endif
