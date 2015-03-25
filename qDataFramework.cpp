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

//First: replace all occurences of 'qDataFramework' by your own plugin class name in this file!
#include "qDataFramework.h"

//Qt
#include <QtGui>
#include <iostream>
#include <FileIOFilter.h>
#include <Eigen/Eigen>

//boost
#include <boost/filesystem.hpp>
#include <qt4/QtGui/qfiledialog.h>

#include <pcl/common/transforms.h>
//Default constructor: should mainly be used to initialize
//actions (pointers) and other members
qDataFramework::qDataFramework(QObject* parent/*=0*/)
	: QObject(parent)
{

    actionImport = new QAction("Import XML", this);
    actionExportOnlyXML= new QAction("Export XML", this);
    actionExportDATAXML= new QAction("Export XML DATA", this);

    connect(actionImport, SIGNAL(triggered()), this, SLOT(doActionImport()));
    connect(actionExportDATAXML, SIGNAL(triggered()), this, SLOT(doActionExport()));

}

//This method should enable or disable each plugin action
//depending on the currently selected entities ('selectedEntities').
//For example: if none of the selected entities is a cloud, and your
//plugin deals only with clouds, call 'm_action->setEnabled(false)'
void qDataFramework::onNewSelection(const ccHObject::Container& selectedEntities)
{
	//if (m_action)
	//	m_action->setEnabled(!selectedEntities.empty());
}

//This method returns all 'actions' of your plugin.
//It will be called only once, when plugin is loaded.
void qDataFramework::getActions(QActionGroup& group)
{
	//default action (if it has not been already created, it's the moment to do it)


    group.addAction(actionImport);
    group.addAction(actionExportDATAXML);

}

//This is an example of an action's slot called when the corresponding action
//is triggered (i.e. the corresponding icon or menu entry is clicked in CC
//main's interface). You can access to most of CC components (database,
//3D views, console, etc.) via the 'm_app' attribute (ccMainAppInterface
//object).

void qDataFramework::doActionExport()
{
    assert(m_app);
    if (!m_app)
        return;

    //QString datasetName = "/home/michal/tests/modelICP.xml";
    QString datasetName =QFileDialog::getSaveFileName(0,tr("Save Metamodel"), "~", tr("XML (*.xml)"));
    boost::filesystem::path filePath (datasetName.toStdString());
    boost::filesystem::path rootPath = filePath.remove_filename();
    boost::filesystem::path folderPath = filePath/("data_cc");
    
    boost::filesystem::create_directories(folderPath);
    m_app->dispToConsole("[qDataFramework] created folder :"+QString::fromStdString(folderPath.c_str()),ccMainAppInterface::STD_CONSOLE_MESSAGE);
    
    data_model savingModel;
    std::vector<ccPointCloud*> pointcloudCollection;
    iterateChildren(m_app->dbRootObject(), pointcloudCollection);
    m_app->dispToConsole("[qDataFramework] number exported objects :"+QString::number(pointcloudCollection.size()),ccMainAppInterface::STD_CONSOLE_MESSAGE);
    
    for (int i=0; i < pointcloudCollection.size(); i++)
    {
        ccGLMatrix m_ccGLMat = pointcloudCollection[i]->getGLTransformationHistory();
        Eigen::Matrix4f transform(m_ccGLMat.data()) ;
        std::string id = pointcloudCollection[i]->getName().toStdString();
        std::string fn = id+".pcd";
        savingModel.setAffine(id, transform);
        savingModel.setPointcloudName(id, fn);
        
        boost::filesystem::path fullPath = folderPath/fn;
        fromCCtoPCL(fullPath.c_str(), pointcloudCollection[i]);
    }
    savingModel.setDataSetPath("data_cc");
    savingModel.saveFile(datasetName.toStdString());
    
}


void qDataFramework::doActionImport()
{
	//(--> pure internal check)
	assert(m_app);
	if (!m_app)
		return;

    QString datasetName =QFileDialog::getOpenFileName(0,tr("Open Metamodel"), "~", tr("XML (*.xml)"));
   
    //QString datasetName = "/home/michal/ug_parking_robotic/modelICP.xml";
    m_app->dispToConsole("[qDataFramework] will load XML MetaModel "+datasetName,ccMainAppInterface::STD_CONSOLE_MESSAGE);
    workingModel.loadFile(datasetName.toStdString());
    std::vector<std::string> ids;
    workingModel.getAllScansId(ids);
    for (int i=0;i < ids.size(); i++)
    {
        std::string fn;
        fn = workingModel.getFullPathOfPointcloud(ids[i]);
        Eigen::Affine3f mat;
        workingModel.getAffine(ids[i], mat.matrix());
        ccPointCloud *pc = new ccPointCloud(ids[i].c_str());
        pc->setGLTransformationHistory(mat.data());
        
        
        pcl::PointCloud<PointT>pcl_pc;
        pcl::io::loadPCDFile(fn, pcl_pc);
        m_app->dispToConsole("[qDataFramework] will load PCD "+QString::fromStdString(fn),ccMainAppInterface::STD_CONSOLE_MESSAGE);
        //transform pointcloud 
        pcl::transformPointCloud(pcl_pc,pcl_pc, mat);
        fromPCLtoCC(pcl_pc, pc);
        m_app->addToDB(pc);
    }



	/*** HERE ENDS THE ACTION ***/

}

void qDataFramework::iterateChildren(ccHObject * object, std::vector<ccPointCloud *> &pointcloudCollection)
{
    // we are intrested only in pointclouds
    ccPointCloud *pc = dynamic_cast<ccPointCloud*>(object);
    if (pc != NULL)
    {
        std::cout << "OBJECT :" << object->getName().toStdString()<<"\t"<< object->getUniqueID()<<"\n";
        pointcloudCollection.push_back(pc);
    }

    for (int i=0; i< object->getChildrenNumber(); i++)
    {
        iterateChildren(object->getChild(i), pointcloudCollection);
    }
}

void qDataFramework::fromPCLtoCC (pcl::PointCloud<PointT> pcl_pc, ccPointCloud* cc_pc)
{

    if (cc_pc == NULL) return;
    unsigned int pcSize = pcl_pc.size();
    cc_pc->reserve(pcSize);

    for (int i=0; i< pcSize; i++)
    {
        PointT ptn = pcl_pc[i];
        cc_pc->addPoint(CCVector3(ptn.x, ptn.y, ptn.z));
    }
}
void qDataFramework::fromCCtoPCL (std::string fn, ccPointCloud* cc_pc)
{
    ccGLMatrix m_ccGLMat = cc_pc->getGLTransformationHistory();
    Eigen::Matrix4f transform(m_ccGLMat.data()) ;
    Eigen::Affine3f inv = Eigen::Affine3f(transform);
    inv = inv.inverse();
    pcl::PointCloud<PointT> pcl_pc;
    for (int i=0;i < cc_pc->size(); i++)
    {
        PointT ptn;
        CCVector3 cc_ptn;
        cc_pc->getPoint(i, cc_ptn); 
        ptn.x = cc_ptn.x;
        ptn.y = cc_ptn.y;
        ptn.z = cc_ptn.z;
        pcl_pc.push_back(ptn);
    }
    pcl::transformPointCloud(pcl_pc, pcl_pc, inv);
    pcl::io::savePCDFile(fn,pcl_pc);
}
    

//This method should return the plugin icon (it will be used mainly
//if your plugin as several actions in which case CC will create a
//dedicated sub-menu entry with this icon.
QIcon qDataFramework::getIcon() const
{
    //open qDataFramework.qrc (text file), update the "prefix" and the
	//icon(s) filename(s). Then save it with the right name (yourPlugin.qrc).
    //(eventually, remove the original qDataFramework.qrc file!)
    return QIcon(":/CC/plugin/qDataFramework/icon.png");
}

#ifndef CC_QT5
//Don't forget to replace 'qDataFramework' by your own plugin class name here also!
Q_EXPORT_PLUGIN2(qDataFramework,qDataFramework);
#endif
