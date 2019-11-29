#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

// Qt
#include <QTimer>

// RobWork includes
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rwlibs/simulation/GLFrameGrabber25D.hpp>

#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>

// RobWorkStudio includes
#include <RobWorkStudioConfig.hpp> // For RWS_USE_QT5 definition
#include <rws/RobWorkStudioPlugin.hpp>

// OpenCV 3
#include <opencv2/opencv.hpp>


#include "ui_SamplePlugin.h"

#include <rws/RobWorkStudio.hpp>

#include <QPushButton>

#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellFactory.hpp>

#include <functional>

class SamplePlugin: public rws::RobWorkStudioPlugin, private Ui::SamplePlugin {
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "plugin.json")
public:
    SamplePlugin();
    virtual ~SamplePlugin();
    virtual void open(rw::models::WorkCell* workcell);
    virtual void close();
    virtual void initialize();

private slots:
    void btnPressed();
    void timer();
    void getImage();
    void get25DImage();
    void stateChangedListener(const rw::kinematics::State& state);
    bool checkCollisions(rw::models::Device::Ptr device,
                         const rw::kinematics::State &state,
                         const rw::proximity::CollisionDetector &detector,
                         const rw::math::Q &q);
    void createPathRRTConnect(rw::math::Q from, rw::math::Q to,  double extend, double maxTime);

private:
    static cv::Mat toOpenCVImage(const rw::sensor::Image& img);
    QTimer* _timer;
    QTimer* _timer25D;
    rw::models::WorkCell::Ptr _wc;
    rw::kinematics::State _state;
    rwlibs::opengl::RenderImage *_textureRender, *_bgRender;
    rwlibs::simulation::GLFrameGrabber* _framegrabber;
    rwlibs::simulation::GLFrameGrabber25D* _framegrabber25D;    
    std::vector<std::string> _cameras;
    std::vector<std::string> _cameras25D;
    rw::models::Device::Ptr _device;
    rw::trajectory::QPath _path;
    unsigned int _step;

};

#endif /*RINGONHOOKPLUGIN_HPP_*/
