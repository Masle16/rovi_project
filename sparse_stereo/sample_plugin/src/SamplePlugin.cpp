
#include "SamplePlugin.hpp"
#include "utils.hpp"

SamplePlugin::SamplePlugin():RobWorkStudioPlugin("SamplePluginUI", QIcon(":/pa_icon.png")) {
    setupUi(this);

    _timer = new QTimer(this);
    connect(_timer, SIGNAL(timeout()), this, SLOT(timer()));

    // now connect stuff from the ui component
    connect(_btn_move,       SIGNAL(pressed()),         this, SLOT(btnPressed()) );
    connect(_btn_im,         SIGNAL(pressed()),         this, SLOT(btnPressed()) );
    connect(_btn_scan,       SIGNAL(pressed()),         this, SLOT(btnPressed()) );
    connect(_btn0,           SIGNAL(pressed()),         this, SLOT(btnPressed()) );
    connect(_btn1,           SIGNAL(pressed()),         this, SLOT(btnPressed()) );
    connect(_spinBox,        SIGNAL(valueChanged(int)), this, SLOT(btnPressed()) );

    connect(_btn_pose,       SIGNAL(pressed()),         this, SLOT(btnPressed()) );

    _framegrabber = NULL;

    _cameras = {"Camera_Right", "Camera_Left"};
    _cameras25D = {"Scanner25D"};
}

SamplePlugin::~SamplePlugin() {
    delete _textureRender;
    delete _bgRender;
}

void SamplePlugin::initialize() {
    log().info() << "INITALIZE" << "\n";

    getRobWorkStudio()->stateChangedEvent().add(std::bind(&SamplePlugin::stateChangedListener, this, std::placeholders::_1), this);

    // Auto load workcell
    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(WC_FILE);
    getRobWorkStudio()->setWorkCell(wc);

}

void SamplePlugin::open(rw::models::WorkCell* workcell)
{
    log().info() << "OPEN" << "\n";
    _wc = workcell;
    _state = _wc->getDefaultState();

    log().info() << workcell->getFilename() << "\n";

    if (_wc != NULL) {
    // Add the texture render to this workcell if there is a frame for texture
    rw::kinematics::Frame* textureFrame = _wc->findFrame("MarkerTexture");
    if (textureFrame != NULL) {
        getRobWorkStudio()->getWorkCellScene()->addRender("TextureImage",_textureRender,textureFrame);
    }
    // Add the background render to this workcell if there is a frame for texture
    rw::kinematics::Frame* bgFrame = _wc->findFrame("Background");
    if (bgFrame != NULL) {
        getRobWorkStudio()->getWorkCellScene()->addRender("BackgroundImage",_bgRender,bgFrame);
    }

    // Create a GLFrameGrabber if there is a camera frame with a Camera property set
    rw::kinematics::Frame* cameraFrame = _wc->findFrame(_cameras[0]);
    if (cameraFrame != NULL) {
        if (cameraFrame->getPropertyMap().has("Camera")) {
            // Read the dimensions and field of view
            double fovy;
            int width, height;
            std::string camParam = cameraFrame->getPropertyMap().get<std::string>("Camera");
            std::istringstream iss (camParam, std::istringstream::in);
            iss >> fovy >> width >> height;
            // Create a frame grabber
            _framegrabber = new rwlibs::simulation::GLFrameGrabber(width,height,fovy);
            rw::graphics::SceneViewer::Ptr gldrawer = getRobWorkStudio()->getView()->getSceneViewer();
            _framegrabber->init(gldrawer);
        }
    }

    rw::kinematics::Frame* cameraFrame25D = _wc->findFrame(_cameras25D[0]);
    if (cameraFrame25D != NULL) {
        if (cameraFrame25D->getPropertyMap().has("Scanner25D")) {
            // Read the dimensions and field of view
            double fovy;
            int width,height;
            std::string camParam = cameraFrame25D->getPropertyMap().get<std::string>("Scanner25D");
            std::istringstream iss (camParam, std::istringstream::in);
            iss >> fovy >> width >> height;
            // Create a frame grabber
            _framegrabber25D = new rwlibs::simulation::GLFrameGrabber25D(width,height,fovy);
            rw::graphics::SceneViewer::Ptr gldrawer = getRobWorkStudio()->getView()->getSceneViewer();
            _framegrabber25D->init(gldrawer);
        }
    }

    _device = _wc->findDevice("UR-6-85-5-A");
    _step = -1;

    }
}


void SamplePlugin::close() {
    log().info() << "CLOSE" << "\n";

    // Stop the timer
    _timer->stop();
    // Remove the texture render
    rw::kinematics::Frame* textureFrame = _wc->findFrame("MarkerTexture");
    if (textureFrame != NULL) {
        getRobWorkStudio()->getWorkCellScene()->removeDrawable("TextureImage",textureFrame);
    }
    // Remove the background render
    rw::kinematics::Frame* bgFrame = _wc->findFrame("Background");
    if (bgFrame != NULL) {
        getRobWorkStudio()->getWorkCellScene()->removeDrawable("BackgroundImage",bgFrame);
    }
    // Delete the old framegrabber
    if (_framegrabber != NULL) {
        delete _framegrabber;
    }
    _framegrabber = NULL;
    _wc = NULL;
}

cv::Mat SamplePlugin::toOpenCVImage(const rw::sensor::Image& img) {
    cv::Mat res(img.getHeight(),img.getWidth(), CV_8SC3);
    res.data = (uchar*)img.getImageData();
    return res;
}


void SamplePlugin::btnPressed() {
    QObject *obj = sender();
    if ( obj==_btn0 ){
//		log().info() << "Button 0\n";
//		// Toggle the timer on and off
//		if (!_timer25D->isActive())
//		    _timer25D->start(100); // run 10 Hz
//		else
//			_timer25D->stop();
        _timer->stop();
        rw::math::Math::seed();
        double extend = 0.05;
        double maxTime = 60;
        rw::math::Q from(6, 1.571, -1.572, -1.572, -1.572, 1.571, 0);
        rw::math::Q to(6, 1.847, -2.465, -1.602, -0.647, 1.571, 0); //From pose estimation
        createPathRRTConnect(from, to, extend, maxTime);
    }
    else if(obj==_btn1) {
        log().info() << "Button 1\n";
        // Toggle the timer on and off
        if (!_timer->isActive()){
            _timer->start(100); // run 10 Hz
            _step = 0;
        }
        else
            _step = 0;
    }
    else if(obj==_spinBox) {
        log().info() << "spin value:" << _spinBox->value() << "\n";
    }
    else if( obj==_btn_im ) {
        getImage();
    }
    else if (obj == _btn_scan) {
        get25DImage();
    }
    else if (obj == _btn_move) {
        moveFrameRandom(_wc, _state, "Duck");
        getRobWorkStudio()->setState(_state);
    }
    else if (obj == _btn_pose) {
        cv::Mat proj_l, proj_r, cam_mat_l, cam_mat_r;
        getCamerasInfo(proj_l, proj_r, cam_mat_l, cam_mat_r);

        // get initial points
        std::vector<cv::Mat> src_pnts_3d;
        getInitial3dPnts(proj_l, proj_r, src_pnts_3d);

        // get initial pose
        Pose src_pose;
        getInitialPose(src_pose);

        getImage();

        // get new points
        std::vector<cv::Mat> new_pnts_3d;
        getNew3dPnts(proj_l, proj_r, new_pnts_3d);

        // calc transformation
        Pose estimatedPose = calcTransformationP2P(src_pnts_3d, new_pnts_3d, src_pose);

        // calc error
        calcError(_wc, _state, estimatedPose);
    }
}

void SamplePlugin::get25DImage() {
    if (_framegrabber25D != NULL) {
        // Create pcd file
        for (size_t i = 0; i < _cameras25D.size(); i++) {
            // Get the image as a RW image
            rw::kinematics::Frame* cameraFrame25D = _wc->findFrame(_cameras25D[i]); // "Camera");
            _framegrabber25D->grab(cameraFrame25D, _state);

            //const Image& image = _framegrabber->getImage();

            const rw::geometry::PointCloud* img = &(_framegrabber25D->getImage());

            std::ofstream output(_cameras25D[i] + ".pcd");
            output << "# .PCD v.5 - Point Cloud Data file format\n";
            output << "FIELDS x y z\n";
            output << "SIZE 4 4 4\n";
            output << "TYPE F F F\n";
            output << "WIDTH " << img->getWidth() << "\n";
            output << "HEIGHT " << img->getHeight() << "\n";
            output << "POINTS " << img->getData().size() << "\n";
            output << "DATA ascii\n";
            for(const auto &p_tmp : img->getData()) {
                rw::math::Vector3D<float> p = p_tmp;
                output << p(0) << " " << p(1) << " " << p(2) << "\n";
            }
            output.close();
        }
    }
}

void SamplePlugin::getImage() {
    if (_framegrabber != NULL) {
        for (size_t i = 0; i < _cameras.size(); i++) {
            // Get the image as a RW image
            rw::kinematics::Frame* cameraFrame = _wc->findFrame(_cameras[i]); // "Camera");
            _framegrabber->grab(cameraFrame, _state);

            const rw::sensor::Image* rw_image = &(_framegrabber->getImage());

            // Convert to OpenCV matrix.
            cv::Mat image = cv::Mat(rw_image->getHeight(), rw_image->getWidth(), CV_8UC3, (rw::sensor::Image*)rw_image->getImageData());

            // Convert to OpenCV image
            cv::Mat imflip, imflip_mat;
            cv::flip(image, imflip, 1);
            cv::cvtColor( imflip, imflip_mat, cv::COLOR_RGB2BGR );

            cv::imwrite(_cameras[i] + ".png", imflip_mat );

            // Show in QLabel
            QImage img(imflip.data, imflip.cols, imflip.rows, imflip.step, QImage::Format_RGB888);
            QPixmap p = QPixmap::fromImage(img);
            unsigned int maxW = 480;
            unsigned int maxH = 640;
            _label->setPixmap(p.scaled(maxW,maxH,Qt::KeepAspectRatio));
        }
    }
}

void SamplePlugin::timer() {
    if(0 <= _step && _step < _path.size()){
        _device->setQ(_path.at(_step),_state);
        getRobWorkStudio()->setState(_state);
        _step++;
    }
}

void SamplePlugin::stateChangedListener(const rw::kinematics::State& state) {
  _state = state;
}

bool SamplePlugin::checkCollisions(rw::models::Device::Ptr device,
                                   const rw::kinematics::State &state,
                                   const rw::proximity::CollisionDetector &detector,
                                   const rw::math::Q &q) {
    rw::kinematics::State testState;
    rw::proximity::CollisionDetector::QueryResult data;
    bool colFrom;

    testState = state;
    device->setQ(q,testState);
    colFrom = detector.inCollision(testState,&data);
    if (colFrom) {
        std::cerr << "Configuration in collision: " << q << std::endl;
        std::cerr << "Colliding frames: " << std::endl;
        rw::kinematics::FramePairSet fps = data.collidingFrames;
        for (rw::kinematics::FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) {
            std::cerr << (*it).first->getName() << " " << (*it).second->getName() << std::endl;
        }
        return false;
    }
    return true;
}

void SamplePlugin::createPathRRTConnect(rw::math::Q from, rw::math::Q to,  double extend, double maxTime){
    _device->setQ(from,_state);
    getRobWorkStudio()->setState(_state);
    rw::proximity::CollisionDetector detector(_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());
    rw::pathplanning::PlannerConstraint constraint = rw::pathplanning::PlannerConstraint::make(&detector,_device,_state);
    rw::pathplanning::QSampler::Ptr sampler = rw::pathplanning::QSampler::makeConstrained(rw::pathplanning::QSampler::makeUniform(_device),constraint.getQConstraintPtr());
    rw::math::QMetric::Ptr metric = rw::math::MetricFactory::makeEuclidean<rw::math::Q>();
    rw::pathplanning::QToQPlanner::Ptr planner = rwlibs::pathplanners::RRTPlanner::makeQToQPlanner(constraint, sampler, metric, extend, rwlibs::pathplanners::RRTPlanner::RRTConnect);

    _path.clear();
    if (!checkCollisions(_device, _state, detector, from))
        std::cout << from << " is in colission!" << std::endl;
    if (!checkCollisions(_device, _state, detector, to))
        std::cout << to << " is in colission!" << std::endl;;
    rw::common::Timer t;
    t.resetAndResume();
    planner->query(from, to, _path, maxTime);
    t.pause();


    if (t.getTime() >= maxTime) {
        std::cout << "Notice: max time of " << maxTime << " seconds reached." << std::endl;
    }

    const int duration = 10;

    if(_path.size() == 2){  //The interpolated path between Q start and Q goal is collision free. Set the duration with respect to the desired velocity
        rw::trajectory::LinearInterpolator<rw::math::Q> linInt(from, to, duration);
        rw::trajectory::QPath tempQ;
        for(int i = 0; i < duration+1; i++){
            tempQ.push_back(linInt.x(i));
        }
        _path = tempQ;
    }
}
