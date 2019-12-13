#include "CommonSlam.h"

std::string getTimeTag()
{
    std::ostringstream oss;
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    oss << std::put_time(&tm, "%d-%m-%Y %H-%M-%S");
    return oss.str();
}

void commonSlam(
    OkvisSLAMSystem &slam,
    ImuCamera &camera,
    const cv::FileStorage &configFile,
    bool isReplay
) {
        //setup display
    if (!MyGUI::Manager::init())
    {
        fprintf(stdout, "Failed to initialize GLFW\n");
        exit(-1);
    }
    //Window for displaying the path
    MyGUI::CameraWindow traj_win("Traj Viewer", 640 * 2, 480 * 2);
    MyGUI::ARCameraWindow ar_win("AR Viewer", 640 * 2.5, 480 * 2.5, GL_LUMINANCE, GL_UNSIGNED_BYTE, 6.16403320e+02, 6.16171021e+02, 3.18104584e+02, 2.33643127e+02, 0.01, 100);
    traj_win.set_pos(640 * 2.5, 100);
    ar_win.set_pos(0, 100);
    MyGUI::Path path1("path1", Eigen::Vector3d(1, 0, 0));
    MyGUI::Axis axis1("axis1", .1);
    MyGUI::Axis axis2("axis2", 1);
    MyGUI::Grid grid1("grid1", 10, 1);
    traj_win.add_object(&path1);
    traj_win.add_object(&axis1);
    traj_win.add_object(&axis2);
    traj_win.add_object(&grid1);
    ar_win.add_object(&axis1);
    std::vector<MyGUI::Object *> cubes;
    std::vector<Eigen::Matrix4d> T_K_cubes;
    std::vector<MapKeyFrame::Ptr> K_cubes;

    //Recieves output from SLAM system and displays to the screen
    FrameAvailableHandler handler([&path1, &axis2, &ar_win, &cubes, &T_K_cubes, &K_cubes](MultiCameraFrame::Ptr frame) {
        Eigen::Affine3d transform(frame->T_WC(3));
        path1.add_node(transform.translation());
        axis2.set_transform(transform);
        ar_win.set_camera(transform);
        ar_win.set_image(frame->images_[3]);
        if (ar_win.clicked())
        {

            std::string cube_name = std::string("CubeNum") + std::to_string(cubes.size());
            MyGUI::Object *obj = new MyGUI::Cube(cube_name, 0.1, 0.1, 0.1);
            obj->set_transform(transform);
            cubes.push_back(obj);
            T_K_cubes.push_back(frame->T_KS_);
            K_cubes.push_back(frame->keyframe_);
            std::cout << "Adding cube " << cube_name << std::endl;
            ar_win.add_object(obj); //NOTE: this is bad, should change objects to shared_ptr
        }
    });
    slam.AddFrameAvailableHandler(handler, "mapping");

    KeyFrameAvailableHandler kfHandler([](MultiCameraFrame::Ptr frame) {
        frame->saveSimple("map_images/");
    });
    //slam.AddKeyFrameAvailableHandler(kfHandler, "saving");

    LoopClosureDetectedHandler loopHandler([&slam, &path1, &cubes, &T_K_cubes, &K_cubes](void) {
        std::vector<Eigen::Matrix4d> traj;
        slam.getTrajectory(traj);
        path1.clear();
        for (size_t i = 0; i < traj.size(); i++)
        {
            path1.add_node(traj[i].block<3, 1>(0, 3));
        }
        for (size_t i = 0; i < cubes.size(); i++)
        {
            if (K_cubes[i] != nullptr)
                cubes[i]->set_transform(Eigen::Affine3d(K_cubes[i]->T_WS() * T_K_cubes[i]));
        }
    });
    slam.AddLoopClosureDetectedHandler(loopHandler, "trajectoryUpdate");
    //run until display is closed
    okvis::Time start(0.0);
    camera.start();

    while (MyGUI::Manager::running())
    {
        //Update the display
        MyGUI::Manager::update();

        try
        {
            //Get current camera frame
            MultiCameraFrame::Ptr frame(new MultiCameraFrame);
            camera.update(*frame);

            const auto frameId = frame->frameId_;
            if (frameId < 0) {
                std::cout << "Data end reached\n";
                break;
            }
            const auto &infrared = frame->images_[0];
            const auto &infrared2 = frame->images_[1];
            const auto &rgb = frame->images_[3];


            //std::cout << "RGB: " << rgb.total() << "\n";
            //cv::imshow(std::string(camera.getModelName()) + " RGB", rgb);
            // tmp fix for the preview window in MyGUI
            //cv::cvtColor(rgb, rgb, CV_RGB2BGR); 
            cv::imshow(std::string(camera.getModelName()) + " Infrared", infrared);
            //Get or wait for IMU Data until current frame
            //std::cout << "frame size: " << rgb.total() << std::endl;
            std::vector<ImuPair> imuData;
            camera.getImuToTime(frame->timestamp_, imuData);
            std::cout << "numimu: " << imuData.size() << std::endl;
            std::cout << "timestamp: " << std::setprecision(15) << frame->timestamp_ << std::endl;

            //Add data to SLAM system
            slam.PushIMU(imuData);
            // make it the same as real camera
            frame->images_.resize(4);
            slam.PushFrame(frame);
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n'; // or whatever
        }
        catch (...)
        {
            std::cout << "ex catched\n";
        }
        const auto isReset = slam.okvis_estimator_->isReset();
        std::cout << "slam reset? : " << isReset << "\n";
        if (isReset) {
            traj_win.msg_ = " *Reseting*";
            path1.clear();
        } else {
            traj_win.msg_ = " ";
        }
        int k = cv::waitKey(1);
        if (k == 'q' || k == 'Q' || k == 27)
            break; // 27 is ESC
    }
    const auto &nodes = path1.nodes;
    if (!nodes.empty()) {
        if (((int)configFile["saveTraj"]) == 1) {
            // const path trajBin = std::string("./traj_") + getTimeTag() + ".bin";
            // std::ofstream intrin_ofs(trajBin.string());
            // boost::archive::text_oarchive oa(intrin_ofs);
            // oa << path1;
        }
        if (((int)configFile["printLoopError"]) == 1) {
            const auto &first = nodes.front();
            const auto &last = nodes.back();
            const auto &diff = last - first;
            std::cout << "Distance from first to end: " << sqrt(diff.dot(diff)) << "\n";
            cv::waitKey(0);
        }
    }

    printf("\nTerminate...\n");
    // Clean up
    slam.ShutDown();
}