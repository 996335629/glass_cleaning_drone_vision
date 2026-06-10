#include "camera/gemini_camera.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>

#ifdef HAS_ORBBEC_SDK
#include <libobsensor/ObSensor.hpp>
#endif

class GeminiCamera::Impl {
public:
    Config config;
    bool running = false;
    std::string last_error;
    CameraInfo camera_info;

#ifdef HAS_ORBBEC_SDK
    std::shared_ptr<ob::Pipeline> pipeline;
    std::shared_ptr<ob::Config> ob_config;
    std::shared_ptr<ob::Device> device;
    std::shared_ptr<ob::Sensor> accel_sensor;
    std::shared_ptr<ob::Sensor> gyro_sensor;
#endif

    // IMU数据缓存（由回调线程写，主线程读）
    mutable std::mutex imu_mutex;
    IMUData latest_imu;

    Impl()
    {
        camera_info.name = "Unknown";
        camera_info.serial_number = "";
        camera_info.pid = 0;
        camera_info.vid = 0;
    }

    IMUData getLatestIMU() const {
        std::lock_guard<std::mutex> lk(imu_mutex);
        return latest_imu;
    }
};

GeminiCamera::GeminiCamera() : impl_(std::make_unique<Impl>()) {}

GeminiCamera::~GeminiCamera()
{
    stop();
}

bool GeminiCamera::initialize()
{
    return initialize(Config());
}

bool GeminiCamera::initialize(const Config &config)
{
    impl_->config = config;

#ifdef HAS_ORBBEC_SDK
    try {
        // 创建Pipeline
        impl_->pipeline = std::make_shared<ob::Pipeline>();

        // 获取并保存设备（IMU需要直接访问device sensor）
        impl_->device = impl_->pipeline->getDevice();
        if (impl_->device) {
            auto info = impl_->device->getDeviceInfo();
            impl_->camera_info.name = info->name();
            impl_->camera_info.serial_number = info->serialNumber();
            impl_->camera_info.pid = info->pid();
            impl_->camera_info.vid = info->vid();

            std::cout << "[GeminiCamera] Device: " << impl_->camera_info.name << std::endl;
            std::cout << "[GeminiCamera] Serial: " << impl_->camera_info.serial_number << std::endl;
        }

        // 创建配置
        impl_->ob_config = std::make_shared<ob::Config>();

        // 配置彩色流
        if (config.enable_color) {
            try {
                auto profiles = impl_->pipeline->getStreamProfileList(OB_SENSOR_COLOR);
                if (profiles && profiles->count() > 0) {
                    std::shared_ptr<ob::VideoStreamProfile> selected = nullptr;

                    // 查找匹配分辨率
                    for (uint32_t i = 0; i < profiles->count(); i++) {
                        auto p = profiles->getProfile(i)->as<ob::VideoStreamProfile>();
                        if (p->width() == config.color_width &&
                            p->height() == config.color_height) {
                            selected = p;
                            break;
                        }
                    }

                    if (!selected) {
                        selected = profiles->getProfile(0)->as<ob::VideoStreamProfile>();
                    }

                    std::cout << "[GeminiCamera] Color: " << selected->width() << "x"
                        << selected->height() << " @ " << selected->fps() << " fps" << std::endl;
                    impl_->ob_config->enableStream(selected);
                }
            }
            catch (ob::Error &e) {
                std::cerr << "[GeminiCamera] Color stream error: " << e.getMessage() << std::endl;
            }
        }

        // 配置深度流
        if (config.enable_depth) {
            try {
                auto profiles = impl_->pipeline->getStreamProfileList(OB_SENSOR_DEPTH);
                if (profiles && profiles->count() > 0) {
                    std::shared_ptr<ob::VideoStreamProfile> selected = nullptr;

                    for (uint32_t i = 0; i < profiles->count(); i++) {
                        auto p = profiles->getProfile(i)->as<ob::VideoStreamProfile>();
                        if (p->width() == config.depth_width &&
                            p->height() == config.depth_height) {
                            selected = p;
                            break;
                        }
                    }

                    if (!selected) {
                        selected = profiles->getProfile(0)->as<ob::VideoStreamProfile>();
                    }

                    std::cout << "[GeminiCamera] Depth: " << selected->width() << "x"
                        << selected->height() << " @ " << selected->fps() << " fps" << std::endl;
                    impl_->ob_config->enableStream(selected);
                }
            }
            catch (ob::Error &e) {
                std::cerr << "[GeminiCamera] Depth stream error: " << e.getMessage() << std::endl;
            }
        }

        // 深度对齐到彩色
        if (config.align_depth_to_color) {
            impl_->ob_config->setAlignMode(ALIGN_D2C_HW_MODE);
        }

        // 预获取IMU sensor（验证设备支持）
        if (config.enable_imu && impl_->device) {
            try {
                impl_->accel_sensor = impl_->device->getSensorList()->getSensor(OB_SENSOR_ACCEL);
                impl_->gyro_sensor  = impl_->device->getSensorList()->getSensor(OB_SENSOR_GYRO);
                if (impl_->accel_sensor && impl_->gyro_sensor) {
                    std::cout << "[GeminiCamera] IMU sensors found (accel + gyro)" << std::endl;
                } else {
                    std::cout << "[GeminiCamera] IMU sensors not available on this device" << std::endl;
                }
            }
            catch (ob::Error &e) {
                std::cerr << "[GeminiCamera] IMU not supported: " << e.getMessage() << std::endl;
                impl_->accel_sensor = nullptr;
                impl_->gyro_sensor  = nullptr;
            }
        }

        std::cout << "[GeminiCamera] Initialized successfully" << std::endl;
        return true;

    }
    catch (ob::Error &e) {
        impl_->last_error = std::string("Init error: ") + e.getMessage();
        std::cerr << "[GeminiCamera] " << impl_->last_error << std::endl;
        return false;
    }
#else
    std::cout << "[GeminiCamera] Simulation mode (no SDK)" << std::endl;
    impl_->camera_info.name = "Simulated Gemini336";
    impl_->camera_info.serial_number = "SIM001";
    return true;
#endif
}

bool GeminiCamera::start()
{
#ifdef HAS_ORBBEC_SDK
    try {
        if (impl_->pipeline && impl_->ob_config) {
            impl_->pipeline->start(impl_->ob_config);
            impl_->running = true;

            // 启动加速度计回调流
            if (impl_->accel_sensor) {
                auto profiles = impl_->accel_sensor->getStreamProfileList();
                auto profile  = profiles->getProfile(OB_PROFILE_DEFAULT);
                impl_->accel_sensor->start(profile, [this](std::shared_ptr<ob::Frame> frame) {
                    auto accel_frame = frame->as<ob::AccelFrame>();
                    if (!accel_frame) return;
                    auto val = accel_frame->value();
                    std::lock_guard<std::mutex> lk(impl_->imu_mutex);
                    impl_->latest_imu.accel_x    = val.x;
                    impl_->latest_imu.accel_y    = val.y;
                    impl_->latest_imu.accel_z    = val.z;
                    impl_->latest_imu.temperature = accel_frame->temperature();
                    impl_->latest_imu.timestamp   = accel_frame->timeStamp();
                    impl_->latest_imu.valid       = true;
                });
                std::cout << "[GeminiCamera] Accel stream started" << std::endl;
            }

            // 启动陀螺仪回调流
            if (impl_->gyro_sensor) {
                auto profiles = impl_->gyro_sensor->getStreamProfileList();
                auto profile  = profiles->getProfile(OB_PROFILE_DEFAULT);
                impl_->gyro_sensor->start(profile, [this](std::shared_ptr<ob::Frame> frame) {
                    auto gyro_frame = frame->as<ob::GyroFrame>();
                    if (!gyro_frame) return;
                    auto val = gyro_frame->value();
                    std::lock_guard<std::mutex> lk(impl_->imu_mutex);
                    impl_->latest_imu.gyro_x = val.x;
                    impl_->latest_imu.gyro_y = val.y;
                    impl_->latest_imu.gyro_z = val.z;
                });
                std::cout << "[GeminiCamera] Gyro stream started" << std::endl;
            }

            std::cout << "[GeminiCamera] Started" << std::endl;
            return true;
        }
        impl_->last_error = "Pipeline not initialized";
        return false;
    }
    catch (ob::Error &e) {
        impl_->last_error = std::string("Start error: ") + e.getMessage();
        std::cerr << "[GeminiCamera] " << impl_->last_error << std::endl;
        return false;
    }
#else
    impl_->running = true;
    std::cout << "[GeminiCamera] Started (simulation)" << std::endl;
    return true;
#endif
}

void GeminiCamera::stop()
{
#ifdef HAS_ORBBEC_SDK
    if (impl_->running) {
        try {
            if (impl_->accel_sensor) impl_->accel_sensor->stop();
        } catch (...) {}
        try {
            if (impl_->gyro_sensor)  impl_->gyro_sensor->stop();
        } catch (...) {}
        try {
            if (impl_->pipeline) impl_->pipeline->stop();
        } catch (...) {}
    }
#endif
    impl_->running = false;
    std::cout << "[GeminiCamera] Stopped" << std::endl;
}

GeminiCamera::Frame GeminiCamera::getFrame(int timeout_ms)
{
    Frame frame;
    frame.valid = false;
    frame.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

#ifdef HAS_ORBBEC_SDK
    if (!impl_->running || !impl_->pipeline) {
        impl_->last_error = "Camera not running";
        return frame;
    }

    try {
        auto frameset = impl_->pipeline->waitForFrames(timeout_ms);
        if (!frameset) {
            impl_->last_error = "Timeout waiting for frames";
            return frame;
        }

        // 彩色帧
        auto color_frame = frameset->colorFrame();
        if (color_frame) {
            int w = color_frame->width();
            int h = color_frame->height();
            auto fmt = color_frame->format();

            if (fmt == OB_FORMAT_RGB888) {
                cv::Mat rgb(h, w, CV_8UC3, color_frame->data());
                cv::cvtColor(rgb, frame.color, cv::COLOR_RGB2BGR);
            }
            else if (fmt == OB_FORMAT_YUYV || fmt == OB_FORMAT_YUY2) {
                cv::Mat yuyv(h, w, CV_8UC2, color_frame->data());
                cv::cvtColor(yuyv, frame.color, cv::COLOR_YUV2BGR_YUYV);
            }
            else if (fmt == OB_FORMAT_MJPG) {
                std::vector<uint8_t> data((uint8_t *)color_frame->data(),
                    (uint8_t *)color_frame->data() + color_frame->dataSize());
                frame.color = cv::imdecode(data, cv::IMREAD_COLOR);
            }
            else if (fmt == OB_FORMAT_BGR) {
                frame.color = cv::Mat(h, w, CV_8UC3, color_frame->data()).clone();
            }
            else {
                // 尝试作为RGB处理
                cv::Mat rgb(h, w, CV_8UC3, color_frame->data());
                cv::cvtColor(rgb, frame.color, cv::COLOR_RGB2BGR);
            }

            frame.timestamp = color_frame->timeStamp();
        }

        // 深度帧
        auto depth_frame = frameset->depthFrame();
        if (depth_frame) {
            int w = depth_frame->width();
            int h = depth_frame->height();
            frame.depth = cv::Mat(h, w, CV_16UC1, depth_frame->data()).clone();
        }

        frame.valid = !frame.color.empty();
        frame.imu   = impl_->getLatestIMU();

    }
    catch (ob::Error &e) {
        impl_->last_error = std::string("Frame error: ") + e.getMessage();
    }
#else
    // 模拟模式：生成测试图像
    int w = impl_->config.color_width;
    int h = impl_->config.color_height;

    frame.color = cv::Mat(h, w, CV_8UC3, cv::Scalar(80, 80, 80));

    // 绘制网格模拟建筑
    for (int y = 0; y < h; y += 60) {
        for (int x = 0; x < w; x += 60) {
            cv::rectangle(frame.color, cv::Rect(x + 5, y + 5, 50, 50),
                cv::Scalar(150, 150, 150), -1);
            cv::rectangle(frame.color, cv::Rect(x + 10, y + 10, 15, 20),
                cv::Scalar(100, 80, 60), -1);  // 窗户
            cv::rectangle(frame.color, cv::Rect(x + 30, y + 10, 15, 20),
                cv::Scalar(100, 80, 60), -1);
        }
    }

    cv::putText(frame.color, "SIMULATION MODE", cv::Point(w / 2 - 100, 30),
        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);

    frame.depth = cv::Mat(impl_->config.depth_height, impl_->config.depth_width,
        CV_16UC1, cv::Scalar(800));
    frame.valid = true;

    std::this_thread::sleep_for(std::chrono::milliseconds(33));
#endif

    return frame;
}

bool GeminiCamera::isRunning() const
{
    return impl_->running;
}

GeminiCamera::CameraInfo GeminiCamera::getCameraInfo() const
{
    return impl_->camera_info;
}

std::vector<GeminiCamera::CameraInfo> GeminiCamera::listDevices()
{
    std::vector<CameraInfo> devices;

#ifdef HAS_ORBBEC_SDK
    try {
        ob::Context ctx;
        auto list = ctx.queryDeviceList();

        for (uint32_t i = 0; i < list->deviceCount(); i++) {
            auto dev = list->getDevice(i);
            auto info = dev->getDeviceInfo();

            CameraInfo ci;
            ci.name = info->name();
            ci.serial_number = info->serialNumber();
            ci.pid = info->pid();
            ci.vid = info->vid();
            devices.push_back(ci);
        }
    }
    catch (ob::Error &e) {
        std::cerr << "[GeminiCamera] Error listing devices: " << e.getMessage() << std::endl;
    }
#else
    // 模拟模式返回虚拟设备
    CameraInfo sim;
    sim.name = "Simulated Gemini336";
    sim.serial_number = "SIM001";
    sim.pid = 0;
    sim.vid = 0;
    devices.push_back(sim);
#endif

    return devices;
}

std::string GeminiCamera::getLastError() const
{
    return impl_->last_error;
}
