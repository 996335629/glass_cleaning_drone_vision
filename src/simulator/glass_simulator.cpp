#include "simulator/glass_simulator.h"
#include "simulator/reflection_simulator.h"
#include <random>

class GlassSimulator::Impl {
public:
    SceneConfig config;
    cv::Point2f drone_position;
    cv::Point2f drone_velocity;
    cv::Point2f prev_position;
    cv::Mat background;
    ReflectionSimulator reflection_simulator;
    std::mt19937 rng;

    Impl(const SceneConfig &cfg)
        : config(cfg)
        , drone_position(cfg.frame_size.width / 2.0f, cfg.frame_size.height / 2.0f)
        , drone_velocity(0, 0)
        , prev_position(drone_position)
        , reflection_simulator(ReflectionSimulator::Config{ cfg.glass.reflectivity, cfg.glass.blur_strength })
        , rng(std::random_device{}())
    {
        generateBackground();
    }

    void generateBackground()
    {
        background = cv::Mat(config.frame_size, CV_8UC3);

        // 创建更丰富的纹理背景（模拟建筑物外墙）
        std::uniform_int_distribution<int> color_dist(120, 200);
        std::uniform_int_distribution<int> size_dist(30, 80);

        // 基础颜色
        background.setTo(cv::Scalar(180, 175, 170));

        // 添加随机矩形（模拟窗户、砖块等）
        for (int i = 0; i < 50; i++) {
            int x = rng() % config.frame_size.width;
            int y = rng() % config.frame_size.height;
            int w = size_dist(rng);
            int h = size_dist(rng);
            int c = color_dist(rng);
            cv::rectangle(background,
                cv::Rect(x, y, w, h),
                cv::Scalar(c, c - 10, c - 20), -1);
        }

        // 添加窗户网格
        for (int y = 40; y < background.rows; y += 100) {
            for (int x = 40; x < background.cols; x += 120) {
                // 窗框
                cv::rectangle(background,
                    cv::Rect(x, y, 80, 60),
                    cv::Scalar(80, 80, 90), -1);
                // 窗玻璃
                cv::rectangle(background,
                    cv::Rect(x + 5, y + 5, 70, 50),
                    cv::Scalar(200, 210, 220), -1);
                // 窗户分隔线
                cv::line(background,
                    cv::Point(x + 40, y + 5),
                    cv::Point(x + 40, y + 55),
                    cv::Scalar(80, 80, 90), 2);
                cv::line(background,
                    cv::Point(x + 5, y + 30),
                    cv::Point(x + 75, y + 30),
                    cv::Scalar(80, 80, 90), 2);
            }
        }

        // 添加一些随机纹理噪声
        cv::Mat noise(config.frame_size, CV_8UC3);
        cv::randn(noise, 0, 15);
        cv::add(background, noise, background);

        // 轻微模糊使纹理更自然
        cv::GaussianBlur(background, background, cv::Size(3, 3), 0.5);
    }
};

GlassSimulator::GlassSimulator(const SceneConfig &config)
    : impl_(std::make_unique<Impl>(config))
{
}

GlassSimulator::~GlassSimulator() {}

void GlassSimulator::generateFrame(cv::Mat &left_frame, cv::Mat &right_frame, cv::Mat &depth_map)
{
    impl_->prev_position = impl_->drone_position;

    // 复制背景
    left_frame = impl_->background.clone();

    // 绘制无人机（更详细的形状）
    cv::Point2f pos = impl_->drone_position;
    int cx = static_cast<int>(pos.x);
    int cy = static_cast<int>(pos.y);

    // 无人机机身
    cv::Rect body_rect(cx - 25, cy - 15, 50, 30);
    if (body_rect.x >= 0 && body_rect.y >= 0 &&
        body_rect.br().x < left_frame.cols && body_rect.br().y < left_frame.rows) {

        // 机身主体
        cv::rectangle(left_frame, body_rect, cv::Scalar(40, 40, 40), -1);
        cv::rectangle(left_frame, body_rect, cv::Scalar(20, 20, 20), 2);

        // 四个旋翼
        int arm_len = 35;
        std::vector<cv::Point> rotor_centers = {
            cv::Point(cx - arm_len, cy - arm_len / 2),
            cv::Point(cx + arm_len, cy - arm_len / 2),
            cv::Point(cx - arm_len, cy + arm_len / 2),
            cv::Point(cx + arm_len, cy + arm_len / 2)
        };

        for (const auto &rc : rotor_centers) {
            if (rc.x > 15 && rc.x < left_frame.cols - 15 &&
                rc.y > 15 && rc.y < left_frame.rows - 15) {
                // 机臂
                cv::line(left_frame, cv::Point(cx, cy), rc, cv::Scalar(30, 30, 30), 3);
                // 旋翼
                cv::circle(left_frame, rc, 12, cv::Scalar(60, 60, 60), -1);
                cv::circle(left_frame, rc, 12, cv::Scalar(40, 40, 40), 2);
            }
        }

        // 相机/传感器
        cv::circle(left_frame, cv::Point(cx, cy), 8, cv::Scalar(0, 0, 80), -1);
        cv::circle(left_frame, cv::Point(cx, cy), 5, cv::Scalar(0, 0, 120), -1);
    }

    // 添加玻璃反射效果（模糊的倒影）
    cv::Mat reflection_layer = cv::Mat::zeros(left_frame.size(), CV_8UC3);

    // 创建无人机倒影（位置偏移、模糊）
    cv::Point2f reflection_offset(5, 8);  // 反射偏移
    int rx = static_cast<int>(pos.x + reflection_offset.x);
    int ry = static_cast<int>(pos.y + reflection_offset.y);

    cv::Rect ref_rect(rx - 30, ry - 18, 60, 36);
    if (ref_rect.x >= 0 && ref_rect.y >= 0 &&
        ref_rect.br().x < reflection_layer.cols && ref_rect.br().y < reflection_layer.rows) {
        cv::rectangle(reflection_layer, ref_rect, cv::Scalar(30, 30, 35), -1);
    }

    // 模糊反射层
    cv::GaussianBlur(reflection_layer, reflection_layer,
        cv::Size(15, 15), impl_->config.glass.blur_strength);

    // 叠加反射
    cv::addWeighted(left_frame, 1.0, reflection_layer,
        impl_->config.glass.reflectivity * 0.5, 0, left_frame);

    // 添加传感器噪声
    if (impl_->config.add_noise) {
        cv::Mat noise(left_frame.size(), CV_8UC3);
        cv::randn(noise, 0, 3);
        cv::add(left_frame, noise, left_frame);
    }

    // 生成右目图像（添加视差）
    right_frame = left_frame.clone();
    float disparity = 8.0f;  // 模拟双目视差
    cv::Mat shift_mat = (cv::Mat_<float>(2, 3) << 1, 0, disparity, 0, 1, 0);
    cv::warpAffine(left_frame, right_frame, shift_mat, right_frame.size(),
        cv::INTER_LINEAR, cv::BORDER_REPLICATE);

    // 生成深度图
    depth_map = cv::Mat(impl_->config.frame_size, CV_16U, cv::Scalar(1000));

    // 无人机区域深度更近
    cv::Rect drone_area(cx - 40, cy - 30, 80, 60);
    if (drone_area.x >= 0 && drone_area.y >= 0 &&
        drone_area.br().x < depth_map.cols && drone_area.br().y < depth_map.rows) {
        depth_map(drone_area).setTo(cv::Scalar(500));  // 更近的深度
    }
}

void GlassSimulator::updateDroneMotion(const cv::Point2f &velocity, float angular_velocity)
{
    (void)angular_velocity;
    impl_->drone_velocity = velocity;
    impl_->drone_position += velocity;

    // 边界限制
    float margin = 60.0f;
    impl_->drone_position.x = std::max(margin, std::min(impl_->drone_position.x,
        impl_->config.frame_size.width - margin));
    impl_->drone_position.y = std::max(margin, std::min(impl_->drone_position.y,
        impl_->config.frame_size.height - margin));
}

void GlassSimulator::addRandomDisturbance()
{
    std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
    impl_->drone_velocity.x += dist(impl_->rng) * 0.5f;
    impl_->drone_velocity.y += dist(impl_->rng) * 0.5f;
}

cv::Point2f GlassSimulator::getGroundTruthDisplacement() const
{
    return impl_->drone_position - impl_->prev_position;
}

void GlassSimulator::reset()
{
    impl_->drone_position = cv::Point2f(impl_->config.frame_size.width / 2.0f,
        impl_->config.frame_size.height / 2.0f);
    impl_->drone_velocity = cv::Point2f(0, 0);
    impl_->prev_position = impl_->drone_position;
    impl_->generateBackground();
}
