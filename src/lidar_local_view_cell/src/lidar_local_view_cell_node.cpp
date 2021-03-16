//
// Created by Zhuang on 19.09.20.
//

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <lidar_local_view_cell/LiDARViewTemplate.h>

int HASH_MASK = 10;
double MATCH_ERROR_THRESHOLD = 0.100;
double ANGLE_TOLERANCE = 5;
int IGNORE_RESENT_LV = 20;
int PROCESS_SCAN_STEP = 1;
int TEMPLATE_SIZE = 128;

template<int Radius>
static inline float Lanczos(float x) {
    if (x == 0.0) return 1.0;
    if (x <= -Radius || x >= Radius) return 0.0;
    float pi_x = x * M_PI;
    return Radius * sin(pi_x) * sin(pi_x / Radius) / (pi_x * pi_x);
}

const int FilterRadius = 3;

static inline void Resample
        (const std::vector<float>& source,
         std::vector<float>& result, size_t dest_len) {
    size_t src_len = source.size();

    const float blur = 1.0;
    const float factor = dest_len / (float) src_len;

    float scale = std::min(factor, 1.0f) / blur;
    float support = FilterRadius / scale;

    std::vector<float> contribution(std::min(src_len, 5 + size_t(2 * support)));
    /* 5 = room for rounding up in calculations of start, stop and support */

    if (support <= 0.5f) {
        support = 0.5f + 1E-12;
        scale = 1.0f;
    }

    for (size_t x = 0; x < dest_len; ++x) {
        float center = (x + 0.5f) / factor;
        size_t start = (size_t) std::max(center - support + 0.5f, (float) 0);
        size_t stop = (size_t) std::min(center + support + 0.5f, (float) src_len);
        float density = 0.0f;
        size_t nmax = stop - start;
        float s = start - center + 0.5f;
        result[x] = 0;
        for (size_t n = 0; n < nmax; ++n, ++s) {
            contribution[n] = Lanczos<FilterRadius>(s * scale);
            density += contribution[n];
            result[x] += source[start + n] * contribution[n];
        }
        if (density != 0.0 && density != 1.0) {
            /* Normalize. */
            result[x] /= density;
        }
    }
}


struct lidar_view_t {
    int id;
    int hash;
    std::vector<float> ranges;
};

ros::Publisher view_template_pub;
std::vector<lidar_view_t> views;

void onLaserScanCallback(const sensor_msgs::LaserScan::Ptr &laserScanPtr) {
    static int counter = 0;
    counter = (counter + 1) % PROCESS_SCAN_STEP;
    if (counter != 0) {
        return;
    }

    float rangeMin = laserScanPtr->range_min;
    float rangeMax = laserScanPtr->range_max;
    float angleRange = (laserScanPtr->angle_max - laserScanPtr->angle_min) / M_PI * 180.0;

    double range_total = 0;
    int range_size = laserScanPtr->ranges.size();
    for (float range : laserScanPtr->ranges) {
        if (rangeMin <= range && range <= rangeMax) {
            range_total += range;
        }
    }
    int hash = ((int) range_total) / HASH_MASK;
    // ROS_INFO("on pointcloud size %lu, min %f, max %f, range count %d, hash %d", laserScanPtr->ranges.size(), rangeMin,
    // rangeMax, valid_range_count, hash);

    double min_error = 9999;
    lidar_view_t *min_error_view = nullptr;
    std::vector<float> down_sampled_ranges(TEMPLATE_SIZE);
    Resample(laserScanPtr->ranges, down_sampled_ranges, TEMPLATE_SIZE);
    for (int v_i = 0; v_i < (int) views.size() - IGNORE_RESENT_LV; v_i++) {
        auto &view = views[v_i];
        if (hash == view.hash) {
            // ROS_INFO("hash matched, id %d", view.id);
            for (int offset = -(ANGLE_TOLERANCE / angleRange * TEMPLATE_SIZE);
                 offset < ANGLE_TOLERANCE / 360.0 * TEMPLATE_SIZE; offset += (TEMPLATE_SIZE / angleRange)) {
                double error = 0;
                int error_count = 0;
                for (int i = 0; i < view.ranges.size(); i++) {
                    if (i + offset < 0 || i + offset >= TEMPLATE_SIZE) {
                        continue;
                    }
                    if (rangeMin <= view.ranges[i] && view.ranges[i] <= rangeMax &&
                        rangeMin <= down_sampled_ranges[i + offset] && down_sampled_ranges[i + offset] <= rangeMax) {
                        error += std::abs(view.ranges[i] - down_sampled_ranges[i + offset]);
                        error_count++;
                    }
                }
                error /= error_count;
                if (error < min_error) {
                    min_error = error;
                    min_error_view = &view;
                }
            }
        }
    }

    lidar_view_t *matched_view = nullptr;
    if (min_error_view != nullptr) {
        // ROS_INFO("min error: %f, hash %d", min_error, hash);
    }
    if (min_error < MATCH_ERROR_THRESHOLD && min_error_view != nullptr) {
        matched_view = min_error_view;
        if (min_error_view->id < (int) views.size() - IGNORE_RESENT_LV * 1.3) {
            ROS_WARN_THROTTLE(3, "matched view found: id %d / %d, error %f", min_error_view->id, (int) views.size(),
                              min_error);
        }
    } else {
        lidar_view_t new_view;
        new_view.id = views.size();
        new_view.ranges = down_sampled_ranges;
        new_view.hash = hash;
        views.push_back(new_view);
        matched_view = &views.back();
        // ROS_INFO("new view: id %d, hash %d", new_view.id, hash);
    }

    static lidar_local_view_cell::LiDARViewTemplate viewTemplate;
    viewTemplate.header.stamp = ros::Time::now();
    viewTemplate.header.seq++;
    viewTemplate.view_id = matched_view->id;
    // ROS_INFO("send vt seq %d, cur_id %d", viewTemplate.header.seq, viewTemplate.current_id);
    view_template_pub.publish(viewTemplate);
}

int main(int argc, char *argv[]) {
    ROS_INFO("LiDAR Local View Cells");
    ros::init(argc, argv, "lidar_local_view_cells");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Subscriber subscriber = nh.subscribe("/scan", 1, onLaserScanCallback);
    view_template_pub = nh.advertise<lidar_local_view_cell::LiDARViewTemplate>("/lidar_local_view", 10);

    HASH_MASK = private_nh.param<int>("hash_mask", 10);
    MATCH_ERROR_THRESHOLD = private_nh.param<double>("match_threshold", 0.100);
    ANGLE_TOLERANCE = private_nh.param<double>("angle_tolerance", 5);
    IGNORE_RESENT_LV = private_nh.param<int>("ignore_resent_lv_num", 20);
    PROCESS_SCAN_STEP = private_nh.param<int>("process_step", 1);
    TEMPLATE_SIZE = private_nh.param<int>("template_size", 360);

    ros::spin();

    return 0;
}
