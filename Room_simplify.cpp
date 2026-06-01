#include <windows.h>

#include <cmath>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#ifdef _DEBUG
#pragma comment(lib, "opencv_world4110d.lib")
#else
#pragma comment(lib, "opencv_world4110.lib")
#endif

namespace {

constexpr char kLidarPacketHeader = 0x54;
constexpr std::size_t kLidarPacketSize = 47;
constexpr DWORD kLidarBaudRate = 230400;
constexpr DWORD kReadIntervalTimeoutMs = 10;
constexpr DWORD kReadTimeoutConstantMs = 10;
constexpr DWORD kReadTimeoutMultiplierMs = 1;
constexpr float kFullRotationDegrees = 360.0f;
constexpr float kAngleStepDegrees = 0.8f;
constexpr double kPi = 3.14159265358979323846;
constexpr int kScanSleepMs = 10;
constexpr int kPostPlotSleepMs = 1000;
constexpr int kImageWidth = 720;
constexpr int kImageHeight = 720;
constexpr int kPlotRangeMin = -3000;
constexpr int kPlotRangeMax = 3000;
constexpr int kCannyThreshold1 = 50;
constexpr int kCannyThreshold2 = 200;
constexpr int kCannyApertureSize = 3;
constexpr int kInitialHoughThreshold = 70;
constexpr int kMinHoughThreshold = -100;
constexpr int kMaxHoughThreshold = 100;
constexpr int kHoughThresholdStep = 10;
constexpr std::size_t kMinDetectedLines = 2;
constexpr std::size_t kMaxDetectedLines = 10;
constexpr int kHoughMinLineLength = 160;
constexpr int kHoughMaxLineGap = 200;
constexpr int kColorConversionMap[] = {0, 2, 0, 1, 0, 0};
constexpr const char* kDefaultComPort = "\\\\.\\COM7";
constexpr const char* kResultDirectory = "%PATH%\\result";

/**
 * @brief Convert an angle in degrees to radians.
 * @param angle_degrees Angle in degrees.
 * @return Angle in radians.
 */
double degrees_to_radians(const float angle_degrees) {
    return static_cast<double>(angle_degrees) * kPi / 180.0;
}

/**
 * @brief Generate a timestamp string in YYYYMMDDHHMM format.
 * @return Current local timestamp string.
 */
std::string make_timestamp() {
    const std::time_t current_time = std::time(nullptr);
    std::tm local_time{};
    localtime_s(&local_time, &current_time);

    std::ostringstream stream;
    stream << std::put_time(&local_time, "%Y%m%d%H%M");
    return stream.str();
}

struct ScanPoint {
    float angle_degrees;
    uint16_t distance_mm;
    uint8_t intensity;
};

/**
 * @brief Manage the LD19 serial port with RAII.
 */
class LidarDevice {
public:
    LidarDevice() = default;

    ~LidarDevice() {
        close();
    }

    /**
     * @brief Open the LiDAR serial port and configure communication settings.
     * @param port_name COM port name such as "\\\\.\\COM7".
     * @return True when the port is ready to use.
     */
    bool open(const std::string& port_name) {
        port_ = CreateFileA(port_name.c_str(), GENERIC_READ, 0, nullptr, OPEN_EXISTING, 0, nullptr);
        if (port_ == INVALID_HANDLE_VALUE) {
            return false;
        }

        DCB dcb{};
        dcb.DCBlength = sizeof(dcb);
        if (GetCommState(port_, &dcb) == 0) {
            close();
            return false;
        }

        dcb.BaudRate = kLidarBaudRate;
        dcb.ByteSize = 8;
        dcb.Parity = NOPARITY;
        dcb.StopBits = ONESTOPBIT;
        if (SetCommState(port_, &dcb) == 0) {
            close();
            return false;
        }

        COMMTIMEOUTS timeouts{};
        timeouts.ReadIntervalTimeout = kReadIntervalTimeoutMs;
        timeouts.ReadTotalTimeoutConstant = kReadTimeoutConstantMs;
        timeouts.ReadTotalTimeoutMultiplier = kReadTimeoutMultiplierMs;
        if (SetCommTimeouts(port_, &timeouts) == 0) {
            close();
            return false;
        }

        return true;
    }

    /**
     * @brief Read one synchronized LiDAR packet.
     * @param packet Output packet buffer.
     * @return True when one full packet was read successfully.
     */
    bool read_packet(std::vector<uint8_t>& packet) const {
        if (port_ == INVALID_HANDLE_VALUE) {
            return false;
        }

        uint8_t header_byte = 0;
        DWORD bytes_read = 0;

        while (true) {
            if (ReadFile(port_, &header_byte, 1, &bytes_read, nullptr) == 0 || bytes_read != 1) {
                return false;
            }

            if (header_byte == kLidarPacketHeader) {
                break;
            }
        }

        packet.assign(kLidarPacketSize, 0);
        packet[0] = kLidarPacketHeader;

        if (ReadFile(port_, &packet[1], static_cast<DWORD>(kLidarPacketSize - 1), &bytes_read, nullptr) == 0) {
            return false;
        }

        return bytes_read == kLidarPacketSize - 1;
    }

private:
    void close() {
        if (port_ == INVALID_HANDLE_VALUE) {
            return;
        }

        CloseHandle(port_);
        port_ = INVALID_HANDLE_VALUE;
    }

    HANDLE port_ = INVALID_HANDLE_VALUE;
};

/**
 * @brief Determine whether one full rotation has been scanned.
 * @param start_angle Current packet start angle.
 * @param last_angle Previous packet start angle.
 * @param angle_accumulator Accumulated rotation angle.
 * @param first_packet True only for the first packet.
 * @return True when a full rotation has been completed.
 */
bool has_completed_full_scan(
    const float start_angle,
    float& last_angle,
    float& angle_accumulator,
    bool& first_packet) {
    if (first_packet) {
        first_packet = false;
        last_angle = start_angle;
        return false;
    }

    float angle_delta = start_angle - last_angle;
    if (angle_delta < 0.0f) {
        angle_delta += kFullRotationDegrees;
    }

    angle_accumulator += angle_delta;
    last_angle = start_angle;
    return angle_accumulator >= kFullRotationDegrees;
}

/**
 * @brief Append scan points from one packet.
 * @param packet Raw LiDAR packet.
 * @param scan_points Output scan point list.
 */
void append_scan_points(const std::vector<uint8_t>& packet, std::vector<ScanPoint>& scan_points) {
    const uint8_t count = packet[1] & 0x1F;
    const float start_angle = static_cast<float>(packet[4] | (packet[5] << 8)) / 100.0f;

    for (int index = 0; index < count; ++index) {
        const int packet_index = 6 + index * 3;
        const uint16_t distance = packet[packet_index] | (packet[packet_index + 1] << 8);
        const uint8_t intensity = packet[8 + 3 * index];
        const float angle = start_angle + static_cast<float>(index) * kAngleStepDegrees;
        scan_points.push_back({angle, distance, intensity});
    }
}

/**
 * @brief Save scan points as Cartesian coordinates to a CSV file.
 * @param output_path CSV file path.
 * @param scan_points Scan results in polar coordinates.
 * @return True when the file was written successfully.
 */
bool save_scan_points_to_csv(const std::string& output_path, const std::vector<ScanPoint>& scan_points) {
    std::ofstream output_stream(output_path);
    if (!output_stream.is_open()) {
        return false;
    }

    for (const auto& point : scan_points) {
        const double angle_radians = degrees_to_radians(point.angle_degrees);
        const double x_value = static_cast<double>(point.distance_mm) * std::cos(angle_radians);
        const double y_value = static_cast<double>(point.distance_mm) * std::sin(angle_radians);
        output_stream << x_value << ',' << y_value << '\n';
    }

    return true;
}

/**
 * @brief Collect one full LiDAR scan and save it as CSV.
 * @param output_path Output CSV path.
 * @return 0 on success, 1 on failure.
 */
int collect_scan_data(const std::string& output_path) {
    LidarDevice lidar_device;
    if (!lidar_device.open(kDefaultComPort)) {
        std::cerr << "Can't open port\n";
        return 1;
    }

    std::cout << "Scanning..." << std::endl;

    std::vector<ScanPoint> scan_points;
    std::vector<uint8_t> packet;
    float last_angle = -1.0f;
    float angle_accumulator = 0.0f;
    bool first_packet = true;

    while (true) {
        if (!lidar_device.read_packet(packet)) {
            std::cerr << "[read failed]" << std::endl;
            continue;
        }

        if (packet.size() != kLidarPacketSize) {
            std::cerr << "[invalid packet size]" << std::endl;
            continue;
        }

        if (packet[0] != kLidarPacketHeader) {
            std::cerr << "[header mismatch]" << std::endl;
            continue;
        }

        const float start_angle = static_cast<float>(packet[4] | (packet[5] << 8)) / 100.0f;
        if (has_completed_full_scan(start_angle, last_angle, angle_accumulator, first_packet)) {
            std::cout << "Scan complete (" << scan_points.size() << " points)\n";
            break;
        }

        append_scan_points(packet, scan_points);
        Sleep(kScanSleepMs);
    }

    if (!save_scan_points_to_csv(output_path, scan_points)) {
        std::cerr << "Failed to save CSV file\n";
        return 1;
    }

    std::cout << "Save complete\n";
    return 0;
}

/**
 * @brief Plot a CSV file with gnuplot and save it as PNG.
 * @param timestamp Timestamp used in the output file name.
 * @return 0 on success, 1 on failure.
 */
int export_plot_with_gnuplot(const std::string& timestamp) {
    const std::string csv_filename = "result_" + timestamp + ".csv";
    const std::string png_filename = "result_" + timestamp + ".png";

    FILE* gnuplot_pipe = _popen("gnuplot -persist", "w");
    if (gnuplot_pipe == nullptr) {
        std::cerr << "Failed to start gnuplot" << std::endl;
        return 1;
    }

    std::fprintf(gnuplot_pipe, "set datafile separator ','\n");
    std::fprintf(gnuplot_pipe, "cd '%s'\n", kResultDirectory);
    std::fprintf(gnuplot_pipe, "set term png size %d,%d\n", kImageWidth, kImageHeight);
    std::fprintf(gnuplot_pipe, "set border 1248\n");
    std::fprintf(gnuplot_pipe, "unset tics\n");
    std::fprintf(gnuplot_pipe, "unset key\n");
    std::fprintf(gnuplot_pipe, "set output '%s'\n", png_filename.c_str());
    std::fprintf(
        gnuplot_pipe,
        "plot [%d:%d][%d:%d] '%s' pt 7 ps 0.5 lc 'black'\n",
        kPlotRangeMin,
        kPlotRangeMax,
        kPlotRangeMin,
        kPlotRangeMax,
        csv_filename.c_str());
    std::fprintf(gnuplot_pipe, "set output\n");
    std::fflush(gnuplot_pipe);
    _pclose(gnuplot_pipe);

    std::cout << "PNG data has been saved" << std::endl;
    return 0;
}

/**
 * @brief Tune the Hough threshold to obtain a moderate number of detected lines.
 * @param edge_image Edge image for Hough transform.
 * @param lines Output detected line segments.
 * @return True when suitable lines were detected.
 */
bool detect_reasonable_lines(const cv::Mat& edge_image, std::vector<cv::Vec4i>& lines) {
    int threshold = kInitialHoughThreshold;

    while (threshold >= kMinHoughThreshold && threshold <= kMaxHoughThreshold) {
        lines.clear();
        cv::HoughLinesP(
            edge_image,
            lines,
            1,
            CV_PI / 180.0,
            threshold,
            kHoughMinLineLength,
            kHoughMaxLineGap);

        if (lines.size() > kMaxDetectedLines) {
            std::cout << "Too many lines detected. Retrying..." << std::endl;
            threshold += kHoughThresholdStep;
            continue;
        }

        if (lines.size() < kMinDetectedLines) {
            std::cout << "Too few lines detected. Retrying..." << std::endl;
            threshold -= kHoughThresholdStep;
            continue;
        }

        std::cout << "Line detection complete" << std::endl;
        return true;
    }

    return false;
}

/**
 * @brief Detect lines from the generated PNG image and save the result.
 * @param input_image_path Input PNG path.
 * @return 0 on success, 1 on failure.
 */
int detect_lines_with_hough(const std::string& input_image_path) {
    std::cout << "Hough line detection" << std::endl;

    try {
        const cv::Mat source = cv::imread(input_image_path, cv::IMREAD_GRAYSCALE);
        if (source.empty()) {
            std::cerr << "Failed to read image: " << input_image_path << std::endl;
            return 1;
        }

        cv::Mat edge_image;
        cv::Canny(source, edge_image, kCannyThreshold1, kCannyThreshold2, kCannyApertureSize);

        cv::Mat destination = cv::Mat::zeros(source.rows, source.cols, CV_8UC3);
        cv::mixChannels(&source, 1, &destination, 1, kColorConversionMap, 3);

        std::vector<cv::Vec4i> lines;
        if (!detect_reasonable_lines(edge_image, lines)) {
            std::cerr << "Failed to find a suitable Hough threshold" << std::endl;
            return 1;
        }

        for (const auto& line : lines) {
            cv::line(
                destination,
                cv::Point(line[0], line[1]),
                cv::Point(line[2], line[3]),
                cv::Scalar(0, 0, 255),
                2);
        }

        cv::namedWindow("dst", 1);
        cv::imshow("dst", destination);
        cv::imwrite(std::string(kResultDirectory) + "\\Hough_result.png", destination);
        cv::waitKey(0);
        cv::destroyAllWindows();
        return 0;
    } catch (const cv::Exception& exception) {
        std::cerr << exception.err << std::endl;
    }

    cv::destroyAllWindows();
    return 1;
}

}  // namespace

int main() {
    const std::string timestamp = make_timestamp();
    std::cout << "Now: " << timestamp << std::endl;
    std::cout << "CSV Filename: result_" << timestamp << std::endl;

    const std::string csv_path = std::string(kResultDirectory) + "\\result_" + timestamp + ".csv";
    const std::string png_path = std::string(kResultDirectory) + "\\result_" + timestamp + ".png";

    std::cout << "CSV Path: " << csv_path << std::endl;

    if (collect_scan_data(csv_path) != 0) {
        return 1;
    }

    if (export_plot_with_gnuplot(timestamp) != 0) {
        return 1;
    }

    std::cout << "gnuplot export ended!" << std::endl;
    Sleep(kPostPlotSleepMs);

    if (detect_lines_with_hough(png_path) != 0) {
        return 1;
    }

    return 0;
}
