#include<windows.h>
#include<iostream>
#include<stdio.h>
#include<iterator>
#include<cstdio>
#include<cmath>

#define _USE_MATH_DEFINES
#include<math.h>
#include<conio.h>
#include<time.h>
#include<chrono>
#include<fstream>
#include<vector>
#include<iomanip>
#include<string>
#include<sstream>
#include<cstdlib>
#include <opencv2/opencv.hpp>
//#include "utm_com.h"
//#include "common.h"

#ifdef _DEBUG
#pragma comment(lib, "opencv_world4110d.lib")
#else
#pragma comment(lib, "opencv_world4110.lib")
#endif


using namespace std;
using namespace cv;


#define SCREEN_CLS 1000

bool timer(bool outer) {

    chrono::system_clock::time_point be_time, af_time;
    be_time = chrono::system_clock::now();
    for (;;) {
        af_time = chrono::system_clock::now();
        int af_bf = chrono::duration_cast<chrono::milliseconds>(af_time - be_time).count();
        if (af_bf >= SCREEN_CLS) {
            break;
        }
    }
    return 0;
}


class Clocker {
private:
    char now_time[60] = {};


public:
    void clocker() {

        time_t t = time(NULL);
        struct tm local;
        //char now[60] = {};

        localtime_s(&local, &t);

        strftime(now_time, 60, "%Y%m%d%H%M", &local);
        cout << "Now :" << now_time << endl;
       
    }

    const char* get_time() {
        return now_time;
    }

};


struct ScanPoint {
    float angle;       // degrees
    uint16_t distance; // mm
    uint8_t intensity;
};

HANDLE lidarPort;

bool openLidarPort(const char* portName) {
    lidarPort = CreateFileA(portName, GENERIC_READ, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (lidarPort == INVALID_HANDLE_VALUE) return false;

    DCB dcb = { 0 };
    dcb.DCBlength = sizeof(dcb);
    GetCommState(lidarPort, &dcb);
    dcb.BaudRate = 230400;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    SetCommState(lidarPort, &dcb);

    COMMTIMEOUTS timeouts = { 0 };
    timeouts.ReadIntervalTimeout = 10;
    timeouts.ReadTotalTimeoutConstant = 10;
    timeouts.ReadTotalTimeoutMultiplier = 1;
    SetCommTimeouts(lidarPort, &timeouts);

    return true;
}

/*bool readLidarPacket(std::vector<uint8_t>& packet) {
    packet.resize(47); // expected typical packet size
    DWORD bytesRead = 0;
    return ReadFile(lidarPort, packet.data(), packet.size(), &bytesRead, NULL) && bytesRead == packet.size();
}*/

bool readLidarPacket(std::vector<uint8_t>& packet) {
    uint8_t byte;
    DWORD bytesRead = 0;

    // パケット先頭 0x54 が来るまで読み飛ばす（同期）
    while (true) {
        if (!ReadFile(lidarPort, &byte, 1, &bytesRead, NULL) || bytesRead != 1)
            return false;
        if (byte == 0x54)
            break;
    }

    // すでに1バイト読んでいるので、残り46バイト読み込む
    packet.resize(47);
    packet[0] = 0x54;

    if (!ReadFile(lidarPort, &packet[1], 46, &bytesRead, NULL) || bytesRead != 46)
        return false;

    return true;
}


int utm_get_data(const string dir) {
    if (!openLidarPort("\\\\.\\COM7")) {
        cerr << "Can't open port\n";
        return 1;
    }

    cout << "Scaning..." << endl;

    vector<ScanPoint> scanPoints;
    vector<uint8_t> packet;
    float last_angle = -1.0f;
    float angle_accumulator = 0.0f;
    bool first_packet = true;

    float x_value, y_value;

    while (true) {

        packet = {};

        if (!readLidarPacket(packet)) {
            cerr << "[read failed]" << std::endl;
            continue;
        }

        // 読み込んだサイズが期待より小さい場合はスキップ
        if (packet.size() < 2) {
            cerr << "[packet too small]" << std::endl;
            continue;
        }

        //std::cout << "[packet read, byte0-1: " << std::hex << (int)packet[0] << "," << (int)packet[1] << std::dec << "]" << std::endl;

        //cout << packet[0] << "," << packet[1] << endl;

        //if (kbhit())break;

        if (packet[0] != 0x54) {
            std::cerr << "[header mismatch]" << std::endl;
            continue;
        }

        uint8_t count = packet[1] & 0x1F;
        uint16_t start_angle_raw = packet[4] | (packet[5] << 8);
        uint16_t end_angle_raw = packet[6 + count * 3] | (packet[7 + count * 3] << 8);
        float start_angle = start_angle_raw / 100.0f;
        float end_angle = end_angle_raw / 100.0f;

        //std::cout << start_angle << "," << last_angle << std::endl;

        if (!first_packet) {
            float delta = start_angle - last_angle;
            if (delta < 0) delta += 360.0f;
            angle_accumulator += delta;

            if (angle_accumulator >= 360.0f) {
                std::cout << "Scan complete（" << scanPoints.size() << "point）\n";
                break;
            }
        }
        else {
            first_packet = false;
        }

        last_angle = start_angle;

        for (int i = 0; i < count; ++i) {
            int idx = 6 + i * 3;
            uint16_t dist = packet[idx] | (packet[idx + 1] << 8);
            uint8_t intensity = packet[8 + 3 * i];
            //float angle = start_angle + i * (end_angle - start_angle) / (count - 1);
            float angle = start_angle + i * 0.8;
            /*if (angle >= 360) {
                angle -= 360;
                Sleep(300);
            }*/
            scanPoints.push_back({ angle, dist, intensity });
        }

        Sleep(10);
    }

    CloseHandle(lidarPort);

    // CSV出力
    /*ofstream ofs(dir);
    for (auto& p : scanPoints) {
        x_value = p.distance * cos(p.angle * M_PI / 180);
        y_value = p.distance * sin(p.angle * M_PI / 180);
        ofs << x_value << "," << y_value << endl;

    }*/

    ofstream ofs(dir);
    for (auto p = scanPoints.begin(); p != scanPoints.end(); ++p) {
        x_value = p->distance * cos(p->angle * M_PI / 180);
        y_value = p->distance * sin(p->angle * M_PI / 180);
        ofs << x_value << "," << y_value << endl;
    }

    cout << "Save complete\n";

    return 0;
}


int gnuwrite(const char* filename){

	//char filename[60] = {};
	//get_time(filename);

    string gnufile = string("result_") + filename + ".csv";


	FILE* gp;
	gp = _popen("gnuplot -persist", "w");

	fprintf(gp, "set datafile separator ',' \n");
	fprintf(gp, "cd 'C:/program2/result' \n");

	fprintf(gp, "set term png size 720,720 \n");
	fprintf(gp, "set border 1248 \n");
	fprintf(gp, "unset tics \n");
	fprintf(gp, "unset key \n");

	fprintf(gp, "set output 'result-1.png' \n");
	fprintf(gp, "plot [-3000:3000][-3000:3000]  '%s' pt 7 ps 2 lc 'black' \n", gnufile.c_str());


	cout << "PNG data have saved" << endl;
	
	fprintf(gp, "set output \n");


	//fprintf(gp, "terminal win \n");
	fflush(gp);

	_pclose(gp);

	return 0;

}

int Hough_line(const char* filename, string dir) {

	cout << "Hough_line" << endl;
	int hr = -1;

	try {

		Mat src, edge, dst;
		vector<Vec4i> lines;
		int counter = -1, threshold = 70;
		size_t size = lines.size();
		double* slope;
		slope = new double[lines.size()];

		src = imread(dir, IMREAD_GRAYSCALE);

		Mat gray;
	
		Canny(src, edge, 50, 200, 3);

		dst = Mat::zeros(src.rows, src.cols, CV_8UC3);

		int fromTo[] = { 0,2,0,1,0,0 };

		mixChannels(&src, 1, &dst, 1, fromTo, 3);


		if (src.empty()) {
			cout << "src = empty" << endl;
		}

		while (1) {

			HoughLinesP(
				edge,
				lines,
				1,
				CV_PI / 180.0,
				threshold,
				160,
				200
			);

			if (abs(threshold) <= 100) {

				if (lines.size() >= 10) {

					cout << "lines weren't ditected" << endl;
					threshold += 10;

				}
				if (lines.size() <= 2) {
					cout << "lines weren't ditected" << endl;
					threshold -= 10;
				}

				else break;

			}
			else break;

		}

		for (auto line : lines) {

			counter++;
			cv::line(dst, Point(line[0], line[1]), Point(line[2], line[3]), Scalar(0, 0, 255), 2);

		}	

		namedWindow("dst", 1);
		imshow("dst", dst);
		imwrite("C:\\program2\\result\\Hough_result.png", dst);
		
		waitKey(0);

		hr = 0;
	}

	catch (Exception ex) {

		cout << ex.err << endl;

	}

	destroyAllWindows();
	return hr;
}


int main(int argc, char* argv[]) {

    Clocker now;
    now.clocker();
    cout << "CSV Filename : result_" << now.get_time() << endl;

    string path = string("C:\\program2\\result\\result_") + now.get_time() + ".csv";
    cout << "Path = " << path << endl;

	utm_get_data(path);

	gnuwrite(now.get_time());
	cout << "gnuwrite ended!" << endl;

	Sleep(1000);

	Hough_line(now.get_time(), path);

	return 0;
}
