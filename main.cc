#include <cmath>
#include <csignal>
#include <mutex>
#include <vector>
#include <string>
#include <fstream>
#include <deque>
#include <algorithm>
#include "message_def.h"

using namespace std;

//消息类型定义
using gpsMsgType = GpsMsg_ulhk;
string groundTruthFile="/home/idriver/code/dataset/ulhk/groundtruth/20190331.tum";

#define PI 3.14159265358979
bool flg_exit=false;
constexpr double KSINSDEGTORAD = 0.017453292519943;
constexpr double KSINSRADTODEG = 57.295779513082323;
constexpr double KUTMSCALEFACTOR = 0.9996;
constexpr double KSMA = 6378137.0;
constexpr double KSMB = 6356752.31425;



class GpsTransform {
   public:
    ros::NodeHandle nh;
    ros::Subscriber subGpsMsg;
    std::mutex mtx;
    std::ofstream ofs;
    vector<double> gpsPosition;
    deque<vector<double>> gpsMsgQue;

    struct UTMPoint {
        double t=0;
        double x = 0;
        double y = 0;
        double z=0;
    };
    

    string gpsTopic;
    UTMPoint utmPoint;

    explicit GpsTransform():ofs(groundTruthFile ,ios::out)
    {   
        //读取参数
        nh.param<string>("gpsTopic",gpsTopic,"/navsat/fix");

        //改变数组大小
        gpsPosition.reserve(4);

        //检测输出文件流
        if (!ofs.is_open()) {
            cout << "Failed to open ground truth file: " <<groundTruthFile;
            return;
        }
        ofs << "#timestamp x y z q_x q_y q_z q_w" << std::endl;
        ofs.flush();

        subGpsMsg=nh.subscribe<gpsMsgType>(gpsTopic,1000000,&GpsTransform::utmPointWriteOut, this, ros::TransportHints().tcpNoDelay());
    }
    ~GpsTransform()
    {
        ofs.close();
    }

    void utmPointWriteOut(const gpsMsgType::ConstPtr &gpsMsg)
    {   
        std::lock_guard<std::mutex> lock1(mtx);
        memset(&utmPoint,0,sizeof(UTMPoint));

        utmPoint.t=gpsMsg->header.stamp.toSec();
        utmPoint.z=gpsMsg->altitude;
        gpsPosition[0]=gpsMsg->longitude;
        gpsPosition[1]=gpsMsg->latitude;
           
        LatLonToUtmXY(gpsPosition[0]* KSINSDEGTORAD, gpsPosition[1] * KSINSDEGTORAD, utmPoint);
        cout << std::fixed << std::setprecision(6) 
            << utmPoint.t << " "
            << utmPoint.x << " "
            << utmPoint.y << " "
            << utmPoint.z << " "
            << 0 <<" " << 0 <<" " << 0 <<" " << 0 <<endl;
        ofs << std::fixed << std::setprecision(6) 
            << utmPoint.t << " "
            << utmPoint.x << " "
            << utmPoint.y << " "
            << utmPoint.z << " "
            << 0 <<" " << 0 <<" " << 0 <<" " << 0 <<endl;
        ofs.flush(); 
    }
    
    //经纬坐标转换成utm的xy坐标
    void LatLonToUtmXY(double lon_rad, double lat_rad, UTMPoint &xyz)
    {
        int zone = 0;
        zone = static_cast<int>((lon_rad * KSINSRADTODEG + 180) / 6) + 1;
        //仿佛在放置中心坐标
        double medium_point = (-183.0 + (zone * 6.0)) * KSINSDEGTORAD;
        //改变了xyz中的xy的值
        MapLatLonToXY(lat_rad, lon_rad, medium_point, xyz);

        /* Adjust easting and northing for UTM system. */
        xyz.x = xyz.x * KUTMSCALEFACTOR + 500000.0;
        xyz.y = xyz.y * KUTMSCALEFACTOR;
        if (xyz.y < 0.0) {
            xyz.y += 10000000.0;
        }
    }

   private:

    static double Arclength0fMeridian(const double phi)
    {
        /* Precalculate n */
        double n = (KSMA - KSMB) / (KSMA + KSMB);

        /* Precalculate alpha */
        double alpha = ((KSMA + KSMB) / 2.0) * (1.0 + (pow(n, 2.0) / 4.0) + (pow(n, 4.0) / 64.0));

        /* Precalculate beta */
        double beta = (-3.0 * n / 2.0) + (9.0 * pow(n, 3.0) / 16.0) + (-3.0 * pow(n, 5.0) / 32.0);

        /* Precalculate gamma */
        double gamma = (15.0 * pow(n, 2.0) / 16.0) + (-15.0 * pow(n, 4.0) / 32.0);

        /* Precalculate delta */
        double delta = (-35.0 * pow(n, 3.0) / 48.0) + (105.0 * pow(n, 5.0) / 256.0);

        /* Precalculate epsilon */
        double epsilon = (315.0 * pow(n, 4.0) / 512.0);

        /* Now calculate the sum of the series and return */
        double result = alpha * (phi + (beta * sin(2.0 * phi)) + (gamma * sin(4.0 * phi)) + (delta * sin(6.0 * phi)) +
                                (epsilon * sin(8.0 * phi)));

        return result;
    }

    static void MapLatLonToXY(double phi, double lambda, double lambda0, UTMPoint &xy)
    {
        /* Precalculate ep2 */
        double ep2 = (pow(KSMA, 2.0) - pow(KSMB, 2.0)) / pow(KSMB, 2.0);

        /* Precalculate nu2 */
        double nu2 = ep2 * pow(cos(phi), 2.0);

        /* Precalculate nn */
        double nn = pow(KSMA, 2.0) / (KSMB * sqrt(1 + nu2));

        /* Precalculate t */
        double t = tan(phi);
        double t2 = t * t;
        // double tmp = (t2 * t2 * t2) - pow(t, 6.0);

        /* Precalculate l */
        double l = lambda - lambda0;

        /* Precalculate coefficients for l**nn in the equations below
        so a normal human being can read the expressions for easting
        and northing
        -- l**1 and l**2 have coefficients of 1.0 */
        double l3coef = 1.0 - t2 + nu2;

        double l4coef = 5.0 - t2 + 9 * nu2 + 4.0 * (nu2 * nu2);

        double l5coef = 5.0 - 18.0 * t2 + (t2 * t2) + 14.0 * nu2 - 58.0 * t2 * nu2;

        double l6coef = 61.0 - 58.0 * t2 + (t2 * t2) + 270.0 * nu2 - 330.0 * t2 * nu2;

        double l7coef = 61.0 - 479.0 * t2 + 179.0 * (t2 * t2) - (t2 * t2 * t2);

        double l8coef = 1385.0 - 3111.0 * t2 + 543.0 * (t2 * t2) - (t2 * t2 * t2);

        /* Calculate easting (x) */
        xy.x = nn * cos(phi) * l + (nn / 6.0 * pow(cos(phi), 3.0) * l3coef * pow(l, 3.0)) +
            (nn / 120.0 * pow(cos(phi), 5.0) * l5coef * pow(l, 5.0)) +
            (nn / 5040.0 * pow(cos(phi), 7.0) * l7coef * pow(l, 7.0));

        /* Calculate northing (y) */
        xy.y = Arclength0fMeridian(phi) + (t / 2.0 * nn * pow(cos(phi), 2.0) * pow(l, 2.0)) +
            (t / 24.0 * nn * pow(cos(phi), 4.0) * l4coef * pow(l, 4.0)) +
            (t / 720.0 * nn * pow(cos(phi), 6.0) * l6coef * pow(l, 6.0)) +
            (t / 40320.0 * nn * pow(cos(phi), 8.0) * l8coef * pow(l, 8.0));
        return;
    }

    

};

void SigHandler(int sig) {
    ROS_INFO("%s is received, Terminating the node...",strsignal(sig));
    flg_exit=true;
}

int main(int argc,char** argv){
    ros::init(argc, argv, "gps_trans_utm");

    GpsTransform GT;

    // 处理ctrl-c
    signal(SIGINT, SigHandler);
    while(ros::ok())
    {
        ROS_INFO("loop");
        if (flg_exit) break;
        ros::spin();
    }
    return 0;
}

//没写CMAKELISTS