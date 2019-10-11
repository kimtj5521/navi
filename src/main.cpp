#include "main.h"

Euler Quarter2Eular(Quaternion _q)
{
    Euler out;

    float sinr = +2.0 * (_q.w * _q.x + _q.y * _q.z);
    float cosr = +1.0 -2.0 * (_q.x * _q.x + _q.y * _q.y);

    out.roll = atan2(sinr,cosr);

    float sinp = +2.0 * (_q.w * _q.y - _q.z * _q.x);

    if(fabs(sinp) >=1)
    {
        out.pitch = copysign(PI / 2, sinp);
    }
    else
    {
        out.pitch = asin(sinp);
    }

    float siny = 2.0 * (_q.w * _q.z + _q.x * _q.y);
    float cosy = 1.0 -2.0 * (_q.y * _q.y + _q.z * _q.z);
    out.yaw = atan2(siny,cosy);

    return out;
}

int reference_point_check(void)
{
    int row_size = sizeof(gps_ref) / sizeof (gps_ref[0]);
    int index = 0;

    for(int i=0; i < row_size; i++){
        distanceTemp[i] = sqrt(pow((gps_ref[i][0]*PI/180.0 - m_gps_data.latitude*PI/180.0),2) + pow((gps_ref[i][1]*PI/180.0 - m_gps_data.longitude*PI/180.0),2));
    }
    minimum_value = distanceTemp[0];
    for(int k=0; k < row_size; k++){
        if(minimum_value > distanceTemp[k]){
            minimum_value = distanceTemp[k];
            index = k;
        }
    }
    first_call = false;
    return index;
}

void subscribeCallback_imu(const sensor_msgs::Imu::ConstPtr& msg)
{
    m_imu_data_prev = m_imu_data;

    m_imu_data.angular_vel_x = msg->angular_velocity.x;
    m_imu_data.angular_vel_y = msg->angular_velocity.y;
    m_imu_data.angular_vel_z = msg->angular_velocity.z;

    m_imu_data.linear_acceleration_x = msg->linear_acceleration.x;
    m_imu_data.linear_acceleration_y = msg->linear_acceleration.y;
    m_imu_data.linear_acceleration_z = msg->linear_acceleration.z;

    // swap X/Y, invert Z
    // microstrain 3dm-gx5 package is modified by author
    m_imu_data.orientation_w = msg->orientation.w;
    m_imu_data.orientation_x = msg->orientation.y;
    m_imu_data.orientation_y = msg->orientation.x;
    m_imu_data.orientation_z = -1.0*msg->orientation.z;

    // swap X/Y, invert Z
    // microstrain 3dm-gx5 package is modified by author
    Quaternion tmp;
    tmp.w = msg->orientation.w;
    tmp.x = msg->orientation.y;
    tmp.y = msg->orientation.x;
    tmp.z = -1.0*msg->orientation.z;

    Euler e = Quarter2Eular(tmp);

    m_imu_data.euler_angle_x = e.roll*180.0/PI;
    m_imu_data.euler_angle_y = e.pitch*180.0/PI;
    m_imu_data.euler_angle_z = e.yaw*180.0/PI;
}

void subscribeCallback_gps(const std_msgs::Float64MultiArray& msg)
{
    m_gps_data_prev = m_gps_data;

    int time_hour = 0;
    int time_minute = 0;
    double time_second = 0.0;
    int time_ms = 0;

    time_hour = msg.data.at(0);
    time_minute = msg.data.at(1);
    time_second = msg.data.at(2);

    time_ms = time_hour*3600000 + time_minute*60000 + time_second*1000;

    int rmc_time_hour = 0;
    int rmc_time_minute = 0;
    double rmc_time_second = 0.0;
    int rmc_time_ms = 0;

    rmc_time_hour = msg.data.at(12);
    rmc_time_minute = msg.data.at(13);
    rmc_time_second = msg.data.at(14);

    rmc_time_ms = rmc_time_hour*3600000 + rmc_time_minute*60000 + rmc_time_second*1000;

    double gps_height1 = 0.0;
    double gps_height2 = 0.0;

    gps_height1 = msg.data.at(10); // height (at virtual sea level)
    gps_height2 = msg.data.at(11); // height (difference between real sea level and virtual sea level)

    double dGPS_DMLAT = 0.0;
    double dGPS_DMLONG = 0.0;
    dGPS_DMLAT = msg.data.at(3); // latitude
    dGPS_DMLONG = msg.data.at(5); // longitude

    // DegMin -> DegDeg
    m_CoordConv.GPSWGS84_DM2DD(dGPS_DMLAT, dGPS_DMLONG);

    m_gps_data.time = time_ms; // gps time
    m_gps_data.latitude = m_CoordConv.dWGS84_DDLAT;
    m_gps_data.lat_direction = msg.data.at(4); // latitude direction
    m_gps_data.longitude = m_CoordConv.dWGS84_DDLON;
    m_gps_data.long_direction = msg.data.at(6); // longitude direction
    m_gps_data.quality_indicator = msg.data.at(7); // mode indicator
    m_gps_data.num_use_satellite = msg.data.at(8); // # of using satellites
    m_gps_data.HDOP = msg.data.at(9); // Horizontal dillusion of position
    m_gps_data.height = gps_height1 + gps_height2;

    m_gps_data.rmc_time = rmc_time_ms;
    m_gps_data.gprmc_speed = msg.data.at(20); // GxRMC knots to m/s speed
    m_gps_data.gprmc_gps_heading = msg.data.at(21); // GxRMC heading degree (north base clock-wise direction 0~360 deg)

    if(first_call){
        ref_index = reference_point_check();
    }
    //cout << "Reference_point_index : " << ref_index << endl;

    m_CoordConv.WGS2UTM(gps_ref[ref_index][0], gps_ref[ref_index][1]);
    m_dRef_UTM_X = m_CoordConv.dUTM_X;
    m_dRef_UTM_Y = m_CoordConv.dUTM_Y;
    m_CoordConv.WGS2UTM(m_gps_data.latitude, m_gps_data.longitude);
    m_dGPS_UTM_X = m_CoordConv.dUTM_X;
    m_dGPS_UTM_Y = m_CoordConv.dUTM_Y;

    m_gps_data.pos_x = m_dGPS_UTM_X - m_dRef_UTM_X;
    m_gps_data.pos_y = m_dGPS_UTM_Y - m_dRef_UTM_Y;
}

void subscribeCallback_odom(const navi::rio_to_pc& msg)
{
    m_odom_data_prev = m_odom_data;

    m_odom_data.velocity = msg.vel_encoder;
}

void subscribeCallback_init(const std_msgs::String::ConstPtr& msg)
{
    GPS_init = true;
}

int main(int argc, char** argv)
{
    interval = 0.01;

    ros::init(argc, argv, "navi");
    ros::NodeHandle nh;

    ros::Publisher pub_pose_estimation = nh.advertise<std_msgs::Float64MultiArray>("POS_T",1);
    ros::Subscriber listener_imu = nh.subscribe("imu/data",1, subscribeCallback_imu, ros::TransportHints().tcpNoDelay());
    ros::Subscriber listener_gps = nh.subscribe("GpsData",1, subscribeCallback_gps, ros::TransportHints().tcpNoDelay());
    ros::Subscriber listener_odom = nh.subscribe("rio_to_pc_l2r",1, subscribeCallback_odom, ros::TransportHints().tcpNoDelay());
    ros::Subscriber listener_init = nh.subscribe("init",100, subscribeCallback_init, ros::TransportHints().tcpNoDelay());

    std_msgs::Float64MultiArray msg_ekf_pose_result_publish;

    ros::Rate loop_rate(100);

    bool cmp_gps_time = false;
    bool cmp_gps_rmc_time = false;
    bool cmp_num_setellite_and_dev = false;
    int countpos = 0;

    bool get_position = false;
    bool get_heading = false;
    bool POS_init = true;

    double hcount = 0.0;
    double hbias = 0.0;
    double h_gps = 0.0;

    double vcount = 0.0;
    double v_gain = 1.0;
    double v_gain_temp = 0.0;

    double heading_error = 0.0;
    double pos_error = 0.0;

    initial_heading = -29.545*PI/180.0;
    h_gps = initial_heading;

    double vel_ave = 0.0;
    double c_yaw_rate = 0.0;

    vel_and_yawRate = Mat::zeros(2, 1, CV_64FC1);
    GPS_data = Mat::zeros(3, 1, CV_64FC1);

    bool true_false_1st = false;
    bool true_false_2nd = false;
    bool ekf_update_bool = false;
    bool pos_update_boolean = false;
    bool heading_update_boolean = false;

    for(int n =0; n<11; n++){
        m_d_nav_result[n] = 0.0;
    }

    m_ekf.EKF_Initialization();

    while (ros::ok()) {
        // interval time starting point
        clock_gettime(CLOCK_MONOTONIC, &prev_time);

        msg_ekf_pose_result_publish.data.clear();
        ros::spinOnce();

        imu_yaw_rate = (m_imu_data.angular_vel_z + m_imu_data_prev.angular_vel_z)*0.5;
        vehicle_speed = (m_odom_data.velocity + m_odom_data_prev.velocity)*0.5;

        ////////////// get_position check ///////////////////
        if(m_gps_data.time != m_gps_data_prev.time){
            cmp_gps_time = true;
        }
        else {
            cmp_gps_time = false;
        }

        if((m_gps_data.num_use_satellite > 10) && (m_gps_data.HDOP < 1.5)){
            cmp_num_setellite_and_dev = true;
        }
        else {
            cmp_num_setellite_and_dev = false;
        }

        if(cmp_num_setellite_and_dev == true){
            countpos +=1;
            if(countpos > 250) { countpos = 250; }
        }
        else{
            countpos = 0;
        }

        if((countpos == 250) && (cmp_gps_time == true)){
            get_position = true;
        }
        else {
            get_position = false;
        }
        /////////////////////////////////////////////////////

        ////////////// get_heading check ////////////////////
        if(m_gps_data.rmc_time != m_gps_data_prev.rmc_time){
            cmp_gps_rmc_time = true;
        }
        else {
            cmp_gps_rmc_time = false;
        }

        if((countpos == 250) && (cmp_gps_rmc_time == true)){
            get_heading = true;
        }
        else {
            get_heading = false;
        }
        /////////////////////////////////////////////////////

        //////// forward/backward heading calculate /////////
        if(vehicle_speed > update_threshold_velocity){
            h_gps = (-1.0*m_gps_data.gprmc_gps_heading+90.0)*PI/180.0;
            h_gps = m_ekf.AngDiff(h_gps);
        }
        if(vehicle_speed < (-1.0*update_threshold_velocity)){ // backward threshold is -3.0 km/h
            h_gps = (-1.0*m_gps_data.gprmc_gps_heading+90.0)*PI/180.0 - PI;
            h_gps = m_ekf.AngDiff(h_gps);
        }
        /////////////////////////////////////////////////////

        ////////////// heading bias calculate ///////////////
        if (fabs(vehicle_speed) <= update_threshold_velocity){
            hcount += 1.0;
            if(hcount > 18446744073709551615.0){ hcount = 18446744073709551615.0; }
            hbias = (hbias*hcount + imu_yaw_rate)/hcount;
        }
        /////////////////////////////////////////////////////

        ////////////// velocity gain calculate //////////////
        v_gain_temp = m_gps_data.gprmc_speed / fabs(vehicle_speed);

        if ((fabs(vehicle_speed) > 15) && (cmp_gps_rmc_time == true) && (fabs(v_gain_temp - 1.0) <0.2)){
            vcount += 1.0;
            if(vcount > 18446744073709551615.0){ vcount = 18446744073709551615.0; }
            v_gain = (v_gain*vcount + v_gain_temp)/vcount;

        }
        /////////////////////////////////////////////////////

        vel_ave = (vehicle_speed * v_gain)/3.6;
        if( fabs(vehicle_speed) <= update_threshold_velocity){
            c_yaw_rate = 0.0;
        }
        else {
            //c_yaw_rate = (imu_yaw_rate - hbias)*(-1.0); // this case : left turn is plus
            c_yaw_rate = (imu_yaw_rate - hbias); // this case : right turn is plus
        }

        vel_and_yawRate.ptr<double>(0)[0] = vel_ave;
        vel_and_yawRate.ptr<double>(1)[0] = c_yaw_rate;

        GPS_data.ptr<double>(0)[0] = m_gps_data.pos_x;
        GPS_data.ptr<double>(1)[0] = m_gps_data.pos_y;
        GPS_data.ptr<double>(2)[0] = h_gps;

        m_ekf.EKF_Predictionstep(m_ekf.m_xhat, m_ekf.m_Phat, vel_and_yawRate, interval);

        heading_error = fabs(m_ekf.AngDiff(h_gps - m_ekf.m_xhat.ptr<double>(2)[0]));

        if(((get_heading == true) || (get_position == true)) && (fabs(vehicle_speed) > update_threshold_velocity)){
            true_false_1st = true;
        }
        else {
            true_false_1st = false;
        }
        if((heading_error < 1.0) || (POS_init == true)){
            true_false_2nd = true;
        }
        else {
            true_false_2nd = false;
        }
        if((true_false_1st == true) && (true_false_2nd == true)){
            ekf_update_bool = true;
        }
        else {
            ekf_update_bool = false;
        }

        if((ekf_update_bool == true) && (GPS_init == false)){
            pos_error = sqrt(pow((m_ekf.m_xhat.ptr<double>(0)[0] - m_gps_data.pos_x),2) + pow((m_ekf.m_xhat.ptr<double>(1)[0] - m_gps_data.pos_y),2));

            if((heading_error < 0.05) && (pos_error < 2.5)){
                POS_init = false;
            }

            if(((pos_error < 50) && (get_position == true)) || (POS_init == true)){
                pos_update_boolean = true;
            }
            else {
                pos_update_boolean = false;
            }

            if(((heading_error < 0.1) && (get_heading == true)) || (POS_init == true)){
                heading_update_boolean = true;
            }
            else {
                heading_update_boolean = false;
            }

            m_ekf.EKF_Correctionstep(m_ekf.m_xhat, m_ekf.m_Phat, m_gps_data.quality_indicator, pos_update_boolean, heading_update_boolean, GPS_data);
        }
        else {
            if (((POS_init == true) && (fabs(vehicle_speed) < update_threshold_velocity)) || (GPS_init == true)){
                if(GPS_init == true){
                    m_ekf.m_xhat.ptr<double>(0)[0] = m_gps_data.pos_x;
                    m_ekf.m_xhat.ptr<double>(1)[0] = m_gps_data.pos_y;
                    m_ekf.m_xhat.ptr<double>(2)[0] = h_gps;
                    GPS_init = false;
                }
                else {
                    m_ekf.m_xhat.ptr<double>(0)[0] = m_gps_data.pos_x;
                    m_ekf.m_xhat.ptr<double>(1)[0] = m_gps_data.pos_y;
                    m_ekf.m_xhat.ptr<double>(2)[0] = h_gps;
                }
            }
            else {

            }
        }

        m_d_nav_result[0] = m_ekf.m_xhat.ptr<double>(0)[0];
        m_d_nav_result[1] = m_ekf.m_xhat.ptr<double>(1)[0];
        m_d_nav_result[2] = m_ekf.m_xhat.ptr<double>(2)[0]*180.0/PI;
        m_d_nav_result[3] = m_gps_data.pos_x;
        m_d_nav_result[4] = m_gps_data.pos_y;
        m_d_nav_result[5] = h_gps*180.0/PI;

        m_d_nav_result[6] = (double)get_position;
        m_d_nav_result[7] = (double)get_heading;
        m_d_nav_result[8] = (double)POS_init;
        m_d_nav_result[9] = (double)GPS_init;
        m_d_nav_result[10] = m_odom_data.velocity;

        for(int j=0; j<11; j++){
            msg_ekf_pose_result_publish.data.push_back(m_d_nav_result[j]);
        }
        pub_pose_estimation.publish(msg_ekf_pose_result_publish);

        m_imu_data_prev = m_imu_data;
        m_odom_data_prev = m_odom_data;
        m_gps_data_prev = m_gps_data;

        loop_rate.sleep();

        // calculate interval time
        clock_gettime(CLOCK_MONOTONIC, &curr_time);
        interval = (curr_time.tv_nsec - prev_time.tv_nsec)/1000000000.0; /* seconds */
        if (!(interval > 0.0098 && interval < 0.012)){
            interval = 0.01;
        }
        //cout << "interval: " << interval << endl;
    }

    return 0;
}
