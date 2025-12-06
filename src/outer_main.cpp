#include <xarm/wrapper/xarm_api.h>
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>
float rad2deg(float r){
    return r * (180.0f /static_cast<float>(M_PI));
}
float deg2rad(float d){
    return d * (static_cast<float>(M_PI)/180.0f);
}
int main(int argc , char ** argv){
    if(argc != 2){
        std::cout << "Usage: ./program <path/to/csv_file>\n";
        return -1;
    }
    std::string waypoint_file= argv[1];
    std::cout<< "waypoint file path is  :"<<waypoint_file<<std::endl;
    // open file  
    std::ifstream file(waypoint_file);
    if(!file.is_open()){
        std::cout << "Could not open file\n";
        return -1 ; 
    }
    // read file and store it in vector  
    std::vector<std::vector<double>> csv_data;
    std::string line;
    while(std::getline(file,line)){
        std::stringstream ss(line);
        std::string cell;
        std::vector<double> row;
        while (std::getline(ss, cell, ',')) {
            row.push_back(std::stod(cell));
        }
        if (row.size() ==6) { //csv input will be x,y,z  only  
           csv_data.push_back(row);
        }
    }
    
    // create  robot  instance  
    std::string port("192.168.1.221");
    XArmAPI *arm = new XArmAPI(port,true); //make orientation in radain
    sleep_milliseconds(500);
    if (arm->error_code != 0) arm->clean_error();
    if(arm->warn_code != 0)  arm->clean_warn();
        //setup tcp tool shift  
    float tcp_offset[6] = {
        -34.03f,   // x offset (mm)
        -3.22f,    // y offset (mm)
        338.32f,   // z offset (mm)
        0.0f,      // roll offset (rad)
        0.0f,      // pitch offset (rad)
        0.0f       // yaw offset (rad)
    };
    int ret = arm->set_tcp_offset(tcp_offset,true);
    std::cout << "set_tcp_offset ret = " << ret << std::endl;
    if (ret != 0) {
        std::cerr << "Error: set_tcp_offset failed, aborting.\n";
        arm->disconnect();
        delete arm;
        return -1;
    }
    std::cout<<"tcp shift after set offset"<<std::endl;
    std::cout << "Current TCP Offset:" << std::endl;
    std::cout << "  x = " << arm->tcp_offset[0] << " mm" << std::endl;
    std::cout << "  y = " << arm->tcp_offset[1] << " mm" << std::endl;
    std::cout << "  z = " << arm->tcp_offset[2] << " mm" << std::endl;
    std::cout << " roll = " << arm->tcp_offset[3] << " rad" << std::endl;
    std::cout << "pitch = " << arm->tcp_offset[4] << " rad" << std::endl;
    std::cout << "  yaw = " << arm->tcp_offset[5] << " rad" << std::endl;
    sleep_milliseconds(500);

    arm->motion_enable(true);
    arm->set_mode(0);  // position mode 
    arm->set_state(0); // set  state to 0 to make robot  ready 
    // Wait until robot is actually ready
    int tries = 100;
    while ((arm->state != 0 || arm->mode != 0) && tries-- > 0) {
        sleep_milliseconds(50);
    }

    if (arm->state != 0 || arm->mode != 0) {
        std::cerr << "❌ Robot is not ready after mode/state commands!\n";
        return -1;
    }
    float kp[6] = { 0.005, 0.005, 0.005, 0.005, 0.005, 0.005 }; // range: 0 ~ 0.05
    float ki[6] = { 0.00006, 0.00006, 0.00006, 0.00006, 0.00006, 0.00006 }; // range: 0 ~ 0.0005
    float kd[6] = { 0 }; // range: 0 ~ 0.05
    float xe_limits[6] = { 100.0, 100.0, 100.0, 100.0, 100.0, 100.0 }; // max adjust velocity(mm/s), range: 0 ~ 200

    int coord = 1; // 0 : base , 1 : tool
    int c_axis[6] = { 0, 0, 1, 0, 0, 0 }; // only control force along z axis
    // MAKE SURE reference frame and force taget sign are correct !!
    float f_ref[6] = { 0.0, 0, 0.6, 0, 0, 0 }; // the force(N) that xArm will apply to the environment
    float limits[6] = { 0 }; // limits are reserved, just give zeros
    int ret_ft;

    // set pid parmeters for force control
    ret_ft = arm->set_ft_sensor_force_parameters(kp, ki, kd, xe_limits);
    printf("set_ft_sensor_force_parameters(pid), ret=%d\n", ret_ft);

    ret_ft = arm->set_ft_sensor_force_parameters(coord, c_axis, f_ref, limits);
    printf("set_ft_sensor_force_parameters, ret=%d\n", ret_ft);

    // enable ft sensor communication
    ret_ft = arm->set_ft_sensor_enable(1);
    printf("set_ft_sensor_enable, ret=%d\n", ret_ft);
    // will overwrite previous sensor zero and payload configuration
    ret_ft = arm->set_ft_sensor_zero(); // remove this if zero_offset and payload already identified & compensated!
    printf("set_ft_sensor_zero, ret=%d\n", ret_ft);
    sleep_milliseconds(200); // wait for writting zero operation to take effect, do not remove
    // move to first point  - 5 mm in z axis  
        // setup oreintation for robot tcp pose  
    float  speed = 10 ;
    float  acc = 100 ; 
    float  mvtime = 0 ;
    // angles from [180,-180] 
    // float roll  = 3.1364975f; 179.713  [120 ,-120]
    // float pitch = -1.0373678f; 59.44 [0,120]
    // float yaw   = 0.00118106f; 0.067 [-60,60]
    // will start after set_state(0)
    ret_ft = arm->set_state(0);

    // keep for 5 secs
    sleep_milliseconds(1000 * 5);
    // Read raw values from CSV (in radians)
    float x = static_cast<float>(csv_data[0][0]);
    float y = static_cast<float>(csv_data[0][1]);
    float z = static_cast<float>(csv_data[0][2]);

    // float roll_raw  = static_cast<float>(csv_data[0][3]);
    // float pitch_raw = static_cast<float>(csv_data[0][4]);
   // float yaw_raw   = static_cast<float>(csv_data[0][5]);
    // float yaw_lim = deg2rad(std::clamp(rad2deg(yaw_raw),-40.0f,40.0f));
    // float yaw_lim = deg2rad(std::clamp(rad2deg(yaw_raw),-40.0f,40.0f));
    // float yaw_lim = deg2rad(std::clamp(rad2deg(yaw_raw),-40.0f,40.0f));

    float roll_raw  =3.1364975f ;
    float pitch_raw = -1.0373678f;
    float yaw_raw   = 0.8f; //25 degree
        // Fill pose array
     float pose[6] = {
            x, y, z,
            roll_raw,
            pitch_raw,
            yaw_raw
        };

        // === PRINT POSE (degrees for rotation) ===
        std::cout << "[ Moving to: "
                << "x=" << x
                << ", y=" << y
                << ", z=" << z
                << ", roll(deg)="  << rad2deg(roll_raw)
                << ", pitch(deg)=" << rad2deg(pitch_raw)
                << ", yaw(deg)="   << rad2deg(yaw_raw)
                //<< ", yaw(deg)lim="   << rad2deg(yaw_lim)
                << " ]" << std::endl;
    int ret_first = arm->set_position(
            pose,
            -1.0f,   // radius: -1 = no blending
            speed,
            acc,
            mvtime,
            true 
        );

        if (ret_first != 0) {
            std::cerr << "set_position failed at index " 
                      << " with ret=" << ret_first << std::endl;
            return -1;
        }

    // move robot in force control
    ret_ft = arm->set_ft_sensor_mode(2);
    printf("set_ft_sensor_mode, ret=%d\n", ret);
    // will start after set_state(0)
    ret_ft = arm->set_state(0);

    // keep for 5 secs
    sleep_milliseconds(1000 * 5);
   std::cout<<"robot is ready to move now\n";
    




    std::cout << "Starting Cartesian path execution...\n";
    size_t N = csv_data.size();       // total number of points
    size_t n1 = N / 5;         // 20%
    size_t n2 = (2 * N) / 5;   // 40%
    size_t n3 = (3 * N) / 5;   // 60%
    size_t n4 = (4 * N) / 5;   // 80%
    for (size_t i = 0; i < csv_data.size(); ++i) {
        if (arm->error_code != 0) {
            std::cerr << "Robot error detected (error_code="
                      << arm->error_code << "), stopping.\n";
            break;
        }

        // Read raw values from CSV (in radians)
        float x = static_cast<float>(csv_data[i][0]);
        float y = static_cast<float>(csv_data[i][1]);
        float z = static_cast<float>(csv_data[i][2]+ 1.5f);

        // float roll_raw  = static_cast<float>(csv_data[i][3]);
        // float pitch_raw = static_cast<float>(csv_data[i][4]);
        //float yaw_raw   = static_cast<float>(csv_data[i][5]);
        // float yaw_lim = deg2rad(std::clamp(rad2deg(yaw_raw),-40.0f,40.0f));
        roll_raw  =3.1364975f ;
        pitch_raw = -1.0373678f;
        if (i < n1) {
            yaw_raw =  0.8f;      // Region 1 (0–20%)
        }
        else if (i < n2) {
            yaw_raw =  0.4f;      // Region 2 (20–40%)
        }
        else if (i < n3) {
            yaw_raw =  0.0f;      // Region 3 (40–60%)
        }
        else if (i < n4) {
            yaw_raw = -0.4f;      // Region 4 (60–80%)
        }
        else {
            yaw_raw = -0.8f;      // Region 5 (80–100%)
        }

        // Fill pose array
        float pose[6] = {
                x, y, z,
                roll_raw,
                pitch_raw,
                yaw_raw
            };

            // === PRINT POSE (degrees for rotation) ===
            std::cout << "[ Moving to: "
                    << "x=" << x
                    << ", y=" << y
                    << ", z=" << z
                    << ", roll(deg)="  << rad2deg(roll_raw)
                    << ", pitch(deg)=" << rad2deg(pitch_raw)
                    << ", yaw(deg)="   << rad2deg(yaw_raw)
                   // << ", yaw(deg)lim="   << rad2deg(yaw_lim)
                    << " ]" << std::endl;

        int  ret = arm->set_position(
            pose,
            -1.0f,   // radius: -1 = no blending
            speed,
            acc,
            mvtime,
            true 
        );

        if (ret != 0) {
            std::cerr << "set_position failed at index " << i
                      << " with ret=" << ret << std::endl;
            break;
        }
        // read  force  torque  raw force and  external force  
        print_nvect("raw_force: ", arm->ft_raw_force, 6);
        print_nvect("ext_force: ", arm->ft_ext_force, 6);


        // small delay between commands
         sleep_milliseconds(100);
    }

    // remember to reset ft_sensor_app when finished
    ret_ft = arm->set_ft_sensor_mode(0);
    printf("set_ft_sensor_modeset_ft_sensor_mode, ret=%d\n", ret_ft);
    ret_ft = arm->set_ft_sensor_enable(0);
    printf("set_ft_sensor_enable, ret=%d\n", ret_ft);
    arm->set_state(4);  // 4 = stopped (optional but nice)
    arm->disconnect();
    delete arm;
    
    return 0 ; 
}

