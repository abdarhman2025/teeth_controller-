#include <xarm/wrapper/xarm_api.h>
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
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
        if (row.size() ==3) { //csv input will be x,y,z  only  
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
   std::cout<<"robot is ready to move now\n";
  

    // setup oreintation for robot tcp pose  
    float  speed = 10 ;
    float  acc = 100 ; 
    float  mvtime = 0 ; 
    float roll  = 3.1364975f;
    float pitch = -1.0373678f;
    float yaw   = 0.00118106f;


    std::cout << "Starting Cartesian path execution...\n";
    for (size_t i = 0; i < csv_data.size(); ++i) {
        if (arm->error_code != 0) {
            std::cerr << "Robot error detected (error_code="
                      << arm->error_code << "), stopping.\n";
            break;
        }

        // CSV row: [x, y, z, 1]
        float pose[6] = {
            static_cast<float>(csv_data[i][0]),   // x (mm)
            static_cast<float>(csv_data[i][1]),   // y (mm)
            static_cast<float>(csv_data[i][2]),   // z (mm)
            roll,
            pitch,
            yaw
        };

        std::cout << "[" << i << "] Moving to: "
                  << "x=" << pose[0]
                  << ", y=" << pose[1]
                  << ", z=" << pose[2] << std::endl;

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

        // small delay between commands
         sleep_milliseconds(100);
    }


    arm->set_state(4);  // 4 = stopped (optional but nice)
    arm->disconnect();
    delete arm;
    
    return 0 ; 
}

