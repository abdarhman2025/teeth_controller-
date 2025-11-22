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
        if (row.size() ==4) {
           csv_data.push_back(row);
        }
    }
    
    // create  robot  instance  
    std::string port("192.168.1.221");
    XArmAPI *arm = new XArmAPI(port);
    sleep_milliseconds(500);
    if (arm->error_code != 0) arm->clean_error();
    if(arm->warn_code != 0)  arm->clean_warn();
    arm->motion_enable(true);

    arm->set_mode(0);  // position mode 
    arm->set_state(0); // set  state to 0 to make robot  ready 
    sleep_milliseconds(500);
    
    // setup oreintation for robot tcp pose  
    float  speed = 20 ;
    float  acc = 200 ; 
    float  mvtime = 0 ; 
    float roll  = 3.1364975f;
    float pitch = -1.0373678f;
    float yaw   = 0.00118106f;
    for (size_t i =0 ; i< csv_data.size();++i){
       
        float  pose[6] = {
                         (float)csv_data[i][0],  
                         (float)csv_data[i][1],
                         (float)csv_data[i][2],
                         roll,
                         pitch,
                         yaw};
        
        arm->set_position(
            pose,
            -1,       // no blending
            20,       // speed mm/s
            200,      // acc mm/s2
            0,        // mvtime = automatic
            false     // non-blocking 
        );

        sleep_milliseconds(10); // wait for 10 ms  

    }


    
    return 0 ; 
}

