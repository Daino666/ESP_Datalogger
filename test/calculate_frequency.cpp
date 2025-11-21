#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <numeric>


/*

this code takes the data from amount of rows from the sensors_data.csv,
calculate time stamps differnece and gives each cycle frequency and each sensors frequency 

*/

int main(){

    std::fstream file("/home/daino/Desktop/Research Projects/FCDataLab/05_Sensor_integeration/ESP_DataLogger/ESP_Datalogger/Data_stored/sensor_data.csv");
    std::string line;
    std::string time_stamp_string;
    std::vector<int> time_stamps;
    int time_stamp;

    while (std::getline(file,line)){
        std::stringstream ss(line);
        
        std::getline(ss, time_stamp_string, ',' );

        if ( time_stamp_string == "Timestamp(ms)" ) 
            continue;

        time_stamp = std::stoi(time_stamp_string);
        
        time_stamps.push_back(time_stamp);

    }

    int size = (time_stamps.size())-1;

    int time_stamps_diff[size];

    for(int i =0;  i<size ; i++){

        time_stamps_diff[i] = time_stamps[i+1]-time_stamps[i];

    }

    double frequency  =1/ ((1e-3* (std::accumulate(time_stamps_diff, time_stamps_diff+size , 0.0))) /size);

    std::cout<<"frequency of one cycle is "<< frequency<<std::endl;
    std::cout<<"frequency of single sensors is "<< frequency * 5<<"\n" ;
    std::cout<<"Amount of readings used to calculate this data is " <<size+1<<"\n";

    std::cout<<"Note that this calculations take into account that there is some other operations happen like calculating the diffrernece between sensors and printing the results"<<"\n";


}