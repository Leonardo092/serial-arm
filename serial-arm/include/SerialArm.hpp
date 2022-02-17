#ifndef SERIAL_ARM_H
#define SERIAL_ARM_H

#include <math.h>
#include <cmath>
#include <string>
#include <vector>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <mutex>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include "tinyxml2.h"

const float PI = 3.14159265359;

class SerialArm
{
    private:

        //members
        std::vector<std::vector<float>> end_poses_;
        std::vector<std::vector<float>> joints_;
        std::vector<std::pair<int, std::vector<float>>> samples_; //tracks joint value acquisition
        std::vector<Eigen::Matrix3f> inv_t_matrix_vec_; //stores meaningful matrices from DGM to IKM
        std::string input_filename_, output_filename_; 
        int sample_counter_, DGM_counter_, publish_counter_;
        bool xml_created_, csv_read_; //read-write flags
        tinyxml2::XMLDocument xmlDoc;

        Eigen::Matrix3f m4R3;

        //methods
        void read_csv();
        void xml_output(std::pair<int, std::vector<float>>, std::vector<float>, std::vector<float>);
        void DGM(std::vector<float>, Eigen::Matrix3f&);
        void IKM(std::vector<float>, Eigen::Matrix3f);
        void Producer();
        void Consumer();
        void Logger();

    public:

        //Constructor
        SerialArm();

        //Desctructor
        ~SerialArm();

        //members

        //methods
        void generate_trajectory(int);
        int get_counter();
};

#endif /* SERIAL_ARM_H */