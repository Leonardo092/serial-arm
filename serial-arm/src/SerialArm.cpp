#include "../include/SerialArm.hpp"
#include <unistd.h>
#include <algorithm>
#include <ginac/ginac.h>

SerialArm::SerialArm() 
{
    std::string curr_dir = get_current_dir_name();
    input_filename_ = curr_dir + "/files/dataJoint.csv";
    output_filename_ = curr_dir + "/files/Trajectory.xml";
    sample_counter_ = 0;
    DGM_counter_ = 0;
    publish_counter_ = 1;
    csv_read_ = false;
    xml_created_ = false;
}

SerialArm::~SerialArm() {}

void SerialArm::read_csv()
{
    std::pair<int, std::vector<float>> sample;
	std::vector<float> row;
	std::string line, word;

    int current_row = 0;
 
	std::fstream file;
    file.open(input_filename_);

	if(file.good())
	{
		while(std::getline(file, line))
		{
            std::stringstream str(line);
            row.clear();

            while(std::getline(str, word, ','))
                row.push_back(std::atof(word.c_str()));

            //assign row to the sample structure (timestamp + joints)
            sample.first = (int)(row[0]);
            sample.second.insert(sample.second.begin(), row.begin() + 1, row.end());

            //track current acquisition
            samples_.push_back(sample);

            //clear sample for next acquisition
            sample.second.clear();
        }

        //remove header from acquisitions
        samples_.erase(samples_.begin());

        //mark document as processed
        csv_read_ = true;
	}
	else
		std::cout<<"Could not open the file\n";

    file.close();
}

void SerialArm::xml_output(std::pair<int, std::vector<float>> sample, std::vector<float> ee_sample, std::vector<float> ikm)
{
    tinyxml2::XMLDocument doc;
    const char * path = output_filename_.c_str();

    doc.LoadFile(path);

    tinyxml2::XMLElement * pTop = doc.RootElement();

    tinyxml2::XMLElement * pFirst = pTop->FirstChildElement("Trajectory");

    tinyxml2::XMLElement* pSample = doc.NewElement("Sample");
    pFirst->InsertEndChild(pSample);

    tinyxml2::XMLElement* pTimestamp = doc.NewElement("Timestamp");
    pTimestamp->SetText((std::to_string(sample.first)).c_str());
    pSample->InsertEndChild(pTimestamp);

    tinyxml2::XMLElement* pJoints = doc.NewElement("Joints");
    pJoints->SetAttribute("q1", sample.second[0]);
    pJoints->SetAttribute("q2", sample.second[1]);
    pJoints->SetAttribute("q3", sample.second[2]);
    pJoints->SetAttribute("q4", sample.second[3]);
    pJoints->SetAttribute("q5", sample.second[4]);
    pJoints->SetAttribute("q6", sample.second[5]);
    pSample->InsertEndChild(pJoints);

    tinyxml2::XMLElement* pTarget = doc.NewElement("End-Effector");
    pTarget->SetAttribute("x", ee_sample[0]);
    pTarget->SetAttribute("y", ee_sample[1]);
    pTarget->SetAttribute("z", ee_sample[2]);
    pTarget->SetAttribute("Roll", ee_sample[3]);
    pTarget->SetAttribute("Pitch", ee_sample[4]);
    pTarget->SetAttribute("Yaw", ee_sample[5]);
    pSample->InsertEndChild(pTarget);

    tinyxml2::XMLElement* pIKM = doc.NewElement("IKM-Result");
    pIKM->SetAttribute("q1", ikm[0]);
    pIKM->SetAttribute("q2", ikm[1]);
    pIKM->SetAttribute("q3", ikm[2]);
    pIKM->SetAttribute("q4", ikm[3]);
    pIKM->SetAttribute("q5", ikm[4]);
    pIKM->SetAttribute("q6", ikm[5]);
    pSample->InsertEndChild(pIKM);

    doc.SaveFile(output_filename_.c_str());
}

void SerialArm::DGM(std::vector<float> joints, Eigen::Matrix3f& inv_prismatic_matrix) //computes forward kinematics for each joint acquisition
{
    //vector to store revolute part of prismatic matrices for IKM
    std::vector<Eigen::Matrix3f> IKM_prism_mat;

    //Denavit-Hartenberg parameters
    std::vector<float> alpha = {0, (PI/2), (PI/2), 0, -(PI/2), -(PI/2), 0}; //angle between Zj-1 and Zj about Xj-1
    std::vector<float> d = {0, 0, 0, 0, 0, 0, 0}; //distance between Zj-1 and Zj along Xj-1
    std::vector<float> theta = {0, (PI/2), 0, joints[3], joints[4], joints[5], -(PI/2)}; //angle between Xj-1 and Xj about Zj
    std::vector<float> r = {joints[0], joints[1], joints[2], 0, 0, 0, 0.01}; //distance between Xj-1 and Xj along Zj
    //N.B. last element represents the transformation between the last joint and the end-effector

    //define rotation matrices
    Eigen::Matrix4f transformation_matrix, prismatic_matrix, revolute_matrix;
    Eigen::Matrix4f base_to_frame0;
    base_to_frame0 << 0, 0, 1, 0,
                      0, -1, 0, 0, 
                      1, 0, 0, 0, 
                      0, 0, 0, 1;

    //initialize transformation matrices
    prismatic_matrix = base_to_frame0; //takes into account the base (world) frame
    revolute_matrix = Eigen::Matrix4f::Identity();

    //find intermediate homogeneous transformation and create the base_T_3 matrix
    for(uint i = 0; i < 3; i++)
    {
        Eigen::Matrix4f current_transform;
        current_transform << cos(theta[i]), -sin(theta[i]), 0, d[i],
                             cos(alpha[i])*sin(theta[i]), cos(alpha[i])*cos(theta[i]), -sin(alpha[i]), -r[i]*sin(alpha[i]),
                             sin(alpha[i])*sin(theta[i]), sin(alpha[i])*cos(theta[i]), cos(alpha[i]), r[i]*cos(alpha[i]),
                             0, 0, 0, 1;
        
        prismatic_matrix = prismatic_matrix*current_transform;
        IKM_prism_mat.push_back(prismatic_matrix.block<3, 3>(0, 0)); //extract rotation matrix
    }

    //find intermediate homogeneous transformation and create the 3_T_end matrix
    for(uint i = 3; i < alpha.size(); i++)
    {
        Eigen::Matrix4f current_transform;
        current_transform << cos(theta[i]), -sin(theta[i]), 0, d[i],
                             cos(alpha[i])*sin(theta[i]), cos(alpha[i])*cos(theta[i]), -sin(alpha[i]), -r[i]*sin(alpha[i]),
                             sin(alpha[i])*sin(theta[i]), sin(alpha[i])*cos(theta[i]), cos(alpha[i]), r[i]*cos(alpha[i]),
                             0, 0, 0, 1;
        
        revolute_matrix = revolute_matrix*current_transform;
    }

    //calculate base_T_end matrix
    transformation_matrix = prismatic_matrix*revolute_matrix;

    //Create matrix 3_R_0 for IKM
    std::reverse(IKM_prism_mat.begin(), IKM_prism_mat.end());

    for (int i = 0; i < IKM_prism_mat.size(); i++)
        inv_prismatic_matrix = IKM_prism_mat[i].inverse()*inv_prismatic_matrix;

    //calculate End-effector pose (X,Y,Z,Roll,Pitch,Yaw)
    std::vector<float> end_pose;

    //insert position
    end_pose.push_back(transformation_matrix.coeff(0,3)); //X
    end_pose.push_back(transformation_matrix.coeff(1,3)); //Y
    end_pose.push_back(transformation_matrix.coeff(2,3)); //Z

    //insert orientation (Factor Rx,Ry,Rz)
    if (transformation_matrix.coeff(0,2) < 1)
    {
        if (transformation_matrix.coeff(0,2) > -1)
        {
            end_pose.push_back(atan2(-transformation_matrix.coeff(1,2), transformation_matrix.coeff(2,2))); //Roll
            end_pose.push_back(asin(transformation_matrix.coeff(0,2))); //Pitch
            end_pose.push_back(atan2(-transformation_matrix.coeff(0,1), transformation_matrix.coeff(0,0))); //Yaw
        }
        else
        {
            end_pose.push_back(-atan2(transformation_matrix.coeff(1,0), transformation_matrix.coeff(1,1))); //Roll
            end_pose.push_back(-PI/2); //Pitch
            end_pose.push_back(0); //Yaw
        }
    }

    //track end-effector pose
    std::mutex write_pose;
    write_pose.lock();
    end_poses_.push_back(end_pose);
    write_pose.unlock();
}

void SerialArm::IKM(std::vector<float> ee_pose, Eigen::Matrix3f inv_t_matrix)
{
    //Initialize IKM joint vector
    std::vector<float> joints;

    //define symbolic variables
    GiNaC::symbol q1("q1"), q2("q2"), q3("q3");

    //Denavit-Hartenberg parameters (symbolic)
    std::vector<float> alpha = {0, (PI/2), (PI/2), 0, -(PI/2), -(PI/2), 0}; //angle between Zj-1 and Zj about Xj-1
    std::vector<float> d = {0, 0, 0, 0, 0, 0, 0}; //distance between Zj-1 and Zj along Xj-1

    std::vector<float> theta = {0, (PI/2), 0}; //angle between Xj-1 and Xj about Zj

    std::vector<GiNaC::symbol> r = {q1, q2, q3}; //angle between Xj-1 and Xj about Zj

    //Symbolic translational inverse
    GiNaC::matrix inv_prismatic_matrix, revolute_matrix;
    GiNaC::matrix base_to_frame0 = {{0, 0, 1, 0},
                                    {0, -1, 0, 0}, 
                                    {1, 0, 0, 0}, 
                                    {0, 0, 0, 1}};

    //initialize inverse transformation matrices
    inv_prismatic_matrix = base_to_frame0.inverse(); //takes into account the base (world) frame

    for(uint i = 0; i < 3; i++)
    {
        GiNaC::matrix current_transform;
        current_transform = {{ cos(theta[i]), -sin(theta[i]), 0, d[i] },
                             { cos(alpha[i])*sin(theta[i]), cos(alpha[i])*cos(theta[i]), -sin(alpha[i]), -r[i]*sin(alpha[i]) }, 
                             { sin(alpha[i])*sin(theta[i]), sin(alpha[i])*cos(theta[i]), cos(alpha[i]), r[i]*cos(alpha[i]) },
                             { 0, 0, 0, 1 }};

        inv_prismatic_matrix = current_transform.inverse().mul(inv_prismatic_matrix);
    }

    //Calculate analytical vector of solutions (prismatic joints)
    GiNaC::matrix ee_position = {{ee_pose[0]}, {ee_pose[1]}, {ee_pose[2]}, {1}};
    auto t_anal_solution = inv_prismatic_matrix.mul(ee_position);

    //calculate numerical solution (prismatic joints)
    GiNaC::lst t_eq = {t_anal_solution(0,0) == 0, t_anal_solution(1,0) == 0, t_anal_solution(2,0) == 0};
    GiNaC::lst t_vars = {q1, q2, q3};
    auto t_solution = GiNaC::lsolve(t_eq, t_vars);

    //convoluted way to convert GiNaC expressions to float (no easier way found so far) and store them
    std::stringstream buffer0, buffer1, buffer2;
    buffer0 << t_solution[0] << std::endl;
    buffer1 << t_solution[1] << std::endl;
    buffer2 << t_solution[2] << std::endl;

    //save prismatic joints values
    joints.push_back(std::stof(buffer0.str().substr(4)));
    joints.push_back(std::stof(buffer1.str().substr(4)));
    joints.push_back(std::stof(buffer2.str().substr(4)));

    //Calculate revolute joints
    /* Assumption: given the geometry of the robot, the revolute contribution to the end effector is given from 3_R_6
        Being 3_R_0 expressed as Q = [[Fx, Fy, Fz], [Gx, Gy, Gz], [Hx, Hy, Hz]], we can say:
        3_R_0 (numerical) = 3_R_6 (analytical) ==> 4_R_3 * Q = 4_R_6. 
        From heer, solutions to the joint values are given by the following system of equations:

        1) -Hx*sin(q4) + Hy*cos(q4) = 0 ==> q4 = atan2(Hy, Hx) || q4 = q4 + PI

        2) {-sin(q5) = Hx*cos(q4) + Hy*sin(q4) 
                                                ==> q5 = ata2(Hx*cos(q4) + Hy*sin(q4), Hz)
            {-cos(q5) = Hz

        2) {-sin(q6) = -Fx*sin(q4) + Fy*cos(q4) 
                                                ==> q6 = ata2(Fx*sin(q4) - Fy*cos(q4), Gx*sin(q4) - Gy*cos(q4))
            {-cos(q6) = -Gx*sin(q4) + Gy*cos(q4) 
            
        reference: https://hal.inria.fr/cel-02129939/document */

    auto m3_R_0 = inv_t_matrix_vec_[DGM_counter_-1];

    auto q4 = atan2(m3_R_0(1,2), m3_R_0(0,2));
    auto q5 = atan2((cos(q4)*m3_R_0(0,2) + sin(q4)*m3_R_0(1,2)), m3_R_0(2,2));
    auto q6 = atan2((sin(q4)*m3_R_0(0,0) - cos(q4)*m3_R_0(1,0)), (sin(q4)*m3_R_0(0,1) - cos(q4)*m3_R_0(1,1)));

    //save revolute joints values
    joints.push_back(q4);
    joints.push_back(q5);
    joints.push_back(q6);

    //save IKM joint values
    joints_.push_back(joints);
}


int SerialArm::get_counter()
{
    return publish_counter_;
}


void SerialArm::Producer()
{
    std::cout<<"Start Producer"<<std::endl;

    //Create Inverse Kinematics meaningful matrices
    Eigen::Matrix3f IKM_t_matrix = Eigen::Matrix3f::Identity();

    //start time counter
    auto start_time = std::chrono::high_resolution_clock::now();

    //read from file
    if (!csv_read_)
        read_csv();

    //Virtual wait to simulate acquisition time (we wait as long as the sample timestamp dictates)
    int time_gap = 0;

    if (sample_counter_ == 0)
        time_gap = samples_[sample_counter_].first;
    else
        time_gap = samples_[sample_counter_].first - samples_[sample_counter_ - 1].first;

    while(true)
    {
        if((std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count()) > time_gap)
            break; 
    }

    //calculate Direct Geometric Model
    DGM(samples_[sample_counter_].second, IKM_t_matrix);

    //Store  numerical rotation matrix for IKM
    std::mutex save_matrix;
    save_matrix.lock();
    inv_t_matrix_vec_.push_back(IKM_t_matrix);
    save_matrix.unlock();

    //update reading counter
    sample_counter_++;
}

void SerialArm::Consumer()
{
    std::cout<<"Start Consumer"<<std::endl;

    //calculate Inverse Kinematics
    if(DGM_counter_ < sample_counter_) //it checks DGM is executed before IKM
    {
        IKM(end_poses_[DGM_counter_], inv_t_matrix_vec_[DGM_counter_]);

        //update DGM counter
        DGM_counter_++;
    }
}

void SerialArm::Logger()
{
    //start timer
    auto start_time = std::chrono::high_resolution_clock::now();
    std::cout<<"Start Logger "<<std::endl;

    if(DGM_counter_ > 0) //it checks Producer and Consumer ran at least once
    {
        //temporary target
        auto ee_sample = end_poses_[DGM_counter_-1];
        auto j_sample = samples_[DGM_counter_-1];
        auto ikm_sample = joints_[DGM_counter_-1];

        //set new timestamp
        j_sample.first = 10*publish_counter_;

        //Write to xml
        if(!xml_created_)
        {
            tinyxml2::XMLDocument xmlDoc;

            tinyxml2::XMLElement* pRoot = xmlDoc.NewElement("file");
            xmlDoc.InsertFirstChild(pRoot);

            tinyxml2::XMLElement* pTrajectory = xmlDoc.NewElement("Trajectory");
            pRoot->InsertEndChild(pTrajectory);

            xmlDoc.SaveFile(output_filename_.c_str());

            //update xml status
            xml_created_ = true;
        }

        xml_output(j_sample, ee_sample, ikm_sample);

        //update output counter
        publish_counter_++;

        //Publish every 10 ms
        while(true)
        {
            if((std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count()) > 10)
                break; 
        }
    }
}

void SerialArm::generate_trajectory(int i)
{
    if (i == 0)
        Producer();
    else if (i == 1)
        Consumer();
    else if (i == 2)
        Logger();
    else
        std::cout<<"index out of range"<<std::endl;
}


