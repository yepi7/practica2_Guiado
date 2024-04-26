#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "cmath.h"
#include "tinyxml2.h"
#include <tf/transform_datatypes.h>


struct Point {
    double x;
    double y;
    double theta;
};
struct Vector {
    double x;
    double y;
};

double quatx;
double quaty;
double quatz;
double quatw;

// Declaracion de variables
std::vector<float> laser_ranges;
std::vector<double> vector_vff, target_vector, repulsion_vector;
Point arrayOfPoints[4];
Point robotPosicion;
double alpha = 0.1; // Modificar el valor segun necesario.
double roll, pitch, yaw;
double threshold = 0.1;
float linearposx;
float linearposy;
// robotPosicion =
void odometryCallback(const nav_msgs::OdometryConstPtr& msg){
	std::cout << "Position: {x:" << msg->pose.pose.position.x << ", y:" << msg->pose.pose.position.y << ", w:"<< msg->pose.pose.orientation.w << "}" << std::endl;
	robotPosicion.x = msg->pose.pose.position.x;
	robotPosicion.y = msg->pose.pose.position.y;
	// point_z = msg->pose.pose.position.z;
	// orien_w = msg->pose.pose.orientation.w;

    linearposx=msg->pose.pose.position.x;
    linearposy=msg->pose.pose.position.y;
    double quatx= msg->pose.pose.orientation.x;
    double quaty= msg->pose.pose.orientation.y;
    double quatz= msg->pose.pose.orientation.z;
    double quatw= msg->pose.pose.orientation.w;

    tf::Quaternion q(quatx, quaty, quatz, quatw);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    ROS_INFO("Roll: %f, Pitch: %f, Yaw: %f", roll, pitch, yaw);
}
// Para ver las medidas debe existir el TOPIC del laser, para ello usar la GUI del simulador.
void laserCallback(const sensor_msgs::LaserScanPtr& msg){
    laser_ranges = msg->ranges;

    // for(auto range : msg->ranges){
    //     int i=0;
    //     std::cout << "LaserScan: " << range << " en la posicion: "<< i <<std::endl;
    //     i++;
    // }
}

void aplicarMatrizGiro(Point punto){
    double xAux, yAux;
    xAux = punto.x;
    yAux = punto.y;
    punto.x = std::cos(punto.theta)*xAux - std::sin(punto.theta)*yAux;
    punto.y = std::sin(punto.theta)*xAux + std::cos(punto.theta)*yAux;
}

void calculaVectorRepulsion(Point robotTarget, std::vector<float> laser_ranges){
    double sensor_meas;
    std::vector<float> vector_obs;
    vector_obs[0];
    for(int i=-45; i<45; i++){
        vector_obs[0] += 1/laser_ranges[i]*sin(robotTarget.theta);
    }
    for(int i=-45; i<45; i++){
        vector_obs[1] += 1/laser_ranges[i]*cos(robotTarget.theta);
    }
}

void calculaTargetVector(Point robotPosic, Point robotTarget){
    Point targetVector;
    targetVector.x = robotPosic.x - robotTarget.x;
    targetVector.y = robotPosic.y - robotTarget.y;
    aplicarMatrizGiro(targetVector);
}

// void calculateDirectionVector(){
//     calculaTargetVector();
//     calculaVectorRepulsion();
//     vector_vff = target_vector + repulsion_vector * alpha;
// }

void try_move(Point robotPosic, geometry_msgs::Twist speed ){
    double error_orientation, error_distance;
    error_orientation = atan2((arrayOfPoints[0].y-robotPosic.y)/(arrayOfPoints[0].x-robotPosic.x));
    error_distance = sqrt(pow((arrayOfPoints[0].x-robotPosic.x),2) + pow((arrayOfPoints[0].y-robotPosic.y),2));
    
    if(error_orientation > 0.01){
        speed.angular.z = 0.1;
        speed.linear.x = 0;
    }
    else if(error_distance > 0.01){
        speed.angular.z = 0;
        speed.linear.x = 0.1;
    }
}

int main(int argc, char** argv){


	ros::init(argc,argv,"firstProj");	
	ros::NodeHandle nh;	

//---------------------Parte del XML----------------------------
    tinyxml2::XMLDocument doc;
    doc.LoadFile("/home/alumno/robotica_movil_ws/src/secondProj/src/puntos.xml");
    if (doc.Error()) {
        std::cout << "Error al cargar el XML!" << std::endl;
        return -1;
    }
    // Obtiene el elemento 'nav-points'
    tinyxml2::XMLElement* navPoints = doc.FirstChildElement("map")->FirstChildElement("nav-points");
    if (!navPoints) {
        std::cout << "No se encontraron puntos de navegación!" << std::endl;
        return -1;
    }
    // Itera sobre cada elemento 'point'
    for (tinyxml2::XMLElement* point = navPoints->FirstChildElement("point"); point != NULL; point = point->NextSiblingElement("point")) {
        int i=0;
        int x,y;
        point->QueryIntAttribute("x", &x);
        point->QueryIntAttribute("y", &y);
        arrayOfPoints[i].x=x;
        arrayOfPoints[i].y=y;
        std::cout << "Punto: X=" << x << ", Y=" << y << std::endl;
    }
//-------------------------Fin del XML-------------------------------------------

	ros::Publisher speed_pub = nh.advertise<geometry_msgs::Twist>("/robot0/cmd_vel",1000);
    ros::Subscriber laser_meas = nh.subscribe("/robot0/laser_0",1000, laserCallback);
    ros::Subscriber odom = nh.subscribe("/robot0/odom",1000, odometryCallback);

    ros::Rate loop(10); // Ejecuta a hercios
	while(ros::ok()){ // Espera a que el master este listo para comunicarse
		geometry_msgs::Twist speed;
		//geometry_msgs::Pose position;
        
        // for(auto range : laser_ranges){
        //     int i=0;
        //     // std::cout << "LaserScan: " << range << " en la posicion: "<< i <<std::endl;
        //     ROS_INFO("rango: %d",range);
        //     i++;
        // }
        // robotPosicion =
        // Genera ERROR de SegmentationFault
        try_move(robotPosicion,speed);
		speed_pub.publish(speed);

        if(repulsion_vector[0] > threshold){
            speed.linear.x = 0.2;
            speed.angular.z = vector_vff[0] * alpha;
        }
        speed_pub.publish(speed);

		ros::spinOnce();
		loop.sleep();
	}

    return 0;
}
