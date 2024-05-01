#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "tinyxml2.h"
#include <tf/transform_datatypes.h>
#include <cmath>

struct Point {
    double x;
    double y;
    double theta;
};
struct Vector {
    double x;
    double y;
};

double quatx = 0;
double quaty = 0;
double quatz = 0;
double quatw = 0;

// Declaracion de variables
std::vector<float> laser_ranges; // Los valores queda el laser son float
std::vector<double> vector_vff = {0.0, 0.0};
std::vector<double> target_vector = {0.0, 0.0};
std::vector<double> repulsion_vector = {0.0, 0.0};
Point arrayOfPoints[4];
Point robotPosicion;
double alpha = 0.1; // Modificar el valor segun necesario.
double roll, pitch, yaw;
double error_orientation = 0;
double error_distance = 0;
double threshold = 0.1;
float linearposx;
float linearposy;
// robotPosicion =
void odometryCallback(const nav_msgs::OdometryConstPtr& msg){
	// std::cout << "Position: {x:" << msg->pose.pose.position.x << ", y:" << msg->pose.pose.position.y << ", w:"<< msg->pose.pose.orientation.w << "}" << std::endl;
	robotPosicion.x = msg->pose.pose.position.x;
	robotPosicion.y = msg->pose.pose.position.y;
	// point_z = msg->pose.pose.position.z;
	// orien_w = msg->pose.pose.orientation.w;

    linearposx=msg->pose.pose.position.x;
    linearposy=msg->pose.pose.position.y;
    quatx= msg->pose.pose.orientation.x;
    quaty= msg->pose.pose.orientation.y;
    quatz= msg->pose.pose.orientation.z;
    quatw= msg->pose.pose.orientation.w;

    tf::Quaternion q(quatx, quaty, quatz, quatw);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, robotPosicion.theta);
    robotPosicion.theta *= 180/M_PI; // Convierte a grados
    // ROS_INFO("Roll: %f, Pitch: %f, Yaw: %f", roll, pitch, yaw);
    ROS_INFO("RobotPostion: (%f,%f,%f)",robotPosicion.x,robotPosicion.y,robotPosicion.theta); // Funciona bien
}
// Para ver las medidas debe existir el TOPIC del laser, para ello usar la GUI del simulador.
// Asumiendo un eje cartesiano  cuyo angulo cero esta al ESTE y aumenta en sentido antihorario,
// en este robot, el valor cero apunta al valor que esta a la espalda de donde apunta el robot.
// Por lo tanto, la posicion del array con la distancia en el angulo de apuntamiento del robot
// es 180.
void laserCallback(const sensor_msgs::LaserScanPtr& msg){
    
    // ROS_INFO("Longitud del laser de ROS: %ld",msg->ranges.size());
    // Redimensionamos el tamano del vector para evitar problemas.
    laser_ranges.resize(msg->ranges.size());
    // Guarda el array de distancias. El anguulo de appuntamiento esta en la posicion 180.
    laser_ranges = msg->ranges;
    // ROS_INFO("Longitud del vector laser_ranges: %ld",laser_ranges.size());
    // // Imprimir los valores del laser
    // ROS_INFO("Valores del laser:");
    // for (int i = 0; i < msg->ranges.size(); ++i){
    //     ROS_INFO("Índice %d: %f", i, msg->ranges[i]);
    // }
}

void aplicarMatrizGiro(std::vector<double> &vector){
    vector[0] = std::cos(robotPosicion.theta)*vector[0] - std::sin(robotPosicion.theta)*vector[1];
    vector[1] = std::sin(robotPosicion.theta)*vector[0] + std::cos(robotPosicion.theta)*vector[1];
}

void calculaTargetVector(Point &robotPosic, Point &robotTarget){
    target_vector[0] = robotTarget.x - robotPosic.x;
    target_vector[1] = robotTarget.y - robotPosic.y;
    ROS_INFO("El TargetVector ORIGINAL es: (%f,%f)",target_vector[0],target_vector[1]);
    aplicarMatrizGiro(target_vector);
    ROS_INFO("El TargetVector GIRADO es: (%f,%f)",target_vector[0],target_vector[1]);
}

void calculaObstacleVector(Point &robotTarget, std::vector<float> &laser_ranges){
    if (laser_ranges.size() < 135) {
    // ROS_INFO("La longitud del laser es: %ld",laser_ranges.size());
    // ROS_WARN("laser_ranges no tiene suficientes elementos.");
    return;
    }
    // ------------CARTESIANAS------------
    repulsion_vector[0] = 0;
    float angle = -45*M_PI/180;
    for(int i=135; i<225; i++){ // Probar con los 360 valores a ver si mejora  la repulsion.
        repulsion_vector[0] += 1/laser_ranges[i]*cos(angle);
        repulsion_vector[1] += 1/laser_ranges[i]*sin(angle);
        angle += M_PI/180; 
    } 

    // ------------POLARES------------
    // repulsion_vector[0] = 0; // Inicializamos el valor del vector
    // float angle = 135;
    // for(int i=135; i<225; i++){ // Probar con los 360 valores a ver si mejora  la repulsion.
    //     repulsion_vector[0] += 1/laser_ranges[i];
    //     repulsion_vector[1] += angle;
    //     angle += M_PI/180;
    // }
    ROS_INFO("El vector de REPULSION en polares es: (%f,%f)", repulsion_vector[0], repulsion_vector[1]);
}

void calculateDirectionVector(Point &robotPosicion, Point &robotTarget, std::vector<float> &laser_ranges, double &alpha){
    calculaTargetVector(robotPosicion, robotTarget); // OK
    calculaObstacleVector(robotPosicion, laser_ranges); 
    // vector_vff[0] = target_vector[0] + repulsion_vector[0] * alpha; // quitar el alpha, no afecta al movimiento, variarlo segun mapa
    // vector_vff[1] = target_vector[1] + repulsion_vector[1] * alpha;
    
    // Nueva version:
    vector_vff[0] = target_vector[0] + repulsion_vector[0];
    vector_vff[1] = target_vector[1] + repulsion_vector[1];
}

void try_move(Point robotPosic, geometry_msgs::Twist &speed, Point &robotTarget){
    std::cout << "--------------------------------------" << std::endl;
    error_orientation = atan2((robotTarget.y-robotPosic.y),(robotTarget.x-robotPosic.x))*180/M_PI - robotPosic.theta;
    error_distance = sqrt(pow((robotTarget.x-robotPosic.x),2) + pow((robotTarget.y-robotPosic.y),2));
    // ROS_INFO("El punto objetivo es: (%f,%f)",robotTarget.x,robotTarget.y);
    // ROS_INFO("La posicion theta es: %f",robotPosic.theta);
    // ROS_INFO("El angulo objetivo es: %f",atan2((robotTarget.y-robotPosic.y),(robotTarget.x-robotPosic.x))*180/M_PI);
    // ROS_INFO("Error orientation: %f --- Error distance: %f",error_orientation,error_distance);
    if(error_orientation > 3){
        speed.angular.z = 0.05; // Revisar si esto afecta al tambaleo del robot
        speed.linear.x = 0;
        // ROS_INFO("Entra en el Primer IF");
    }
    else if(error_distance > 1){
        speed.angular.z = 0;
        speed.linear.x = 0.1;
        // ROS_INFO("Entra en el segundo IF");
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


    // Imprimir el vector de obstaculos


    // ========================================================
    // Empieza el bucle WHILE
    // ========================================================
    
    ros::Rate loop(1); // Ejecuta a hercios
	while(ros::ok()){ // Espera a que el master este listo para comunicarse
		geometry_msgs::Twist speed;
		//geometry_msgs::Pose position;
        
        // Genera ERROR de SegmentationFault
        try_move(robotPosicion,speed,arrayOfPoints[0]);
		speed_pub.publish(speed); // Quitar

        // Aqui va la funcion que calcula el vector VFF
        calculateDirectionVector(robotPosicion,arrayOfPoints[0],laser_ranges,alpha);
        // if(repulsion_vector[0] > threshold){ // threshold varia si es con robot real o no
        //     speed.linear.x = 0.2;
        //     speed.angular.z = vector_vff[0] * alpha; // revisar si va el 0 o el 1.
        // }

        // Finalmente actualiza la velocidad
        // speed_pub.publish(speed);

		ros::spinOnce();
		loop.sleep();
	}

    return 0;
}
