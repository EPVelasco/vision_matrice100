#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Dense>

// Variables globales
Eigen::Vector3d gravity_vector;
ros::Publisher new_imu_pub; // Declaración del publisher

// Función de callback para el mensaje de la IMU
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  // Crear una nueva IMU para publicar
  sensor_msgs::Imu new_imu;

  

  // Copiar los datos de orientación y velocidad lineal de la IMU original a la nueva IMU
  new_imu.orientation = msg->orientation;
  new_imu.angular_velocity = msg->angular_velocity;


  new_imu.header = msg->header;
  new_imu.header.stamp = msg->header.stamp;
  // Obtener los datos de la aceleración de la IMU original
  double ax = msg->linear_acceleration.x;
  double ay = msg->linear_acceleration.y;
  double az = msg->linear_acceleration.z;

  // Obtener los datos de los cuaterniones de la IMU
  double qx = msg->orientation.x;
  double qy = msg->orientation.y;
  double qz = msg->orientation.z;
  double qw = msg->orientation.w;

  // Calcular la matriz de rotación a partir de los cuaterniones
  Eigen::Quaterniond q(qw, qx, qy, qz);
  Eigen::Matrix3d rotation_matrix = q.normalized().toRotationMatrix();

  // Ajustar el vector de gravedad según la orientación de la IMU
  gravity_vector = rotation_matrix * Eigen::Vector3d(0, 0, -9.8);

  // Aumentar la aceleración por el vector de gravedad ajustado
  new_imu.linear_acceleration.x = ax + gravity_vector.x();
  new_imu.linear_acceleration.y = ay + gravity_vector.y();
  new_imu.linear_acceleration.z = az + gravity_vector.z();

  // Publicar la nueva IMU
  // (asumiendo que tienes un publisher llamado "new_imu_pub" definido en tu nodo)
  new_imu_pub.publish(new_imu);
}

int main(int argc, char** argv)
{
  // Inicializar el nodo ROS
  ros::init(argc, argv, "imu_gravity_compensation");
  ros::NodeHandle nh;

  ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/dji_sdk/imu", 10, imuCallback);
  new_imu_pub = nh.advertise<sensor_msgs::Imu>("/dji_sdk/imu_gravity", 10);


  ros::spin();

  return 0;
}
