Programming the Titan
=====================


Motor Setup
-----------

.. tabs::
   
    .. tab:: Java

        .. code-block:: java
            :linenos:

            //import the TitanQuad Library
            import com.studica.frc.TitanQuad;

            //Create the TitanQuad Object
            private TitanQuad motor;

            //Constuct a new instance
            motor = new TitanQuad(TITAN_CAN_ID, TITAN_MOTOR_NUMBER);
    
        .. note:: ``TITAN_CAN_ID`` is the CAN id for the Titan, by defualt it is 42. ``TITAN_MOTOR_NUMBER`` is the motor port to be used. Valid range is ``0 - 3``, this corresponds to the M0 - M3 on the Titan. 

    .. tab:: C++ (Header)

        .. code-block:: c++
            :linenos:

            //Include the TitanQuad Library
            #include <studica/TitanQuad.h>

            //Constuct a new instance
            private:
                studica::TitanQuad motor{TITAN_CAN_ID, TITAN_MOTOR_NUMBER};
        
        .. note:: ``TITAN_CAN_ID`` is the CAN id for the Titan, by defualt it is 42. ``TITAN_MOTOR_NUMBER`` is the motor port to be used. Valid range is ``0 - 3``, this corresponds to the M0 - M3 on the Titan.
        
    .. tab:: Roscpp
    
        .. code-block:: c++
            :linenos:
            
            //Include the TitanQuad Library
            #include "TitanDriver_ros_wrapper.h"
            
            ros::NodeHandle nh; //internal reference to the ROS node that the program will use to interact with the ROS system
            VMXPi vmx(true, (uint8_t)50); //realtime bool and the update rate to use for the VMXPi AHRS/IMU interface, default is 50hz within a valid range of 4-200Hz
            
            TitanDriverROSWrapper titan(&nh, &vmx);
          
        .. note:: ``TITAN_CAN_ID`` is the CAN id for the Titan, by defualt it is 42. ``TITAN_MOTOR_NUMBER`` is the motor port to be used. Valid range is ``0 - 3``, this corresponds to the M0 - M3 on the Titan.

Setting Motor Speed
-------------------

.. tabs::
   
    .. tab:: Java

        .. code-block:: java
            :linenos:

            /**
             * Sets the speed of a motor
             * <p>
             * @param speed range -1 to 1 (0 stop)
             */
            public void setMotorSpeed(double speed)
            {
                motor.set(speed);
            }

    .. tab:: C++ (Source)

        .. code-block:: c++
            :linenos:

            /**
             * Sets the speed of a motor
             * <p>
             * @param speed range -1 to 1 (0 stop)
             */
            void ClassName::SetMotorSpeed(double speed)
            {
                motor.Set(speed);
            }
            
    .. tab:: Roscpp
    
         .. code-block:: c++
            :linenos:
            
            /**
             * Sets the speed of a motor
             * 
             * @param speed range -1.0 to 1.0 (0 stop)
             */
             
             ros::ServiceClient set_m_speed = nh->serviceClient<vmxpi_ros::MotorSpeed>("titan/set_motor_speed");
             
             vmxpi_ros::MotorSpeed msg;

             msg.request.speed = rightSpeed;
             msg.request.motor = 0;
             set_m_speed.call(msg1);
            
        .. note:: This is a demonstration of calling the motor speed service using the ``set_motor_speed`` server.


Full Example
------------

.. tabs::
   
    .. tab:: Java

        .. code-block:: java
            :linenos:

            package frc.robot.subsystems;

            //Subsystem Base import
            import edu.wpi.first.wpilibj2.command.SubsystemBase;

            //Titan import
            import com.studica.frc.TitanQuad;

            public class Example extends SubsystemBase
            {
                /**
                 * Motors
                 */
                private TitanQuad motor;

                public Example()
                {
                    //Motors
                    motor = new TitanQuad(TITAN_CAN_ID, TITAN_MOTOR_NUMBER);
                }

                /**
                 * Sets the speed of a motor
                 * <p>
                 * @param speed range -1 to 1 (0 stop)
                 */
                public void setMotorSpeed(double speed)
                {
                    motor.set(speed);
                }
            }
            

    .. tab:: C++ (Header)

        .. code-block:: c++
            :linenos:

            #pragma once

            //Include SubsystemBase
            #include <frc2/command/SubsystemBase.h>

            //Include Titan Library
            #include "studica/TitanQuad.h"

            class Example : public frc2::SubsystemBase
            {
                public:
                    Example();
                    void SetMotorSpeed(double speed);

                private:
                    studica::TitanQuad motor(TITAN_CAN_ID, TITAN_MOTOR_NUMBER);
            };
    
    .. tab:: C++ (Source)

            .. code-block:: c++
                :linenos:
    
                //Include Header
                #include "subsystems/Example.h"

                //Constructor
                Example::Example(){}

                /**
                 * Sets the speed of a motor
                 * <p>
                 * @param speed range -1 to 1 (0 stop)
                 */
                void Example::SetMotorSpeed(double speed)
                {
                    motor.Set(speed);
                }        


    .. tab:: Roscpp

            .. code-block:: c++  
                :linenos:
               
                //Include the TitanQuad Library
                #include "TitanDriver_ros_wrapper.h"
                
                double motor1_speed;
                
                // Returns the speed of motor 1
                void motor1_speed_callback(const std_msgs::Float32::ConstPtr& msg)
                {
                   motor1_speed = msg->data;
                }
               
                int main(int argc, char **argv)
                {
                  
                   ros::init(argc, argv, "titan_node");
                  
                   /**
                    * Constructor
                    * Titan's ros threads (publishers and services) will run asynchronously in the background
                    */
                   
                   ros::NodeHandle nh; //internal reference to the ROS node that the program will use to interact with the ROS system
                   VMXPi vmx(true, (uint8_t)50); //realtime bool and the update rate to use for the VMXPi AHRS/IMU interface, default is 50hz within a valid range of 4-200Hz
                  
                   ros::ServiceClient set_m_speed;
                   ros::Subscriber motor1_speed_sub;
                  
                   TitanDriverROSWrapper titan(&nh, &vmx);

                  /**
                   * Sets the speed of a motor
                   * 
                   * @param speed range -1.0 to 1.0 (0 stop)
                   */
                
                   set_m_speed = nh.serviceClient<vmxpi_ros::MotorSpeed>("titan/set_motor_speed");
                  
                   vmxpi_ros::MotorSpeed msg;

                   msg.request.speed = 1.0; //Setting the motor 1 speed to 1.0
                   msg.request.motor = 0;
                   set_m_speed.call(msg1);
                  
                   // Subscribing to Motor 1 speed topic to access the speed data
                   motor1_speed_sub = nh.subscribe("titan/motor1/speed", 1, motor1_speed_callback);
                 
                   ros::spin(); //ros::spin() will enter a loop, pumping callbacks to obtain the latest sensor data
                     
                   return 0;
                }
               
            .. important:: Subscribe to Titan topics to access the data being published and write callbacks to pass messages between various processes.
            
            .. note:: For more information on programming with ROS, refer to: `ROS Tutorials <http://wiki.ros.org/ROS/Tutorials>`__.