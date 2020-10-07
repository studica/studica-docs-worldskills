Adding Vendor Libraries
=======================

Adding a vendor library is simple. Open VS Code then the command palette using ``Ctrl+Shift+P`` or ``F1`` and type the following command ``WPILib: Manage Vendor Libraries``. This will open a list of choices as shown. 

.. figure:: images/adding-vendor-libraries-1.png
   :align: center
   
- *Manage current libraries* - Shows the current libraries installed and allows you to remove them.
- *Check for updates(offline)* - will check if there is an update for a library in the offline folder.
- *Check for updates(online)* - will check if there is an update for a library online. 
- *Install new libraries(offline)* - will install a new library in the offline folder.
- *Install new libraries(online)* - will install a new library from the internet.

For this guide select **Install new libraries(online)**.

There are multiple vendor libraries available. The ones supported on the VMXpi are listed below.

- Kauailabs' NavX Library
   
   https://www.kauailabs.com/dist/frc/2020/navx_frc.json
   
- Studica's Titan Library

   http://dev.studica.com/releases/2020/Studica.json
   
- REVRobotics SparkMax Library

   https://www.kauailabs.com/dist/frc/2020/REVRobotics.json
   
   Enabling the SparkMax requires adding some code to two sections.
   
   .. tabs::
   
      .. tab:: Java
      
         In ``Robot.java`` add the following lines of code where appropriate
         
         .. code-block:: java
            :linenos:
            
            import com.revrobotics.jni.CANSparkMaxJNI;

            @Override
            public void autonomousPeriodic()
            {
               CANSparkMaxJNI.c_SparkMax_SetEnable(true); // Periodically ensure motor controller outputs are enabled
            }

            @Override
            public void teleopPeriodic()
            {
               CANSparkMaxJNI.c_SparkMax_SetEnable(true); // Periodically ensure motor controller outputs are enabled
            }
      
      .. tab:: C++
      
         In ``Robot.cpp`` add the following lines of code where appropriate
         
         .. code-block:: c++
            :linenos:
            
            #include <rev/CANSparkMaxLowLevel.h>
            
            void Robot::AutonomousPeriodic()
            {
               rev::CANSparkMaxLowLevel::SetEnable(true); // Periodically ensure motor controller outputs are enabled
            }

            void Robot::TeleopPeriodic()
            {
               rev::CANSparkMaxLowLevel::SetEnable(true); // Periodically ensure motor controller outputs are enabled
            }
   
- CTRE Phoenix Library

   http://www.kauailabs.com/dist/frc/2020/Phoenix-vmx.json
   
   Enabling a Phoenix controlled device requires adding some code to two sections.
   
   .. tabs::
   
      .. tab:: Java
      
         In ``Robot.java`` add the following lines of code where appropriate
         
         .. code-block:: java
            :linenos:
            
            import com.ctre.phoenix.unmanaged.UnmanagedJNI;

            @Override
            public void autonomousPeriodic()
            {
               UnmanagedJNI.JNI_FeedEnable(100); // Enable Phoenix CAN Devices for 100 Milliseconds
            }

            @Override
            public void teleopPeriodic()
            {
               UnmanagedJNI.JNI_FeedEnable(100); // Enable Phoenix CAN Devices for 100 Milliseconds
            }
            
      .. tab:: C++
      
         In ``Robot.cpp`` add the following lines of code where appropriate  

         .. code-block:: c++
            :linenos:
            
            #include <ctre/phoenix/cci/Unmanaged_CCI.h>

            void AutonomousPeriodic() override
            {
               c_FeedEnable(100); // Enable Phoenix CAN Devices for 100 Milliseconds
            }

            void TeleopPeriodic() override
            {
               c_FeedEnable(100); // Enable Phoenix CAN Devices for 100 Milliseconds
            }
      