Robot
=====

There are only few changes required in the main ``Robot`` class. 

.. tabs::
   
    .. tab:: Java

        .. code-block:: java
            :linenos:

            import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
            import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
            import frc.robot.commands.auto.DriveMotor;

            /**
             * This function is called once each time the robot enters Disabled mode.
             */
            @Override
            public void disabledInit()
            {
                //Check to see if autoChooser has been created
                if(null == RobotContainer.autoChooser)
                {
                RobotContainer.autoChooser = new SendableChooser<>();
                }
                //Add the default auto to the auto chooser
                RobotContainer.autoChooser.setDefaultOption("Drive Motor", "Drive Motor");
                RobotContainer.autoMode.put("Drive Motor", new DriveMotor());
                SmartDashboard.putData(RobotContainer.autoChooser);
            }

- Lines ``1`` - ``3`` are the required imports.
- Lines ``8`` - ``20`` are the modifications made to the previously empty ``disabledInit()``. 
- Lines ``12`` - ``15`` check if autoChooser has beend created yet. If not then it creates autoChooser as a new SendableChooser.
- Lines ``17`` - ``19`` add the default option to autoChooser and add autoChooser to the smartdashboard.