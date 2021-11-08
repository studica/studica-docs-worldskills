Deploying To The Robot
======================

A few steps will be followed to deploy the code to the **VMX** / **Robot**.

Ensure Target is set to VMX
---------------------------

.. important:: The VMX must be connected to the internet for this step to work!

Using the ``WSR VMX`` extension, the command ``VMX WSR: Set the deploy target to VMX (from RoboRIO)`` will be used.

This will ensure that the project is configured for the VMX and run a build to download and cache the correct files for the VMX. 

Connect to the VMX WiFi AP
--------------------------

Once the development computer is connected to the VMX via the VMX WiFi AP, the code can be deployed to the VMX. To deploy the code us the extension ``WPILib`` and use the command ``WPILib: Deploy Robot Code``. The command will deploy the compiled code onto the VMX for the robot manager to run. 

Team Number is not 1234
^^^^^^^^^^^^^^^^^^^^^^^

If the VMX team number is not 1234, the team number will have to be set correctly. To set the team number correctly, use the ``WPILib`` extension and the command ``WPILib: Set Team Number``. The command palette will ask for a team number to be entered, and then save the team number hit ``Enter``. It is a good idea to rebuild the project code if the team number is changed. To rebuild the project code us the extension ``WPILib`` and the command ``WPILib: Build Robot Code``

