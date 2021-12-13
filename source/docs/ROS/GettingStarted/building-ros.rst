Building ROS
============

CMakeLists.txt
--------------

The ``CMakeLists.txt`` file is the input to CMake, which is a system that manages the build process in a compiler-independent manner. To access this software, CMakeLists.txt configuration files are created that detail how the code should be built, these in turn generate the standard makefiles for compiling a program on Linux operating systems like the Rasbian for the VMX-pi. In the ``catkin_ws``, the CMakeLists.txt used is a standard CMakeLists.txt file with a few more restrictions.

Structure
^^^^^^^^^

1. Required CMake Version (cmake_minimum_required)

2. Package Name (project())

3. Find other CMake/Catkin packages needed for build (find_package())

4. Message/Service/Action Generators (add_message_files(), add_service_files(), add_action_files())

5. Invoke message/service/action generation (generate_messages())

6. Specify package build info export (catkin_package())

7. Libraries/Executables to build (add_library()/add_executable()/target_link_libraries())

Writing a CMakeLists.txt file
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For the purposes of this demonstration we will use the CMakeLists.txt file for the main ``vmxpi_ros_bringup`` package for reference.

.. code-block:: rst
   :linenos:
   
   cmake_minimum_required(VERSION 3.0.2)
   project(vmxpi_ros_bringup)

   ## Compile as C++11, supported in ROS Kinetic and newer
   # add_compile_options(-std=c++11)

   ## Find catkin macros and libraries
   ## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
   ## is used, also find other catkin packages
   find_package(catkin REQUIRED COMPONENTS
     roscpp
     rospy
     dynamic_reconfigure
     vmxpi_ros
     vmxpi_ros_titan
     vmxpi_ros_cobra
     vmxpi_ros_sharp
     vmxpi_ros_ultrasonic
     vmxpi_ros_navx
     vmxpi_ros_servo
     vmxpi_ros_io

   )


   ###################################
   ## catkin specific configuration ##
   ###################################
   ## The catkin_package macro generates cmake config files for your package
   ## Declare things to be passed to dependent projects
   ## INCLUDE_DIRS: uncomment this if your package contains header files
   ## LIBRARIES: libraries you create in this project that dependent projects also need
   ## CATKIN_DEPENDS: catkin_packages dependent projects also need
   ## DEPENDS: system dependencies of this project that dependent projects also need
   catkin_package(
   #  INCLUDE_DIRS include
   #  LIBRARIES vmxpi_ros_bringup
   #  CATKIN_DEPENDS roscpp rospy
   #  DEPENDS system_lib
   )

   ###########
   ## Build ##
   ###########

   ## Specify additional locations of header files
   ## Your package locations should be listed before other locations
   include_directories(
     include
     ${catkin_INCLUDE_DIRS}
     ../vmxpi_ros_titan/include
     ../vmxpi_ros_navx/include
     ../vmxpi_ros_sensors/vmxpi_ros_cobra/include
     ../vmxpi_ros_sensors/vmxpi_ros_sharp/include
     ../vmxpi_ros_sensors/vmxpi_ros_ultrasonic/include
     ../vmxpi_ros_servo/include
     ../vmxpi_ros_utils/include
     ../vmxpi_ros_io/include
     ../vmxpi_ros/include
     /usr/local/include/vmxpi
   )

   add_library(vmxpi_hal SHARED IMPORTED GLOBAL)
   set_target_properties(vmxpi_hal PROPERTIES IMPORTED_LOCATION "/usr/local/lib/vmxpi/libvmxpi_hal_cpp.so")

   add_library(navx_ros_wrapper SHARED IMPORTED GLOBAL)
   # set_target_properties(navx_ros_wrapper PROPERTIES IMPORTED_LOCATION "/home/pi/catkin_ws/devel/lib/libnavx_ros_wrapper.so")
   set_target_properties(navx_ros_wrapper PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/../../../devel/lib/libnavx_ros_wrapper.so)

   add_library(titandriver_ros_wrapper SHARED IMPORTED GLOBAL)
   # set_target_properties(titandriver_ros_wrapper PROPERTIES IMPORTED_LOCATION "/home/pi/catkin_ws/devel/lib/libtitandriver_ros_wrapper.so")
   set_target_properties(titandriver_ros_wrapper PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/../../../devel/lib/libtitandriver_ros_wrapper.so)

   add_library(titandriver_ros SHARED IMPORTED GLOBAL)
   # set_target_properties(titandriver_ros PROPERTIES IMPORTED_LOCATION "/home/pi/catkin_ws/devel/lib/libtitandriver_ros.so")
   set_target_properties(titandriver_ros PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/../../../devel/lib/libtitandriver_ros.so)

   add_library(cobra_ros SHARED IMPORTED GLOBAL)
   set_target_properties(cobra_ros PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/../../../devel/lib/libcobra_ros.so)

   add_library(sharp_ros SHARED IMPORTED GLOBAL)
   # set_target_properties(sharp_ros PROPERTIES IMPORTED_LOCATION "/home/pi/catkin_ws/devel/lib/libsharp_ros.so")
   set_target_properties(sharp_ros PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/../../../devel/lib/libsharp_ros.so)

   add_library(servo_ros SHARED IMPORTED GLOBAL)
   # set_target_properties(servo_ros PROPERTIES IMPORTED_LOCATION "/home/pi/catkin_ws/devel/lib/libservo_ros.so")
   set_target_properties(servo_ros PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/../../../devel/lib/libservo_ros.so)

   add_library(ultrasonic_ros SHARED IMPORTED GLOBAL)
   # set_target_properties(ultrasonic_ros PROPERTIES IMPORTED_LOCATION "/home/pi/catkin_ws/devel/lib/libultrasonic_ros.so")
   set_target_properties(ultrasonic_ros PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/../../../devel/lib/libultrasonic_ros.so)

   add_library(iowd_ros SHARED IMPORTED GLOBAL)
   # set_target_properties(iowd_ros PROPERTIES IMPORTED_LOCATION "/home/pi/catkin_ws/devel/lib/libiowd_ros.so")
   set_target_properties(iowd_ros PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/../../../devel/lib/libiowd_ros.so)

   add_library(digitalin_ros SHARED IMPORTED GLOBAL)
   # set_target_properties(digitalin_ros PROPERTIES IMPORTED_LOCATION "/home/pi/catkin_ws/devel/lib/libdigitalin_ros.so")
   set_target_properties(digitalin_ros PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/../../../devel/lib/libdigitalin_ros.so)

   add_library(digitalout_ros SHARED IMPORTED GLOBAL)
   # set_target_properties(digitalout_ros PROPERTIES IMPORTED_LOCATION "/home/pi/catkin_ws/devel/lib/libdigitalout_ros.so")
   set_target_properties(digitalout_ros PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/../../../devel/lib/libdigitalout_ros.so)

   add_executable(test_node src/test_node.cpp)
   target_link_libraries(test_node PRIVATE
      vmxpi_hal 
      navx_ros_wrapper 
      titandriver_ros 
      titandriver_ros_wrapper 
      cobra_ros 
      sharp_ros
      servo_ros 
      ultrasonic_ros
      iowd_ros
      digitalin_ros
      digitalout_ros
      ${catkin_LIBRARIES}
     
   ) 
   add_dependencies(test_node 
      navx_ros_wrapper 
      titandriver_ros 
      titandriver_ros_wrapper 
      cobra_ros 
      sharp_ros
      servo_ros 
      ultrasonic_ros
      iowd_ros
      digitalin_ros
      digitalout_ros
      ${PROJECT_NAME}_gencfg)


   add_executable(main_node src/main.cpp)
   target_link_libraries(main_node PRIVATE
      vmxpi_hal 
      navx_ros_wrapper 
      titandriver_ros 
      titandriver_ros_wrapper 
      cobra_ros 
      sharp_ros
      servo_ros 
      ultrasonic_ros
      iowd_ros
      digitalin_ros
      digitalout_ros
      ${catkin_LIBRARIES}
   )
   add_dependencies(main_node 
      navx_ros_wrapper 
      titandriver_ros 
      titandriver_ros_wrapper 
      cobra_ros 
      sharp_ros
      servo_ros 
      ultrasonic_ros
      iowd_ros
      digitalin_ros
      digitalout_ros
      ${PROJECT_NAME}_gencfg)
      
Explaining the File
~~~~~~~~~~~~~~~~~~~

1. Before starting any CMakeLists.txt file, the first thing to add is the version of CMake. Catkin requires version 2.8.3 or higher.

.. code-block:: rst
   
   cmake_minimum_required(VERSION 3.0.2)
   
2. The next section is specifying the package name using the CMake ``project()`` function, here is where the ``vmxpi_ros_bringup`` package is declared. In CMake, the project name can be referenced using the ``${PROJECT_NAME}`` variable.

.. code-block:: rst
   
   project(vmxpi_ros_bringup)

3. Using the CMake ``find_package()`` function, we specify the packages that the project needs to find before building. ``catkin REQUIRED`` must be passed to this function, from the code-block below, there are other dependencies added to this package such as ``roscpp``, ``rospy``, and the various other packages in Studica's ROS library needed for this wrapper package. Note, the "wet" packages must be turned into components of catkin using the ``COMPONENTS`` argument.

.. code-block:: rst
   
   find_package(catkin REQUIRED COMPONENTS
     roscpp
     rospy
     dynamic_reconfigure
     vmxpi_ros
     vmxpi_ros_titan
     vmxpi_ros_cobra
     vmxpi_ros_sharp
     vmxpi_ros_ultrasonic
     vmxpi_ros_navx
     vmxpi_ros_servo
     vmxpi_ros_io

   )
   
When a package is found following the function call, this leads to the generation of environment variables  that can be utilized later in the CMake script. The environment variables indicate the locations of the headers and source files for the packages, the libraries that the package depends on, as well as the path to those libraries. The naming convention follows <PACKAGE NAME>_<PROPERTY>, for example:

- <NAME>_FOUND - Set to true if the library is found, otherwise false

- <NAME>_INCLUDE_DIRS or <NAME>_INCLUDES - The include paths exported by the package

- <NAME>_LIBRARIES or <NAME>_LIBS - The libraries exported by the package

Remember, catkin packages are not components of catkin, they must be specified as compnents using CMake's components feature to save time. Calling ``find_package()`` on catkin packages is beneficial since their files, paths, and libraries are added as catkin_variables as mentioned earlier.

4. The catkin_package macro generates cmake config files for your package. This is required to declare things to be passed to dependent projects. Note, this function must be called before the ``add_library()`` or ``add_executable()``.

.. code-block:: rst
   
   catkin_package(
   #  INCLUDE_DIRS include
   #  LIBRARIES vmxpi_ros_bringup
   #  CATKIN_DEPENDS roscpp rospy
   #  DEPENDS system_lib
   )
   
- INCLUDE_DIRS - The exported include paths (i.e. cflags) for the package

- LIBRARIES - The exported libraries from the project

- CATKIN_DEPENDS - Other catkin projects that this project depends on

- DEPENDS - Non-catkin CMake projects that this project depends on.

Uncommenting the lines in the code-block above, this indicates that exported headers go in the include folder of the package. We know the ``${PROJECT_NAME}`` variable is the value passed in the ``project()`` fucntion from before, ``roscpp`` and ``rospy`` are packages needed in order to build/run this package, and finally the package depends on ``system_lib``.

5. Specify additional locations of header files, the current packages ``/include/`` directory should be listed before other ``/include`` locations.

.. code-block:: rst

   include_directories(
     include
     ${catkin_INCLUDE_DIRS}
     ../vmxpi_ros_titan/include
     ../vmxpi_ros_navx/include
     ../vmxpi_ros_sensors/vmxpi_ros_cobra/include
     ../vmxpi_ros_sensors/vmxpi_ros_sharp/include
     ../vmxpi_ros_sensors/vmxpi_ros_ultrasonic/include
     ../vmxpi_ros_servo/include
     ../vmxpi_ros_utils/include
     ../vmxpi_ros_io/include
     ../vmxpi_ros/include
     /usr/local/include/vmxpi
   )

6. The add_library() CMake function is used to specify libraries to build, the ``SHARED IMPORTED GLOBAL`` arguments set the type of library to be created. For non-Windows platforms like Rasbian, the primary library file for a ``SHARED`` library is the ``.so`` file, the ``GLOBAL`` option extends the scope of the target (first argument) in the directory it is created and beyond.

.. code-block:: rst

   add_library(vmxpi_hal SHARED IMPORTED GLOBAL)
   
7. Imported targets are used to convert files outside of a CMake project into logical targets inside of the project. The ``set_target_properties()`` function gives the ability to set the properties of the target depending on the options passed after the target. Here, the imported location of the target is pointed to the imported target ``libvmxpi_hal_cpp.so`` file created earlier via ``add_library()`` in ``/usr/local/lib/vmxpi/libvmxpi_hal_cpp.so``.

.. code-block:: rst

   set_target_properties(vmxpi_hal PROPERTIES IMPORTED_LOCATION "/usr/local/lib/vmxpi/libvmxpi_hal_cpp.so")
   
8. Specify an executable target to be built with the ``add_executable()`` function. 

.. code-block:: rst
   
   add_executable(test_node src/test_node.cpp)
   
9. Set the libraries that an executable target links against using the ``target_link_libraries``. The ``PRIVATE`` option indicates that all the following will be used for the current target only, meaning the ``test_node`` target is linked against the shared libraries (``.so`` since Rasbian is Linux-based) of the other packages.

.. code-block:: rst

   target_link_libraries(test_node PRIVATE
      vmxpi_hal
      navx_ros_wrapper
      titandriver_ros
      titandriver_ros_wrapper
      cobra_ros
      sharp_ros
      servo_ros
      ultrasonic_ros
      iowd_ros
      digitalin_ros
      digitalout_ros
      ${catkin_LIBRARIES}

10. Add dependencies using ``add_dependencies`` to the target (``test_node``) defined in the ``add_executable()`` call prior, this is done for targets that depend on other targets than need messages, services, and actions to be built. Essentially, messages from other packages inside the catkin workspace need a dependency added to their generation targets, this is often the case as one of the primary uses of ROS is this message-passing aspect between packages.

.. code-block:: rst

   add_dependencies(test_node
      navx_ros_wrapper
      titandriver_ros
      titandriver_ros_wrapper
      cobra_ros
      sharp_ros
      servo_ros
      ultrasonic_ros
      iowd_ros
      digitalin_ros
      digitalout_ros
      ${PROJECT_NAME}_gencfg)

11. The macros ``add_message_files(...)``, ``add_service_files(...)`, ``add_action_files(...)``, ``generate_messages(...)`` were not included in the example for the ``vmxpi_ros_bringup`` package, but the functions must come BEFORE the ``catkin_package`` macro in this order:
 
.. code-block:: rst

   find_package(catkin REQUIRED COMPONENTS ...)
   add_message_files(...)
   add_service_files(...)
   add_action_files(...)
   generate_messages(...)
   catkin_package(...)

``add_message_files(...)``, ``add_service_files(...)`, ``add_action_files(...)`` handle messages, services, and actions respectively, followed by a call to invoke generation:

.. code-block::

   generate_messages(...)

.. note:: It is important to adhere to the structure of the CMakeLists.txt file as outlined above. Refer to `CMakeLists.txt <http://wiki.ros.org/catkin/CMakeLists.txt>`__ for more information.

Configuring CMakeLists.txt
^^^^^^^^^^^^^^^^^^^^^^^^^^

The previous section analyzed the major sections of a ``CMakeLists.txt`` file, luckily most of the work is already done when the repository is cloned. The main thing to remember when it is time to build your programs are to generate executables, set dependencies, and set libraries to link the target against. To do this, add the following lines at the end of the ``vmxpi_ros_bringup`` CMakeLists.txt file:

.. code-block:: rst

   add_executable(...)
   target_link_libraries(...)
   add_dependencies(...)
   
.. note:: The CMakeLists.txt file has already been configured to build the ``main_node`` executable with all the currently available packages in Studica's ROS library, hence you can simply begin writing your program in ``main.cpp``.