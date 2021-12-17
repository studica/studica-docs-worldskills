ROS Setup
=========

Installing ROS Manually
-----------------------

Open a terminal window and run the ``installNoetic.sh`` script to install ROS Noetic. For more information, visit `ROS Noetic <http://wiki.ros.org/noetic/Installation/Debian>`__.

.. code-block:: rst

   ./installNoetic.sh

.. note:: Running the ``installNoetic.sh`` script takes approximately 3 hours, this includes most of the required tools and dependencies needed for the ROS Package, however this does not include ``catkin-tools``.

To install the required ``catkin-tools``, run the following command:

.. code-block:: rst

   sudo apt install python3-catkin-tools python3-osrf-pycommon

Using the ROS Image
-------------------

To get started, navigate to the `ROS Image <https://studicalimited.sharepoint.com/:f:/s/SR-Resources/EsU13cdnTWNFkZK6vXhLDhEBh7I-i7Ov-6tFDjWFbTGjOg?e=dBrPiC>`__ and download the `ROSImage.zip` file. Unzip the downloaded zip file and refer to the section on flashing image files to an SD card `here <https://docs.wsr.studica.com/en/latest/docs/VMX/os-image.html>`__.


.. note:: The ``ROSImage`` file includes all the required tools and dependencies needed for the ROS Package, however this will require approximately 4.8Gb of disk space.

Insert the SD card into the VMX-pi and continue with the instructions below.


Creating a ROS Workspace
^^^^^^^^^^^^^^^^^^^^^^^^

1. Create a directory ``catkin_ws`` by running ``mkdir -p ~/catkin_ws/src``, preferrably in ``/home/pi``.

.. code-block:: rst
   
   mkdir -p ~/catkin_ws/src
   
2. Then ``cd catkin_ws`` and run ``catkin init`` to initialize the workspace environment.

.. code-block:: rst
   
   cd catkin_ws
   catkin init
   
3. Next, run ``catkin build``.

.. code-block:: rst
   
   catkin build

This will create two additional ``build`` and ``devel`` folders in the ``catkin_ws`` directory.

.. figure:: images/catkin_ws_img.JPG
    :align: center
    :width: 70%

4. Now clone the VMX-ROS repo into the ``src`` folder.

.. code-block:: rst
   
   git clone https://github.com/studica/VMX-ROS.git
   
   
5. Lastly, run ``catkin build`` once again to build the newly cloned repository in the catkin workspace.

.. code-block:: rst
   
   catkin build -cs



