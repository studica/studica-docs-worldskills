Installing ROS
==============

Downloading the Base Repo
-------------------------

To get started make sure ROS is installed on your machine:

If you haven't already, run ``installNoetic.sh`` to install ROS Noetic. For more information, visit `ROS Noetic <http://wiki.ros.org/noetic/Installation/Debian>`__

.. code-block:: rst
   
   ./installNoetic.sh

.. important:: Running the ``installNoetic.sh`` script includes all the required tools and dependencies needed for the ROS Package, however this will require some time as the process can take approximately 3 hours. 

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

.. figure:: images/catkin_ws_img.jpg
    :align: center
    :width: 70%

Now clone the ros-dev repo into the ``src`` folder.

.. code-block:: rst
   
   git clone <REPO LINK>



