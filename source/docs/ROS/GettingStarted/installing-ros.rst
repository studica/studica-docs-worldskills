Installing ROS Manually
=======================

To get started, we need to install ROS on your machine:

1. Open the terminal window and run the following code-block to allow your system to accept software from packages.ros.org. The VMX-pi uses Rasbian, which is a Debian-based operating system for the Raspberry Pi, since ROS Noetic ONLY supports Debian 10 (Buster), later versions like Rasbian Bullseye will not support the Noetic distribution.

.. code-block:: rst
   
   sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
   
2. Set up your keys

.. code-block:: rst
   
   sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

3. Run ``sudo apt update`` to ensure that your Debain package index is up to date.

.. code-block:: rst

   sudo apt update

4. Install the full version of the ROS Noetic distribution, this includes catkin, the official build system of ROS, and tools like the 2D/3D simulators and the ``rqt`` framework.

.. code-block:: rst
   
   sudo apt install ros-noetic-desktop-full
   
.. note:: There are numerous packages available in ROS, access these packages by simply running ``sudo apt install ros-noetic-PACKAGE``, for example ``sudo apt install ros-noetic-turtlesim``. For a list of available ROS Noetic packages, visit the `ROS Index <https://index.ros.org/packages/page/1/time/#noetic>`__.


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

4. Now clone the ros-dev repo into the ``src`` folder.

.. code-block:: rst
   
   git clone <REPO LINK>
   
   
5. Lastly, run ``catkin build`` once again to build the newly cloned repository in the catkin workspace.

.. code-block:: rst
   
   catkin build -cs



