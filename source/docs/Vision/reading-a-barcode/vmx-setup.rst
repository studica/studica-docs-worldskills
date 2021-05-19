Setup Dependencies on the VMX
=============================

Before anything can be done, some packages and dependencies must be installed. 

Switching to WiFi Client Mode
-----------------------------

To install packages, the VMX must be connected to the internet. The easiest way to do this is to put the VMX into client mode. Open Terminal and run the command below to enter client mode. 

.. code-block:: text

    setupWifiClient.sh 

This will change the VMX from an access point to a client. In client mode, the VMX can then be connected to your local WiFi. 

.. note:: The robot manager and connection to the control station does not work in this mode. 

Packages to Install
-------------------

Three packages need to be installed.

Pyzbar
^^^^^^

This package is used to read the barcode data. In the terminal, run the following commands:

.. code-block:: text

    sudo apt-get install libzbar0
    pip install pyzbar 
    pip install pyzbar[scripts]

pynetworktables
^^^^^^^^^^^^^^^

This package is what is used to communicate with the robot and shuffleboard. 

.. code-block:: text

    pip install pynetworktables

Watchdog
^^^^^^^^

This package acts as a watchdog that is used to check filesystem changes. The main reason for this package is that currently, Pyzbar and pynetworktables do not interact appropriately. 

.. code-block:: text

    pip install watchdog


Going back to WiFi AP Mode
--------------------------

After these packages are installed, it is good to go back to WiFi AP mode to prevent issues down the line.

.. code-block:: text

    setupWifiAP.sh