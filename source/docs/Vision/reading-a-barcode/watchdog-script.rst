Watchdog Listener Script 
========================

The watchdog listener monitors changes in the barcodes.txt file and sends those changes back to the robot and shuffleboard. The watchdog listener is also used to read the networktables and see if a new barcode must be read. 

Imports
-------

.. code-block:: python
    :linenos: 
    :lineno-start: 4

    from watchdog.observers import Observer
    from watchdog.events import FileSystemEventHandler
    from networktables import NetworkTables
    from networktables.util import ntproperty
    import threading
    import os

There are a few more imports for this script over the barcode script. 

- Line ``4`` is the observer import and is used to create the listener for a file.
- Line ``5`` is the FileSystemEventHandler which creates functions that check for changes to files
- Line ``6`` is the main NetworkTables import, used to send and receive info from the robot. 
- Line ``7`` is the ntproperty import and is used to create tables and properties. 
- Line ``8`` is the thread import to create threads.
- Line ``9`` is the os import and used for sending commands to the terminal.

Create Barcodes File
--------------------

The watchdog script will be run as a startup script, which makes it a good idea to create the barcodes.txt file if it does not exist to avoid errors when reading the file. 

.. code-block:: python
    :linenos: 
    :lineno-start: 12

    f = open('/home/pi/barcodes.txt', 'w')
    f.close()

- Line ``12`` will create the barcodes.txt if it does not exist
- Line ``13`` closes the file to prevent issues of the file being open when it should be closed

Connect to NetworkTables
------------------------

Before anything can happen, a connection to NetworkTables needs to be established. NetworkTables does not run right away and needs some time for the server and client to start. The below code handles this.

.. code-block:: python
    :linenos:
    :lineno-start: 15

    #Create thread to make sure networktables is connected
    cond = threading.Condition()
    notified = [False]

    #Create a listener 
    def connectionListener(connected, info):
        with cond:
            notified[0] = True
            cond.notify()

    #Instantiate NetworkTables
    NetworkTables.initialize(server="10.12.34.2")
    NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)

    #Wait until connected
    with cond:
        if not notified[0]:
            cond.wait()

The above may look complicated, but it is pretty simple. 

- Line ``16`` creates a conditional thread 
- Line ``17`` creates a boolean array
- Line ``20`` defines a new function call connectionListener; this function listens for a connection and changes the condition when connected. 
- Lines ``21 - 23`` is a statement that when connected to update conditions
- Line ``26`` will initialize a connection 
- Line ``27`` adds a listener to check if connected 
- Lines ``30 - 32`` will hang until a connection is made 

.. important:: Make sure the IP address used in line 26 matches the IP address of the VMX WiFi AP.

Create the Vision Tables 
------------------------

To successfully send data between the watchdog and the robot code, it is good to create a couple of properties to hold this data. The properties can be read on the watchdog side and the robot side. 

.. code-block:: python
    :linenos:
    :lineno-start: 35

    ntBarcodeData = ntproperty('/Vision/barcodeData', "null")
    ntBarcodeType = ntproperty('/Vision/barcodeType', "null")
    ntReadBarcode = ntproperty('/Vision/readBarcode', False)

    #Get Table
    table = NetworkTables.getTable('Vision')

- Lines ``35 & 36`` create the barcode properties. 
- Line ``37`` creates the command property used for executing the barcode script for a new barcode.
- Line ``40`` assigns the ``Vision`` table to a variable for later use. 

The first value of property is the **key** and the second is the default value. The default value also creates the property type. In the above cases ``ntBarcodeData`` & ``ntBarcodeType`` will be **strings**, whereas ``ntReadBarcode`` is a **boolean**.

.. note:: When creating a table, the **key** of the table must always start with a ``/``. 

File System Handler 
-------------------

To optimize the code that it does not open, read, close the code continuously. A FileSystemEventHandler can be used. In this case, the use of watchdog is perfect. This way, nothing will happen unless the file is modified. If there is no modification to the file, i.e., a new barcode is read, it will do nothing. 

.. code-block:: python
    :linenos:
    :lineno-start: 43

    class MyHandler(FileSystemEventHandler):
        def on_modified(self, event):
            try:
                file = open('./barcodes.txt', 'r')
                table.putString('barcodeData', file.readline())
                table.putString('barcodeType', file.readline())
                file.close()
            except:
                pass #when file is not created yet
 
    event_handler = MyHandler()
    observer = Observer()
    observer.schedule(event_handler, path='./barcodes.txt', recursive=False)
    observer.start()

- Line ``43`` creates a class that uses the FileSystemEventHandler
- Line ``44`` creates the function **on_modified** which is an extension of the FileSystemEventHandler. This function will be called when the event handler detects a modification to the file. 
- Line ``45 & 50`` is used to catch errors. 
- Line ``46`` opens the **barcodes.txt** in read mode.
- Line ``47`` will read the first line of the file and add it as the **barcodeData** data. 
- Line ``48`` will read the second line of the file and add it as the **barcodeType** data.
- Line ``49`` closes the file to ensure no issues. 
- Line ``53`` creates the event handler
- Line ``54`` creates the observer
- Line ``55`` configures the observer with the event handler and file to watch 
- Line ``56`` starts the observer thread 

Forever Loop
------------

This script needs to run forever and handle a flag sent from the robot to take a new barcode reading. 

.. code-block:: python
    :linenos:
    :lineno-start: 59

    while(True):
        if table.getBoolean('readBarcode', False) == True:
            table.putBoolean('readBarcode', False)
            os.system('python3 /home/pi/readBarcode.py')
        try:
            pass
        except KeyboardInterrupt:
            observer.stop()

- Line ``59`` is the while loop that never ends
- Line ``60`` checks to see if the robot code is requesting a new barcode scan. 
- Line ``61`` flips the **readBarcode** flag to False to prevent a double read. 
- Line ``62`` runs the ``readBarcode.py`` script
- Lines ``63 - 66`` are used if the script ever wants to end. In this case, a KeyboardInterrupt is required to end. As this script will run as a startup script, the observer should never end. 

Setting the Script to run as at Startup
---------------------------------------

Setting the watchdog script to run at startup is very simple. 

In terminal open rc.local

.. code-block:: bash

    sudo nano /etc/rc.local

Scroll to the bottom with the arrow keys. 

Above ``exit 0`` put:

.. code-block:: bash

    python3 /home/pi/watchdogListener.py &

To save, hit ``CTRL + X`` then ``Y`` and hit ``Enter``. 

.. important:: Including the ``&`` at the end will fork the process to the background and prevent other processes from hanging. 

It is now possible to reboot, and the script will be running. 

.. note:: There will be no indication that the script is running. 

Full Script
-----------

.. code-block:: python
    :linenos:

    #!/usr/bin/python3

    #imports
    from watchdog.observers import Observer
    from watchdog.events import FileSystemEventHandler
    from networktables import NetworkTables
    from networktables.util import ntproperty
    import threading
    import os

    #Create the barcodes file
    f = open('/home/pi/barcodes.txt', 'w')
    f.close()

    #Create thread to make sure networktables is connected
    cond = threading.Condition()
    notified = [False]

    #Create a listener 
    def connectionListener(connected, info):
        with cond:
            notified[0] = True
            cond.notify()

    #Instantiate NetworkTables
    NetworkTables.initialize(server="10.12.34.2")
    NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)

    #Wait until connected
    with cond:
        if not notified[0]:
            cond.wait()

    #Create the vision Table
    ntBarcodeData = ntproperty('/Vision/barcodeData', "null")
    ntBarcodeType = ntproperty('/Vision/barcodeType', "null")
    ntReadBarcode = ntproperty('/Vision/readBarcode', False)

    #Get Table
    table = NetworkTables.getTable('Vision')

    #Create the system handler
    class MyHandler(FileSystemEventHandler):
        def on_modified(self, event):
            try:
                file = open('./barcodes.txt', 'r')
                table.putString('barcodeData', file.readline())
                table.putString('barcodeType', file.readline())
                file.close()
            except:
                pass #when file is not created yet
    
    event_handler = MyHandler()
    observer = Observer()
    observer.schedule(event_handler, path='./barcodes.txt', recursive=False)
    observer.start()

    #The forever loop
    while(True):
        if table.getBoolean('readBarcode', False) == True:
            table.putBoolean('readBarcode', False)
            os.system('python3 /home/pi/readBarcode.py')
        try:
            pass
        except KeyboardInterrupt:
            observer.stop()


