Barcode Python Script
=====================

The script for reading a barcode is quite simple and easy to use. There are five sections to the script, and each is explained below. 

Imports
-------

.. code-block:: python 
    :linenos:
    :lineno-start: 2

    import cv2 as cv
    from pyzbar.pyzbar import decode

Lines ``2`` and ``3`` are the imports required for this script. Line ``2`` imports open cv, which is used for taking the picture with the camera. Line ``3`` imports pyzbar, which is used for decoding the snapshot taken by the camera.

Variable Initialization
-----------------------

The ``BarcodeData`` and ``BarcodeType`` variables need to be initialized to prevent an error if no barcode is found.

.. code-block:: python
    :linenos:
    :lineno-start: 6

    BarcodeData = 'No Barcode Found'
    BarcodeType = 'null'

Snapping an Image 
-----------------

This section will snap a single image from the camera.

.. code-block:: python
    :linenos:
    :lineno-start: 10

    cap = cv.VideoCapture(0)
    cap.set(3, 640) # Width
    cap.set(4, 480) # Height
    ret, raw = cap.read()
    raw = cv.flip(raw, -1)
    cap.release()

- Line ``10`` creates the camera and is using port **0** 
- Line ``11`` sets the Width 
- Line ``12`` sets the Height
- Line ``13`` snaps the image
- Line ``14`` rotates the image 180 as the way the camera is mounted it is upside down 
- line ``15`` closes the camera resources so that the script is not leaving the camera hanging. 

Barcode Decoding 
----------------

In this section, the image will be decoded to reveal barcode data. 

.. code-block:: python
    :linenos:
    :lineno-start: 18

    for barcode in decode(raw):
        BarcodeData = barcode.data.decode("utf-8")
        BarocdeType = barcode.type
        #print("Found {} barcode: {}".format(BarcodeType, BarcodeData)) #For debugging

- Line ``18`` is the for loop that will run through all the found barcodes. 
- Line ``19`` assigns the barcode data to BarcodeData 
- Line ``20`` assigns the barcode type to BarcodeType
- Line ``21`` is used as a debug print statement that will print the results to the terminal. 

.. note:: If there are multiple barcodes, it will only take the last one found, as each loop overrides the previous results. 

Write to File 
-------------

A simple text file is used to pass the barcode data from this script to the watchdog script.

.. code-block:: python
    :linenos:
    :lineno-start: 23

    file = open('/home/pi/barcodes.txt', 'w')
    file.write(BarcodeData)
    file.write('/n')
    file.write(BarcodeType)
    file.close()

- Line ``23`` opens the file in write mode 
- Line ``24`` writes the barcode data to the first line of the file
- Line ``25`` moves the file to the next line
- Line ``26`` writes the barcode type to the second line of the file
- Line ``27`` closes the file. 

Full Script
-----------

.. code-block:: python
    :linenos:

    #imports
    import cv2 as cv
    from pyzbar.pyzbar import decode

    #initialize variables to prevent errors if no barcode found 
    BarcodeData = 'No Barcode Found'
    BarcodeType = 'null'

    # Snap an image
    cap = cv.VideoCapture(0)
    cap.set(3, 640) # Width
    cap.set(4, 480) # Height
    ret, raw = cap.read()
    raw = cv.flip(raw, -1)
    cap.release()

    # process image and output barcode data to file
    for barcode in decode(raw):
        BarcodeData = barcode.data.decode("utf-8")
        BarcodeType = barcode.type
        #print("Found {} barcode: {}".format(barcode.type, barcode.data.decode("utf-8"))) # For debugging
        
    file = open('/home/pi/barcodes.txt', 'w')
    file.write(BarcodeData)
    file.write('\n')
    file.write(BarcodeType)
    file.close()
