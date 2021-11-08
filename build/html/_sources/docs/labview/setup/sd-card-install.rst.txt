Installing LabVIEW on VMX
=========================

LabVIEW for VMX requires a different SD-card image than that which comes standard on the VMX. 

Downloading the image
---------------------

.. note:: The image is a ``3.34 GB`` download and can be downloaded `here. <https://studicalimited.sharepoint.com/:f:/s/SR-Resources/EgnfDGDd0rBJrMD_vu-JwIwBssQHTi8erH9I4Ab9AibqwQ?e=t9wGdE>`__


Flashing the image
------------------

Once downloaded a flashing software is required to flash the image to the SD Card. The recommended software to do this is `Etcher <https://www.balena.io/etcher>`__. 

.. important:: It is highly recommended to use a Samsung 32GB or 64GB EVO Plus micro SD card. 

To start flashing the SD card, first plug the SD card into your computer. Open ``Etcher``, you will notice that it has auto-detected the SD card. If it has not detected the SD card, you can manually select and find it. 

.. figure:: images/labview-image-1.png
    :align: center

Hit ``Select image`` and find the ``HG_WSI_X.X.X.X-XXX.zip`` file that was downloaded before. 

.. figure:: images/labview-image-2.png
    :align: center

``Flash`` will now be available. Hit Flash to start flashing the SD card image to the SD card. Note this can take a while depending on your computer. 

.. figure:: images/labview-image-3.png
    :align: center

After flashing, Etcher will automatically start to validate the flash to ensure that the flash was successful. 

.. figure:: images/labview-image-4.png
    :align: center

When complete, the SD card will be auto ejected and can be stuck directly back into the VMX.

.. figure:: images/labview-image-5.png
    :align: center
