Setting up the WiFi
===================

WiFi and Ethernet are already set up on the LabVIEW image. However, sometimes the WiFi SSID and password must be changed. 

Download the WiFi Tool Project
------------------------------

The LabVIEW project for modifying the WiFi can be downloaded `here <https://studicalimited.sharepoint.com/:f:/s/SR-Resources/EoNpuC5Lc-9KrmoRBr56GegBApdr93Yy120Kwe3Ip-VBdw?e=dVKDMm>`__.

Connecting to the WiFi
----------------------

When the VMX is powered on, the default WiFi of the LabVIEW image will be active. 

.. figure:: images/wifi-setup-1.png
    :align: center

WiFi Settings

- **SSID**: high-genius
- **PASS**: high-genius

Connect to the WiFi by selecting ``high-genius`` and using ``high-genius`` as the password.

.. figure:: images/wifi-setup-2.png
    :align: center

Changing the WiFi
-----------------

1. Open LabVIEW 2020 Community Edition and select ``File`` -> ``Open Project``.

    .. figure:: images/wifi-setup-3.png
        :align: center

2. Select the ``HG_WSI_Tool xx.lvproj`` that was downloaded above.

    .. figure:: images/wifi-setup-4.png
        :align: center

3. Connect the project to the VMX Target by right-clicking on ``Raspberry Pi (172.16.0.1)`` and selecting ``Connect``.

    .. figure:: images/wifi-setup-5.png
        :align: center

4. A window will pop up showing the project trying to establish a connection. 

    .. figure:: images/wifi-setup-6.png
        :align: center

5. The tiny green dot next to the ``Raspberry Pi (172.16.0.1)`` should now be bright green.

    .. figure:: images/wifi-setup-7.png
        :align: center

6. Click on the plus next to the ``Raspberry Pi (172.16.0.1)`` and double click on ``WIFI Setup.vi``.

    .. figure:: images/wifi-setup-8.png
        :align: center

7. The WiFi Setup vi will open up.

    .. figure:: images/wifi-setup-9.png
        :align: center

8. Change the ``Name`` and ``Password`` to what is required for you. 

    .. important:: The **Name** and **Password** must be more ``8`` digits. 
    
    .. figure:: images/wifi-setup-10.png
        :align: center

    Here the Name / SSID will be set to ``StudicaLabVIEW``, and the password will be set to ``password``.

9. Select the ``Restart after final?`` push button.

    .. figure:: images/wifi-setup-11.png
        :align: center

    This will reboot the VMX after the WiFi has been set. 

10. Hit the run button to execute the change.

    .. figure:: images/wifi-setup-12.png
        :align: center

11. A window will pop up saying the connection has been lost. Hit ``ok``.

    .. figure:: images/wifi-setup-13.png
        :align: center

12. Checking the WiFi again, we can see that the change has occurred.

    .. figure:: images/wifi-setup-14.png
        :align: center

13. To check that everything is still working, repeat steps ``3`` to ``5`` to check that the project can still connect to the VMX.

    .. figure:: images/wifi-setup-7.png
        :align: center

