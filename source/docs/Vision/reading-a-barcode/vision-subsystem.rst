Vision Subsystem
================

The Vision Subsystem will house the core of the code. It is always a good idea to have a Vision subsystem that will hold all the getters and setters for the vision on the robot. 

VisionSubsystem.java
--------------------

Imports
^^^^^^^

.. code-block:: java
    :linenos:

    package frc.robot.subsystems;

    import edu.wpi.first.networktables.NetworkTable;
    import edu.wpi.first.networktables.NetworkTableEntry;
    import edu.wpi.first.networktables.NetworkTableInstance;
    import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
    import edu.wpi.first.wpilibj2.command.SubsystemBase;

- Line ``1`` is the package setup, and will assign this class to the package. 
- Line ``3`` is the import for networktables; this allows us to talk with the vision Scripts.
- Line ``4`` is the import for network table entries and allows us to read the data from the table.
- Line ``5`` is the import for the networktables instance and is used to get the table.
- Line ``6`` is the import for SmartDashboard, which will be used to put values for user display.
- Line ``7`` is the import for SubsystemBase, which is required to have this class become a subsystem.

Class
^^^^^

.. code-block:: java
    :linenos:
    :lineno-start: 9

    public class VisionSubsystem extends SubsystemBase

- Line ``9`` creates the class ``VisionSubsystem`` and extends SubsystemBase to have this class be a subsystem.

Objects
^^^^^^^

.. code-block:: java
    :linenos:
    :lineno-start: 11

    private NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private NetworkTable table = inst.getTable("Vision");
    private NetworkTableEntry data;

- Line ``11`` creates the NetworkTableInstance
- Line ``12`` creates the table 
- Line ``13`` creates the table entry reference

Constructor
^^^^^^^^^^^

.. code-block:: java
    :linenos:
    :lineno-start: 15

    public VisionSubsystem()
    {
        SmartDashboard.putBoolean("Get New Barcode", false);
    }

- Line ``15`` is the constructor and will create the VisionSubsystem when called.
- Line ``17`` Will create an entry in the smartdashboard called ``Get New Barcode`` and sets its default value to **false**.

Setter
^^^^^^

.. code-block:: java
    :linenos:
    :lineno-start: 20

    public void readBarcode()
    {
        table.getEntry("readBarcode").setBoolean(true);
    }

- Line ``20`` is the setter that is called when a new barcode should be read.
- Line ``22`` will update the readBarcode flag in networktables to **true**. This, in turn, will tell the vision scripts to read a new barcode and update the data keys.

Getter 
^^^^^^

.. code-block:: java
    :linenos:
    :lineno-start: 25

    public void printBarcode()
    {
        data = table.getEntry("barcodeData");
        SmartDashboard.putString("Barcode Data", data.getString("Nothing was read"));
    }

- Line ``25`` is the method that will be called to get the current value of the **barcodeData** entry.
- Line ``27`` assigns the entry to the **data** object.
- Line ``28`` places the string value of **data** to the dashboard. 

Periodic Loop
^^^^^^^^^^^^^

The periodic loop is used to check the current value of the **barcodeData** entry for every robot loop. 

.. code-block:: java 
    :linenos:
    :lineno-start: 31

    @Override
    public void periodic()
    {
        printBarcode();
    }

- Line ``31`` is the required Override to tell the compiler to use this periodic method and not the one built into SubsystemBase.
- Line ``32`` is the periodic method.
- Line ``34`` will call the ``printBarcode`` method every robot loop.


Full Subsystem Code
-------------------

.. code-block:: java
    :linenos:

    package frc.robot.subsystems;

    import edu.wpi.first.networktables.NetworkTable;
    import edu.wpi.first.networktables.NetworkTableEntry;
    import edu.wpi.first.networktables.NetworkTableInstance;
    import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
    import edu.wpi.first.wpilibj2.command.SubsystemBase;

    public class VisionSubsystem extends SubsystemBase
    {
        private NetworkTableInstance inst = NetworkTableInstance.getDefault();
        private NetworkTable table = inst.getTable("Vision");
        private NetworkTableEntry data;

        public VisionSubsystem()
        {
            SmartDashboard.putBoolean("Get New Barcode", false);
        }

        public void readBarcode()
        {
            table.getEntry("readBarcode").setBoolean(true);
        }

        public void printBarcode()
        {
            data = table.getEntry("barcodeData");
            SmartDashboard.putString("Barcode Data", data.getString("Nothing was read"));
        }

        @Override
        public void periodic()
        {
            printBarcode();
        }
    }
    

