Constants
=========

The ``Constants`` class will hold the two constants required for the Motor definitions on the Titan Quad Motor Controller.

.. tabs::
   
    .. tab:: Java

        .. code-block:: java
            :linenos:

            package frc.robot;

            public final class Constants
            {
                /**
                 * Motor Constants
                 */
                public static final int TITAN_ID        = 42;
                public static final int MOTOR           = 2;
            }

- The constant ``TITAN_ID`` is the CAN ID for the Titan Quad. **Out of box the ID is 42**
- The constant ``MOTOR`` is the motor port on the Titan Quad that the motor is attached to. In this case that is M2. 