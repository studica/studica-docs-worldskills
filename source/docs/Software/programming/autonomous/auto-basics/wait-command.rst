Wait Command
============

The ``WaitCommand()`` is useful for a when a timed wait period is required. 

Example
-------

.. tabs::
   
    .. tab:: Java

        .. code-block:: java
            :linenos:

            // Waits 10 seconds
            new WaitCommand(10);


    .. tab:: C++

        .. code-block:: c++
            :linenos:

            // Waits 10 seconds
            frc2::WaitCommand(10.0_s);
