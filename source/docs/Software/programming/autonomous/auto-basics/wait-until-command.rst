Wait Until Command 
==================

The ``WaitUntilCommand`` is an upgraded version of ``WaitCommand`` as a boolean condition can be added. 

Example
-------

.. tabs::
   
    .. tab:: Java

        .. code-block:: java
            :linenos:

            // Waits 10 seconds
            new WaitUntilCommand(10);

            // Waits for limit switch to be true 
            new WaitUntilCommand(isLimitHit());


    .. tab:: C++

        .. code-block:: c++
            :linenos:

            // Waits 10 seconds
            frc2::WaitUntilCommand(10.0_s);

            // Waits for limit switch to be true
            frc2.WaitUntilCommand([&Limit] {return isLimitHit();});
