Conditional Command 
===================

The ``ConditionalCommand`` will run one command or another based on a condtion that must be met. 

Example
-------

.. tabs::
   
    .. tab:: Java

        .. code-block:: java
            :linenos:

            // Base parameters
            new ConditionalCommand(trueCommand, falseCommand, boolean condition);

            // Use case
            new ConditionalCommand(new DriveForward(), new DriveReverse(), isLimitHit());

    .. tab:: C++

        .. code-block:: c++
            :linenos:

            frc2::ConditionalCommand(trueCommand, falseCommand, [&limit] {return isLimitHit();});
