Command Decorators
==================

Command decorators take the base command and add additonal functionalities to it. 

withTimeout()
-------------

Adds a timeout to the command. When the timeout expires the command will be interrupted and end. 

.. tabs::
   
    .. tab:: Java

        .. code-block:: java
            :linenos:

            // Add a 10 second timeout
            new command.withTimeout(10);


    .. tab:: C++

        .. code-block:: c++
            :linenos:

            // Add a 10 second timeout
            command.WithTimeout(10.0_s);

withInterrupt()
---------------

Adds a condition that will interrupt the command.

.. tabs::
   
    .. tab:: Java

        .. code-block:: java
            :linenos:

            new command.withInterrupt(isLimitHit());


    .. tab:: C++

        .. code-block:: c++
            :linenos:

            command.WithInterrupt([&limit]{return isLimitHit();});

andThen()
---------

Adds a method that is executed after the command ends.

.. tabs::
   
    .. tab:: Java

        .. code-block:: java
            :linenos:

            new command.andThen(() -> System.out.println("Command Finished"));


    .. tab:: C++

        .. code-block:: c++
            :linenos:

            command.AndThen([] {std::cout<< "Command Finished";});

beforeStarting()
----------------

Adds a method that is executed before the command starts.

.. tabs::
   
    .. tab:: Java

        .. code-block:: java
            :linenos:

            new command.beforeStarting(() -> System.out.println("Command Starting"));


    .. tab:: C++

        .. code-block:: c++
            :linenos:

            command.BeforeStarting([] {std::cout<< "Command Starting"});