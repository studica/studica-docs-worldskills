Data Types
==========

Most programing languages use data types to tell the compiler how the program is using data. Using the correct data type is always important. Good programmers always try to optimize the code as much as possible. The easiest way to optimize code is to use the correct data type. Data types are in many shapes and sizes. Java has eight primitive data types.

.. list-table:: Primitive Data Types
   :widths: 25 50 25
   :header-rows: 1
   :align: center
   
   * - Name
     - Range
     - Size
   * - byte
     - -128 -> 127
     - 8-bit signed
   * - short
     - -32768 -> 32767
     - 16-bit signed
   * - int 
     - -2147483648 -> 2147483647
     - 32-bit signed
   * - long
     - -9223372036854775808 -> 9223372036854775807
     - 64-bit signed
   * - float
     - ±3.4028235E+38
     - 32-bit IEEE 754
   * - double
     - ±1.79769313486231570E+308
     - 64-bit IEEE 754
   * - char
     - 0 -> 65536
     - 16-bit unsigned
   * - boolean
     - true or false
     - 1-bit 
     
.. note:: Java uses signed data types except for char's. The difference between unsigned and signed data types is that unsigned cannot hold negative values. However unsigned data types have larger positive values. 

Primitive data types are split into two categories ``Integers`` and ``Floating Points``. Integers are used for whole numbers, whereas floating points are used for numbers that have decimal points. 

Integers
--------

Byte
^^^^ 

Bytes are the smallest type of integers. They hava range of -128 to 127. When an integer is required and the value will be in that range it is best to use a byte as it will save memory. 

.. code-block:: java
   :linenos:
   
   byte variable = 42;
   System.out.println(variable);
   
Output

.. code-block:: text
   
   42
   
Short
^^^^^

Shorts are used when the integer will fall in between -32768 to 32767.

.. code-block:: java
   :linenos:
   
   short variable = -4242;
   System.out.println(variable);
   
Output

.. code-block:: text

   -4242
   
Int
^^^

Int is the most used data type for integers. Unless there is a specific reason to use one of the other integer types ``int`` is always preferred and used. Int's have a range of -2147483648 to 2147483647.

.. code-block:: java
   :linenos:
   
   int variable = 123456789;
   System.out.println(variable);
   
Output

.. code-block:: text

   123456789

Long
^^^^

Long data types are used when an ``int`` is not big enough. Longs have a range of -9223372036854775808 to 9223372036854775807.

.. note:: Longs require an ``L`` to be added at the end of the value.

.. code-block:: java
   :linenos:
   
   long variable = 9999999999L;
   System.out.println(variable);
   
Output

.. code-block:: text

   9999999999
   
Floating Points 
---------------
   
float
^^^^^

Floats are used for numbers that have decimals. The range for floats is ±3.4028235E+38. 

.. note:: floats require an ``f`` to be added at the end of the value.

.. code-block:: java
   :linenos:
   
   float variable = 1.2345f;
   System.out.println(variable);
   
Output

.. code-block:: text

   1.2345   
   
.. note:: floats are good when a precision of six to seven decimal points are required.

double
^^^^^^

Doubles are used for numbers that have lots of decimals. Unlike floats, doubles have a precision of fifteen decimal points. 

.. code-block:: java
   :linenos:
   
   double variable = 42.42;
   System.out.println(variable);
   
Output

.. code-block:: text

   42.42
   
.. note:: Unlike floats the value of a double does not require a ``d`` at the end.

Scientific Numbers
^^^^^^^^^^^^^^^^^^

In Java Floating Points can be scientific numbers. 

.. code-block:: java
   :linenos:
   
   double variable = 42.42e6;
   float variable1 = 42.42e-2f;
   System.out.println(variable);
   System.out.println(variable1);
   
Output

.. code-block:: text

   424200.0
   0.4242
   
Boolean
-------

Booleans are a special data type as they don't hold a numeric value. Booleans only have two options, true or false.

.. code-block:: java
   :linenos:
   
   boolean variable = true;
   boolean variable1 = false;
   System.out.println(variable);
   System.out.println(variable1);
   
Output

.. code-block:: text

   true
   false

Char
----

Char is the short form for character. ``char`` will store a single character. Char's use a single quotations ``' '`` to identify. 

.. code-block:: java
   :linenos:
   
   char variable = 'A';
   System.out.println(variable);
   
Output

.. code-block:: text

   A
