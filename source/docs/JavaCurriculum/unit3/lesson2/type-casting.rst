Type Casting
============

Type casting is process of assigning a value of one data type to another data type. There are two types of casts in Java, *Widening* and *Narrowing*.

Widening Casting
----------------

Widening casts are done automatically. A widening cast only happens when going from a smaller data type to a larger data type. The list from smaller to larger is listed below. 

1. byte
2. short
3. char
4. int
5. long
6. float
7. double

.. note:: The data type boolean cannot be type casted.

Examples

``int`` -> ``long``

.. code-block:: java
   :linenos:
   
   int typeInt = 42;
   long typeLong = typeInt;
   
   System.out.println(typeInt);
   System.out.println(typeLong);
   
Output

.. code-block:: text

   42
   42
   
``byte`` -> ``double``

.. code-block:: java
   :linenos:
   
   byte typeByte = 2;
   double typeDouble = typeByte;
   
   System.out.println(typeByte);
   System.out.println(typeDouble);
   
Output

.. code-block:: text

   2
   2.0
   
Narrowing Casting
-----------------

Narrowing casting is done when a larger data type needs to be converted to a smaller data type. 

Examples

``float`` -> ``short``

.. code-block:: java
   :linenos:
   
   float typeFloat = -42.27;
   short typeShort = (short) typeFloat;
   
   System.out.println(typeFloat);
   System.out.println(typeShort);
   
Output

.. code-block:: text

   -42.27
   -42