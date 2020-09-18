Variables
=========

Variables are a way of storing data that can be used and changed throughout the program. 

Variable Declaration
--------------------

Before a variable may be used it must be declared. This tells the compiler to allocate memory for the variable. The format for declaring a variable is shown below. 

.. code-block:: text

   dataType variableName;
   
- dataType is the data type that the variable holds. 
- variableName is the name of the variable you wish to give.

Some examples of real world variables:

.. code-block:: java
   :linenos:
   
   int number;
   double area;
   char letter;
   
Variables of the same data type can be declared together. 

**Instead of:** 

.. code-block:: java
   :linenos:
   
   int a;
   int b;
   int c;

**Use:**

.. code-block:: java
   :linenos:
   
   int a, b, c;
   
Variables should have initial values. It is easy to do this when declaring them.

.. code-block:: java
   :linenos:
   
   int number = 1; 
   double area = 42.5;
   char letter = 'B';
   
   int a = 1, b = 2, c = 3;
   
Variable Naming
---------------

Variables follow a `camel case <https://en.wikipedia.org/wiki/Camel_case>`__ naming structure. This means that the first word is lower case and any preceding word in the variable has a capital first letter. 

.. code-block:: java
   :linenos:
   
   // Bad naming of variables ie. no camelCase
   int Hellothere;
   double thatsreallycool;
   
   // Good naming of variables using camelCase
   int helloThere;
   double thatsReallyCool;
   
If you notice it becomes easier to read the different words in the variable. 

Variables containing more than one word should be joined together.

.. code-block:: java
   :linenos:
   
   // Bad variable naming
   int hello_There;
   double thats-really-cool;

   // Good variable naming
   int helloThere;
   double thatsReallyCool;
   
Variables must always start with a lowercase letter, an underscore ``_`` or a ``$`` sign. Variables cannot start with a number or any other symbol. 

.. code-block:: java
   :linenos:
   
   // Acceptable starts of variables
   int hello;
   double _variable;
   long $money;
   
Variables should always be descriptive but not to long and match the function. 

.. code-block:: java
   :linenos:
   
   // Good Example
   
   // Variables for calculating Pythagorean theorem
   double a, b, c; 
   
   // Bad Example
   
   // Variables for calculating Pythagorean theorem
   double edge, longerEdge, reallyLongEdge;
   
Constants
---------

Constants are a special type of variable that cannot change during the operation of the program. Constants are useful for values that wont change or don't need to change. 

To declare a constant we use:

.. code-block:: text

   final dataType CONSTANT_NAME = valueOfConstant;
   
``final`` tells the compiler that this variable cannot be changed. 

.. note:: Unlike variables constants use all caps for naming. Also if more than one word is in the constant name we use an underscore ``_`` to separate them.

Some examples

.. code-block:: java
   :linenos:
   
   final int CONTROLLER_AXIS = 1;
   final double PI = 3.14159265358979;
   
Using Variables in code
-----------------------

Variables make programing easy lets go through some examples.

.. code-block:: java
   :linenos:
   
   public class Variables
   {
      public static void main(String[] args)
      {
         int x = 1;
         System.out.println(x);
      }
   }
   
Output

.. code-block:: text

   1

Line 5 holds the variable and its initial value. The variable is ``x``, the data type is ``int`` and the value of the variable is ``1``.

Line 6 is the output of the variable. When printing a variable the quotations ``" "`` are not used. 

.. code-block:: java
   :linenos:
   
   public class Variables
   {
      public static void main(String[] args)
      {
         final double PI = 3.14159265358979;
         System.out.println("The Value of pi to 15 decimal places is: " + PI);
      }
   }
   
Output

.. code-block:: text

   The Value of pi to 15 decimal places is: 3.14159265358979
   
Line 5 holds the variable which is being used as a constant. 

Line 6 is the output. Notice how this time we are mixing a String with a variable. The String "The Value of pi to 15 decimal places is: " and variable PI are joined by using ``+``. 

.. code-block:: java
   :linenos:
   
   public class Variables
   {
      public static void main(String[] args)
      {
         double x = 6.84;
         System.out.println("Original Variable x: " + x);
         x = 10.8;
         System.out.println("Changed Variable x: " + x);
      }
   }
   
Output

.. code-block:: text

   Original Variable x: 6.84
   Changed Variable x: 10.8
   
In this example we define the variable ``x`` and give it the value of ``6.84`` on line 5. On line 7 we assign x a new value of ``10.8``. 
   

   