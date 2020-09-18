Errors
======

There are three types of errors in programming; Syntax, Logic and Runtime.

Syntax Errors
-------------

Any error that is detected by the compiler is called a Syntax error. Syntax errors are due to issues in how the code is constructed. Some common syntax errors are misspelled words, forgetting braces ``{}`` and or semi-colons ``;``. 

Some examples are below.

.. code-block:: text
   :linenos:
   
   public class HelloWorld
   {
      public static void main(String[] args)
      {
         //Display message Hello World! on the console
         System.out.println("Hello World!);
      }
   }
   
This example will give 3 errors on compilation. 

On line 6 it will give an ``unclosed string literal`` and a ``';' expected`` error. On line 8 it will give a ``reached end of file while parsing }`` error.

The only actual error is the first one on line 6: ``unclosed string literal``. The other errors are the chain effect of the first error. To fix this error have a look at line 6. 

.. code-block:: text
   :linenos:
   :lineno-start: 6
   
   System.out.println("Hello World!);``

The error is that a string was started but never completed. Remember a string requires an opening ``"`` and closing ``"`` quotation. To fix this line a end ``"`` quotation is needed. The ``"`` is inserted between the ``!`` and the ``)``. 

.. code-block:: java
   :linenos:
   :lineno-start: 6
   
   System.out.println("Hello World!");
   
Logic Errors
------------

Logic errors are when a program is supposed to do one thing but does another. Logic errors are very common and sometimes can lead to long debugging sessions. One of the most common logic errors is comparisons. An example is shown below.  

.. code-block:: java
   :linenos:
   
   public class LogicError
   {
      public static void main(String[] args)
      {
         boolean x = false;
         
         if (x = false)
         {
            System.out.println("X is False!");
         }
         else
         {
            System.out.println("X is True!");
         }
      }
   }
   
Output

.. code-block:: text

   X is True!
   
The code compiles but the output is wrong. The correct output should have been ``X is False!``. The logic error is on line 7. Comparisons and if statements will be covered in later chapters. But for now a logical error is being shown.

On line 7 we have:

.. code-block:: java
   :linenos:
   :lineno-start: 7
   
   if (x = false)
   
Comparisons require a ``==`` not ``=``. 

By changing line 7 to this:

.. code-block:: java
   :linenos:
   :lineno-start: 7
   
   if (x == false)
   
We get the successful output:

.. code-block:: text

   X is False!
   
Runtime Errors
--------------

Runtime errors happen while the Java code is running. Some common runtime errors are input errors, logical errors that cause the program to crash and infinite loops. 

A simple runtime error to show is a division by zero. This is shown below.

.. code-block:: java
   :linenos:
   
   public class RuntimeError
   {
      public static void main(String[] args)
      {
         System.out.println(1 / 0);
      }  
   }

The output console will display this error:

.. code-block:: text

   Exception in thread "main" java.lang.ArithmeticException: / by zero
            at RuntimeError.main(RuntimeError.java:5)
            


