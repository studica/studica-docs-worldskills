Simple Java Program
===================

The code used for any beginner in a new language is "Hello World!". Lets look below at this simple program.

HelloWorld.java
---------------

.. code-block:: java
   :linenos:
   
   public class HelloWorld
   {
      public static void main(String[] args)
      {
         //Display message Hello World! on the console
         System.out.println("Hello World!");
      }
   }
   
When executed we will get the output below in the console window. 

.. code-block:: text

   Hello World!

Lets break down this simple program line by line. 

Line 1: is the class identifier. Every Java program requires at least one class to be defined. Conventionally every class must start with an uppercase letter. The class name used in the example is ``HelloWorld``. Notice how it is one word and not multiple words. If we used ``Hello World`` a syntax error would pop up. 

.. note:: The class name ``HelloWorld`` is the same as the file name ``HelloWorld.java``. Java is case sensitive and these must be the same to avoid a compilation errors. 

Line 2: This is the opening brace ``{``. Braces group the components of a program. To close the group a closing brace ``}`` like on line 8 is required.

Line 3: is the *main* definition. The main method is required for a Java program to execute. There may be multiple methods in a class but the main is the entry point during execution. 

Line 4: This is the opening brace ``{`` for the main method.

Line 5: This is a comment. Comments are little notes left by a programmer that do not get compiled. Comments will be covered more in depth in a later chapter. 

Line 6: Contains the print statement. ``System.out.println`` is a *statement* in Java. This particular statement will display the contents inside the ``()`` parentheses. On line 6 inside the () parentheses we have the *String* "Hello World!". A String is a term for a sequence of characters. A String is always enclosed in ``" "`` quotations. On this line any line of text placed inside the " " will be output to the console. 

.. important:: Notice at the end of line 6 there is a ``;`` Semicolon. In Java Semicolons are required to end a statement. In the case of line 6 we have the statement ``System.out.println("Hello World!")`` then to end it there is the ``;``.

Line 7: Is the closing brace ``}`` for the main method.

Line 8: Is the closing brace ``}`` for the class.


Lets Create some more Simple Programs
-------------------------------------

HelloWorldVersionTwo.java
^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: java
   :linenos:
   
   public class HelloWorldVersionTwo
   {
      public static void main(String[] args)
      {
         //Display messages on the console
         System.out.println("Hello World!");
         System.out.println("We added some more print statements");
         System.out.println("Wohoo!");
      }
   }

Output

.. code-block:: text

   Hello World!
   We added some more print statements
   Wohoo!
   
LetsDoSomeMath.java
^^^^^^^^^^^^^^^^^^^

.. code-block:: java
   :linenos:
   
   public class LetsDoSomeMath
   {
      public static void main(String[] args)
      {
         //Show how some expressions work
         System.out.println("Let's do some math!");
         System.out.print("10 + 2 - 5 = ");
         System.out.println(10 + 2 - 5);
      }
   }
   
Output

.. code-block:: text

   Let's do some math!
   10 + 2 - 5 = 7

Let's look at why there is only two lines in the console and what happened. 

On line 7 notice how it's ``System.out.print`` and not ``System.out.println``.  ``println`` will move the cursor to the start of the next line after displaying it's statement. ``print`` does not move to the next line after displaying the statement. 

On line 8 the math ``10 + 2 - 5 =`` is inside quotations ``" "`` thus identifying it a String and not an expression. 

On line 9 there is no quotations ``" "`` so the statement inside the parentheses ``10 + 2 - 5`` is now considered an expression and will be evaluated. As the expression is in a print statement the result will be outputted to the console. 

 