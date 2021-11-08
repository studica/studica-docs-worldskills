Scanner
=======

Receiving user data is an important part of coding to interact with the user. To receive input from the user, the ``Scanner`` class is used. This is found under the ``java.util`` package.

Hence we will need to import the library first,

.. code-block:: java
   :linenos:
   
   import java.util.Scanner;
   
.. note:: This is the most important step and also the easiest to forget. If the ``Scanner`` class is not implemented, all its function and instances cannot be used.
   
Now we can use the Scanner class by creating an object of the class.

.. code-block:: java 
   :linenos:
   
   Scanner input = new Scanner(System.in);
   
The line above creates a Scanner instance input that will read the user input depending on the methods called. The following are the eight most common methods to retrieve user input depending on the data type.

.. table::
   :align: center
   
   .. csv-table::
      :header: "Methods", "Return Type"
	  
	  "``nextByte()``", "Byte"
	  "``nextShort()``", "Short"
	  "``nextInt()``", "Int"
	  "``nextLong()``", "Long"
	  "``nextFloat()``", "Float"
	  "``nextDouble()``", "Double"
	  "``nextLine()``", "String"
	  "``nextBoolean``", "Boolean"

.. important:: It is important that the input type matches the method's data type, or else you will get an exception/error message.

Setting up with Scanner class
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: java
   :linenos:
   :emphasize-lines: 1
   
   import java.util.Scanner; // Import Scanner library
   
   class TestClass 
   {
      public static void main(String[] args)
	  {
	     // Create an instance of the Scanner class
		 Scanner input = new Scanner(System.in);
		 
		 // Source code as follows
      }
   }
		 
Byte
~~~~

.. code-block:: java
   :linenos:
   
   System.out.println("Enter a byte integer:");
   
   // Reading the input as byte data type
   byte aByte = input.nextByte(); 
   System.out.println("aByte = " + aByte);
   
Output

.. code-block:: text
   :linenos:
   
   Enter a byte integer:
   5
   aByte = 5

Short
~~~~~

.. code-block:: java
   :linenos:
   
   System.out.println("Enter a short integer:");
   
   // Reading the input as short data type
   short aShort = input.nextShort();
   System.out.println("aShort = " + aShort);
   
Output

.. code-block:: text
   :linenos:
   
   Enter a short integer:
   50
   aShort = 50
   
Int
~~~

.. code-block:: java 
   :linenos:
   
   System.out.println("Enter a integer:");
   
   // Reading the input as a int data type
   int aInt = input.nextInt();
   System.out.println("aInt = " + aInt);

Output

.. code-block:: text
   :linenos:

   Enter a integer:
   100
   aInt = 100

Long
~~~~

.. code-block:: java
   :linenos:

   System.out.println("Enter a long integer:");

   // Reading the input as a long data type
   long aLong = input.nextLong();
   System.out.println("aLong = " + aLong);

Output

.. code-block:: text
   :linenos:

   Enter a long integer:
   12345
   aLong = 12345

Float
~~~~~

.. code-block:: java
   :linenos:

   System.out.println("Enter a float:");
   
   // Reading the input as a float data type
   float aFloat = input.nextFloat();
   System.out.println("aFloat = " + aFloat);
   
Output

.. code-block:: text
   :linenos:
   
   Enter a float:
   95.43
   aFloat = 95.43
   
Double
~~~~~~

.. code-block:: java
   :linenos:
   
   System.out.println("Enter a double:");
   
   // Reading the input as a double data type
   double aDouble = input.nextDouble();
   System.out.println("aDouble = " + aDouble);
   
Output

.. code-block:: java
   :linenos:
   
   Enter a double:
   97584.45
   aDouble = 97584.45
   

String 
~~~~~~

.. code-block:: java
   :linenos:
   
   System.out.println("Enter a string:");
   
   // Reading the input as a string data type
   String aString = input.nextLine();
   System.out.println("aString = " + aString);
   
Output 

.. code-block:: text
   :linenos:
   
   Enter a string:
   Hello World
   aString = Hello World
   
Boolean
~~~~~~~

.. code-block:: java
   :linenos:
   
   System.out.println("Enter a boolean:");
   
   // Reading the input as a boolean variable
   boolean aBoolean = input.nextBoolean();
   System.out.println("aBoolean = " + aBoolean);
   
Output 

.. code-block:: text
   :linenos:
   
   Enter a boolean:
   true
   aBoolean = true
   
Example
^^^^^^^

The following block of code shows an example of using the Scanner library.

.. code-block:: java
   :linenos:
   :emphasize-lines: 1
   
   import java.util.Scanner; // Import Scanner library
   
   class TestClass
   {
      public static void main(String[] args) 
	  {
		Scanner input = new Scanner(System.in);
		
		System.out.print("Please enter your name: ");
		String name = input.nextLine();
		
		System.out.println("Hi " + name + ", what is your favourite number?");
		int num = input.nextInt();
		
		System.out.println("Your favourite number is " + num + ".");
	  }
   }
   
Output 

.. code-block:: text
   :linenos:
   
   Please enter your name: Jack
   Hi Jack, what is your favourite number?
   7
   Your favourite number is 7.