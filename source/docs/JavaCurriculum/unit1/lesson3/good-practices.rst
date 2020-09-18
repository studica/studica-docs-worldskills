Good Practices
==============

In Java, a whole program can be done on one line; however, this would make it incredibly hard to read, understand, and maintain. To be readable, code is spread out over multiple lines, and a concept known as white space is used. White space is the area between and around programming elements. Let's look at the Hello World example.

The first example will have everything on one line.

.. code-block:: java
   :linenos:
   
   public class HelloWorld{public static void main(String[] args){/*Display message Hello World! on the console*/System.out.println("Hello World!");}}
   
Notice how it's hard to see what is going on. There is an inline comment which is ok, but there are better ways to comment. In conclusion, it is tough to interpret. 

In this example, proper white space and multiple lines are used to separate everything. 

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
   
This code is now readable and it is easy to follow along. 

Indentation Style
-----------------

There are two popular styles of indentation in programming. *K&R* (Kernighan and Ritchie) and *Allman* (Eric Allman). 

K&R Style
^^^^^^^^^

.. code-block:: java
   :linenos:
   
   public class HelloWorld {
      public static void main(String[] args) {
         //Display message Hello World! on the console
         System.out.println("Hello World!");
      }
   }

Allman Style
^^^^^^^^^^^^

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
   
There are benefits and drawbacks to both methods. K&R allows for saving space but can get be very hard to debug. Allman lines the braces ``{}`` up; this allows for easy debugging; however, it does add extra lines of code. The choice of which one is better comes down to a personal decision. Programmers have long fought over which style is better, but at the end of the day, there is only one rule, be consistent. Don't switch between styles in a project, always maintain the same style throughout the whole project. Mixing styles will cause your brain to use more overhead when trying to debug an error and lead to much frustration. 

.. hint:: Most of the documentation here will use Allman style as it allows for more comfortable reading on new programmers.

Indentation
-----------

Code should always be indented after a brace ``{``. Some examples are shown below.

**Bad Indentation**

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
   
This code has no indentation.

**Good Indentation**

.. code-block:: java
   :linenos:
   :emphasize-lines: 2,4
   
   public class HelloWorld
   {
      public static void main(String[] args)
      {
         //Display message Hello World! on the console
         System.out.println("Hello World!");
      }
   }
   
This code has good indentation. Notice that after the ``{`` in the highlighted lines the next block is indented. 

Spacing
-------

While not necessary, spacing allows for easier readability within code.

**Bad Spacing**

.. code-block:: java
   :linenos:
   
   System.out.println(6*10%5/6.2+"Math is crazy!");

**Good Spacing**

.. code-block:: java
   :linenos:
   
   System.out.println(6 * 10 % 5 / 6.2 + "Math is crazy!");
   
Both forms are acceptable; however, the second is a better practice and allows for easier debugging.  

Commenting
----------

Documentation is fundamental in programming. A good saying is to always comment as if someone else needed to use your code and understand what is going on. There are three types of comments; line, block, and Javadoc. Adding comments to your code will not change the functionality of the code. Comments are not compiled and included in the Java bytecode. 

Line
^^^^

Line comments are the most common type of commenting. A line comment is achieved by using a ``//`` before any line you want to comment or comment out. Some examples are shown below.

.. code-block:: java
   :linenos:
   
   // This is a basic line comment
   
   System.out.println("Hello World!"); // Line comments can be placed after code as well
   
   // Anything after the // will be commented out and excluded this is useful for disabling lines of code

   // System.out.println("Hello World!");
   
If we were to run the code above only the first print statement will be printed to the console. The second print statement has been commented out and will be ignored by the compiler.

Block
^^^^^

Block comments are useful when multiple lines of comments are required. Block comments can also be used to comment out a whole section of code. Block comments start with ``/*`` and to end the comment use ``*\``. Some examples are shown below.

.. code-block:: java
   :linenos:
   
   /*
    * This is a block comment
    * <- Sometimes we add a * or the ide will auto add a * to show a new line in the comment block
    */
    
   int x = 10 /* Block comments can be used inline as well but not preferred */ + 20;
   
   /* The code below is commented out
   public class HelloWorld
   {
      public static void main(String[] args)
      {
         //Display message Hello World! on the console
         System.out.println("Hello World!");
      }
   }
   */ 

Always remember to close the block comment with ``*/`` otherwise all the code after the starting ``/*`` will be commented out. 

Javadoc 
^^^^^^^

Javadoc comments are a particular type of comment. When documentation is generated for a Java project, a Javadoc comment will follow into the docs. Javadoc comments are generally used at the beginning of the program in the title block and at the beginning of every class and method. A Javadoc comment is similar to a block comment with one change. To start a Javadoc comment, use ``/**`` notice the double *****. To end a Javadoc use ``*/``. Some examples are shown below.

.. code-block:: java
   :linenos:
   
   /**
    * This is an example of a Javadoc comment 
    */
    
   /**
    * Javadoc comments have some special features called tags
    * Here are some examples of tags
    * @param variable variable description
    * @return whatever the return statement is 
    * @author authors name 
    */
    
Javadoc comments are very useful and powerful. For a full list of tags and how they are used consult the Javadoc tag conventions `here <https://www.oracle.com/technical-resources/articles/java/javadoc-tool.html#tag>`__.


     
     
   

