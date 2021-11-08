Exercise Answers
================

Question 1:
-----------

.. code-block:: java
   :linenos:
   
   public class QuestionOne
   {
      public static void main(String[] args)
      {
        double X;
        short Y = 10;
        X = Y;
        System.out.println(X);
    
        long Z; 
        int F = 1234567;
        Z = F;
        System.out.println(Z);
      }
   }

**Output**

.. code-block:: text

    10.0
    1234567

Question 2:
-----------
    
.. code-block:: java
    :linenos:
       
    public class QuestionTwo
    {
        public static void main(String[] args)
        {
            double X = 12345.54321789;

            // Show X as a double
            System.out.println("X as a double: " + X);

            // Show X as a float
            System.out.println("X as a float: " + (float) X);
            
            // Show X as an integer
            System.out.println("X as an integer: " + (int) X);

            // Show X as a byte
            System.out.println("X as a byte: " + (byte) X);
        }
    }
    
**Output**
    
.. code-block:: text
    
    X as a double: 12345.54321789
    X as a float: 12345.543
    X as an integer: 12345
    X as a byte: 57

.. note:: It would be good to notice that the accuracy of **X** drops as the cast goes to a smaller sized data type. 

Question 3:
-----------
    
.. code-block:: java
    :linenos:
       
    public class QuestionThree
    {
        public static void main(String[] args)
        {
            int X = 5 + 2;
            int Y = 20 / 2;
            int Z = 10 % 3;
            System.out.println("X = " + X + " Y = " + Y + " Z = " + Z);
            
            X %= 2;
            Y /= 5;
            Z *= 2;
            System.out.println("X = " + X + " Y = " + Y + " Z = " + Z);
            
            System.out.println("(X == 2) = " + (X == 2) + " (Y > 1) = " + (Y > 1) + " (Z <= 5) = " + (Z <= 5));
            
            System.out.println("(5 > 2) && (2 > 2) = " + ((5 > 2) && (2 > 2)));
            System.out.println("(5 <= 5) || (2 == 2) = " + ((5 <= 5) || (2 == 2)));
            System.out.println("!(5 <= 6) || (2 == 2) && (3 > 3) = " + (!(5 <= 6) || (2 == 2) && (3 > 3)));
        }
    }
    
**Output**
    
.. code-block:: text
    
    X = 7 Y = 10 Z = 1
    X = 1 Y = 2 Z = 2
    (X == 2) = false (Y > 1) = true (Z <= 5) = true
    (5 > 2) && (2 > 2) = false
    (5 <= 5) || (2 == 2) = true
    !(5 <= 6) || (2 == 2) && (3 > 3) = false

Challenge Question
------------------

.. code-block:: java
   :linenos:
   
   /*
    * To change this license header, choose License Headers in Project Properties.
    * To change this template file, choose Tools | Templates
    * and open the template in the editor.
    */
   package com.edu.unit3;

   import java.util.Scanner;

   /**
    *
    * @author james
    */
   public class ChallengeQuestion
   {
      public static void main(String[] args)
      {
        Scanner input = new Scanner(System.in);

        System.out.print("Enter purchase amount: $");
        double purchaseAmount = input.nextDouble();
      
        double tax = purchaseAmount * 0.13;
        System.out.println("Sales tax is: $" + (int)(tax * 100) / 100.0);
       }
   }
   
**Output**

.. code-block:: text

    Enter purchase amount: $299.99
    Sales Tax is: $38.99