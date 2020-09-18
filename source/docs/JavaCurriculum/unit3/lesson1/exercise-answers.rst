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
         byte d1 = 125;
         short d2 = 32000;
         int d3 = 500000;
         long d4 = 9999999999L;
         float d5 = 10.4f;
         double d6 = -50112.56;
         char d7 = 'A';
         boolean d8 = true;
           
         System.out.println("byte: " + d1);
         System.out.println("short: " + d2);
         System.out.println("int: " + d3);
         System.out.println("long: " + d4);
         System.out.println("float: " + d5);
         System.out.println("double: " + d6);
         System.out.println("char: " + d7);
         System.out.println("boolean: " + d8);
      }
   }
   
Question 2:
-----------

.. note:: There are two solutions to this question

.. code-block:: java
   :linenos: 
   
   public class QuestionTwo
   {
      public static void main(String[] args)
      {
         int x = 105;
         
         System.out.println("Original: " + x);
         System.out.println("Change 1: " + (x = 110));
         System.out.println("Change 2: " + (x = -15));
         System.out.println("Change 3: " + (x = 12));
      }
   }


or

.. code-block:: java
   :linenos:
   
   public class QuestionTwo
   {
      public static void main(String[] args)
      {
         int x = 105;
         
         System.out.println("Original: " + x);
         x = 110;
         System.out.println("Change 1: " + x);
         x = -15;
         System.out.println("Change 2: " + x);
         x = 12;
         System.out.println("Change 3: " + x);
      }
   }
   
Question 3:
-----------

.. code-block:: java
   :linenos:
   
   public class QuestionThree
   {
      public static void main(String[] args)
      {
         int x = 10;
         int y = 12;
         int z = 42;
           
         System.out.println(x + " + " + y + " + " + z + " = " + (x + y + z));
      }
   }
   
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
         System.out.println("Welcome to the Pythagorean Calculator!!!\n");
         System.out.print("Enter the length of side A: ");
         double a = input.nextDouble();
         System.out.print("Enter the length of side B: ");
         double b = input.nextDouble();
         System.out.println("Calculating");
         double result = Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2));
         System.out.println("The length of the hypotenuse is: " + result);
       }
   }
   
Output

.. code-block:: text

   Welcome to the Pythagorean Calculator!!!
   
   Enter the length of side A: 3
   Enter the length of side B: 4
   Calculating
   The length of the hypotenuse is: 5.0