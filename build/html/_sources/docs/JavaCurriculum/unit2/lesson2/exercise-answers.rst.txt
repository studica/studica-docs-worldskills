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
         //Display some messages on the console
         System.out.println("Hello World!");
         System.out.println("It is very nice to meet you.");
         System.out.println("This is one of my first Java programs.");
      }
   }
   
Question 2:
-----------

.. code-block:: java
   :linenos:
   
   public class QuestionTwo
   {
      public static void main(String[] args)
      {
         //Display message Robot on the console
         System.out.println("RRRRR    OOO    BBBBB    OOO   TTTTT");
         System.out.println(" R  R   O   O   B   B   O   O    T");
         System.out.println(" RRRR  O     O  BBBBB  O     O   T");
         System.out.println(" R  R   O   O   B   B   O   O    T");
         System.out.println("RR   R   OOO    BBBBB    OOO     T");
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
           //Display a table on the console
           System.out.println("x     x^2      x^3      x^4");
           System.out.println("1     1        1        1");
           System.out.println("2     4        8        16");
           System.out.println("3     9        27       81");
           System.out.println("4     16       64       256");
           System.out.println("5     25       125      625");
       }
   }

Question 4:
-----------

.. code-block:: java
   :linenos:
   
   public class QuestionFour
   {
       public static void main(String[] args)
       {
           //Display some math on the console
           System.out.print("1 + 2 + 3 + 4 + 5 + 6 + 7 + 8 + 9 = ");
           System.out.println(1 + 2 + 3 + 4 + 5 + 6 + 7 + 8 + 9);
       }
   }
   
Output

.. code-block:: text

   1 + 2 + 3 + 4 + 5 + 6 + 7 + 8 + 9 = 45
   
Question 5:
-----------

.. code-block:: java
   :linenos:
   
   public class QuestionFive
   {
       public static void main(String[] args)
       {
           //Display some math on the console
           System.out.println((10.6 * 4.0 - 3.2 * 1.0) / (20.6 - 1.8));
       }
   }
   
Output

.. code-block:: text

   2.085106382978723
   
Question Six
------------

.. code-block:: java
   :linenos:
   
   public class QuestionSix
   {
       public static void main(String[] args)
       {
           //Display some math on the console
           System.out.print((9.0 / 5.0) * 30 + 32);
           System.out.println("°F");
       }
   }
   
Output

.. code-block:: text

   86°F
   
.. important:: If you got 62°F as your answer there is a logic error in your code. In Java :math:`\begin{equation}\frac{9}{5}\end{equation}` would result in 1. This is due to integer division. Integers do not allow decimal points. :math:`\begin{equation}\frac{9}{5}\end{equation}` should be 1.8 but the result is 1 as .8 is discarded. To eliminate integer division we add .0 to the integer as shown in the answer.
   
Challenge Question
------------------

.. code-block:: java
   :linenos:
   
   /*
    * To change this license header, choose License Headers in Project Properties.
    * To change this template file, choose Tools | Templates
    * and open the template in the editor.
    */
   package com.edu.firstproject;

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
         System.out.print("Enter the temperature to convert in °C: ");
         double temp = input.nextDouble();
         System.out.println(temp + "°C is equal to " + ((9.0 / 5.0) * temp + 32) + "°F");
      }
   }
   
Example Output

.. code-block:: text

   Enter the temperature to covert in °C: 21.5
   21.5°C is equal to 70.7°F