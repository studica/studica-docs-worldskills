Exercises
=========

Below are a list of exercises to complete. Answers will be provided in the next section.

.. important:: To learn properly it is recommended to attempt the exercises before looking at the answers.

1. What are the results of these casts?

.. code-block:: java

    double X;
    short Y = 10;
    X = Y;
    System.out.println(X);

    long Z; 
    int F = 1234567;
    Z = F;
    System.out.println(Z);

2. Add to the code below:

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

            
            // Show X as an integer


            // Show X as a byte

        }
    }

3. What are the results of these operations:

.. code-block:: java

    int X = 5 + 2;
    int Y = 20 / 2;
    int Z = 10 % 3;

    X %= 2;
    Y /= 5;
    Z *= 2;

    X == 2;
    Y > 1;
    Z <= 5;

    (5 > 2) && (2 > 2)
    (5 <= 5) || (2 == 2)
    !(5 <= 6) || (2 == 2) && (3 > 3)

Challenge Question
------------------
    
.. note:: Challenge questions are here to stump you and test your problem solving skills. The content in a challenge question will be covered in later units.

4. Create a sales tax calculator. The user is required to input the value of the item. The output may only have **2** decimal places and the sales tax rate is **13%**.

Example output

.. code-block:: text

    Enter purchase amount: $299.99
    Sales Tax is: $38.99

