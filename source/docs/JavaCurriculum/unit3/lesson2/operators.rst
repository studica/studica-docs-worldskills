Operators
=========

Operators are functions that act upon elements to create new elements. There are 5 types of operator groups in Java. 

- Arithmetic
- Assignment
- Bitwise
- Comparison
- Logical

Arithmetic
----------

Arithmetic operators are common mathematical operators. 

.. list-table:: Arithmetic Operators
   :widths: 25 50 25 25
   :header-rows: 1
   :align: center
   
   * - Operator
     - Meaning
     - Example
     - Result
   * - **+**
     - Addition
     - 10 + 5
     - 15
   * - **-**
     - Subtraction
     - 50 - 8
     - 42 
   * - **\***
     - Multiplication
     - 4 * 6
     - 24
   * - **/**
     - Division
     - 2.0 / 1.0
     - 1.0
   * - **++**
     - Increment by 1
     - z++
     - z + 1
   * - **--**
     - Decrement by 1
     - z--
     - z - 1 
   * - **%**
     - Modulo
     - 5 % 2
     - 1

Assignment
----------

Assignment operators are used to store values in defined variables.

.. list-table:: Assignment Operators
   :widths: 25 50 25 25
   :header-rows: 1
   :align: center
   
   * - Operator
     - Meaning
     - Example
     - Example expanded
   * - **=**
     - Assign
     - x = 10
     - x = 10
   * - **\*=**
     - Multiply the current value by
     - x *= 5
     - x = x * 5 
   * - **/=**
     - Divide the current value by
     - x /= 5
     - x = x / 5
   * - **%=**
     - Mudulo the current value by
     - x %= 5
     - x = x % 5
   * - **+=**
     - Add to the current value by
     - x += 5
     - x = x + 5
   * - **-=**
     - Subtract the current value by
     - x -= 5
     - x = x - 5
   * - **<<=**
     - Shift the current value left by 
     - x <<= 8
     - x = x << 8
   * - **>>=**
     - Shift the current value right by
     - x >>= 8
     - x = x >> 8
   * - **&=**
     - Bitwise AND the current value by
     - x &= 0xFF
     - x = x & 0xFF
   * - **^=**
     - Bitwise XOR the current value by
     - x ^= 0xFF
     - x = x ^ 0xFF
   * - **|=**
     - Bitwise OR the current value by
     - x |= 0xFF
     - x = x | 0xFF

Bitwise
-------

Bitwise operators manipulate the individual bits of numbers. 

**X = 10** and **Y = 2**

.. list-table:: Bitwise Operators
   :widths: 25 50 25 25
   :header-rows: 1
   :align: center
   
   * - Operator
     - Meaning
     - Example
     - Result
   * - **~**
     - Unary NOT
     - ~X
     - -11
   * - **&**
     - AND
     - X & Y
     - 2
   * - **|**
     - OR
     - X | Y
     - 10
   * - **^**
     - XOR
     - X ^ Y
     - 8
   * - **>>**
     - Shift Right
     - X >> 1
     - 5
   * - **<<**
     - Shift Left
     - X << 1
     - 20
   * - **>>>**
     - Shift right zero fill
     - X >>> 1
     - 5

Comparison
----------

Comparison operators are used to compare two values.

**X = 5** and **Y = 2**

.. list-table:: Comparison Operators
   :widths: 25 50 25 25
   :header-rows: 1
   :align: center
   
   * - Operator
     - Meaning
     - Example
     - Result
   * - **==**
     - Equal to
     - X == Y
     - false
   * - **!=**
     - Not Equal
     - X != Y
     - true
   * - **>**
     - Greater than
     - X > Y
     - true
   * - **<**
     - Less than
     - X < Y
     - false
   * - **>=**
     - Greater than or equal to
     - X >= Y
     - true
   * - **<=**
     - Less than or equal to
     - X <= Y
     - false

Logical
-------

Logical operators determine logic betwen boolean statements.

**X = 2**

.. list-table:: Logical Operators
   :widths: 25 50 25 25
   :header-rows: 1
   :align: center
   
   * - Operator
     - Meaning
     - Example
     - Result
   * - **&&**
     - AND
     - (X < 5) && (X < 10)  
     - true
   * - **||**
     - OR
     - (X < 5) || (X < 1)
     - true
   * - **!**
     - NOT
     - !(X < 5)
     - false