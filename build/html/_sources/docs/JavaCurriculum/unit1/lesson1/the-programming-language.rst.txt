The Programming Language
========================

A machine does not understand any human languages. Therefore programs are written in languages the machine can interoperate and use. Before a machine can use instructions outlined in a program, that program is required to be translated into a language, the machine CPU can execute. 

There are three main programming language levels:

1. Machine Language
2. Assembly Language
3. High-Level Language

Machine Language
----------------

Machine language is the most primitive instructions used by a machine. It is sometimes referred to as the *native* language. Machine language can be complicated to understand as it is only represented in *binary code*. It is possible to edit and make programs in raw binary code; however, it is not ideal and can lead down a rabbit hole very fast if something is wrong.

Assembly Language
-----------------

Due to the nature of machine language being very hard to write and read in, *assembly* was created. Assembly uses a short word known as *mnemonic* to represent each of the machine language instructions. For example ``add`` (addition), ``sub`` (subtraction), and ``mov`` (move). Assembly makes programming the machine easier; however, the machine can not read assembly language. To convert assembly to machine language, a program called an *assembler* was created. This will take the mnemonic instructions and convert them into the binary code used by the CPU. 

High-Level Language
-------------------

Assembly is almost still one for one the same as machine language just wrapped to be more readable. This makes assembly still very difficult to program in and requires excellent knowledge of the CPU architecture. This is why *high-level* languages were created. High-Level languages are close to English, which allows for better readability and use. As stated in the introduction, there are many languages, and each one serves a specific purpose and is better suited based on the application. Some of the most popular high-level languages in order of popularity on GitHub (The worlds leading software platform) in 2019.

-  JavaScript - Used in web development
-  Python - Scripting language useful for short programs
-  Java - Object-orientated language widely used for platform independent applications
-  PHP - Scripting language for web development
-  C# - Object-orientated language developed by Microsoft and used for desktop applications
-  C++ - Object-orientated language based on C
-  TypeScript - Superset of JavaScript, allows for static typing
-  Shell - Scripting language used for running tasks on a command-line interpreter
-  C - Very close to assembly but has the ease of a high-level language
-  Ruby - Similar to Python but is mostly used for web applications

The data for the above list can be viewed in the GitHub year of review report found `here <https://octoverse.github.com/>`__.

Programs written in high-level languages are called source code. A machine cannot natively run source code. Source code must be translated into binary code for the CPU to execute the source code. A program called a compiler is used to compile the source code into usable binary code for the CPU. 

