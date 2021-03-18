# Project Info

## CPP Signature

```CPP
/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description:
----------------------------------------------------------------- */
```

## Coding Practice

1. Classes are wrapped in namespaces. So the class name need not be appended with the namespace name.
2. Naming Practice (functions and variables)
    1. Private: all_lower_case, separation using underscore.
    2. Protected: camelCase.
    3. Public: PascalCasing
    4. Define: ALL_CAPS
3. Classes are divided into,
    1. Properties: values share among multiple systems.
    2. Controller: control and logic functions for each sub-system.
    3. Main: Initializing all the system, threads, processes.

## Namespaces

1. Actuators -> Contains libraries that output to hardware
2. Sensors   -> Contains libraries that get information from sensors
3. Utilities   -> Contains libraries that provide Utilities of the framework

## Useful links

### g++

1. [Add libraries to path](https://stackoverflow.com/questions/6141147/how-do-i-include-a-path-to-libraries-in-g)
2. [Working with dynamic libraries](https://www.cprogramming.com/tutorial/shared-libraries-linux-gcc.html)
3. Working with MPI and OpenMP.
4. [Logging with Boost](https://www.boost.org/doc/libs/1_64_0/libs/log/doc/html/log/tutorial.html)
