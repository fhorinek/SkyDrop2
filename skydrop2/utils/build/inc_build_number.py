#!/usr/bin/python

''' Eclipse is calling this script every build '''

with open("../utils/build/build_number.txt", "r") as f:
    number = int(f.readline())

number += 1

print("Current build number is: %d" % number)

with open("../utils/build/build_number.txt", "w") as f:
    f.write("%d" % number)

with open("../src/build_number.h", "w") as f:
    f.write("//generated by inc_build_number.py\n")
    f.write("\n")
    f.write("#ifndef BUILD_NUMBER_H\n")
    f.write("#define BUILD_NUMBER_H\n")
    f.write("#define BUILD_NUMBER %dul\n" % number)
    f.write("#endif\n")
