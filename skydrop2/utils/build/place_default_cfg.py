#!/usr/bin/python

import struct

placeholder = struct.pack("I", 0xAABBCCDD) + "*** Default configuration placeholder ***".encode('ASCII')

input_fw = open("DROP2.BIN", "rb").read()
input_cfg = open("DROP2.EE", "rb").read()

index = input_fw.find(placeholder)

print("Firmware length      %6u" % len(input_fw))
print("Configuration length %6u" % len(input_cfg))
print("Placeholder index    0x%04x" % index)

zeros = "\0" * (len(input_cfg) - len(placeholder))
placeholder = placeholder + zeros.encode("ASCII")

output_fw = input_fw.replace(placeholder, input_cfg)

f = open("DROP2.BIN", "wb")
f.write(output_fw)
f.close()

