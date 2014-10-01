#! /usr/bin/python
from subprocess import *
import sys
import os

filename = sys.argv[1]

call(['objcopy','-I','elf32-little','-j','.rodata','-O','binary',filename,'temp.bin'])

p = Popen(['/bin/sh', '-c', 'readelf -s '+ filename +' | grep FW'], stdout=PIPE)

for line in p.stdout:
    args = line.split()
    
    print "Found", args[7], "offset", int(args[1],16), "count", args[2]
    call(['dd','if=temp.bin','bs=1','count='+args[2], 'skip='+str(int(args[1],16)),'of='+args[7] + ".fw"])

os.unlink('temp.bin')
