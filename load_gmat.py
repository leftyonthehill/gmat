""" GMAT Application Programmer's Interface
- Coded by D. Conway. Thinking Systems, Inc. """

import sys
from os import path

apistartup = "api_startup_file.txt"
GmatInstall = "C:/gmat-win-R2026a"
GmatBinPath = GmatInstall + "/bin"
Startup = GmatBinPath + "/" + apistartup

if path.exists(Startup):


   sys.path.insert(1, GmatBinPath)

   import gmatpy as gmat
   gmat.Setup(Startup)

else:
   print("Cannot find ", Startup)
   print()
   print("Please set up a GMAT startup file named ", apistartup, " in the ", 
      GmatBinPath, " folder.")
