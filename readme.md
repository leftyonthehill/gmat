# Closed-Loop Station Keeping State Machine

## Description:
This repo was created to study techniques in closed-loop station keeping for satellites in proliferated 
low Earth orbit (PLEO) satellite constellations. The underlying dynamics are computed using API calls to 
NASA's General Mission Analysis Tool (GMAT). 

## Requirements
- Python 3.10+
- NASA GMAT-2025+
- Numpy (Python library)
- Matplotlib (Python library)

## Instructions:
1. Download and install GMAT from NASA's SourceForge page:
   https://sourceforge.net/projects/gmat/
    
2. Navigate to .../GMAT Install/application/api and open BuildApiStartupFile.py. In the terminal enter:
   python BuildApiStartupFile.py

3. Copy the path to .../GMAT Install/application/bin and paste it in this repo's load_gmat.py script for
   either the desktop path or the laptop path (this will change later, was intended to help me
   switch between two of my computers while traveling)

4. Install supporting Python libraries:
   - Numpy
   - Matplotlib

5.  Run testThrusting.py and analyze the station keeping data!

    