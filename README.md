# Closed-Loop Station Keeping State Machine
This repo was created to study techniques in **closed-loop station keeping** for satellites in proliferated 
Low Earth Orbit (PLEO) satellite constellations. The underlying dynamics are computed using API calls to 
NASA's **General Mission Analysis Tool (GMAT)**. 

The simulation models tight operational bounds typical of PLEO satellites. Station-keeping control logic
(including the state machine) lives in `testThrusting.py`, which also serves as the main entry point for the
project.

## Features
- Full GMAT API integration for high-fidelity orbit propagation
- Custom satellite, propagator, and force model creation
- Closed-loop thrusting logic for station keeping
- Coordinate transformations (ECI <-> RIC frame)
- Data visualization with Matplotlib

## Requirements
- **Python** 3.10+
- **NASA GMAT-2026**
- The following **Python** libraries:
   - Numpy
   - Matplotlib

## Instructions
1. **Installing GMAT** - 
   Download and install the latest version of GMAT from NASA's SourceForge page:
   https://sourceforge.net/projects/gmat/
    
2. **Create API Connection** - 
   Navigate to `.../GMAT Install/application/api` and open BuildApiStartupFile.py. In the terminal enter:
   ```bash
   python BuildApiStartupFile.py

3. **Clone Repository** - 
   Add this repo to your coding environment:
   ```bash
   git clone https://github.com/leftyonthehill/gmat.git

4. **Connect API to Repo** - 
   Copy the path to `.../GMAT Install` and paste it in this repo's `load_gmat.py` script for
   either the desktop path or the laptop path (this will change later, was intended to help me
   switch between two of my computers while traveling)

5. **Install libraries** - 
   Install supporting **Python** libraries by running the following command:
   ```bash
   pip install numpy matplotlib

6.  **Test for correct install** - 
   Run `testThrusting.py` and analyze the station keeping data!

## Contributing
This is currently a personal research project, but feel free to open issues or pull requests if 
you have suggestions for improving the station-keeping logic or modularity.
    