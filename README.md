# GNCSIM
GNC Simulation, Integration, and Modeling

GNC Simulation, Integration, and Modeling (GNCSIM) is the future of how GNC performs simulations and control system design for its rockets. The idea behind GNCSIM is that an object-oriented approach to simulations and controls allow for rapid simulation development for new vehicles as well as quick adjustments later in the design process. Additionally, the object-oriented approach allows for an entire simulations suite to exist within a mere handful of class files, which is an improvement upon the oftentimes messy jet-vanes-simulations repository that was previously utilized. A “master file” then acts to represent the simulations for a given rocket design.  

## Documentation

The [GNCSIM Documentation](https://gtvault.sharepoint.com/:f:/s/gnc/Es1GMQJQkK1IlyImzKzuF2cBYLz4-xXhQ_Y9vMDLjO26Zw?e=yQhzuy) is available on the GNC SharePoint.

## Environment Class
Environment Class Requires Python3 Environment in order to regenerate location data. If not, existing location data will be used.
Must be Python 3.9 or newer. 3.13 is recommended.

The following python packages are required (Installing Just Rocketpy should automatically install the rest):
- Rocketpy
- requests
- Numpy >= 1.13
- Scipy >= 1.0
- Matplotlib >= 3.0
- netCDF4 >= 1.6.4
- windrose >= 1.6.8
- requests
- pytz
- simplekml

Packages Required that do not come with Rocketpy:
- sys
- csv
- argparse
- datetime
- os
