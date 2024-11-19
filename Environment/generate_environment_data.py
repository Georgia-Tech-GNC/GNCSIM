# Core RocketPy library
from rocketpy import Environment, SolidMotor, Rocket, Flight  # Import specific classes/modules as needed

# RocketPy dependencies
import requests              # For fetching online weather data
import numpy as np           # Core numerical computations
import scipy                 # Scientific and numerical methods
from scipy.optimize import minimize  # Example: for optimization
import matplotlib.pyplot as plt  # Plotting and visualization
from netCDF4 import Dataset   # For handling NetCDF atmospheric data
from windrose import WindroseAxes  # For wind rose diagrams
import pytz                  # For handling time zones
import simplekml             # For generating KML files

# Other Imports
import sys
import csv
import argparse
import os
from datetime import datetime, timedelta

# main

# Parsing Location information

# Create an argument parser
parser = argparse.ArgumentParser(description="Script to accept launch site parameters.")

# Add arguments for latitude, longitude, and launch altitude
parser.add_argument("latitude", type=float, help="Latitude of the launch site (degrees)")
parser.add_argument("longitude", type=float, help="Longitude of the launch site (degrees)")
parser.add_argument("launch_altitude", type=float, help="Launch altitude of the site (meters)")

# Parse the arguments
args = parser.parse_args()

# Store arguments in variables
latitude_s = args.latitude
longitude_s = args.longitude
launch_altitude_s = args.launch_altitude

# Using rocketpy to determine information

tomorrow = datetime.now() + timedelta(days=1)

# Create an environment using the forecast model
env = Environment(
    latitude=latitude_s,         # Latitude of the launch site
    longitude=longitude_s,       # Longitude of the launch site
    elevation=launch_altitude_s,           # Elevation above sea level (meters)
    date=tomorrow # Date and time of the launch (year, month, day, hour)
)

# Set the atmospheric model to "Forecast"
env.set_atmospheric_model(type="forecast", file="GFS")

# Range of Altitudes
max_altitude = 20000
altitudes = range(0, max_altitude, 10)

# Initializing Profiles
density_profile = []
pressure_profile = []
temperature_profile = []
speed_of_sound_profile = []
wind_speed_profile = []


# Extract specific atmospheric profiles
for altitude in altitudes:
    density = env.density(altitude)  # Get air density at the current altitude
    pressure = env.pressure(altitude)
    temperature = env.temperature(altitude)  
    speed_of_sound = env.speed_of_sound(altitude)  
    wind_speed = env.wind_speed(altitude)  
    density_profile.append(float(density))  # Add the density value to the list
    pressure_profile.append(float(pressure))
    temperature_profile.append(float(temperature))
    speed_of_sound_profile.append(float(speed_of_sound))
    wind_speed_profile.append(float(wind_speed))

# Writing Information to csv file

# Ensure the "Environment" folder exists in the script's directory
folder_name = "Environment"
if not os.path.exists(folder_name):
    os.makedirs(folder_name)  # Create the folder if it doesn't exist

# Define the path to the CSV file inside the "Environment" folder
csv_filename = os.path.join(folder_name, "EnvironmentData.csv")

# Write data to the CSV file
with open(csv_filename, mode="w", newline="") as file:
    writer = csv.writer(file)
    
    # Write column titles
    writer.writerow(["Altitude", "Density", "Pressure", "Temperature", "Speed of Sound", "Wind Speed"])
    
    # Write rows of altitude and density data
    for alt, dens, pres, temp, sos, wspeed in zip(altitudes, density_profile, pressure_profile, temperature_profile, speed_of_sound_profile, wind_speed_profile):
        writer.writerow([alt, dens, pres, temp, sos, wspeed])
