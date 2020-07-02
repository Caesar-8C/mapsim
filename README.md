# Mapsim

## Prerequisites

Python3
Modules used:
pygame
numpy
networkx

## File structure
#### main.py
Main script that contains Map class that starts pygame, builds the graph and runs drawing of objects on the screen.

#### eventHandlerLoop.py
Extension of Map class. Handles pygame events.

#### draw.py
Contains extension for Map class: all custom draw functions for corridors, agents etc. Also contains Draw class that handles scaling and shifting of the screen.

#### utils.py
Contains auxiliary classes for Robot, Agents and Rooms.

#### fileManager.py
Extension of Map class that handles loading the map from a file and saving it back.

#### data/graph2.txt
Sample map description.
