#!python3
# Author: Theodor Giles
# Created: 5/21/21
# Last Edited 5/26/21
# Description:
# REMOTE CONTROL! probably will put this into the commander at some point, but it's nice to have as like a random input


def get_wasdqerv_directional() -> int:
    directval = input("MOVE: ")
    if directval == "w" or directval == "W":
        directval = 1
    elif directval == "a" or directval == "A":
        directval = 2
    elif directval == "s" or directval == "S":
        directval = 3
    elif directval == "d" or directval == "D":
        directval = 4
    elif directval == "q" or directval == "Q":
        directval = 5
    elif directval == "e" or directval == "E":
        directval = 6
    elif directval == "r" or directval == "R":
        directval = 7
    elif directval == "v" or directval == "V":
        directval = 8
    elif directval == "o" or directval == "O":
        directval = -1
    elif directval == "u" or directval == "U":
        directval = -2
    else:
        directval = 0
    return directval
