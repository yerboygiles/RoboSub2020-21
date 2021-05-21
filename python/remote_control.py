#!python3
# Author: Theodor Giles
# Created: 5/21/21
# Description:
# REMOTE CONTROOOOOOL


def get_wasd_input() -> int:
    directval = input("WASD")
    if directval == "w" or directval == "W":
        directval = 1
    elif directval == "a" or directval == "A":
        directval = 2
    elif directval == "s" or directval == "S":
        directval = 3
    elif directval == "d" or directval == "D":
        directval = 4
    else:
        directval = 0
    return directval
