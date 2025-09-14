import time

# 7 8
# 4 5
# 1 2

BRAILLE_MAP_INVERTED = {
    "a": {7},
    "b": {7, 4},
    "c": {7, 8},
    "d": {7, 8, 2},
    "e": {7, 2},
    "f": {7, 4, 8},
    "g": {7, 4, 8, 5},
    "h": {7, 4, 5},
    "i": {4, 8},
    "j": {4, 8, 5},
    "k": {7, 1},
    "l": {7, 4, 1},
    "m": {7, 8, 1},
    "n": {7, 8, 5, 1},
    "o": {7, 1, 5},
    "p": {7, 4, 8, 1},
    "q": {7, 4, 8, 1, 2},
    "r": {7, 4, 1, 5},
    "s": {4, 8, 1},
    "t": {4, 8, 1, 5},
    "u": {7, 1, 2},
    "v": {7, 4, 1, 2},
    "w": {4, 8, 5, 2},
    "x": {7, 8, 1, 2},
    "y": {7, 8, 1, 2, 5},
    "z": {7, 1, 5, 2},
    " ": {0}
}

def send_to_motor(letter):
    # Placeholder function to simulate sending a letter to a motor
    if letter == "!":
        print("Motor reset command received")
        time.sleep(1)  # Simulate time taken to reset motor
    if letter in BRAILLE_MAP_INVERTED:
        print(f"Motor received letter: {BRAILLE_MAP_INVERTED.get(letter)}")
        time.sleep(0.25)  # Simulate time taken to send to motor