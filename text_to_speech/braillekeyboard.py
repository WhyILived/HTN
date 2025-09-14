from pynput import keyboard

BRAILLE_MAP = {
    frozenset([7]): "a",
    frozenset([7, 4]): "b",
    frozenset([7, 8]): "c",
    frozenset([7, 8, 2]): "d",
    frozenset([7, 2]): "e",
    frozenset([7, 4, 8]): "f",
    frozenset([7, 4, 8, 5]): "g",
    frozenset([7, 4, 5]): "h",
    frozenset([4, 8]): "i",
    frozenset([4, 8, 5]): "j",
    frozenset([7, 1]): "k",
    frozenset([7, 4, 1]): "l",
    frozenset([7, 8, 1]): "m",
    frozenset([7, 8, 5, 1]): "n",
    frozenset([7, 1, 5]): "o",
    frozenset([7, 4, 8, 1]): "p",
    frozenset([7, 4, 8, 1, 2]): "q",
    frozenset([7, 4, 1, 5]): "r",
    frozenset([4, 8, 1]): "s",
    frozenset([4, 8, 1, 5]): "t",
    frozenset([7, 1, 2]): "u",
    frozenset([7, 4, 1, 2]): "v",
    frozenset([4, 8, 5, 2]): "w",
    frozenset([7, 8, 1, 2]): "x",
    frozenset([7, 8, 1, 2, 5]): "y",
    frozenset([7, 1, 5, 2]): "z",
    frozenset([0]): " "   # space
}

pressed_keys = set()
word = ""      
sentence = ""  
final_sentence = {"text": None}  # container to pass out of listener


def on_press(key):
    try:
        if hasattr(key, 'char') and key.char:
            pressed_keys.add(key.char)
    except AttributeError:
        pass


def on_release(key):
    global pressed_keys, word, sentence, final_sentence

    if hasattr(key, 'char') and key.char:
        try:
            chord = frozenset(int(k) for k in pressed_keys)
        except ValueError:
            chord = frozenset()
        if chord:
            letter = BRAILLE_MAP.get(chord, "")
            if letter == " ":
                sentence += word + " "
                print("Current word:", word)
                print("Current string:", sentence)
                word = ""
            else:
                word += letter

            print("→", letter)
            print("Word so far:", word)
        pressed_keys = set()

    if key == keyboard.Key.esc:
        if word:
            sentence += word
        final_sentence["text"] = sentence  # store final sentence
        print("\nFinal sentence:", sentence)
        return False  # stop listener


def capture_sentence():
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()
    return final_sentence["text"]


if __name__ == "__main__":
    result = capture_sentence()
    print("Returned sentence:", result)