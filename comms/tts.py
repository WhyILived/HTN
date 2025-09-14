from pynput import keyboard
import os
from dotenv import load_dotenv
from elevenlabs.client import ElevenLabs
from playsound import playsound
import tempfile

# Braille mapping
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

# Load environment variables
load_dotenv()
api_key = os.environ.get("ELEVENLABS_API_KEY")

# State
pressed_keys = set()
word = ""
sentence = ""
final_sentence = {"text": None}

# Hardcoded sentences (demo)
hardcodedsentences = [
    "Hi, my name is Sy",
    "How are you doing?",
    "It was a great night",
    "Hack the north was a great experience",
    "My project is called 'beyond sight'",
    "Can you help me?",
    "Nice to meet you!",
    "Goodbye!",
    "Good morning!",
    "Cool!"
]

# Mapping for numpad vk → numbers
NUMPAD_MAP = {96 + i: i for i in range(10)}  # 96=0, 105=9


def on_press(key):
    global pressed_keys
    try:
        if hasattr(key, "char") and key.char and key.char.isdigit():
            pressed_keys.add(int(key.char))
        elif hasattr(key, "vk") and key.vk in NUMPAD_MAP:
            pressed_keys.add(NUMPAD_MAP[key.vk])
            pressed_keys.add("numpad")  # to differentiate from top row numbers
    except Exception:
        pass


def on_release(key):
    global pressed_keys, word, sentence, final_sentence
    try:
        if pressed_keys:
            if "numpad" in pressed_keys:
                pressed_keys.remove("numpad")
                pressed_key = pressed_keys.pop()
                
                final_sentence["text"] = hardcodedsentences[pressed_key]
                return False  # Stop listener
                
            chord = frozenset(pressed_keys)
            letter = BRAILLE_MAP.get(chord, "")
            if letter == " ":
                sentence += word + " "
                word = ""
            else:
                word += letter
            pressed_keys.clear()
    except Exception:
        pass

    if key == keyboard.Key.esc:
        if word:
            sentence += word
        final_sentence["text"] = sentence.strip()
        return False


def capture_sentence():
    global word, sentence
    word, sentence = "", ""
    print("Start typing your sentence in Braille (Press ESC to finish):")
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()
    return final_sentence["text"]


def take_input():
    client = ElevenLabs(api_key=api_key)
    all_text = ""

    while True:
        n = input("Press Enter to type with Braille (or * to quit): ")
        if n == "*":
            break

        text = capture_sentence()
        print("Got text:", text)

        # Convert text to speech
        audio_bytes = client.text_to_speech.convert(
            text=text,
            voice_id="JBFqnCBsd6RMkjVDRZzb",
            model_id="eleven_multilingual_v2",
            output_format="mp3_44100_128",
        )

        # Save temp file + play
        with tempfile.NamedTemporaryFile(delete=False, suffix=".mp3") as temp_file:
            for chunk in audio_bytes:
                if chunk:
                    temp_file.write(chunk)
            temp_path = temp_file.name

        playsound(temp_path)
        os.remove(temp_path)
        
        all_text += text + " "
    
    return all_text.strip()


if __name__ == "__main__":
    take_input()
