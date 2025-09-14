from flask import Flask, jsonify
from RealtimeSTT import AudioToTextRecorder
from motor_connection import send_to_motor
import threading
import multiprocessing
from pymongo.mongo_client import MongoClient
import os
from dotenv import load_dotenv
from tts import take_input
from flask_cors import CORS

load_dotenv()

# --- MongoDB setup ---
uri = os.environ.get("MONGODB_URI")

client = MongoClient(uri)
db = client.htn2025  # Use your database
conversations_collection = db.conversations

conversation_id = None

# --- Shared state ---
latest_text = ""
letters_sent = 0
running = True

# --- Flask app ---
app = Flask(__name__)
CORS(app)

@app.route("/status")
def status():
    return jsonify({
        "latest_text": latest_text,
        "letters_sent": letters_sent,
        "running": running
    })
    
@app.route("/all_messages")
def all_messages():
    cursor = conversations_collection.find({}, {"_id": 0, "messages": 1})
    messages = list(cursor)  # convert Cursor to list
    return jsonify(messages)

def run_flask():
    app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False)

# --- RealtimeSTT callback ---
def process_text(text):
    global latest_text, letters_sent, conversation_id
    text = text.lower().strip().replace("\n", " ").replace("\r", " ")
    latest_text = text
    print(f"Processed text: {text}")
    for letter in text:
        send_to_motor(letter)
        letters_sent += 1
        
    # --- MongoDB: save conversation ---
    if conversation_id is None:
        # First entry: create new conversation document
        result = conversations_collection.insert_one({
            "full_text": text,
            "messages": [text]
        })
        conversation_id = result.inserted_id
    else:
        # Update existing conversation
        conversations_collection.update_one(
            {"_id": conversation_id},
            {
                "$push": {"messages": text},
                "$set": {"full_text": {"$concat": ["$full_text", text]}}
            }
        )
    
    # take input from user after processing text
    user_text = take_input()
    
    if user_text:
        # Append user input to the conversation
        if conversation_id is None:
            # Just in case, create new conversation
            result = conversations_collection.insert_one({
                "full_text": user_text,
                "messages": [user_text]
            })
            conversation_id = result.inserted_id
        else:
            conversations_collection.update_one(
                {"_id": conversation_id},
                {
                    "$push": {"messages": user_text},
                    "$set": {"full_text": {"$concat": ["$full_text", user_text]}}
                }
            )
        print(f"User input saved: {user_text}")
    

# --- Main program ---
def main():
    global running
    print("Say 'Computer' to start speaking...")

    recorder = AudioToTextRecorder(
        wake_words="computer",
        enable_realtime_transcription=True
    )

    while True:
        recorder.text(process_text)

# --- Entry point ---
if __name__ == "__main__":
    # Necessary for Windows multiprocessing
    multiprocessing.freeze_support()

    # Start Flask in a separate thread
    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()

    # Start main loop
    main()
