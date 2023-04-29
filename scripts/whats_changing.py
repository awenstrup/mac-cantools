"""
Script to show which CAN messages/signals start changing 
when a user stimulates the vehicle.

Any messages/signals that change during the initial window,
(defined by WAIT_TIME), are assumed to be 'always changing'.

Any messages that are not sent, or are always sent with the same value
during the initial window, are treated as static.

Once the initial window ends, the script will start printing all
changes to messages/signals that were determined to be static.

Example: 
* run whats_changing.py
* wait 5 seconds to learn which messages are always changing
* press the accelerator pedal
* watch the accelerator pedal signal change

Today, only whole messages are tracks (isolating changes to individual
signals is not yet supported)
"""
import time
from hitl.cancontroller import CANController

WAIT_TIME = 5

def ext_callback_1(msg):
    id = msg.arbitration_id
    if id in changing:
        pass
    elif id in static:
        if static[id] != msg.data:
            changing.add(id)
            static.pop(id)
    else:
        static[id] = msg.data


def ext_callback_2(msg, static):
    id = msg.arbitration_id
    if msg in static and msg.data != static[id]:
        print(f"ID: {id}, Data: {msg.data}")


if __name__ == "__main__":
    # Learn which messages are static vs change
    changing = set()
    static = {}

    can = CANController(ext_callback=ext_callback_1)
    start_time = time.time()
    while time.time() - WAIT_TIME < start_time:
        time.sleep(0.1)

    # Clean up 
    del(changing)
    can.reset_rx_thread(ext_callback_2)
    print("Learning complete!")

    # Start printing messages that change
    while True:
        time.sleep(0.1)