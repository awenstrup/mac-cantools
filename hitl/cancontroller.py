# Base python
from configparser import ConfigParser
import csv
import os
import sys
import time
import threading
from typing import Callable, Tuple
import logging

# Extended python
import cantools
import can

# Project Imports 
from hitl.utils import artifacts_path, get_logging_config

config = ConfigParser(interpolation=None)
config.read(os.path.join(artifacts_path, "config.ini"))

class CANController:
    """High level python object to interface with hardware.

    The CANController works **passively** for reading signals; you create one with a path to a DBC, and it keeps track of all states automatically

    :param str channel: The name of the can channel to use.

    :param int bitrate: The bitrate of the can bus. Defaults to 500K.

    :param str can_spec_path: The path to the can spec. For now, we only support ``.dbc`` files. Should be
        stored in ``artifacts``.
    """

    def __init__(
        self, 
        channel: str = config.get("HARDWARE", "can_channel", fallback="PCAN_USBBUS1"), 
        bitrate: int = config.get("HARDWARE", "can_bitrate", fallback=500000), 
        can_spec_path: str = config.get("PATH", "dbc_path", fallback = ""), 
        ext_callback = None  # allow users to define custom RX callback
    ):
        # Create logger
        get_logging_config()
        self.log = logging.getLogger(name=__name__)

        # Create empty list of periodic messages
        self.periodic_messages = {}

        # Parse .dbc and generate internal state dictionary
        self.message_of_signal, self.signals, self.unknown_messages = self._create_state_dictionary(can_spec_path)

        # Start listening
        interface = "pcan"

        try:
            self.can_bus = can.interface.Bus(
                channel=channel, 
                interface=interface, 
                bitrate=bitrate
            )
        except can.interfaces.pcan.pcan.PcanCanInitializationError:
            raise Exception("CAN interface initialization failed. Check if your device is plugged in.")

        global reset_listener
        
        self.listener = threading.Thread(
            target=self._listen,
            name="listener",
            kwargs={
                "can_bus": self.can_bus,
                "callback": self._parse_frame,
                "ext_callback": ext_callback,
            },
        )
        self.listener.daemon = True
        self.listener.start()

    def get_state(self, signal):
        """
        Get the state of a can signal on the car
        """
        try:
            msg = self.message_of_signal[signal]
            return self.signals[msg][signal]
        except KeyError:
            self.log.debug("Signal not found in .dbc file, interpreting as message ID...")
            try:
                msg = self.unknown_messages[signal]
                return msg
            except KeyError:
                raise Exception(f"Cannot get state of signal '{signal}'. It wasn't found.")

    def set_state(self, signal, value):
        """
        Set the state of a can signal on the car

        """
        try:
            #find message name
            msg_name = self.message_of_signal[signal] 

            #update signals dict to new value and pull out message to send
            self.signals[msg_name][signal] = value
            msg = self.db.get_message_by_name(msg_name)

            #encode message data and create message
            data = msg.encode(self.signals[msg_name])
            message = can.Message(arbitration_id=msg.frame_id, data=data)

            #send message
            if msg_name in self.periodic_messages:
                self.periodic_messages[msg_name].modify_data(message)
            else:
                self.can_bus.send(message)
        except KeyError:
            raise Exception(f"Cannot set state of signal '{signal}'. It wasn't found.")

    def set_periodic(self, msg_name, period):
        """
        Set a message to be send periodically, at a specified period

        All states in the message should be set before starting a periodic broadcast,
        though they can be changed without interrupting the periodic broadcast.
        """
        if msg_name not in self.signals:
            raise Exception(f"Message {msg_name} not found in messages.")

        if msg_name in self.periodic_messages:
            raise Exception(f"Message {msg_name} is already being sent periodically.")

        #update signals dict to new value and pull out message to send
        msg = self.db.get_message_by_name(msg_name)

        #encode message data and create message
        data = msg.encode(self.signals[msg_name])
        message = can.Message(arbitration_id=msg.frame_id, data=data)

        #send message
        send_task = self.can_bus.send_periodic(message, period)
        self.periodic_messages[msg_name] = send_task

    
    def reset_rx_thread(self, ext_callback = None):
        reset_listener = True

        self.listener = threading.Thread(
            target=self._listen,
            name="listener",
            kwargs={
                "can_bus": self.can_bus,
                "callback": self._parse_frame,
                "ext_callback": ext_callback,
            },
        )
        self.listener.daemon = True
        self.listener.start()

    def playback(self, path, initial_time=0):
        """
        Play back a CSV log of CAN messages on the interface assigned to CANController object.

        :param path: Path to CSV log file

        :param initial_time: Timestamp of the first CAN message in the CSV file - can be useful 
                             if you only want to test a certain portion of CAN messages. Defaults to 0.
        
        Designed to input messages in following format:

        Timestamp,arbitration_id,signals(0 to 255 value)...

        """
        #Reading and parsing csv file
        log_file = open(path, 'r')
        csvreader = csv.reader(log_file)
        messages = []
        for row in csvreader:
            #remove empty elements from list
            row = list(filter(None, row)) 
            #convert strings to integers
            row = [int(i) for i in row]
            #check if row is within time range
            if row[0] >= initial_time:
                 messages.append(row)
        log_file.close()
        prev_time = initial_time


        start_time = time.time_ns() # initial time
        row = 0 # initial row
        while row < len(messages):
            #calculate time elapsed and check if next message should be sent
            time_elapsed = (time.time_ns() - start_time)/1000000
            if time_elapsed >= int(messages[row][0]):
                #Pull out message data from csv
                data = messages[row][2:]
                #create CAN message
                message = can.Message(arbitration_id=messages[row][1], data=data)
                #send message
                self.can_bus.send(message)
                #increment row
                row += 1


    def _create_state_dictionary(self, path: str) -> None:
        """Generate self.message_of_signal and self.signals

        :param str path: Path the the CAN spec file
        """
        message_of_signal = {}  # signal_name (str): message name (str)
        signals = {}  # message name (str): signals (dict(str: any))
        unknown_messages = {}  # can id (int): data (Message)

        # Create database that has all the messages in the dbc file in type message
        if path:
            self.db = cantools.database.load_file(path)

            # Iterates through messages to create state dictionaries
            for msg in self.db.messages:
                if msg.name not in message_of_signal:
                    signals[msg.name] = {}
                for sig in msg.signals:
                    signals[msg.name][sig.name] = 0
                    message_of_signal[sig.name] = msg.name

        return message_of_signal, signals, unknown_messages

    def _parse_frame(self, msg):
        """
        Callback function to the listener thread.
        
        Parses through a CAN frame and updates the self.states dictionary
        """
        try:
            # Get the message name
            msg_name = self.db.get_message_by_frame_id(msg.arbitration_id).name

            # Decode the data
            data = self.db.decode_message(msg.arbitration_id, msg.data)

            # Update the state dictionary
            self.signals[msg_name].update(data)
        except:
            # If signal not defined in DBC, track raw frame data
            self.unknown_messages[msg.arbitration_id] = msg

        
    def _listen(self, can_bus: can.Bus, callback: Callable, ext_callback: Callable) -> None:
        """Thread that runs all the time to listen to CAN messages

        References:
          - https://python-can.readthedocs.io/en/master/interfaces/socketcan.html
          - https://python-can.readthedocs.io/en/master/
        """
        if ext_callback:
            def rx_function(msg):
                callback(msg)
                ext_callback(msg)
        else:
            def rx_function(msg):
                callback(msg)

        reset_listener = False
        while not reset_listener:
            msg = can_bus.recv(1)  # 1 second receive timeout
            if msg:
                rx_function(msg)

    
    def __del__(self):
        try:
            self.can_bus.shutdown()
        except:
            pass
