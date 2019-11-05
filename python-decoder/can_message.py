#!venv/bin/python3

import os.path

class canMessage():
    def __init__(self):
        self.raw_messages_list = []

    def handle_messages(self, raw_filename):
        for raw_message in self.raw_messages_list:
            decoded_message = self.read_bytes(raw_message[0].split('|'))
            timestamp = raw_message[1]

            self.append_to_log(raw_filename, decoded_message, timestamp)

    def read_bytes(self, message):
        pass

    def append_to_log(self, raw_filename, decoded_message, timestamp):
        complete_filename = "Logs/Tratados/{:s}/{:s}".format(
            raw_filename.split('.')[0],
            self.log_filename
        )

        if(not os.path.isfile(complete_filename)):
            os.system("mkdir -p Logs/Tratados/{:s}".format(raw_filename.split('.')[0]))

            with open(complete_filename, 'w') as file:
                file.write(self.log_header)
                file.close()

        with open(complete_filename, 'a') as file:
            file.write("{:s};{:s}\n".format(decoded_message, timestamp))
            file.close()