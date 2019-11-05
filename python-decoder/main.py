#!venv/bin/python3

import os

from message_list import *


def find_log_files(log_folders):
    log_files = []

    for folder in log_folders:
        files = os.listdir("Logs/Brutos/{:s}".format(folder))

        for file in files:
            log_files.append("Logs/Brutos/{:s}/{:s}".format(folder, file))

    return(log_files)


def get_corrupted_percentage(filename, total_messages, corrupted_messages):
    print("File {:s} had {:.4f}% of messages corrupted!".format(
        filename,
        (corrupted_messages / total_messages) * 100
    ))


def main():
    log_folders = os.listdir("Logs/Brutos")

    log_files = find_log_files(log_folders)

    for log_file in log_files:
        with open(log_file, 'r') as file:
            raw_lines = file.readlines()
            file.close()

        messages_with_error = 0
        for line in raw_lines:
            try:
                message_id, message_data, message_timestamp = line.replace('\n', '').split(';')

                try:
                    message_list[int(message_id, 16)].raw_messages_list.append((message_data, message_timestamp))

                # message_id not mapped! #
                except KeyError as error:
                    pass

            # corrupted message! #
            except ValueError as error:
                messages_with_error += 1

        short_filename = '/'.join(log_file.split('/')[2:])

        get_corrupted_percentage(short_filename, len(raw_lines), messages_with_error)

        for message in message_list:
            message_list[message].handle_messages(short_filename)


main()
