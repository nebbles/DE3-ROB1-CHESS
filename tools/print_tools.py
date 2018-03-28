import os
import platform


class Print:
    _purple = '\033[95m'
    _cyan = '\033[96m'
    _darkcyan = '\033[36m'
    _blue = '\033[94m'
    _green = '\033[92m'
    _yellow = '\033[93m'
    _red = '\033[91m'
    _bold = '\033[1m'
    _underline = '\033[4m'
    _end = '\033[0m'

    @staticmethod
    def clear_screen():  # Clears screen based on platform OS
        current_platform = platform.system()
        if current_platform == 'Windows':
            os.system('cls')
        elif current_platform == 'Darwin':
            os.system('clear')
        else:
            print("\n" * 100)

    @staticmethod
    def purple(msg):
        print(Print._purple + msg + Print._end)

    @staticmethod
    def cyan(msg):
        print(Print._cyan + msg + Print._end)

    @staticmethod
    def darkcyan(msg):
        print(Print._darkcyan + msg + Print._end)

    @staticmethod
    def blue(msg):
        print(Print._blue + msg + Print._end)

    @staticmethod
    def green(msg):
        print(Print._green + msg + Print._end)

    @staticmethod
    def yellow(msg):
        print(Print._yellow + msg + Print._end)

    @staticmethod
    def red(msg):
        print(Print._red + msg + Print._end)

    @staticmethod
    def bold(msg):
        print(Print._bold + msg + Print._end)

    @staticmethod
    def underline(msg):
        print(Print._underline + msg + Print._end)

    @staticmethod
    def warning(msg):
        print(Print._bold + Print._yellow + "Warning:    " + Print._end
              + Print._yellow + msg + Print._end)

    @staticmethod
    def danger(msg):
        print(Print._bold + Print._red + "DANGER:     " + Print._end
              + Print._red + msg + Print._end)

    @staticmethod
    def success(msg):
        print(Print._bold + Print._green + "Success:    " + Print._end
              + Print._green + msg + Print._end)

    @staticmethod
    def message(msg):
        print(Print._bold + Print._blue + "Message:    " + Print._end
              + Print._blue + msg + Print._end)
