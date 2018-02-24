class Print:  # Debug window giving information about the program and user state
    purple = '\033[95m'
    cyan = '\033[96m'
    darkcyan = '\033[36m'
    blue = '\033[94m'
    green = '\033[92m'
    yellow = '\033[93m'
    red = '\033[91m'
    bold = '\033[1m'
    underline = '\033[4m'
    end = '\033[0m'

    @staticmethod
    def clear_screen():  # Defines how to clear screen based on platform OS
        import os
        import platform
        current_platform = platform.system()
        if current_platform == 'Windows':
            os.system('cls')
        elif current_platform == 'Darwin':
            os.system('clear')
        else:
            print("\n" * 100)

    @staticmethod
    def warning(msg):
        print(Print.bold + Print.yellow + "Warning:    " + Print.end
              + Print.yellow + msg + Print.end)

    @staticmethod
    def danger(msg):
        print(Print.bold + Print.red + "DANGER:     " + Print.end
              + Print.red + msg + Print.end)

    @staticmethod
    def success(msg):
        print(Print.bold + Print.green + "Success:    " + Print.end
              + Print.green + msg + Print.end)

    @staticmethod
    def message(msg):
        print(Print.bold + Print.blue + "Message:    " + Print.end
              + Print.blue + msg + Print.end)
