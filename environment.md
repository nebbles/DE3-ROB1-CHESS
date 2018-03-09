# Requirements and Environments

This project utilises some specific packages to run properly. In order to run this source code, it is recommended you use a virtual environment to install the required packages, so that you can delete it later without affecting your base Python installation.

We recommend you use [virtualenvwrapper](https://virtualenvwrapper.readthedocs.io/en/latest/index.html) to manage your virtual environments. Once set up, create a virtual environment for this project (when in the project directory) using ``mkvirtualenv CHESS_PROJECT``.

You should see a ``(CHESS_PROJECT)`` in your terminal window indicating you are in the virtual environment. You can now install the requirements of the project using ``pip install -r requirements.txt``. **Note: this project uses the Python 2.7 interpreter due to the dependency on ROS.**
