# Compatibility

We are going to be using Python 2.7 to run our project due to the critical dependency on ROS which currently only supports version 2.7.

We need to maintain our support for Python 3.x so keep writing your print statements as print(str) and keep test running your code via a 3.x interpreter.

Based on this webpage (http://python-future.org/imports.html), to make your Python 3.x code compatible with 2.7 add the following lines to the very top of your python files (keep the order as is):

```python
from __future__ import absolute_import, division, print_function
from builtins import *
```
