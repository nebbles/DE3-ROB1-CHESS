***************************
DE Programming Ground Rules
***************************

1. **Always put code on GitHub**, never use Google Drive or direct messaging.

  a. This means when working on a particular function, you will be working in a smaller team and contributing to a particular branch you set up together.
  b. Working on a branch in a small team (maybe 2-3) requires coordination so that you don’t create conflicts when pushing your changes to the branch.
  c. Remember to push changes to a branch, AND sync!

2. **When writing Python, keep to the** `PEP8 Python styling guide`_.

   This style guide is usually enabled in PyCharm to help you. This means you get a squiggle and a yellow/brown mark next to a violation (settings are in ``Preferences > Editor > Inspections > Python``).
   The key takeaways are:

  a. Naming conventions using: ClassNames, function_names, variable_names
  b. `White space`_ (PyCharm will give you help here)
  c. Tabs vs Space, PyCharm actually converts the tab key, and 4 spaces, into the same thing by default (so just don’t change this from default values, and you can use either).

3. **Comment your code! Plus see bullet 4.**

  a. This can be in two main forms, inline or block. `Again, keep to PEP8`_ to avoid differences between code. It’s the simplest one to keep to.

4. **Document your code!**

   Use the PEP257 docstring convention and keep to it well - you can read the `short version PEP257`_, or the `complete PEP257`_.

  a. This means that Sphinx (an automatic documentation generator) will work and display things properly.
  b. `An example of this can be seen on GitHub`_. It has library, modules, and comments.

5. UML Diagrams... USE THEM

6. `Handle errors properly with exceptions`_. If necessary, build your own errors for safety measures and use try..except..raise blocks in your code.

7. Global variables are bad. Functions are better than types. Objects are likely to be better than complex data structures.

8. Avoid ``from X import *`` as much as possible. It’s much better to import a specific function ``from X import Y`` or to use it in context of your code ``import X; X.Y(arg)``. Otherwise we will need to `start handling lists of import names`_.

.. _`PEP8 Python styling guide`: https://www.python.org/dev/peps/pep-0008/
.. _`White space`: https://www.python.org/dev/peps/pep-0008/#whitespace-in-expressions-and-statements
.. _`Again, keep to PEP8`: https://www.python.org/dev/peps/pep-0008/#comments
.. _`short version PEP257`: http://docs.python-guide.org/en/latest/writing/documentation/
.. _`complete PEP257`: https://www.python.org/dev/peps/pep-0257/
.. _`An example of this can be seen on GitHub`: https://github.com/brandon-rhodes/sphinx-tutorial/tree/master/triangle-project/trianglelib
.. _`Handle errors properly with exceptions`: https://docs.python.org/2/tutorial/errors.html
.. _`start handling lists of import names`: https://docs.python.org/3/tutorial/modules.html#importing-from-a-package
