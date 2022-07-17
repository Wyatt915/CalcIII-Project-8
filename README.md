# Setup
I use Linux. These steps *should* work for MacOS. No clue about windows.

  * Install python on your system
  * set up a virtualenv with `python3 -m venv venv`
    * Activate the virtual environment with `source venv/bin/activate`
    * Install [Manim](https://github.com/ManimCommunity/manim/) with `pip install manim`
    * Create an animation with `manim -pql field.py LineIntegral`
  * Type `deactivate` to leave the virtualenv
