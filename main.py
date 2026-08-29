import os
import sys

# Add the workspace root to python path
sys.path.append(os.path.abspath(os.path.dirname(__file__)))

from robot.main import main

if __name__ == "__main__":
    main()
