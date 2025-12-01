import sys
import os

def main():
    print("Hello from the Quadruped Robot environment!")
    print(f"Python version: {sys.version}")
    print(f"Current working directory: {os.getcwd()}")
    
    # Check if running in a virtual environment
    if sys.prefix != sys.base_prefix:
        print("Running in a virtual environment.")
    else:
        print("NOT running in a virtual environment.")

if __name__ == "__main__":
    main()
