#!/usr/bin/env python3
import sys
import os

if __name__ == "__main__":
    # Ensure the workspace root is in PYTHONPATH
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from teleop_quest.main_quest import main
    main()
