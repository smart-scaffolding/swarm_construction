from pathlib import Path
from install_requires import convert

path = Path(__file__).parent
convert("requirements.txt", "setup.py", path)
