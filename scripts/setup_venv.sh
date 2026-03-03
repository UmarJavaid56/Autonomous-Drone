#!/bin/bash

# Setup Python Virtual Environment Script
# Creates a portable Python venv in the project directory

set -e  # Exit on any error

# Get the project root directory (parent of scripts directory)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
VENV_DIR="$PROJECT_ROOT/.venv"

echo "=========================================="
echo "Setting up Python Virtual Environment"
echo "=========================================="

# Check Python version
if ! command -v python3 &> /dev/null; then
    echo "ERROR: python3 not found. Please install Python 3."
    exit 1
fi

PYTHON_VERSION=$(python3 --version | cut -d' ' -f2)
echo "Python version: $PYTHON_VERSION"

# Check if python3-venv is installed
if ! python3 -m venv --help &> /dev/null; then
    echo "ERROR: python3-venv module not found."
    echo "Install it with: sudo apt install python3-venv"
    exit 1
fi

# Create venv if it doesn't exist
if [ ! -d "$VENV_DIR" ]; then
    echo "Creating virtual environment at: $VENV_DIR"
    python3 -m venv "$VENV_DIR"
    echo "Virtual environment created"
else
    echo "Virtual environment already exists at: $VENV_DIR"
fi

# Activate venv and upgrade pip, setuptools, wheel
echo "Upgrading pip, setuptools, and wheel..."
source "$VENV_DIR/bin/activate"
pip install --upgrade pip setuptools wheel

# Install project-specific Python packages if needed
# Add any project-specific Python dependencies here
# pip install package_name

echo "Virtual environment setup complete!"
echo ""
echo "To activate the virtual environment:"
echo "  source $VENV_DIR/bin/activate"
echo ""
echo "To deactivate:"
echo "  deactivate"
