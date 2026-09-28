# Settings
# SHELL := /bin/bash

# Variables
MICROMAMBA = micromamba
ENV = env
VENV = eee4022s
BOLD = $$(tput bold)
RESET = $$(tput sgr0)
# source ~/.bashrc && micromamba

.PHONY: micromamba dep run clean

# Create a Python virtual environment, assuming micromamba is installed
micromamba:
	@$(MICROMAMBA) env create -f $(ENV)/environment.yml
	@wget -P ./mediapipe_models https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task

# Install dependencies
dep:
	@pip install -r $(ENV)/requirements.txt
	@wget -P ./mediapipe_models https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task

# Run program
run:
	@$(MICROMAMBA) run -n $(VENV) python main.py

# Delete buildable files
clean:
	@echo "TODO: Implement"
