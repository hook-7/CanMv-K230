# Gemini Project Analysis: Real-Time Object Detection

## Project Overview

This project is designed for real-time object detection. It contains two main variations:

1.  **Embedded Version (`20class_detect.py`):** This script is written in MicroPython for Maix hardware (e.g., K210 series). It utilizes the on-chip KPU (Kendryte Processing Unit) to run a custom `.smodel` YOLOv2 model for efficient, low-power object detection.
2.  **Desktop Version (`object_detection.py`):** This is a standard Python script that uses the OpenCV DNN module to perform object detection on a desktop computer. It uses a webcam as the video source and runs a YOLOv2 model using `.cfg` and `.weights` files.

The goal is to detect 20 different classes of objects, as defined in `20_categories_of_object_classification_models/labels.txt`.

**Key Technologies:**
*   Python 3
*   MicroPython (for embedded script)
*   OpenCV
*   NumPy
*   uv (for package management)

## Building and Running

This project uses `uv` and a `.venv` for dependency management.

### Dependencies

The required Python packages are:
*   `numpy`
*   `opencv-python`
*   `psutil`

These can be installed by activating the virtual environment.

### Running the Desktop Version

The primary script for use on a standard computer is `object_detection.py`.

**1. Prerequisites:**

*   Ensure you have a webcam connected.
*   Download the pre-trained model weights file:
    *   **File:** `tiny-yolo-voc.weights`
    *   **Download Link:** `https://raw.githubusercontent.com/DevendraPratapYadav/tiny_yolo/master/tiny-yolo-voc.weights`
    *   **IMPORTANT:** Place this file in the root directory of the project, alongside `object_detection.py`.

**2. Run Command:**

To run the object detection script, execute the following command from the project's root directory:

```bash
# Activate the virtual environment and run the script
.\.venv\Scripts\python.exe object_detection.py
```

Press 'q' in the display window to quit the application.

### Running the Embedded Version

The `20class_detect.py` script is intended for a Maix development board. It cannot be run on a standard PC. It must be flashed and executed on the target hardware using the appropriate Maix IDE or tools (e.g., MaixPy IDE, kflash).

## Development Conventions

*   **Environment:** The project is set up to use a virtual environment located in the `.venv` directory. All development should be done with this environment activated.
*   **Configuration:** The object detection models are configured via `.cfg` files (for OpenCV) or are self-contained in `.smodel` files (for Maix). The class labels are stored in a simple `labels.txt` file.
*   **Scripts:** The original scripts (`20class_detect.py`, `20class_detect_o.py`) appear to be experimental versions for the embedded platform. The more stable and general-purpose script for desktop use is `object_detection.py`.

始终使用中文回答用户问题
